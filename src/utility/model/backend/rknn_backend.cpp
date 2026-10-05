#include "utility/model/backend/rknn_backend.hpp"

#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <rknn_api.h>
#include <string>
#include <utility>

#include <opencv2/imgproc.hpp>

namespace rmcs {

namespace {

    constexpr auto kModelExtension = ".rknn";

    /// Derive the `.rknn` model path from the configured model location, e.g.
    /// `shenzhen-0526.onnx` -> `shenzhen-0526.rknn`.
    auto resolve_model_path(const std::string& location) -> std::string {
        auto path = std::filesystem::path { location };
        path.replace_extension(kModelExtension);
        return path.string();
    }

    auto read_model(const std::string& path, std::vector<std::uint8_t>& buffer) -> bool {
        auto stream = std::ifstream { path, std::ios::binary | std::ios::ate };
        if (!stream.is_open()) return false;

        const auto size = stream.tellg();
        if (size <= 0) return false;

        buffer.resize(static_cast<std::size_t>(size));
        stream.seekg(0, std::ios::beg);
        stream.read(reinterpret_cast<char*>(buffer.data()), size);
        return static_cast<bool>(stream);
    }

    /// Normalize a queried tensor shape to 4 dims, treating a missing leading batch as 1.
    auto as_shape_4d(const rknn_tensor_attr& attr) -> std::array<std::size_t, 4> {
        if (attr.n_dims == 4) {
            return { attr.dims[0], attr.dims[1], attr.dims[2], attr.dims[3] };
        }
        if (attr.n_dims == 3) {
            return { 1, attr.dims[0], attr.dims[1], attr.dims[2] };
        }
        return { 0, 0, 0, 0 };
    }

    auto shape_to_string(const std::array<std::size_t, 4>& shape) -> std::string {
        return "[" + std::to_string(shape[0]) + ", " + std::to_string(shape[1]) + ", "
            + std::to_string(shape[2]) + ", " + std::to_string(shape[3]) + "]";
    }

    /// Whether the RKNN input tensor matches the configured dimensions. The `.rknn`
    /// records its own data format (the toolkit defaults to NHWC), so accept either
    /// the NCHW or the NHWC ordering of the same logical shape.
    auto input_matches(const rknn_tensor_attr& attr, const Dimensions& dimensions) -> bool {
        const auto batch   = static_cast<std::size_t>(dimensions.N);
        const auto channel = static_cast<std::size_t>(dimensions.C);
        const auto width   = static_cast<std::size_t>(dimensions.W);
        const auto height  = static_cast<std::size_t>(dimensions.H);

        const auto actual = as_shape_4d(attr);
        const auto nchw   = std::array<std::size_t, 4> { batch, channel, height, width };
        const auto nhwc   = std::array<std::size_t, 4> { batch, height, width, channel };
        return actual == nchw || actual == nhwc;
    }

} // namespace

RknnBackend::RknnBackend(ModelSpec spec)
    : spec_ { std::move(spec) } { }

RknnBackend::~RknnBackend() {
    {
        auto lock = std::lock_guard { mutex_ };
        stopping_ = true;
    }
    cv_.notify_all();
    for (auto& thread : threads_) {
        if (thread.joinable()) thread.join();
    }
    for (auto& worker : workers_) {
        if (worker.ctx == 0) continue;
        if (worker.input_mem != nullptr) rknn_destroy_mem(worker.ctx, worker.input_mem);
        if (worker.output_mem != nullptr) rknn_destroy_mem(worker.ctx, worker.output_mem);
        rknn_destroy(worker.ctx);
        worker.ctx = 0;
    }
}

auto RknnBackend::setup_worker(rknn_context ctx, Worker& worker)
    -> std::expected<void, std::string> {
    worker.ctx = ctx;

    auto io_num = rknn_input_output_num { };
    if (rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num)) != RKNN_SUCC) {
        return std::unexpected { "rknn_query(IN_OUT_NUM) failed" };
    }
    if (io_num.n_input != 1 || io_num.n_output != 1) {
        return std::unexpected { "Unexpected RKNN inputs/outputs: " + std::to_string(io_num.n_input)
            + "/" + std::to_string(io_num.n_output) };
    }

    if (rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &worker.input_attr, sizeof(worker.input_attr))
        != RKNN_SUCC) {
        return std::unexpected { "rknn_query(INPUT_ATTR) failed" };
    }
    if (!input_matches(worker.input_attr, spec_.dimensions)) {
        return std::unexpected { "RKNN input shape "
            + shape_to_string(as_shape_4d(worker.input_attr)) + " does not match ModelSpec" };
    }

    if (rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &worker.output_attr, sizeof(worker.output_attr))
        != RKNN_SUCC) {
        return std::unexpected { "rknn_query(OUTPUT_ATTR) failed" };
    }
    if (worker.output_attr.n_dims != 2 && worker.output_attr.n_dims != 3) {
        return std::unexpected { "Unexpected RKNN output rank: "
            + std::to_string(worker.output_attr.n_dims) + " shape "
            + shape_to_string(as_shape_4d(worker.output_attr)) };
    }
    if (out_shape_[0] == 0) {
        if (worker.output_attr.n_dims == 2) {
            out_shape_ = { 1, worker.output_attr.dims[0], worker.output_attr.dims[1] };
        } else {
            out_shape_ = { worker.output_attr.dims[0], worker.output_attr.dims[1],
                worker.output_attr.dims[2] };
        }
    }

    // Zero-copy input: bind a u8 NHWC buffer; `pass_through = 0` keeps the
    // model's normalization fused into the NPU. Zero-copy input is NHWC only.
    worker.input_attr.index        = 0;
    worker.input_attr.type         = RKNN_TENSOR_UINT8;
    worker.input_attr.fmt          = RKNN_TENSOR_NHWC;
    worker.input_attr.pass_through = 0;

    worker.input_mem = rknn_create_mem(ctx, worker.input_attr.size_with_stride);
    if (worker.input_mem == nullptr || worker.input_mem->virt_addr == nullptr) {
        return std::unexpected { "rknn_create_mem(input) failed" };
    }

    const auto stride = worker.input_attr.w_stride != 0
        ? static_cast<std::size_t>(worker.input_attr.w_stride) * 3
        : static_cast<std::size_t>(cv::Mat::AUTO_STEP);
    worker.input_view = cv::Mat {
        static_cast<int>(spec_.dimensions.H),
        static_cast<int>(spec_.dimensions.W),
        CV_8UC3,
        worker.input_mem->virt_addr,
        stride,
    };

    if (rknn_set_io_mem(ctx, worker.input_mem, &worker.input_attr) != RKNN_SUCC) {
        return std::unexpected { "rknn_set_io_mem(input) failed" };
    }

    // Zero-copy output: let the runtime write f32 results straight into this
    // buffer, which back InferOutput::data until the next inference.
    worker.output_attr.type = RKNN_TENSOR_FLOAT32;
    worker.output_mem       = rknn_create_mem(
        ctx, worker.output_attr.n_elems * static_cast<std::uint32_t>(sizeof(float)));
    if (worker.output_mem == nullptr || worker.output_mem->virt_addr == nullptr) {
        return std::unexpected { "rknn_create_mem(output) failed" };
    }

    if (rknn_set_io_mem(ctx, worker.output_mem, &worker.output_attr) != RKNN_SUCC) {
        return std::unexpected { "rknn_set_io_mem(output) failed" };
    }

    worker.output_elems = worker.output_attr.n_elems;
    return { };
}

auto RknnBackend::run_worker(Worker& worker) -> void {
    auto lock = std::unique_lock { mutex_ };
    while (true) {
        cv_.wait(lock, [&] { return stopping_ || worker.submitted > worker.completed; });
        if (stopping_) return;

        const auto generation = worker.submitted;
        lock.unlock();
        const auto result = rknn_run(worker.ctx, nullptr);
        lock.lock();

        worker.run_result = result;
        worker.completed  = generation;
        cv_.notify_all();
    }
}

auto RknnBackend::create(const ModelSpec& spec)
    -> std::expected<std::unique_ptr<InferBackend>, std::string> {
    auto backend = std::make_unique<RknnBackend>(spec);

    const auto model_path = resolve_model_path(spec.location);
    if (!read_model(model_path, backend->model_buf_)) {
        return std::unexpected { "Failed to read RKNN model: " + model_path };
    }

    auto& primary = backend->workers_[0];
    auto ret      = rknn_init(&primary.ctx, backend->model_buf_.data(),
        static_cast<std::uint32_t>(backend->model_buf_.size()), 0, nullptr);
    if (ret != RKNN_SUCC) {
        primary.ctx = 0;
        return std::unexpected {
            "rknn_init failed (" + std::to_string(ret) + "): " + model_path,
        };
    }
    ret = rknn_set_core_mask(primary.ctx, RKNN_NPU_CORE_0);
    if (ret != RKNN_SUCC) {
        return std::unexpected { "rknn_set_core_mask failed (" + std::to_string(ret) + ")" };
    }

    // The runtime owns its copy of the model from now on.
    backend->model_buf_.clear();
    backend->model_buf_.shrink_to_fit();

    if (auto setup = backend->setup_worker(primary.ctx, primary); !setup) {
        return std::unexpected { setup.error() };
    }

    // The second context shares the weights and is pinned to the other NPU core.
    auto& secondary = backend->workers_[1];
    ret             = rknn_dup_context(&primary.ctx, &secondary.ctx);
    if (ret != RKNN_SUCC) {
        return std::unexpected { "rknn_dup_context failed (" + std::to_string(ret) + ")" };
    }
    ret = rknn_set_core_mask(secondary.ctx, RKNN_NPU_CORE_1);
    if (ret != RKNN_SUCC) {
        return std::unexpected { "rknn_set_core_mask(second) failed (" + std::to_string(ret)
            + ")" };
    }
    if (auto setup = backend->setup_worker(secondary.ctx, secondary); !setup) {
        return std::unexpected { setup.error() };
    }

    for (std::size_t i = 0; i < backend->workers_.size(); ++i) {
        backend->threads_[i] =
            std::jthread { [ptr = backend.get(), i] { ptr->run_worker(ptr->workers_[i]); } };
    }

    return std::unique_ptr<InferBackend> { std::move(backend) };
}

auto RknnBackend::infer(const cv::Mat& input) noexcept -> std::expected<InferOutput, std::string> {
    try {
        if (input.empty() || input.type() != CV_8UC3) {
            return std::unexpected { "RKNN backend expects a non-empty u8 BGR image" };
        }
        if (input.cols != spec_.dimensions.W || input.rows != spec_.dimensions.H) {
            return std::unexpected { "RKNN input size does not match ModelSpec" };
        }

        const auto current = next_worker_;
        next_worker_       = current ^ 1;
        auto& target       = workers_[current];
        auto& previous     = workers_[current ^ 1];

        {
            auto lock = std::unique_lock { mutex_ };
            cv_.wait(lock, [&] { return stopping_ || target.submitted == target.completed; });
            if (stopping_) {
                return std::unexpected { "RKNN backend is stopping" };
            }
        }

        // Colour-convert straight into this core's zero-copy input memory; the
        // worker only has to issue `rknn_run` once it is handed over.
        cv::cvtColor(input, target.input_view, cv::COLOR_BGR2RGB);

        {
            auto lock        = std::lock_guard { mutex_ };
            target.submitted = ++sequence_;
        }
        cv_.notify_all();

        auto lock = std::unique_lock { mutex_ };
        cv_.wait(lock, [&] { return stopping_ || previous.submitted == previous.completed; });

        if (previous.submitted == 0) {
            // Depth-1 pipeline: the very first frame has no predecessor yet.
            return InferOutput { };
        }
        if (previous.run_result != RKNN_SUCC) {
            return std::unexpected { "rknn_run failed (" + std::to_string(previous.run_result)
                + ")" };
        }
        return InferOutput {
            .data =
                std::span<const float> { static_cast<const float*>(previous.output_mem->virt_addr),
                    previous.output_elems },
            .shape = out_shape_,
        };
    } catch (const std::exception& e) {
        return std::unexpected { std::string { "RKNN inference failed: " } + e.what() };
    }
}

} // namespace rmcs
