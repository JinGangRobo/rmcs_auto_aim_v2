#include "utility/model/backend/rknn_backend.hpp"

#include <cstdint>
#include <exception>
#include <experimental/scope>
#include <filesystem>
#include <fstream>
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
    if (ctx_ != 0) {
        rknn_destroy(ctx_);
        ctx_ = 0;
    }
}

auto RknnBackend::create(const ModelSpec& spec)
    -> std::expected<std::unique_ptr<InferBackend>, std::string> {
    auto backend = std::make_unique<RknnBackend>(spec);

    const auto model_path = resolve_model_path(spec.location);
    if (!read_model(model_path, backend->model_buf_)) {
        return std::unexpected { "Failed to read RKNN model: " + model_path };
    }

    auto ret = rknn_init(&backend->ctx_, backend->model_buf_.data(),
        static_cast<std::uint32_t>(backend->model_buf_.size()), 0, nullptr);
    if (ret != RKNN_SUCC) {
        backend->ctx_ = 0;
        return std::unexpected {
            "rknn_init failed (" + std::to_string(ret) + "): " + model_path,
        };
    }

    // The runtime owns its copy of the model from now on.
    backend->model_buf_.clear();
    backend->model_buf_.shrink_to_fit();

    auto io_num = rknn_input_output_num { };
    ret         = rknn_query(backend->ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC) {
        return std::unexpected { "rknn_query(IN_OUT_NUM) failed (" + std::to_string(ret) + ")" };
    }
    if (io_num.n_input != 1 || io_num.n_output != 1) {
        return std::unexpected { "Unexpected RKNN inputs/outputs: " + std::to_string(io_num.n_input)
            + "/" + std::to_string(io_num.n_output) };
    }

    auto input_attr = rknn_tensor_attr { };
    ret = rknn_query(backend->ctx_, RKNN_QUERY_INPUT_ATTR, &input_attr, sizeof(input_attr));
    if (ret != RKNN_SUCC) {
        return std::unexpected { "rknn_query(INPUT_ATTR) failed (" + std::to_string(ret) + ")" };
    }
    if (!input_matches(input_attr, spec.dimensions)) {
        return std::unexpected { "RKNN input shape " + shape_to_string(as_shape_4d(input_attr))
            + " does not match ModelSpec" };
    }

    auto output_attr = rknn_tensor_attr { };
    ret = rknn_query(backend->ctx_, RKNN_QUERY_OUTPUT_ATTR, &output_attr, sizeof(output_attr));
    if (ret != RKNN_SUCC) {
        return std::unexpected { "rknn_query(OUTPUT_ATTR) failed (" + std::to_string(ret) + ")" };
    }
    if (output_attr.n_dims != 2 && output_attr.n_dims != 3) {
        return std::unexpected { "Unexpected RKNN output rank: "
            + std::to_string(output_attr.n_dims) + " shape "
            + shape_to_string(as_shape_4d(output_attr)) };
    }
    if (output_attr.n_dims == 2) {
        backend->out_shape_ = { 1, output_attr.dims[0], output_attr.dims[1] };
    } else {
        backend->out_shape_ = { output_attr.dims[0], output_attr.dims[1], output_attr.dims[2] };
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

        cv::cvtColor(input, rgb_buf_, cv::COLOR_BGR2RGB);

        auto rknn_in         = rknn_input { };
        rknn_in.index        = 0;
        rknn_in.buf          = rgb_buf_.data;
        rknn_in.size         = static_cast<std::uint32_t>(rgb_buf_.total() * rgb_buf_.elemSize());
        rknn_in.type         = RKNN_TENSOR_UINT8;
        rknn_in.fmt          = RKNN_TENSOR_NHWC;
        rknn_in.pass_through = 0;

        auto ret = rknn_inputs_set(ctx_, 1, &rknn_in);
        if (ret != RKNN_SUCC) {
            return std::unexpected { "rknn_inputs_set failed (" + std::to_string(ret) + ")" };
        }

        ret = rknn_run(ctx_, nullptr);
        if (ret != RKNN_SUCC) {
            return std::unexpected { "rknn_run failed (" + std::to_string(ret) + ")" };
        }

        auto rknn_out       = rknn_output { };
        rknn_out.index      = 0;
        rknn_out.want_float = 1;
        ret                 = rknn_outputs_get(ctx_, 1, &rknn_out, nullptr);
        if (ret != RKNN_SUCC) {
            return std::unexpected { "rknn_outputs_get failed (" + std::to_string(ret) + ")" };
        }
        auto release =
            std::experimental::scope_exit { [&] { rknn_outputs_release(ctx_, 1, &rknn_out); } };

        const auto count = rknn_out.size / sizeof(float);
        const auto* raw  = static_cast<const float*>(rknn_out.buf);
        out_data_.assign(raw, raw + count);
    } catch (const std::exception& e) {
        return std::unexpected { std::string { "RKNN inference failed: " } + e.what() };
    }

    return InferOutput {
        .data  = out_data_,
        .shape = out_shape_,
    };
}

}
