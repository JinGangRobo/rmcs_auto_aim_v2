#pragma once
#include "utility/model/infer_backend.hpp"

#include <array>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <rknn_api.h>

namespace rmcs {

///
/// @brief:
/// Rockchip RKNN (RKNPU2) inference backend running a depth-1 double-buffered
/// pipeline across two NPU cores. The `.rknn` model path is derived from
/// `ModelSpec::location` by swapping the extension, e.g.
/// `shenzhen-0526.onnx` -> `shenzhen-0526.rknn`, so the robot config keeps
/// pointing at the original model file.
///
/// Two contexts share one model (the second is created with `rknn_dup_context`)
/// and are pinned to `RKNN_NPU_CORE_0` and `RKNN_NPU_CORE_1`. Each context has
/// its own worker thread and its own zero-copy input/output buffers, so one
/// frame is inferred on a core while the caller consumes the previous one.
/// `infer()` therefore returns the result of the PREVIOUS frame, which keeps the
/// synchronous `ArmorDetection::sync_detect` contract while roughly doubling
/// throughput. `InferOutput::data` stays valid until the next `infer()`.
///
/// A u8 NHWC RGB image is fed with `pass_through = 0`, letting the runtime apply
/// the model's own normalization; the raw float output tensor is returned
/// untouched.
///
class RknnBackend final : public InferBackend {
public:
    static auto create(const ModelSpec& spec)
        -> std::expected<std::unique_ptr<InferBackend>, std::string>;

    explicit RknnBackend(ModelSpec spec);

    auto infer(const cv::Mat& input) noexcept -> std::expected<InferOutput, std::string> override;

    ~RknnBackend() override;

private:
    /// One NPU core's state: its context, its zero-copy buffers and the pipeline
    /// bookkeeping shared with its worker thread.
    struct Worker {
        rknn_context ctx { 0 };

        /// Backing buffers of the zero-copy input/output of this core.
        rknn_tensor_mem* input_mem { nullptr };
        rknn_tensor_mem* output_mem { nullptr };
        rknn_tensor_attr input_attr { };
        rknn_tensor_attr output_attr { };
        cv::Mat input_view;
        std::size_t output_elems { 0 };

        /// Pipeline counters guarded by `RknnBackend::mutex_`. A worker is idle
        /// when `submitted == completed`; `run_result` is its last `rknn_run`
        /// return code.
        std::uint64_t submitted { 0 };
        std::uint64_t completed { 0 };
        int run_result { 0 };
    };

    /// Query the tensor attributes of `ctx`, validate them against `spec_` and
    /// bind a zero-copy input/output pair into `worker`.
    auto setup_worker(rknn_context ctx, Worker& worker) -> std::expected<void, std::string>;

    /// Runs the blocking inference loop of one core until the backend stops.
    auto run_worker(Worker& worker) -> void;

    ModelSpec spec_;

    /// Backing buffer of the loaded model. The runtime copies it during
    /// `rknn_init`, so it is released as soon as loading succeeds.
    std::vector<std::uint8_t> model_buf_;

    std::array<Worker, 2> workers_ { };
    std::array<std::size_t, 3> out_shape_ { 0, 0, 0 };

    std::mutex mutex_;
    std::condition_variable cv_;
    std::uint64_t sequence_ { 0 };
    std::size_t next_worker_ { 0 };
    bool stopping_ { false };
    std::array<std::jthread, 2> threads_ { };
};

}
