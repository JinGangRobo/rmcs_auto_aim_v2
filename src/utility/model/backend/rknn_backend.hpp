#pragma once
#include "utility/model/infer_backend.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <memory>
#include <string>
#include <vector>

#include <rknn_api.h>

namespace rmcs {

///
/// @brief:
/// Rockchip RKNN (RKNPU2) inference backend. The `.rknn` model path is derived
/// from `ModelSpec::location` by swapping the extension, e.g.
/// `shenzhen-0526.onnx` -> `shenzhen-0526.rknn`, so the robot config keeps
/// pointing at the original model file.
///
/// Inference is zero-copy: the input/output tensor memory is allocated once with
/// `rknn_create_mem` and bound through `rknn_set_io_mem`, so no per-frame
/// staging copy is made. A u8 NHWC RGB image is written straight into the input
/// memory with `pass_through = 0`, letting the runtime apply the model's own
/// normalization; the raw float output is read straight out of the output
/// tensor memory.
///
class RknnBackend final : public InferBackend {
public:
    static auto create(const ModelSpec& spec)
        -> std::expected<std::unique_ptr<InferBackend>, std::string>;

    explicit RknnBackend(ModelSpec spec);

    auto infer(const cv::Mat& input) noexcept -> std::expected<InferOutput, std::string> override;

    ~RknnBackend() override;

private:
    ModelSpec spec_;
    rknn_context ctx_ { 0 };

    /// Backing buffer of the loaded model. The runtime copies it during
    /// `rknn_init`, so it is released as soon as loading succeeds.
    std::vector<std::uint8_t> model_buf_;

    /// Zero-copy tensor memory bound to the model input/output with
    /// `rknn_set_io_mem`; the runtime reads and writes these buffers in place.
    rknn_tensor_mem* input_mem_ { nullptr };
    rknn_tensor_mem* output_mem_ { nullptr };

    /// Attributes used to bind the tensor memory. The input is bound as u8 NHWC
    /// with `pass_through = 0` (the runtime applies the model's normalization);
    /// the output is bound as f32.
    rknn_tensor_attr input_attr_ { };
    rknn_tensor_attr output_attr_ { };

    /// RGB view over the NPU input memory: converts the BGR frame directly into
    /// the zero-copy buffer, honouring the tensor width stride via the Mat step.
    cv::Mat input_view_;

    /// Backs the lifetime of InferOutput::data until the next `infer()`.
    std::array<std::size_t, 3> out_shape_ { 0, 0, 0 };
};

}
