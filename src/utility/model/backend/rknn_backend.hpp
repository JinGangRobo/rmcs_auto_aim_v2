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
    ModelSpec spec_;
    rknn_context ctx_ { 0 };

    /// Backing buffer of the loaded model. The runtime copies it during
    /// `rknn_init`, so it is released as soon as loading succeeds.
    std::vector<std::uint8_t> model_buf_;

    /// Reused RGB buffer backing the NHWC input of each inference.
    cv::Mat rgb_buf_;

    /// Keeps the latest output alive to back the lifetime of InferOutput::data.
    std::vector<float> out_data_;
    std::array<std::size_t, 3> out_shape_ { 0, 0, 0 };
};

}
