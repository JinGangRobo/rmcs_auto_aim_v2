#pragma once
#include "utility/model/infer_backend.hpp"

#include <expected>
#include <memory>
#include <string>

#include <openvino/runtime/compiled_model.hpp>
#include <openvino/runtime/core.hpp>
#include <openvino/runtime/infer_request.hpp>
#include <openvino/runtime/tensor.hpp>

namespace rmcs {

///
/// @brief:
/// OpenVINO-based inference backend supporting ONNX and OpenVINO IR.
/// It is built only when CMake detects OpenVINO; otherwise the detector degrades to unavailable.
///
class OpenVinoBackend final : public InferBackend {
public:
    static auto create(const ModelSpec& spec)
        -> std::expected<std::unique_ptr<InferBackend>, std::string>;

    explicit OpenVinoBackend(ModelSpec spec);

    auto infer(const cv::Mat& input) noexcept -> std::expected<InferOutput, std::string> override;

private:
    ModelSpec spec_;
    ov::Core core_;
    ov::CompiledModel model_;
    ov::InferRequest request_;

    /// Keeps the latest output alive to back the lifetime of InferOutput::data.
    ov::Tensor output_tensor_;
};

}
