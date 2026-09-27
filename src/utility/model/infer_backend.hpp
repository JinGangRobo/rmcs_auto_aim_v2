#pragma once
#include "utility/model/common_model.hpp"

#include <array>
#include <cstddef>
#include <expected>
#include <memory>
#include <span>
#include <string>

#include <opencv2/core/mat.hpp>

namespace rmcs {

///
/// @brief:
/// Engine-agnostic view of an output tensor.
/// `data` points into the backend's internal buffer and stays valid only until the next `infer()`.
///
struct InferOutput {
    std::span<const float> data;
    /// [batch, rows, cols]
    std::array<std::size_t, 3> shape { 0, 0, 0 };
};

///
/// @brief:
/// Inference backend abstraction. It feeds a letterboxed u8 BGR image to the model
/// and returns the raw output tensor.
///
class InferBackend {
public:
    virtual ~InferBackend() = default;

    /// Load the model and create the inference backend.
    /// The backend implementation is chosen at configure time; returns an error
    /// when no backend is compiled in.
    static auto create(const ModelSpec& spec)
        -> std::expected<std::unique_ptr<InferBackend>, std::string>;

    /// @param input u8 BGR image (NHWC) already letterboxed to `spec.dimensions`
    virtual auto infer(const cv::Mat& input) noexcept
        -> std::expected<InferOutput, std::string> = 0;
};

/// Whether any inference backend is available in the current build.
[[nodiscard]] auto has_infer_backend() noexcept -> bool;

}
