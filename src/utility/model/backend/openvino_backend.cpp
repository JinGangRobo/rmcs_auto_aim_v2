#include "utility/model/backend/openvino_backend.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <exception>
#include <span>
#include <string>
#include <utility>

#include <openvino/core/preprocess/pre_post_process.hpp>
#include <openvino/runtime/exception.hpp>
#include <openvino/runtime/properties.hpp>

namespace rmcs {

namespace {
    inline auto kRealTimePerformanceMode =
        ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY);
}

OpenVinoBackend::OpenVinoBackend(ModelSpec spec)
    : spec_ { std::move(spec) } {
    auto raw = core_.read_model(spec_.location);

    auto ppp = ov::preprocess::PrePostProcessor { raw };
    {
        const auto shape = spec_.input_layout.shape(spec_.dimensions);

        auto& input = ppp.input();
        input.tensor()
            .set_element_type(ov::element::u8)
            .set_shape(ov::PartialShape {
                static_cast<std::int64_t>(shape[0]),
                static_cast<std::int64_t>(shape[1]),
                static_cast<std::int64_t>(shape[2]),
                static_cast<std::int64_t>(shape[3]),
            })
            .set_layout(ov::Layout { std::string { spec_.input_layout.str() } })
            .set_color_format(ov::preprocess::ColorFormat::BGR);
        input.preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale({ 255., 255., 255. });
        input.model().set_layout(ov::Layout { std::string { spec_.model_layout.str() } });
    }
    {
        auto& output = ppp.output();
        output.tensor().set_element_type(ov::element::f32);
    }

    model_   = core_.compile_model(ppp.build(), spec_.device, kRealTimePerformanceMode);
    request_ = model_.create_infer_request();
}

auto OpenVinoBackend::create(const ModelSpec& spec)
    -> std::expected<std::unique_ptr<InferBackend>, std::string> {
    try {
        return std::unique_ptr<InferBackend> { new OpenVinoBackend { spec } };
    } catch (const ov::Exception& e) {
        return std::unexpected { std::string { "Failed to load model with OpenVINO: " }
            + e.what() };
    } catch (const std::exception& e) {
        return std::unexpected { std::string { "Failed to load model: " } + e.what() };
    }
}

auto OpenVinoBackend::infer(const cv::Mat& input) noexcept
    -> std::expected<InferOutput, std::string> {
    try {
        const auto shape  = spec_.input_layout.shape(spec_.dimensions);
        auto input_tensor = ov::Tensor {
            ov::element::u8,
            ov::Shape { shape[0], shape[1], shape[2], shape[3] },
        };

        if (input.total() * input.elemSize() != input_tensor.get_byte_size()) {
            return std::unexpected { "Input mat size does not match model input tensor" };
        }
        std::memcpy(input_tensor.data(), input.data, input_tensor.get_byte_size());

        request_.set_input_tensor(input_tensor);
        request_.infer();
        output_tensor_ = request_.get_output_tensor();
    } catch (const ov::Exception& e) {
        return std::unexpected { std::string { "OpenVINO inference failed: " } + e.what() };
    } catch (const std::exception& e) {
        return std::unexpected { std::string { "OpenVINO inference failed: " } + e.what() };
    }

    const auto& shape = output_tensor_.get_shape();
    if (shape.size() < 3) {
        return std::unexpected { "Unexpected OpenVINO output rank" };
    }

    return InferOutput {
        .data =
            std::span<const float> {
                output_tensor_.data<float>(),
                output_tensor_.get_size(),
            },
        .shape =
            std::array<std::size_t, 3> {
                static_cast<std::size_t>(shape[0]),
                static_cast<std::size_t>(shape[1]),
                static_cast<std::size_t>(shape[2]),
            },
    };
}

}
