#pragma once
#include "utility/string.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <ranges>
#include <stdexcept>
#include <string>
#include <string_view>

namespace rmcs {

/// @brief:
/// POD 结构体，用于语义化地设置各个维度的值，比如：
/// ```
/// auto dimensions = Dimensions{ .W = 100, .H = 100 };
/// ```
struct Dimensions {
    using Value = std::int64_t;

    Value N = 1;
    Value C = 3;
    Value W = 0;
    Value H = 0;

    constexpr auto at(char dimension) const -> Value {
        switch (dimension) {
        case 'N':
            return N;
        case 'C':
            return C;
        case 'W':
            return W;
        case 'H':
            return H;
        default:
            throw std::runtime_error("Wrong dimension char, valid: N, C, W, H");
        }
    }
};

///
/// @brief:
/// 模型布局，用于语义化生成 layout, shape 等数据结构
/// Engine-agnostic: it only describes the N/C/W/H ordering.
///
struct TensorLayout {
private:
    std::array<char, 5> chars { '\0', '\0', '\0', '\0', '\0' };

public:
    static constexpr auto is_valid_dimension(char dimension) noexcept {
        return dimension == 'N' || dimension == 'C' || dimension == 'W' || dimension == 'H';
    }
    template <std::ranges::input_range Range>
    static constexpr auto has_unique_dimensions(const Range& parsed) noexcept {
        auto has_unique_dimensions = true;
        for (auto it = std::ranges::begin(parsed); it != std::ranges::end(parsed); ++it) {
            if (std::ranges::find(std::next(it), std::ranges::end(parsed), *it)
                != std::ranges::end(parsed)) {
                has_unique_dimensions = false;
            }
        }
        return has_unique_dimensions;
    }
    static constexpr auto is_valid_description(std::string_view description) noexcept -> bool {
        return std::ranges::all_of(description, is_valid_dimension)
            && has_unique_dimensions(description);
    }

    template <util::StaticString description>
    static consteval auto from() noexcept -> TensorLayout {
        static_assert(description.length() == 5, "Layout description must be exactly 4 characters");

        constexpr auto parsed = std::array<char, 4> {
            description.data[0],
            description.data[1],
            description.data[2],
            description.data[3],
        };
        static_assert(std::ranges::all_of(parsed, is_valid_dimension),
            "The layout description only supports N/C/W/H");
        static_assert(has_unique_dimensions(parsed),
            "The layout description must not contain duplicate dimensions");

        return TensorLayout { std::string_view { parsed.data(), 4 } };
    }

    constexpr explicit TensorLayout(std::string_view description) {
        if (description.size() == 5 && description[4] == '\0') {
            description.remove_suffix(1);
        }
        if (description.size() != 4) {
            throw std::invalid_argument { "Layout description must be exactly 4 characters" };
        }
        if (!is_valid_description(description)) {
            throw std::invalid_argument { "Invalid layout description" };
        }
        std::ranges::copy_n(description.begin(), 4, chars.begin());
    }

    /// 4-character layout string in N/C/W/H order, e.g. "NHWC".
    constexpr auto str() const noexcept -> std::string_view {
        return std::string_view { chars.data(), 4 };
    }

    /// Expand Dimensions into a 4D shape following this layout's dimension order.
    constexpr auto shape(const Dimensions& dimensions) const noexcept {
        return std::array<std::size_t, 4> {
            static_cast<std::size_t>(dimensions.at(chars[0])),
            static_cast<std::size_t>(dimensions.at(chars[1])),
            static_cast<std::size_t>(dimensions.at(chars[2])),
            static_cast<std::size_t>(dimensions.at(chars[3])),
        };
    }
};

///
/// @brief:
/// Engine-agnostic model description. Backends use it to load and preprocess the model,
/// so model types no longer carry their own compilation logic.
///
struct ModelSpec {
    std::string location;

    /// Inference device hint, e.g. "CPU" / "AUTO"; interpreted by the backend.
    std::string device = "AUTO";

    TensorLayout input_layout = TensorLayout::from<"NHWC">();
    TensorLayout model_layout = TensorLayout::from<"NCHW">();
    Dimensions dimensions     = Dimensions { .W = 640, .H = 640 };
};

}
