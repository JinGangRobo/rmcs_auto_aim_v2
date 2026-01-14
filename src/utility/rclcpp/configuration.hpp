#pragma once
#include "utility/rclcpp/parameters.hpp"

#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace rmcs::util {

inline auto configuration(const char* config_file = "config.yaml") {
    static auto location = std::filesystem::path {
        Parameters::share_location(),
    };
    static auto root = YAML::LoadFile(location / config_file);
    return root;
}

}
