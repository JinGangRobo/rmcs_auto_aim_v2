#pragma once
#include "utility/rclcpp/parameters.hpp"

#include <cstdlib>
#include <filesystem>
#include <yaml-cpp/yaml.h>

namespace rmcs::util {

inline auto configuration(const char* config_file = "config.yaml") {
    static auto location = std::filesystem::path {
        Parameters::share_location(),
    };
    static auto root = [config_file]() {
        auto filename = std::string(config_file);
        if (const char* robot_type = std::getenv("RMCS_ROBOT_TYPE")) {
            filename = std::string(robot_type) + ".yaml";
        }
        return YAML::LoadFile(location / filename);
    }();
    return root;
}

}
