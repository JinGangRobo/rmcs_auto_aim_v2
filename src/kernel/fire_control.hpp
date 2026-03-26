#pragma once

#include <expected>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "kernel/tracker.hpp"
#include "module/predictor/snapshot.hpp"
#include "utility/clock.hpp"
#include "utility/pimpl.hpp"
#include "utility/robot/armor.hpp"
#include "utility/shared/context.hpp"

namespace rmcs::kernel {

class FireControl {
    using Clock = util::Clock;

    RMCS_PIMPL_DEFINITION(FireControl)

public:
    struct Result {
        double pitch;
        double yaw;
        double horizon_distance;
    };

    auto initialize(const YAML::Node&) noexcept -> std::expected<void, std::string>;

    auto set_bullet_speed(double speed) -> void;

    auto solve(const predictor::Snapshot& snapshot,
        util::Transform const& odom_to_muzzle_translation,
        const std::optional<std::vector<Armor3D>>& armors_3d, kernel::Tracker& tracker)
        -> std::optional<Result>;
};
}
