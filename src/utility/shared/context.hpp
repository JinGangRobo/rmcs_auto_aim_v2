#pragma once

#include "utility/clock.hpp"
#include "utility/math/linear.hpp"
#include "utility/robot/id.hpp"
#include <cstdint>

namespace rmcs::util {

enum class ShootMode {
    STOPPING,
    OUTPOST,
    BATTLE,
    BUFF_SMALL,
    BUFF_LARGE,
};

enum class SelfColor {
    UNKNOWN,
    RED,
    BLUE,
};

struct Transform {
    Translation position { };
    Orientation orientation { };
};

struct AutoAimState {
    Clock::time_point timestamp { };

    bool shoot_permitted = { false };

    double yaw { 0. };
    double pitch { 0. };

    DeviceId target { DeviceId::UNKNOWN };

    // In Base frame, where target enemy at.
    double target_position[3] { 0, 0, 0 };

    // Only used for autopilot.
    int8_t selected_scan_direction { 0 };

    auto set_identity() noexcept -> void {
        timestamp = Clock::now();

        shoot_permitted         = false;
        yaw                     = 0.;
        pitch                   = 0.;
        target_position[0]      = 0.;
        target_position[1]      = 0.;
        target_position[2]      = 0.;
        selected_scan_direction = 0;
    }
};
static_assert(std::is_trivially_copyable_v<AutoAimState>);

struct ControlState {
    Clock::time_point timestamp { };
    ShootMode shoot_mode { ShootMode::BATTLE };

    double bullet_speed { 0. };
    double yaw { 0. };
    double pitch { 0. };

    DeviceIds invincible_devices { DeviceIds::None() };
    SelfColor self_color { SelfColor::UNKNOWN };

    Transform odom_to_camera_transform { };
    Transform odom_to_base_transform { };
    Transform odom_to_muzzle_transform { };

    auto set_identity() noexcept -> void {
        timestamp                = Clock::now();
        shoot_mode               = ShootMode::STOPPING;
        bullet_speed             = 0.0;
        yaw                      = 0.0;
        pitch                    = 0.0;
        invincible_devices       = DeviceIds::None();
        odom_to_camera_transform = { };
        odom_to_base_transform   = { };
        odom_to_muzzle_transform = { };
    }
};
static_assert(std::is_trivially_copyable_v<ControlState>);
}
