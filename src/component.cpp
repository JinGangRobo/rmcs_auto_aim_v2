#include "kernel/feishu.hpp"
#include "module/debug/framerate.hpp"
#include "utility/clock.hpp"
#include "utility/rclcpp/node.hpp"
#include "utility/rclcpp/visual/transform.hpp"
#include "utility/shared/context.hpp"
#include <eigen3/Eigen/Dense>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs {

using namespace rmcs::util;
using namespace kernel;

class AutoAimComponent final : public rmcs_executor::Component {
public:
    explicit AutoAimComponent() noexcept
        : rclcpp { get_component_name() } {

        register_input("/tf", rmcs_tf);
        register_output("/gimbal/auto_aim/control_direction", auto_aim_control_direction_);
        register_output("/debug/aim/x", debug_aim_x);
        register_output("/debug/aim/y", debug_aim_y);
        register_output("/debug/aim/z", debug_aim_z);
        register_output("/debug/aim/delay", aim_delay_ms);

        using namespace std::chrono_literals;
        framerate.set_interval(2s);

        visual::Transform::Config config {
            .rclcpp       = rclcpp,                     // 当前组件持有的 RclcppNode
            .topic        = "odom_to_camera_transform", // 发布的 topic 名
            .parent_frame = "odom_imu_link",            // 父坐标系
            .child_frame  = "camera_link",              // 子坐标系
        };
        visual_odom_to_camera = std::make_unique<visual::Transform>(config);

        last_valid_aim_time          = Clock::now();
        *auto_aim_control_direction_ = Eigen::Vector3d::Zero();
    }

    auto update() -> void override {
        using namespace rmcs_description;

        if (!rmcs_tf.ready()) [[unlikely]]
            return;

        {
            control_state.timestamp = Clock::now();

            auto odom_to_camera_transform =
                fast_tf::lookup_transform<rmcs_description::OdomImu, rmcs_description::CameraLink>(
                    *rmcs_tf);

            control_state.odom_to_camera_transform.position =
                odom_to_camera_transform.translation();
            control_state.odom_to_camera_transform.orientation =
                Eigen::Quaterniond(odom_to_camera_transform.rotation());

            visual_odom_to_camera->move(control_state.odom_to_camera_transform.position,
                control_state.odom_to_camera_transform.orientation);
            visual_odom_to_camera->update();

            // TODO:无敌状态下的装甲板需要从裁判系统获取并在此更新
            control_state.invincible_devices = DeviceIds::None();

            // TODO:弹速需要进一步确认
            control_state.bullet_speed = 25;
            auto success               = feishu.commit(control_state);

            // TODO:添加错误输出的时间间隔和次数限制
            if (!success) rclcpp.info("commit control state failed!");
        }

        if (feishu.updated()) {
            auto_aim_state = feishu.fetch();

            // timestamp check
            if ((Clock::now() - auto_aim_state.timestamp) < std::chrono::milliseconds(0)
                || (Clock::now() - auto_aim_state.timestamp) > std::chrono::milliseconds(100))
                return;

            Eigen::Vector3d control_direction;
            control_direction.x() = auto_aim_state.x;
            control_direction.y() = auto_aim_state.y;
            control_direction.z() = auto_aim_state.z;

            if (!control_direction.isZero()) {
                auto time_diff = Clock::now() - last_valid_aim_time;
                *aim_delay_ms =
                    std::chrono::duration_cast<std::chrono::microseconds>(time_diff).count()
                    / 1000.0;

                last_valid_aim_time = Clock::now();

                control_direction = Eigen::AngleAxisd { yaw_aim_offset, Eigen::Vector3d::UnitZ() }
                    * control_direction;

                control_direction.normalize();
                *auto_aim_control_direction_ = control_direction;
                // FOR DEBUG
                *debug_aim_x = control_direction.x();
                *debug_aim_y = control_direction.y();
                *debug_aim_z = control_direction.z();
            }
        } else {
            if (Clock::now() - last_valid_aim_time > std::chrono::milliseconds(200)) {
                *auto_aim_control_direction_ = Eigen::Vector3d::Zero();
                // FOR DEBUG
                *debug_aim_x = 0.0;
                *debug_aim_y = 0.0;
                *debug_aim_z = 0.0;
            }
        }
    }

private:
    InputInterface<rmcs_description::Tf> rmcs_tf;
    OutputInterface<Eigen::Vector3d> auto_aim_control_direction_;
    OutputInterface<double> debug_aim_x;
    OutputInterface<double> debug_aim_y;
    OutputInterface<double> debug_aim_z;
    OutputInterface<double> aim_delay_ms;

    // TODO: Move it to config file
    static constexpr double yaw_aim_offset = -0.1;

    RclcppNode rclcpp;
    std::unique_ptr<visual::Transform> visual_odom_to_camera;

    Feishu<RuntimeRole::Control> feishu;
    ControlState control_state;
    AutoAimState auto_aim_state;
    Clock::time_point last_valid_aim_time;

    FramerateCounter framerate;

private:
};

} // namespace rmcs

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs::AutoAimComponent, rmcs_executor::Component)
