#include "navigation_system_provider.hpp"

namespace arch_nav {

NavigationSystemProvider::NavigationSystemProvider(
    rclcpp::Node& node,
    const config::NavigationConfig& config)
    : state_manager_()
    , dispatcher_(node,
                  config.topics.trajectory_setpoint,
                  config.topics.vehicle_command,
                  config.topics.offboard_control_mode,
                  static_cast<uint8_t>(config.target_system + 1))
    , planner_(
          config.local_planner.max_linear_velocity,
          config.local_planner.max_linear_acceleration,
          config.local_planner.max_angular_velocity,
          config.local_planner.max_angular_acceleration,
          config.local_planner.max_vertical_velocity,
          config.local_planner.max_vertical_acceleration,
          config.local_planner.time_step)
    , controller_(state_manager_, planner_, dispatcher_)
    , api_(controller_)
    , local_pos_listener_(node, state_manager_, config.topics.vehicle_local_position)
    , global_pos_listener_(node, state_manager_, config.topics.vehicle_global_position)
    , status_listener_(node, state_manager_, config.topics.vehicle_status) {}

NavigationApi& NavigationSystemProvider::provide() {
  return api_;
}

}  // namespace arch_nav
