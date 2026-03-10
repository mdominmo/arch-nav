#ifndef NAVIGATION_STATE_REGISTER__NAVIGATION_SYSTEM_PROVIDER_HPP_
#define NAVIGATION_STATE_REGISTER__NAVIGATION_SYSTEM_PROVIDER_HPP_

#include "config/navigation_config.hpp"
#include "core/controller/operational_controller.hpp"
#include "core/context/vehicle_context.hpp"
#include "core/planner/kinematic_point_to_point_local_planner.hpp"
#include "dispatchers/px4/px4_command_dispatcher.hpp"
#include "listeners/px4/vehicle_global_position_listener.hpp"
#include "listeners/px4/vehicle_local_position_listener.hpp"
#include "listeners/px4/vehicle_status_listener.hpp"
#include "navigation_api.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arch_nav {

class NavigationSystemProvider {
 public:
  explicit NavigationSystemProvider(
      rclcpp::Node& node,
      const config::NavigationConfig& config);

  NavigationApi& provide();

 private:
  context::VehicleContext                              state_manager_;
  dispatchers::px4::Px4CommandDispatcher               dispatcher_;
  planner::KinematicPointToPointLocalPlanner           planner_;
  controller::OperationalController                    controller_;
  NavigationApi                                        api_;
  listeners::px4::VehicleLocalPositionListener         local_pos_listener_;
  listeners::px4::VehicleGlobalPositionListener        global_pos_listener_;
  listeners::px4::VehicleStatusListener                status_listener_;
};

}  // namespace arch_nav

#endif  // NAVIGATION_STATE_REGISTER__NAVIGATION_SYSTEM_PROVIDER_HPP_
