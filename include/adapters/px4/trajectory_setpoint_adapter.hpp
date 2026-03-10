#ifndef NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__TRAJECTORY_SETPOINT_ADAPTER_HPP_
#define NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__TRAJECTORY_SETPOINT_ADAPTER_HPP_

#include <vector>

#include "core/model/vehicle/trajectory_point.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

namespace arch_nav::adapters::px4 {

class TrajectorySetpointAdapter {
 public:
  static px4_msgs::msg::TrajectorySetpoint map_from(
      const vehicle::TrajectoryPoint& origin);

  static std::vector<px4_msgs::msg::TrajectorySetpoint> map_from(
      const std::vector<vehicle::TrajectoryPoint>& origin);
};

}  // namespace arch_nav::adapters::px4

#endif  // NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__TRAJECTORY_SETPOINT_ADAPTER_HPP_
