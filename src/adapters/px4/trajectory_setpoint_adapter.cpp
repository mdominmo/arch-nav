#include "adapters/px4/trajectory_setpoint_adapter.hpp"

#include <vector>

namespace arch_nav::adapters::px4 {

px4_msgs::msg::TrajectorySetpoint TrajectorySetpointAdapter::map_from(
    const vehicle::TrajectoryPoint& origin) {
  px4_msgs::msg::TrajectorySetpoint msg;

  msg.position[0] = static_cast<float>(origin.x);
  msg.position[1] = static_cast<float>(origin.y);
  msg.position[2] = static_cast<float>(origin.z);

  msg.velocity[0] = static_cast<float>(origin.vx);
  msg.velocity[1] = static_cast<float>(origin.vy);
  msg.velocity[2] = static_cast<float>(origin.vz);

  msg.acceleration[0] = static_cast<float>(origin.ax);
  msg.acceleration[1] = static_cast<float>(origin.ay);
  msg.acceleration[2] = static_cast<float>(origin.az);

  msg.yaw      = static_cast<float>(origin.heading);
  msg.yawspeed = static_cast<float>(origin.omega);

  return msg;
}

std::vector<px4_msgs::msg::TrajectorySetpoint> TrajectorySetpointAdapter::map_from(
    const std::vector<vehicle::TrajectoryPoint>& origin) {
  std::vector<px4_msgs::msg::TrajectorySetpoint> result;
  result.reserve(origin.size());
  for (const auto& point : origin) {
    result.push_back(map_from(point));
  }
  return result;
}

}  // namespace arch_nav::adapters::px4
