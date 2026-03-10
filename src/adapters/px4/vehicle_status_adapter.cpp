#include "adapters/px4/vehicle_status_adapter.hpp"

namespace arch_nav::adapters::px4 {

namespace {

constants::VehicleStatusStates map_arm_state(uint8_t arming_state) {
  if (arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
    return constants::VehicleStatusStates::STATE_DISARMED;
  }
  if (arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
    return constants::VehicleStatusStates::STATE_ARMED;
  }
  return constants::VehicleStatusStates::STATE_UNKNOWN;
}

constants::VehicleStatusStates map_nav_state(uint8_t nav_state) {
  switch (nav_state) {
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF:
      return constants::VehicleStatusStates::STATE_TAKEOFF;
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:
      return constants::VehicleStatusStates::STATE_HOLD;
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION:
      return constants::VehicleStatusStates::STATE_GOING_TO_WAYPOINT;
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL:
      return constants::VehicleStatusStates::STATE_ON_AIR;
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL:
      return constants::VehicleStatusStates::STATE_RTL;
    case px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD:
      return constants::VehicleStatusStates::STATE_OFFBOARD;
    default:
      return constants::VehicleStatusStates::STATE_UNKNOWN;
  }
}

}  // namespace

vehicle::VehicleStatus VehicleStatusAdapter::map_from(
    const px4_msgs::msg::VehicleStatus& origin) {
  return vehicle::VehicleStatus(
      map_nav_state(origin.nav_state),
      map_arm_state(origin.arming_state));
}

}  // namespace arch_nav::adapters::px4
