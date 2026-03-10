#ifndef NAVIGATION__CORE__MODEL__VEHICLE__VEHICLE_STATUS_HPP_
#define NAVIGATION__CORE__MODEL__VEHICLE__VEHICLE_STATUS_HPP_

#include "core/constants/vehicle_status_states.hpp"

namespace arch_nav::vehicle {

using constants::VehicleStatusStates;

struct VehicleStatus {
  VehicleStatusStates nav_state;
  VehicleStatusStates arm_state;

  explicit VehicleStatus(
      VehicleStatusStates nav_state_value = VehicleStatusStates::STATE_UNKNOWN,
      VehicleStatusStates arm_state_value = VehicleStatusStates::STATE_UNKNOWN)
      : nav_state(nav_state_value), arm_state(arm_state_value) {}

  bool is_valid() const {
    return nav_state != VehicleStatusStates::STATE_UNKNOWN &&
           arm_state != VehicleStatusStates::STATE_UNKNOWN;
  }


};

}  // namespace arch_nav::vehicle

#endif  // NAVIGATION__CORE__MODEL__VEHICLE__VEHICLE_STATUS_HPP_
