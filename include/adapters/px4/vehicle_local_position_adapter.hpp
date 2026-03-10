#ifndef NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__VEHICLE_LOCAL_POSITION_ADAPTER_HPP_
#define NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__VEHICLE_LOCAL_POSITION_ADAPTER_HPP_

#include "core/model/vehicle/kinematics.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

namespace arch_nav::adapters::px4 {

class VehicleLocalPositionAdapter {
 public:
  static vehicle::Kinematics map_from(
      const px4_msgs::msg::VehicleLocalPosition& origin);
};

}  // namespace arch_nav::adapters::px4

#endif  // NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__VEHICLE_LOCAL_POSITION_ADAPTER_HPP_
