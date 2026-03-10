#ifndef NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__VEHICLE_GLOBAL_POSITION_ADAPTER_HPP_
#define NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__VEHICLE_GLOBAL_POSITION_ADAPTER_HPP_

#include "core/model/vehicle/global_position.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"

namespace arch_nav::adapters::px4 {

class VehicleGlobalPositionAdapter {
 public:
  static vehicle::GlobalPosition map_from(
      const px4_msgs::msg::VehicleGlobalPosition& origin);
};

}  // namespace arch_nav::adapters::px4

#endif  // NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__VEHICLE_GLOBAL_POSITION_ADAPTER_HPP_
