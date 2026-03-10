#ifndef NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__VEHICLE_STATUS_ADAPTER_HPP_
#define NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__VEHICLE_STATUS_ADAPTER_HPP_

#include "core/model/vehicle/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

namespace arch_nav::adapters::px4 {

class VehicleStatusAdapter {
 public:
  static vehicle::VehicleStatus map_from(
      const px4_msgs::msg::VehicleStatus& origin);
};

}  // namespace arch_nav::adapters::px4

#endif  // NAVIGATION_STATE_REGISTER__PX4__ADAPTERS__VEHICLE_STATUS_ADAPTER_HPP_
