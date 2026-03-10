#include "adapters/px4/vehicle_global_position_adapter.hpp"

namespace arch_nav::adapters::px4 {

vehicle::GlobalPosition VehicleGlobalPositionAdapter::map_from(
    const px4_msgs::msg::VehicleGlobalPosition& origin) {
  vehicle::GlobalPosition state;

  state.lat = origin.lat;
  state.lon = origin.lon;
  state.alt = origin.alt;

  return state;
}

}  // namespace arch_nav::adapters::px4
