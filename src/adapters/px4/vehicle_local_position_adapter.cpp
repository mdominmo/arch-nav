#include "adapters/px4/vehicle_local_position_adapter.hpp"

namespace arch_nav::adapters::px4 {

vehicle::Kinematics VehicleLocalPositionAdapter::map_from(
    const px4_msgs::msg::VehicleLocalPosition& origin) {
  vehicle::Kinematics state;

  state.x = origin.x;
  state.y = origin.y;
  state.z = origin.z;

  state.vx = origin.vx;
  state.vy = origin.vy;
  state.vz = origin.vz;

  state.ax = origin.ax;
  state.ay = origin.ay;
  state.az = origin.az;

  state.ref_lat = origin.ref_lat;
  state.ref_lon = origin.ref_lon;
  state.ref_alt = origin.ref_alt;

  state.heading = origin.heading;

  return state;
}

}  // namespace arch_nav::adapters::px4
