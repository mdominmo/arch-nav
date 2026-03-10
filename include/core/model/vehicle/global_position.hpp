#ifndef NAVIGATION__CORE__MODEL__VEHICLE__GLOBAL_POSITION_HPP_
#define NAVIGATION__CORE__MODEL__VEHICLE__GLOBAL_POSITION_HPP_

#include <cstdint>
#include <cmath>
#include <limits>

namespace arch_nav::vehicle {

struct GlobalPosition {
  double lat;
  double lon;
  double alt;

  explicit GlobalPosition(
      double lat_value = std::numeric_limits<double>::quiet_NaN(),
      double lon_value = std::numeric_limits<double>::quiet_NaN(),
      double alt_value = std::numeric_limits<double>::quiet_NaN())
      : lat(lat_value), lon(lon_value), alt(alt_value) {}

  bool is_valid() const {
    return !std::isnan(lat) && !std::isnan(lon) && !std::isnan(alt);
  }


};

}  // namespace arch_nav::vehicle

#endif  // NAVIGATION__CORE__MODEL__VEHICLE__GLOBAL_POSITION_HPP_
