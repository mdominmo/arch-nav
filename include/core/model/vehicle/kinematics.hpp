#ifndef NAVIGATION__CORE__MODEL__VEHICLE__KINEMATICS_HPP_
#define NAVIGATION__CORE__MODEL__VEHICLE__KINEMATICS_HPP_

#include <cmath>
#include <limits>

namespace arch_nav::vehicle {

struct Kinematics {
  double x;
  double y;
  double z;

  double vx;
  double vy;
  double vz;

  double ax;
  double ay;
  double az;

  double ref_lat;
  double ref_lon;
  double ref_alt;

  double heading;

  explicit Kinematics(
      double x_value = std::numeric_limits<double>::quiet_NaN(),
      double y_value = std::numeric_limits<double>::quiet_NaN(),
      double z_value = std::numeric_limits<double>::quiet_NaN(),
      double vx_value = std::numeric_limits<double>::quiet_NaN(),
      double vy_value = std::numeric_limits<double>::quiet_NaN(),
      double vz_value = std::numeric_limits<double>::quiet_NaN(),
      double ax_value = std::numeric_limits<double>::quiet_NaN(),
      double ay_value = std::numeric_limits<double>::quiet_NaN(),
      double az_value = std::numeric_limits<double>::quiet_NaN(),
      double ref_lat_value = std::numeric_limits<double>::quiet_NaN(),
      double ref_lon_value = std::numeric_limits<double>::quiet_NaN(),
      double ref_alt_value = std::numeric_limits<double>::quiet_NaN(),
      double heading_value = std::numeric_limits<double>::quiet_NaN())
      : x(x_value),
        y(y_value),
        z(z_value),
        vx(vx_value),
        vy(vy_value),
        vz(vz_value),
        ax(ax_value),
        ay(ay_value),
        az(az_value),
        ref_lat(ref_lat_value),
        ref_lon(ref_lon_value),
        ref_alt(ref_alt_value),
        heading(heading_value) {}

  bool is_valid() const {
    return !std::isnan(x) && !std::isnan(y) && !std::isnan(z) &&
           !std::isnan(vx) && !std::isnan(vy) && !std::isnan(vz) &&
           !std::isnan(ax) && !std::isnan(ay) && !std::isnan(az) &&
           !std::isnan(ref_lat) && !std::isnan(ref_lon) &&
           !std::isnan(ref_alt) && !std::isnan(heading);
  }


};

}  // namespace arch_nav::vehicle

#endif  // NAVIGATION__CORE__MODEL__VEHICLE__KINEMATICS_HPP_
