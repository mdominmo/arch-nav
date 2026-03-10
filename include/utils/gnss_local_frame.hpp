#ifndef NAVIGATION_STATE_REGISTER__UTILS__GNSS_LOCAL_FRAME_HPP_
#define NAVIGATION_STATE_REGISTER__UTILS__GNSS_LOCAL_FRAME_HPP_

namespace arch_nav::utils::gnss_local_frame {

double lat_to_ned_x(double lat, double ref_lat);
double lon_to_ned_y(double lon, double ref_lon, double ref_lat);
double alt_to_ned_z(double alt, double ref_alt);

}  // namespace arch_nav::utils::gnss_local_frame

#endif  // NAVIGATION_STATE_REGISTER__UTILS__GNSS_LOCAL_FRAME_HPP_
