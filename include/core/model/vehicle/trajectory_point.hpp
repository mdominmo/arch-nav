#ifndef NAVIGATION__CORE__MODEL__VEHICLE__TRAJECTORY_POINT_HPP_
#define NAVIGATION__CORE__MODEL__VEHICLE__TRAJECTORY_POINT_HPP_

namespace arch_nav::vehicle {

struct TrajectoryPoint {
  double t;

  double x;
  double y;
  double z;

  double vx;
  double vy;
  double vz;

  double ax;
  double ay;
  double az;

  double heading;
  double omega;
};

}  // namespace arch_nav::vehicle

#endif  // NAVIGATION__CORE__MODEL__VEHICLE__TRAJECTORY_POINT_HPP_
