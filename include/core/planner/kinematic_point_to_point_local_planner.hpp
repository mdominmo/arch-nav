#ifndef NAVIGATION_STATE_REGISTER__PLANNER__POINT_TO_POINT_LOCAL_PLANNER_HPP_
#define NAVIGATION_STATE_REGISTER__PLANNER__POINT_TO_POINT_LOCAL_PLANNER_HPP_

#include <vector>

#include "core/planner/local_planner.hpp"
#include "core/model/vehicle/kinematics.hpp"
#include "core/model/vehicle/trajectory_point.hpp"

namespace arch_nav::planner {

class KinematicPointToPointLocalPlanner : public ILocalPlanner {
 public:
  explicit KinematicPointToPointLocalPlanner(
      double max_linear_velocity,
      double max_linear_acceleration,
      double max_angular_velocity,
      double max_angular_acceleration,
      double max_vertical_velocity,
      double max_vertical_acceleration,
      double time_step);

  std::vector<vehicle::TrajectoryPoint> plan_travel(
      const vehicle::Kinematics& from,
      const vehicle::Kinematics& to) const override;

  std::vector<vehicle::TrajectoryPoint> plan_rotation(
      const vehicle::Kinematics& from,
      double target_heading) const override;

  std::vector<vehicle::TrajectoryPoint> plan_vertical(
      const vehicle::Kinematics& from,
      double target_z) const override;

 private:
  double max_linear_velocity_;
  double max_linear_acceleration_;
  double max_angular_velocity_;
  double max_angular_acceleration_;
  double max_vertical_velocity_;
  double max_vertical_acceleration_;
  double time_step_;
};

}  // namespace arch_nav::planner

#endif  // NAVIGATION_STATE_REGISTER__PLANNER__POINT_TO_POINT_LOCAL_PLANNER_HPP_
