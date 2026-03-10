#ifndef NAVIGATION__CORE__PLANNER__LOCAL_PLANNER_HPP_
#define NAVIGATION__CORE__PLANNER__LOCAL_PLANNER_HPP_

#include <vector>

#include "core/model/vehicle/kinematics.hpp"
#include "core/model/vehicle/trajectory_point.hpp"

namespace arch_nav::planner {

class ILocalPlanner {
 public:
  virtual std::vector<vehicle::TrajectoryPoint> plan_travel(
      const vehicle::Kinematics& from,
      const vehicle::Kinematics& to) const = 0;

  virtual std::vector<vehicle::TrajectoryPoint> plan_rotation(
      const vehicle::Kinematics& from,
      double target_heading) const = 0;

  virtual std::vector<vehicle::TrajectoryPoint> plan_vertical(
      const vehicle::Kinematics& from,
      double target_z) const = 0;

  virtual ~ILocalPlanner() = default;
};

}  // namespace arch_nav::planner

#endif  // NAVIGATION__CORE__PLANNER__LOCAL_PLANNER_HPP_
