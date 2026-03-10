#ifndef NAVIGATION__CORE__PLANNER__LINEAR_DECREASING_VELOCITY_PROFILE_HPP_
#define NAVIGATION__CORE__PLANNER__LINEAR_DECREASING_VELOCITY_PROFILE_HPP_

#include <algorithm>

namespace arch_nav::planner {

struct LinearDecreasingVelocityProfile {
  double v_initial;   // m/s, velocity at t=0
  double v_min;       // m/s, minimum velocity (saturation floor)
  double decay_rate;  // m/s per second

  double compute(double elapsed_seconds) const {
    return std::max(v_min, v_initial - decay_rate * elapsed_seconds);
  }
};

}  // namespace arch_nav::planner

#endif  // NAVIGATION__CORE__PLANNER__LINEAR_DECREASING_VELOCITY_PROFILE_HPP_
