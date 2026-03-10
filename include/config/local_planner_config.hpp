#ifndef NAVIGATION_STATE_REGISTER__CONFIG__LOCAL_PLANNER_CONFIG_HPP_
#define NAVIGATION_STATE_REGISTER__CONFIG__LOCAL_PLANNER_CONFIG_HPP_

namespace arch_nav::config {

struct LocalPlannerConfig {
  double max_linear_velocity;
  double max_linear_acceleration;
  double max_angular_velocity;
  double max_angular_acceleration;
  double max_vertical_velocity;
  double max_vertical_acceleration;
  double time_step;
};

}  // namespace arch_nav::config

#endif  // NAVIGATION_STATE_REGISTER__CONFIG__LOCAL_PLANNER_CONFIG_HPP_
