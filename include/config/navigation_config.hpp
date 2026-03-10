#ifndef NAVIGATION_STATE_REGISTER__CONFIG__NAVIGATION_CONFIG_HPP_
#define NAVIGATION_STATE_REGISTER__CONFIG__NAVIGATION_CONFIG_HPP_

#include <cstdint>

#include "config/landing_config.hpp"
#include "config/local_planner_config.hpp"
#include "config/topics_config.hpp"

namespace arch_nav::config {

struct NavigationConfig {
  uint8_t            target_system{0};
  LocalPlannerConfig local_planner;
  LandingConfig      landing;
  TopicsConfig       topics;
};

}  // namespace arch_nav::config

#endif  // NAVIGATION_STATE_REGISTER__CONFIG__NAVIGATION_CONFIG_HPP_
