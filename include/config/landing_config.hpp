#ifndef NAVIGATION_STATE_REGISTER__CONFIG__LANDING_CONFIG_HPP_
#define NAVIGATION_STATE_REGISTER__CONFIG__LANDING_CONFIG_HPP_

namespace arch_nav::config {

struct LandingConfig {
  double v_initial{0.3};    // m/s, initial descent velocity
  double v_min{0.1};        // m/s, minimum descent velocity (saturation)
  double decay_rate{0.05};  // m/s per second of elapsed descent
  double max_duration{120.0};  // s, maximum landing duration
  double time_step{0.05};   // s, setpoint publication interval
};

}  // namespace arch_nav::config

#endif  // NAVIGATION_STATE_REGISTER__CONFIG__LANDING_CONFIG_HPP_
