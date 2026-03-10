#ifndef NAVIGATION_STATE_REGISTER__CONFIG__TOPICS_CONFIG_HPP_
#define NAVIGATION_STATE_REGISTER__CONFIG__TOPICS_CONFIG_HPP_

#include <string>

namespace arch_nav::config {

struct TopicsConfig {
  std::string vehicle_local_position  = "/fmu/out/vehicle_local_position";
  std::string vehicle_global_position = "/fmu/out/vehicle_global_position";
  std::string vehicle_status          = "/fmu/out/vehicle_status";
  std::string trajectory_setpoint     = "/fmu/in/trajectory_setpoint";
  std::string vehicle_command         = "/fmu/in/vehicle_command";
  std::string offboard_control_mode   = "/fmu/in/offboard_control_mode";
};

}  // namespace arch_nav::config

#endif  // NAVIGATION_STATE_REGISTER__CONFIG__TOPICS_CONFIG_HPP_
