#include "config/navigation_config_loader.hpp"

#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace arch_nav::config {

NavigationConfig NavigationConfigLoader::load_from() {
  return load_from(
      ament_index_cpp::get_package_share_directory("arch-nav") +
      "/config/navigation_config.yaml");
}

namespace {
template <typename T>
T get_or(const YAML::Node& node, const std::string& key, T default_value) {
  return node[key] ? node[key].as<T>() : default_value;
}
}  // namespace

NavigationConfig NavigationConfigLoader::load_from(const std::string& path) {
  const YAML::Node root = YAML::LoadFile(path);
  NavigationConfig config;

  config.target_system = get_or<uint8_t>(root, "target_system", config.target_system);

  const YAML::Node planner = root["local_planner"];
  config.local_planner.max_linear_velocity      = planner["max_linear_velocity"].as<double>();
  config.local_planner.max_linear_acceleration  = planner["max_linear_acceleration"].as<double>();
  config.local_planner.max_angular_velocity     = planner["max_angular_velocity"].as<double>();
  config.local_planner.max_angular_acceleration = planner["max_angular_acceleration"].as<double>();
  config.local_planner.max_vertical_velocity    = planner["max_vertical_velocity"].as<double>();
  config.local_planner.max_vertical_acceleration = planner["max_vertical_acceleration"].as<double>();
  config.local_planner.time_step                = planner["time_step"].as<double>();

  if (const YAML::Node landing = root["landing"]) {
    config.landing.v_initial     = get_or<double>(landing, "v_initial",     config.landing.v_initial);
    config.landing.v_min         = get_or<double>(landing, "v_min",         config.landing.v_min);
    config.landing.decay_rate    = get_or<double>(landing, "decay_rate",    config.landing.decay_rate);
    config.landing.max_duration  = get_or<double>(landing, "max_duration",  config.landing.max_duration);
    config.landing.time_step     = get_or<double>(landing, "time_step",     config.landing.time_step);
  }

  if (const YAML::Node topics = root["topics"]) {
    config.topics.vehicle_local_position  = get_or<std::string>(topics, "vehicle_local_position",  config.topics.vehicle_local_position);
    config.topics.vehicle_global_position = get_or<std::string>(topics, "vehicle_global_position", config.topics.vehicle_global_position);
    config.topics.vehicle_status          = get_or<std::string>(topics, "vehicle_status",          config.topics.vehicle_status);
    config.topics.trajectory_setpoint     = get_or<std::string>(topics, "trajectory_setpoint",     config.topics.trajectory_setpoint);
    config.topics.vehicle_command         = get_or<std::string>(topics, "vehicle_command",         config.topics.vehicle_command);
    config.topics.offboard_control_mode   = get_or<std::string>(topics, "offboard_control_mode",   config.topics.offboard_control_mode);
  }

  return config;
}

}  // namespace arch_nav::config
