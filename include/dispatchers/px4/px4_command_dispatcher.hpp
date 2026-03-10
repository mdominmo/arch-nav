#ifndef NAVIGATION_STATE_REGISTER__PX4__DISPATCHERS__PX4_COMMAND_DISPATCHER_HPP_
#define NAVIGATION_STATE_REGISTER__PX4__DISPATCHERS__PX4_COMMAND_DISPATCHER_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "rclcpp/rclcpp.hpp"

#include "config/landing_config.hpp"
#include "dispatchers/i_command_dispatcher.hpp"

namespace arch_nav::dispatchers::px4 {

class Px4CommandDispatcher : public ICommandDispatcher {
 public:
  explicit Px4CommandDispatcher(
      rclcpp::Node& node,
      const std::string& trajectory_setpoint_topic = "/fmu/in/trajectory_setpoint",
      const std::string& vehicle_command_topic = "/fmu/in/vehicle_command",
      const std::string& offboard_control_mode_topic = "/fmu/in/offboard_control_mode",
      uint8_t target_system = 1,
      const rclcpp::QoS& qos = rclcpp::QoS(10).best_effort(),
      const config::LandingConfig& landing_config = config::LandingConfig{});

  void execute_trajectory(
      std::vector<vehicle::TrajectoryPoint> trajectory,
      std::function<void()> on_complete) override;

  void execute_arm() override;
  void execute_disarm() override;
  void execute_land(double z_start) override;

  void stop() override;

 private:
  enum class ControlMode { POSITION, VELOCITY };

  void publish_vehicle_command(uint16_t command, float param1 = 0.0f);
  void publish_offboard_heartbeat();
  void set_control_mode(ControlMode mode);
  static std::vector<double> extract_times(
      const std::vector<vehicle::TrajectoryPoint>& points);

  rclcpp::Node&                                                    node_;
  rclcpp::Clock::SharedPtr                                         clock_;
  uint8_t                                                          target_system_;
  config::LandingConfig                                            landing_config_;
  ControlMode                                                      control_mode_{ControlMode::POSITION};

  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  trajectory_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      command_publisher_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_publisher_;

  rclcpp::CallbackGroup::SharedPtr  heartbeat_callback_group_;
  rclcpp::CallbackGroup::SharedPtr  operation_callback_group_;
  rclcpp::TimerBase::SharedPtr      heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr      operation_timer_;

  // trajectory execution state
  std::vector<px4_msgs::msg::TrajectorySetpoint> trajectory_;
  std::vector<double>                            trajectory_times_;
  std::size_t                                    trajectory_index_{0};
  std::function<void()>                          on_trajectory_complete_;
  rclcpp::Time                                   trajectory_t_start_;

  // land descent state
  double land_elapsed_s_{0.0};
};

}  // namespace arch_nav::dispatchers::px4

#endif  // NAVIGATION_STATE_REGISTER__PX4__DISPATCHERS__PX4_COMMAND_DISPATCHER_HPP_
