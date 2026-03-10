#include "dispatchers/px4/px4_command_dispatcher.hpp"

#include <chrono>
#include <cmath>
#include <limits>

#include "adapters/px4/trajectory_setpoint_adapter.hpp"
#include "core/planner/linear_decreasing_velocity_profile.hpp"

namespace arch_nav::dispatchers::px4 {

Px4CommandDispatcher::Px4CommandDispatcher(
    rclcpp::Node& node,
    const std::string& trajectory_setpoint_topic,
    const std::string& vehicle_command_topic,
    const std::string& offboard_control_mode_topic,
    uint8_t target_system,
    const rclcpp::QoS& qos,
    const config::LandingConfig& landing_config)
    : node_(node),
      clock_(node.get_clock()),
      target_system_(target_system),
      landing_config_(landing_config) {
  trajectory_publisher_ = node.create_publisher<px4_msgs::msg::TrajectorySetpoint>(
      trajectory_setpoint_topic, qos);
  command_publisher_ = node.create_publisher<px4_msgs::msg::VehicleCommand>(
      vehicle_command_topic, qos);
  offboard_publisher_ = node.create_publisher<px4_msgs::msg::OffboardControlMode>(
      offboard_control_mode_topic, qos);
  heartbeat_callback_group_ = node.create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  operation_callback_group_ = node.create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  heartbeat_timer_ = node.create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() { publish_offboard_heartbeat(); },
      heartbeat_callback_group_);
}

// ── trajectory ───────────────────────────────────────────────────────────────

void Px4CommandDispatcher::execute_trajectory(
    std::vector<vehicle::TrajectoryPoint> trajectory,
    std::function<void()> on_complete) {
  stop();
  set_control_mode(ControlMode::POSITION);
  trajectory_             = adapters::px4::TrajectorySetpointAdapter::map_from(trajectory);
  trajectory_times_       = extract_times(trajectory);
  trajectory_index_       = 0;
  on_trajectory_complete_ = std::move(on_complete);
  trajectory_t_start_     = clock_->now();

  operation_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(10),
      [this]() {
        if (trajectory_index_ >= trajectory_.size()) {
          operation_timer_->cancel();
          operation_timer_.reset();
          if (on_trajectory_complete_) on_trajectory_complete_();
          return;
        }
        const double elapsed = (clock_->now() - trajectory_t_start_).seconds();
        if (elapsed >= trajectory_times_[trajectory_index_]) {
          auto msg      = trajectory_[trajectory_index_++];
          msg.timestamp = static_cast<uint64_t>(clock_->now().nanoseconds() / 1000);
          trajectory_publisher_->publish(msg);
        }
      },
      operation_callback_group_);
}

// ── arm / disarm ─────────────────────────────────────────────────────────────

void Px4CommandDispatcher::execute_arm() {
  stop();
  operation_timer_ = node_.create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
      },
      operation_callback_group_);
}

void Px4CommandDispatcher::execute_disarm() {
  stop();
  publish_vehicle_command(
      px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
}

// ── land ─────────────────────────────────────────────────────────────────────

void Px4CommandDispatcher::execute_land(double /*z_start*/) {
  stop();
  set_control_mode(ControlMode::VELOCITY);
  land_elapsed_s_ = 0.0;

  const planner::LinearDecreasingVelocityProfile profile{
      landing_config_.v_initial,
      landing_config_.v_min,
      landing_config_.decay_rate};

  const auto dt = std::chrono::milliseconds(
      static_cast<int>(landing_config_.time_step * 1000));

  operation_timer_ = node_.create_wall_timer(
      dt,
      [this, profile]() {
        if (land_elapsed_s_ >= landing_config_.max_duration) {
          operation_timer_->cancel();
          operation_timer_.reset();
          return;
        }
        const double vz = profile.compute(land_elapsed_s_);
        land_elapsed_s_ += landing_config_.time_step;

        const float nan = std::numeric_limits<float>::quiet_NaN();
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.timestamp    = static_cast<uint64_t>(clock_->now().nanoseconds() / 1000);
        msg.position     = {nan, nan, nan};
        msg.velocity     = {0.0f, 0.0f, static_cast<float>(vz)};
        msg.acceleration = {nan, nan, nan};
        msg.yaw          = nan;
        trajectory_publisher_->publish(msg);
      },
      operation_callback_group_);
}

// ── stop ─────────────────────────────────────────────────────────────────────

void Px4CommandDispatcher::stop() {
  if (operation_timer_) {
    operation_timer_->cancel();
    operation_timer_.reset();
  }
  set_control_mode(ControlMode::POSITION);
  trajectory_.clear();
  on_trajectory_complete_ = nullptr;
  land_elapsed_s_         = 0.0;
}

// ── private ──────────────────────────────────────────────────────────────────

void Px4CommandDispatcher::publish_vehicle_command(uint16_t command, float param1) {
  px4_msgs::msg::VehicleCommand msg;
  msg.timestamp        = static_cast<uint64_t>(clock_->now().nanoseconds() / 1000);
  msg.command          = command;
  msg.param1           = param1;
  msg.target_system    = target_system_;
  msg.target_component = 1;
  msg.source_system    = 1;
  msg.source_component = 1;
  msg.from_external    = true;
  command_publisher_->publish(msg);
}

void Px4CommandDispatcher::set_control_mode(ControlMode mode) {
  control_mode_ = mode;
}

void Px4CommandDispatcher::publish_offboard_heartbeat() {
  px4_msgs::msg::OffboardControlMode msg;
  msg.timestamp    = static_cast<uint64_t>(clock_->now().nanoseconds() / 1000);
  msg.position     = (control_mode_ == ControlMode::POSITION);
  msg.velocity     = (control_mode_ == ControlMode::VELOCITY);
  msg.acceleration = false;
  msg.attitude     = false;
  msg.body_rate    = false;
  offboard_publisher_->publish(msg);
}

std::vector<double> Px4CommandDispatcher::extract_times(
    const std::vector<vehicle::TrajectoryPoint>& points) {
  std::vector<double> times;
  times.reserve(points.size());
  for (const auto& p : points) {
    times.push_back(p.t);
  }
  return times;
}

}  // namespace arch_nav::dispatchers::px4
