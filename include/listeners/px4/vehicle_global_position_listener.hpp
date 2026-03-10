#ifndef NAVIGATION_STATE_REGISTER__PX4__LISTENERS__VEHICLE_GLOBAL_POSITION_LISTENER_HPP_
#define NAVIGATION_STATE_REGISTER__PX4__LISTENERS__VEHICLE_GLOBAL_POSITION_LISTENER_HPP_

#include <string>

#include "core/context/vehicle_context.hpp"
#include "adapters/px4/vehicle_global_position_adapter.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arch_nav::listeners::px4 {

class VehicleGlobalPositionListener {
 public:
  explicit VehicleGlobalPositionListener(
      rclcpp::Node& node,
      context::VehicleContext& state_manager,
      const std::string& topic_name = "/fmu/out/vehicle_global_position",
      const rclcpp::QoS& qos = rclcpp::QoS(10).best_effort());

 private:
  void on_message(const px4_msgs::msg::VehicleGlobalPosition& origin);

  context::VehicleContext& state_manager_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr subscription_;
};

}  // namespace arch_nav::listeners::px4

#endif  // NAVIGATION_STATE_REGISTER__PX4__LISTENERS__VEHICLE_GLOBAL_POSITION_LISTENER_HPP_
