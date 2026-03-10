#ifndef NAVIGATION_STATE_REGISTER__PX4__LISTENERS__VEHICLE_STATUS_LISTENER_HPP_
#define NAVIGATION_STATE_REGISTER__PX4__LISTENERS__VEHICLE_STATUS_LISTENER_HPP_

#include <string>

#include "adapters/px4/vehicle_status_adapter.hpp"
#include "core/context/vehicle_context.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace arch_nav::listeners::px4 {

class VehicleStatusListener {
 public:
  explicit VehicleStatusListener(
      rclcpp::Node& node,
      context::VehicleContext& state_manager,
      const std::string& topic_name = "/fmu/out/vehicle_status",
      const rclcpp::QoS& qos = rclcpp::QoS(10).best_effort());

 private:
  void on_message(const px4_msgs::msg::VehicleStatus& origin);

  context::VehicleContext& state_manager_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr subscription_;
};

}  // namespace arch_nav::listeners::px4

#endif  // NAVIGATION_STATE_REGISTER__PX4__LISTENERS__VEHICLE_STATUS_LISTENER_HPP_
