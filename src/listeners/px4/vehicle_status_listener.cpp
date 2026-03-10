#include "listeners/px4/vehicle_status_listener.hpp"

namespace arch_nav::listeners::px4 {

VehicleStatusListener::VehicleStatusListener(
    rclcpp::Node& node,
    context::VehicleContext& state_manager,
    const std::string& topic_name,
    const rclcpp::QoS& qos)
    : state_manager_(state_manager) {
  subscription_ = node.create_subscription<px4_msgs::msg::VehicleStatus>(
      topic_name,
      qos,
      [this](const px4_msgs::msg::VehicleStatus& origin) {
        on_message(origin);
      });
}

void VehicleStatusListener::on_message(
    const px4_msgs::msg::VehicleStatus& origin) {
  state_manager_.update(adapters::px4::VehicleStatusAdapter::map_from(origin));
}

}  // namespace arch_nav::listeners::px4
