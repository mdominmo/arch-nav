#include "listeners/px4/vehicle_global_position_listener.hpp"

namespace arch_nav::listeners::px4 {

VehicleGlobalPositionListener::VehicleGlobalPositionListener(
    rclcpp::Node& node,
    context::VehicleContext& state_manager,
    const std::string& topic_name,
    const rclcpp::QoS& qos)
    : state_manager_(state_manager) {
  subscription_ = node.create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
      topic_name,
      qos,
      [this](const px4_msgs::msg::VehicleGlobalPosition& origin) {
        on_message(origin);
      });
}

void VehicleGlobalPositionListener::on_message(
    const px4_msgs::msg::VehicleGlobalPosition& origin) {
  state_manager_.update(adapters::px4::VehicleGlobalPositionAdapter::map_from(origin));
}

}  // namespace arch_nav::listeners::px4
