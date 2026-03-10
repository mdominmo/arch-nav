#include "listeners/px4/vehicle_local_position_listener.hpp"

namespace arch_nav::listeners::px4 {

VehicleLocalPositionListener::VehicleLocalPositionListener(
    rclcpp::Node& node,
    context::VehicleContext& state_manager,
    const std::string& topic_name,
    const rclcpp::QoS& qos)
    : state_manager_(state_manager) {
  subscription_ = node.create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      topic_name,
      qos,
      [this](const px4_msgs::msg::VehicleLocalPosition& origin) {
        on_message(origin);
      });
}

void VehicleLocalPositionListener::on_message(
    const px4_msgs::msg::VehicleLocalPosition& origin) {
  state_manager_.update(adapters::px4::VehicleLocalPositionAdapter::map_from(origin));
}

}  // namespace arch_nav::listeners::px4
