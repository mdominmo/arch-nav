#include "navigation_api.hpp"

namespace arch_nav {

NavigationApi::NavigationApi(controller::OperationalController& controller)
    : controller_(controller) {}

void NavigationApi::takeoff(double height) {
  controller_.takeoff(height);
}

void NavigationApi::land() {
  controller_.land();
}

void NavigationApi::waypoint_following(
    std::vector<geographic_msgs::msg::GeoPose> waypoints) {
  controller_.waypoint_following(std::move(waypoints));
}

void NavigationApi::cancel_operation() {
  controller_.stop();
}

constants::OperationStatus NavigationApi::operation_status() const {
  return controller_.operation_status();
}

const report::OperationReport* NavigationApi::last_operation_report() const {
  return controller_.last_operation_report();
}

void NavigationApi::arm() {
  controller_.arm();
}

void NavigationApi::disarm() {
  controller_.disarm();
}

}  // namespace arch_nav
