#ifndef NAVIGATION_STATE_REGISTER__NAVIGATION_API_HPP_
#define NAVIGATION_STATE_REGISTER__NAVIGATION_API_HPP_

#include <vector>

#include "core/constants/operation_status.hpp"
#include "core/controller/operational_controller.hpp"
#include "core/model/report/operation_report.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"

namespace arch_nav {

class NavigationApi {
 public:
  explicit NavigationApi(controller::OperationalController& controller);

  void takeoff(double height);
  void land();
  void waypoint_following(std::vector<geographic_msgs::msg::GeoPose> waypoints);
  void cancel_operation();
  constants::OperationStatus        operation_status() const;
  const report::OperationReport*  last_operation_report() const;
  void arm();
  void disarm();

 private:
  controller::OperationalController& controller_;
};

}  // namespace arch_nav

#endif  // NAVIGATION_STATE_REGISTER__NAVIGATION_API_HPP_
