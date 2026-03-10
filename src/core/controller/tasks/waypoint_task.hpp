#ifndef NAVIGATION__CORE__CONTROLLER__WAYPOINT_TASK_HPP_
#define NAVIGATION__CORE__CONTROLLER__WAYPOINT_TASK_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <vector>

#include "geographic_msgs/msg/geo_pose.hpp"

#include "core/controller/navigation_task.hpp"
#include "core/model/report/waypoint_report.hpp"

namespace arch_nav::controller {

class WaypointTask : public NavigationTask {
 public:
  explicit WaypointTask(std::vector<geographic_msgs::msg::GeoPose> waypoints);

  void start(
      context::VehicleContext& context,
      planner::ILocalPlanner& planner,
      dispatchers::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) override;

  void abort() override;

  std::shared_ptr<report::OperationReport> make_report() override;

 private:
  enum class Phase { ROTATING, TRAVELING };

  void plan_and_execute_phase();
  void on_waypoint_done();

  std::vector<geographic_msgs::msg::GeoPose>  waypoints_;
  std::shared_ptr<report::WaypointReport>     report_;
  std::size_t waypoint_index_{0};
  double      target_x_{0.0};
  double      target_y_{0.0};
  double      target_z_{0.0};
  Phase       phase_{Phase::ROTATING};

  context::VehicleContext*          context_{nullptr};
  planner::ILocalPlanner*           planner_{nullptr};
  dispatchers::ICommandDispatcher*  dispatcher_{nullptr};
  std::function<void()>             on_complete_;
};

}  // namespace arch_nav::controller

#endif  // NAVIGATION__CORE__CONTROLLER__WAYPOINT_TASK_HPP_
