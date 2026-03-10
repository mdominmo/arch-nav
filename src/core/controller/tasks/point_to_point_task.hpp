#ifndef NAVIGATION__CORE__CONTROLLER__POINT_TO_POINT_TASK_HPP_
#define NAVIGATION__CORE__CONTROLLER__POINT_TO_POINT_TASK_HPP_

#include <functional>

#include "core/controller/navigation_task.hpp"

namespace arch_nav::controller {

class PointToPointTask : public NavigationTask {
 public:
  PointToPointTask(double x, double y, double z, double heading);

  void start(
      context::VehicleContext& context,
      planner::ILocalPlanner& planner,
      dispatchers::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) override;

  void abort() override;

  std::shared_ptr<report::OperationReport> make_report() override;

 private:
  double target_x_;
  double target_y_;
  double target_z_;
  double target_heading_;
  dispatchers::ICommandDispatcher* dispatcher_{nullptr};
};

}  // namespace arch_nav::controller

#endif  // NAVIGATION__CORE__CONTROLLER__POINT_TO_POINT_TASK_HPP_
