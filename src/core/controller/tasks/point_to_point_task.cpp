#include "point_to_point_task.hpp"

#include "core/model/vehicle/kinematics.hpp"

namespace arch_nav::controller {

PointToPointTask::PointToPointTask(
    double x, double y, double z, double heading)
    : target_x_(x), target_y_(y), target_z_(z), target_heading_(heading) {}

void PointToPointTask::start(
    context::VehicleContext& context,
    planner::ILocalPlanner& planner,
    dispatchers::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  dispatcher_ = &dispatcher;

  const auto current    = context.get_kinematic();
  const auto trajectory = planner.plan_vertical(current, target_z_);
  dispatcher.execute_trajectory(trajectory, std::move(on_complete));
}

void PointToPointTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> PointToPointTask::make_report() {
  return std::make_shared<report::OperationReport>();
}

}  // namespace arch_nav::controller
