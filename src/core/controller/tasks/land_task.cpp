#include "land_task.hpp"

namespace arch_nav::controller {

LandTask::LandTask(double z_start) : z_start_(z_start) {}

void LandTask::start(
    context::VehicleContext&,
    planner::ILocalPlanner&,
    dispatchers::ICommandDispatcher& dispatcher,
    std::function<void()>) {
  dispatcher_ = &dispatcher;
  dispatcher.execute_land(z_start_);
}

void LandTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

std::shared_ptr<report::OperationReport> LandTask::make_report() {
  return std::make_shared<report::OperationReport>();
}

}  // namespace arch_nav::controller
