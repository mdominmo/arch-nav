#ifndef NAVIGATION__CORE__CONTROLLER__LAND_TASK_HPP_
#define NAVIGATION__CORE__CONTROLLER__LAND_TASK_HPP_

#include "core/controller/navigation_task.hpp"

namespace arch_nav::controller {

class LandTask : public NavigationTask {
 public:
  explicit LandTask(double z_start);

  void start(
      context::VehicleContext& context,
      planner::ILocalPlanner& planner,
      dispatchers::ICommandDispatcher& dispatcher,
      std::function<void()> on_complete) override;

  void abort() override;

  std::shared_ptr<report::OperationReport> make_report() override;

 private:
  double z_start_;
  dispatchers::ICommandDispatcher* dispatcher_{nullptr};
};

}  // namespace arch_nav::controller

#endif  // NAVIGATION__CORE__CONTROLLER__LAND_TASK_HPP_
