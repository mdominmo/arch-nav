#ifndef NAVIGATION__CORE__CONTROLLER__RUNNING_STATE_HPP_
#define NAVIGATION__CORE__CONTROLLER__RUNNING_STATE_HPP_

#include <memory>

#include "core/controller/operational_controller.hpp"
#include "core/controller/navigation_task.hpp"

namespace arch_nav::controller {

struct OperationalController::RunningState : OperationalController::State {
  explicit RunningState(std::unique_ptr<NavigationTask> task);

  void on_enter(OperationalController& ctx) override;
  void on_vehicle_status_update(
      OperationalController& ctx,
      const vehicle::VehicleStatus& status) override;
  void try_stop(OperationalController& ctx) override;

 private:
  std::unique_ptr<NavigationTask> task_;
};

}  // namespace arch_nav::controller

#endif  // NAVIGATION__CORE__CONTROLLER__RUNNING_STATE_HPP_
