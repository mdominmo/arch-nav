#ifndef NAVIGATION__CORE__CONTROLLER__IDDLE_STATE_HPP_
#define NAVIGATION__CORE__CONTROLLER__IDDLE_STATE_HPP_

#include "core/controller/operational_controller.hpp"

namespace arch_nav::controller {

struct OperationalController::IddleState : OperationalController::State {
  void on_vehicle_status_update(
      OperationalController& ctx,
      const vehicle::VehicleStatus& status) override;
  void try_execute(
      OperationalController& ctx,
      std::unique_ptr<NavigationTask> task) override;
  void try_command(
      OperationalController& ctx,
      std::unique_ptr<VehicleCommand> cmd) override;
};

}  // namespace arch_nav::controller

#endif  // NAVIGATION__CORE__CONTROLLER__IDDLE_STATE_HPP_
