#ifndef NAVIGATION__CORE__CONTROLLER__DISARMED_STATE_HPP_
#define NAVIGATION__CORE__CONTROLLER__DISARMED_STATE_HPP_

#include <memory>

#include "core/controller/operational_controller.hpp"
#include "core/controller/vehicle_command.hpp"

namespace arch_nav::controller {

struct OperationalController::DisarmedState : OperationalController::State {
  void try_command(
      OperationalController& ctx,
      std::unique_ptr<VehicleCommand> cmd) override;
  void on_vehicle_status_update(
      OperationalController& ctx,
      const vehicle::VehicleStatus& status) override;

 private:
  std::unique_ptr<VehicleCommand> cmd_;
};

}  // namespace arch_nav::controller

#endif  // NAVIGATION__CORE__CONTROLLER__DISARMED_STATE_HPP_
