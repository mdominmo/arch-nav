#ifndef NAVIGATION__CORE__CONTROLLER__HANDOVER_STATE_HPP_
#define NAVIGATION__CORE__CONTROLLER__HANDOVER_STATE_HPP_

#include "core/controller/operational_controller.hpp"

namespace arch_nav::controller {

struct OperationalController::HandoverState : OperationalController::State {
  void on_vehicle_status_update(
      OperationalController& ctx,
      const vehicle::VehicleStatus& status) override;
};

}  // namespace arch_nav::controller

#endif  // NAVIGATION__CORE__CONTROLLER__HANDOVER_STATE_HPP_
