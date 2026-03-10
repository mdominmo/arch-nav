#include "handover_state.hpp"

#include <memory>

#include "disarmed_state.hpp"
#include "iddle_state.hpp"

#include "core/constants/operation_status.hpp"
#include "core/constants/vehicle_status_states.hpp"

namespace arch_nav::controller {

void OperationalController::HandoverState::on_vehicle_status_update(
    OperationalController& ctx, const vehicle::VehicleStatus& status) {
  if (!status.is_valid() ||
      status.nav_state != constants::VehicleStatusStates::STATE_OFFBOARD) {
    return;
  }
  if (status.arm_state == constants::VehicleStatusStates::STATE_ARMED) {
    ctx.change_state(
        std::make_unique<IddleState>(),
        constants::OperationStatus::IDDLE);
  } else {
    ctx.change_state(
        std::make_unique<DisarmedState>(),
        constants::OperationStatus::DISARMED);
  }
}

}  // namespace arch_nav::controller
