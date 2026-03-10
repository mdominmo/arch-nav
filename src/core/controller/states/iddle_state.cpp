#include "iddle_state.hpp"

#include <memory>

#include "disarmed_state.hpp"
#include "handover_state.hpp"
#include "running_state.hpp"

#include "core/constants/operation_status.hpp"
#include "core/constants/vehicle_status_states.hpp"

namespace arch_nav::controller {

void OperationalController::IddleState::on_vehicle_status_update(
    OperationalController& ctx, const vehicle::VehicleStatus& status) {
  if (!status.is_valid() ||
      status.nav_state != constants::VehicleStatusStates::STATE_OFFBOARD) {
    ctx.change_state(
        std::make_unique<HandoverState>(),
        constants::OperationStatus::HANDOVER);
  } else if (status.arm_state != constants::VehicleStatusStates::STATE_ARMED) {
    ctx.change_state(
        std::make_unique<DisarmedState>(),
        constants::OperationStatus::DISARMED);
  }
}

void OperationalController::IddleState::try_execute(
    OperationalController& ctx, std::unique_ptr<NavigationTask> task) {
  ctx.change_state(
      std::make_unique<RunningState>(std::move(task)),
      constants::OperationStatus::RUNNING);
}

void OperationalController::IddleState::try_command(
    OperationalController& ctx, std::unique_ptr<VehicleCommand> cmd) {
  cmd->execute(ctx.dispatcher_);
}

}  // namespace arch_nav::controller
