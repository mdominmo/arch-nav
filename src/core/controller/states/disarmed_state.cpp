#include "disarmed_state.hpp"

#include <memory>

#include "handover_state.hpp"
#include "iddle_state.hpp"

#include "core/constants/operation_status.hpp"
#include "core/constants/vehicle_status_states.hpp"

namespace arch_nav::controller {

void OperationalController::DisarmedState::try_command(
    OperationalController& ctx, std::unique_ptr<VehicleCommand> cmd) {
  if (cmd_) cmd_->cancel(ctx.dispatcher_);
  cmd_ = std::move(cmd);
  cmd_->execute(ctx.dispatcher_);
}

void OperationalController::DisarmedState::on_vehicle_status_update(
    OperationalController& ctx, const vehicle::VehicleStatus& status) {
  if (status.is_valid() &&
      status.nav_state == constants::VehicleStatusStates::STATE_OFFBOARD &&
      status.arm_state == constants::VehicleStatusStates::STATE_ARMED) {
    if (cmd_) { cmd_->cancel(ctx.dispatcher_); cmd_.reset(); }
    ctx.change_state(
        std::make_unique<IddleState>(),
        constants::OperationStatus::IDDLE);
  } else if (!status.is_valid() ||
             status.nav_state != constants::VehicleStatusStates::STATE_OFFBOARD) {
    if (cmd_) { cmd_->cancel(ctx.dispatcher_); cmd_.reset(); }
    ctx.change_state(
        std::make_unique<HandoverState>(),
        constants::OperationStatus::HANDOVER);
  }
}

}  // namespace arch_nav::controller
