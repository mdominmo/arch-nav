#include "running_state.hpp"

#include <memory>

#include "disarmed_state.hpp"
#include "handover_state.hpp"
#include "iddle_state.hpp"

#include "core/constants/operation_status.hpp"
#include "core/constants/vehicle_status_states.hpp"

namespace arch_nav::controller {

OperationalController::RunningState::RunningState(
    std::unique_ptr<NavigationTask> task)
    : task_(std::move(task)) {}

void OperationalController::RunningState::on_enter(OperationalController& ctx) {
  ctx.last_report_ = task_->make_report();
  task_->start(
      ctx.state_manager_,
      ctx.planner_,
      ctx.dispatcher_,
      [&ctx]() {
        ctx.last_report_->complete();
        ctx.change_state(
            std::make_unique<IddleState>(),
            constants::OperationStatus::IDDLE);
      });
}

void OperationalController::RunningState::try_stop(OperationalController& ctx) {
  task_->abort();
  ctx.last_report_->abort();
  ctx.change_state(
      std::make_unique<IddleState>(),
      constants::OperationStatus::IDDLE);
}

void OperationalController::RunningState::on_vehicle_status_update(
    OperationalController& ctx, const vehicle::VehicleStatus& status) {
  if (status.is_valid() &&
      status.nav_state == constants::VehicleStatusStates::STATE_OFFBOARD &&
      status.arm_state == constants::VehicleStatusStates::STATE_ARMED) {
    return;
  }
  task_->abort();
  ctx.last_report_->abort();
  if (!status.is_valid() ||
      status.nav_state != constants::VehicleStatusStates::STATE_OFFBOARD) {
    ctx.change_state(
        std::make_unique<HandoverState>(),
        constants::OperationStatus::HANDOVER);
  } else {
    ctx.change_state(
        std::make_unique<DisarmedState>(),
        constants::OperationStatus::DISARMED);
  }
}

}  // namespace arch_nav::controller
