#include "core/controller/operational_controller.hpp"

#include <memory>

#include "commands/arm_command.hpp"
#include "commands/disarm_command.hpp"
#include "states/disarmed_state.hpp"
#include "states/handover_state.hpp"
#include "states/iddle_state.hpp"
#include "states/running_state.hpp"
#include "tasks/land_task.hpp"
#include "tasks/point_to_point_task.hpp"
#include "tasks/waypoint_task.hpp"

namespace arch_nav::controller {

OperationalController::OperationalController(
    context::VehicleContext& state_manager,
    planner::ILocalPlanner& planner,
    dispatchers::ICommandDispatcher& dispatcher)
    : state_manager_(state_manager),
      planner_(planner),
      dispatcher_(dispatcher),
      current_state_(nullptr),
      current_status_(constants::OperationStatus::HANDOVER) {
  change_state(
      std::make_unique<HandoverState>(),
      constants::OperationStatus::HANDOVER);
  state_manager_.subscribe_vehicle_status(
      [this](const vehicle::VehicleStatus& status) {
        on_vehicle_status_update(status);
      });
}

OperationalController::~OperationalController() = default;

void OperationalController::takeoff(double height) {
  const auto current = state_manager_.get_kinematic();
  auto task = std::make_unique<PointToPointTask>(
      current.x, current.y, current.z - height, current.heading);
  current_state_->try_execute(*this, std::move(task));
}

void OperationalController::land() {
  const auto current = state_manager_.get_kinematic();
  current_state_->try_execute(*this, std::make_unique<LandTask>(current.z));
}

void OperationalController::waypoint_following(
    std::vector<geographic_msgs::msg::GeoPose> waypoints) {
  auto task = std::make_unique<WaypointTask>(std::move(waypoints));
  current_state_->try_execute(*this, std::move(task));
}

void OperationalController::stop() {
  current_state_->try_stop(*this);
}

void OperationalController::arm() {
  current_state_->try_command(*this, std::make_unique<ArmCommand>());
}

void OperationalController::disarm() {
  current_state_->try_command(*this, std::make_unique<DisarmCommand>());
}

constants::OperationStatus OperationalController::operation_status() const {
  return current_status_;
}

const report::OperationReport* OperationalController::last_operation_report() const {
  return last_report_.get();
}

void OperationalController::on_vehicle_status_update(
    const vehicle::VehicleStatus& status) {
  current_state_->on_vehicle_status_update(*this, status);
}

void OperationalController::change_state(
    std::unique_ptr<State> new_state,
    constants::OperationStatus status) {
  current_status_ = status;
  current_state_ = std::move(new_state);
  current_state_->on_enter(*this);
}

}  // namespace arch_nav::controller
