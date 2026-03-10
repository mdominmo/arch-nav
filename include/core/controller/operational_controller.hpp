#ifndef NAVIGATION_STATE_REGISTER__CORE__CONTROLLER__OPERATIONAL_CONTROLLER_HPP_
#define NAVIGATION_STATE_REGISTER__CORE__CONTROLLER__OPERATIONAL_CONTROLLER_HPP_

#include <memory>
#include <vector>

#include "core/constants/operation_status.hpp"
#include "core/controller/navigation_task.hpp"
#include "core/model/report/operation_report.hpp"
#include "core/controller/vehicle_command.hpp"
#include "core/context/vehicle_context.hpp"
#include "core/model/vehicle/vehicle_status.hpp"
#include "core/planner/local_planner.hpp"
#include "dispatchers/i_command_dispatcher.hpp"
#include "geographic_msgs/msg/geo_pose.hpp"

namespace arch_nav::controller {

class OperationalController {
 public:
  explicit OperationalController(
      context::VehicleContext& state_manager,
      planner::ILocalPlanner& planner,
      dispatchers::ICommandDispatcher& dispatcher);

  ~OperationalController();

  void waypoint_following(std::vector<geographic_msgs::msg::GeoPose> waypoints);
  void takeoff(double height);
  void land();
  void stop();
  void arm();
  void disarm();

  constants::OperationStatus         operation_status() const;
  const report::OperationReport*   last_operation_report() const;

 private:
  struct State {
    virtual void on_enter(OperationalController&) {}
    virtual void on_vehicle_status_update(
        OperationalController&, const vehicle::VehicleStatus&) {}
    virtual void try_execute(
        OperationalController&, std::unique_ptr<NavigationTask>) {}
    virtual void try_command(
        OperationalController&, std::unique_ptr<VehicleCommand>) {}
    virtual void try_stop(OperationalController&) {}
    virtual ~State() = default;
  };

  struct HandoverState;
  struct DisarmedState;
  struct IddleState;
  struct RunningState;

  void on_vehicle_status_update(const vehicle::VehicleStatus& status);
  void change_state(std::unique_ptr<State> new_state, constants::OperationStatus status);

  context::VehicleContext&                   state_manager_;
  planner::ILocalPlanner&                    planner_;
  dispatchers::ICommandDispatcher&           dispatcher_;
  std::unique_ptr<State>                     current_state_;
  constants::OperationStatus                 current_status_;
  std::shared_ptr<report::OperationReport>   last_report_;
};

}  // namespace arch_nav::controller

#endif  // NAVIGATION_STATE_REGISTER__CORE__CONTROLLER__OPERATIONAL_CONTROLLER_HPP_
