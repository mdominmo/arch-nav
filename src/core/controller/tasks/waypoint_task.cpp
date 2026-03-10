#include "waypoint_task.hpp"

#include <cmath>
#include <functional>
#include <limits>

#include "core/model/vehicle/kinematics.hpp"
#include "utils/gnss_local_frame.hpp"

namespace arch_nav::controller {

namespace {
constexpr double kHeadingThreshold = 0.01;

double normalize_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

WaypointTask::WaypointTask(std::vector<geographic_msgs::msg::GeoPose> waypoints)
    : waypoints_(std::move(waypoints)),
      report_(std::make_shared<report::WaypointReport>(waypoints_.size())) {}

void WaypointTask::start(
    context::VehicleContext& context,
    planner::ILocalPlanner& planner,
    dispatchers::ICommandDispatcher& dispatcher,
    std::function<void()> on_complete) {
  context_     = &context;
  planner_     = &planner;
  dispatcher_  = &dispatcher;
  on_complete_ = std::move(on_complete);

  const auto current     = context_->get_kinematic();
  const auto& target_geo = waypoints_[waypoint_index_];
  target_x_ = utils::gnss_local_frame::lat_to_ned_x(
      target_geo.position.latitude, current.ref_lat);
  target_y_ = utils::gnss_local_frame::lon_to_ned_y(
      target_geo.position.longitude, current.ref_lon, current.ref_lat);
  target_z_ = utils::gnss_local_frame::alt_to_ned_z(
      target_geo.position.altitude, current.ref_alt);

  plan_and_execute_phase();
}

void WaypointTask::abort() {
  if (dispatcher_) dispatcher_->stop();
}

void WaypointTask::plan_and_execute_phase() {
  const auto current = context_->get_kinematic();

  if (phase_ == Phase::ROTATING) {
    const double travel_heading =
        std::atan2(target_y_ - current.y, target_x_ - current.x);
    const double delta =
        std::abs(normalize_angle(travel_heading - current.heading));

    if (delta >= kHeadingThreshold) {
      const auto trajectory = planner_->plan_rotation(current, travel_heading);
      phase_ = Phase::TRAVELING;
      dispatcher_->execute_trajectory(
          trajectory,
          std::bind(&WaypointTask::plan_and_execute_phase, this));
      return;
    }
  }

  vehicle::Kinematics goal{};
  goal.x       = target_x_;
  goal.y       = target_y_;
  goal.z       = target_z_;
  goal.vx      = 0.0;
  goal.vy      = 0.0;
  goal.vz      = 0.0;
  goal.heading = std::numeric_limits<double>::quiet_NaN();

  const auto trajectory = planner_->plan_travel(current, goal);
  dispatcher_->execute_trajectory(
      trajectory,
      std::bind(&WaypointTask::on_waypoint_done, this));
}

std::shared_ptr<report::OperationReport> WaypointTask::make_report() {
  return report_;
}

void WaypointTask::on_waypoint_done() {
  report_->increment_completed();
  ++waypoint_index_;

  if (waypoint_index_ >= waypoints_.size()) {
    auto done = std::move(on_complete_);
    done();
    return;
  }

  const auto current     = context_->get_kinematic();
  const auto& target_geo = waypoints_[waypoint_index_];
  target_x_ = utils::gnss_local_frame::lat_to_ned_x(
      target_geo.position.latitude, current.ref_lat);
  target_y_ = utils::gnss_local_frame::lon_to_ned_y(
      target_geo.position.longitude, current.ref_lon, current.ref_lat);
  target_z_ = utils::gnss_local_frame::alt_to_ned_z(
      target_geo.position.altitude, current.ref_alt);
  phase_ = Phase::ROTATING;

  plan_and_execute_phase();
}

}  // namespace arch_nav::controller
