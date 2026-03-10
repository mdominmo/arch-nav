#include "core/planner/kinematic_point_to_point_local_planner.hpp"

#include <algorithm>
#include <cmath>
#include <tuple>


namespace arch_nav::planner {

namespace {

struct TrapezoidalProfile {
  double v0;
  double v_cruise;
  double vf;
  double a_max;
  double t1;
  double t12;
  double t_total;
  double d1;
  double d12;
};

TrapezoidalProfile build_profile(
    double distance,
    double v0,
    double vf,
    double v_max,
    double a_max)
{
  v0 = std::clamp(v0, 0.0, v_max);
  vf = std::clamp(vf, 0.0, v_max);

  const double d_min_decel = (v0 * v0 - vf * vf) / (2.0 * a_max);
  if (d_min_decel > distance) {
    vf = std::sqrt(std::max(0.0, v0 * v0 - 2.0 * a_max * distance));
  }

  double v_cruise = v_max;
  double d_acc = (v_cruise * v_cruise - v0 * v0) / (2.0 * a_max);
  double d_dec = (v_cruise * v_cruise - vf * vf) / (2.0 * a_max);

  if (d_acc + d_dec > distance) {
    v_cruise = std::sqrt(a_max * distance + (v0 * v0 + vf * vf) / 2.0);
    v_cruise = std::min(v_cruise, v_max);
    d_acc = (v_cruise * v_cruise - v0 * v0) / (2.0 * a_max);
    d_dec = (v_cruise * v_cruise - vf * vf) / (2.0 * a_max);
  }

  const double t1 = (v_cruise - v0) / a_max;
  const double d_cruise = std::max(0.0, distance - d_acc - d_dec);
  const double t_cruise = (v_cruise > 1e-9) ? d_cruise / v_cruise : 0.0;
  const double t3 = (v_cruise - vf) / a_max;
  const double t12 = t1 + t_cruise;
  const double d12 = d_acc + d_cruise;

  return {v0, v_cruise, vf, a_max, t1, t12, t12 + t3, d_acc, d12};
}

std::tuple<double, double, double> evaluate_profile(const TrapezoidalProfile & p, double t)
{
  t = std::clamp(t, 0.0, p.t_total);

  if (t <= p.t1) {
    const double tau = t;
    return {
      p.v0 * tau + 0.5 * p.a_max * tau * tau,
      p.v0 + p.a_max * tau,
      p.a_max
    };
  }

  if (t <= p.t12) {
    const double tau = t - p.t1;
    return {
      p.d1 + p.v_cruise * tau,
      p.v_cruise,
      0.0
    };
  }

  const double tau = t - p.t12;
  return {
    p.d12 + p.v_cruise * tau - 0.5 * p.a_max * tau * tau,
    p.v_cruise - p.a_max * tau,
    -p.a_max
  };
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

}  // namespace

KinematicPointToPointLocalPlanner::KinematicPointToPointLocalPlanner(
    double max_linear_velocity,
    double max_linear_acceleration,
    double max_angular_velocity,
    double max_angular_acceleration,
    double max_vertical_velocity,
    double max_vertical_acceleration,
    double time_step)
: max_linear_velocity_(max_linear_velocity),
  max_linear_acceleration_(max_linear_acceleration),
  max_angular_velocity_(max_angular_velocity),
  max_angular_acceleration_(max_angular_acceleration),
  max_vertical_velocity_(max_vertical_velocity),
  max_vertical_acceleration_(max_vertical_acceleration),
  time_step_(time_step)
{
}

std::vector<vehicle::TrajectoryPoint> KinematicPointToPointLocalPlanner::plan_travel(
    const vehicle::Kinematics& from,
    const vehicle::Kinematics& to) const
{
  const double dx = to.x - from.x;
  const double dy = to.y - from.y;
  const double dz = to.z - from.z;
  const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

  std::vector<vehicle::TrajectoryPoint> trajectory;

  if (distance < 1e-6) {
    trajectory.push_back({0.0, from.x, from.y, from.z,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, from.heading, 0.0});
    return trajectory;
  }

  const double ux = dx / distance;
  const double uy = dy / distance;
  const double uz = dz / distance;
  const double travel_heading =
      (std::abs(ux) < 1e-9 && std::abs(uy) < 1e-9 && !std::isnan(to.heading))
      ? to.heading
      : std::atan2(uy, ux);

  const double v0 = from.vx * ux + from.vy * uy + from.vz * uz;
  const double vf = to.vx * ux + to.vy * uy + to.vz * uz;

  const TrapezoidalProfile linear_profile = build_profile(
      distance, v0, vf,
      max_linear_velocity_,
      max_linear_acceleration_);

  const double delta_heading    = normalize_angle(travel_heading - from.heading);
  const double angular_distance = std::abs(delta_heading);
  const double angular_dir      = (delta_heading >= 0.0) ? 1.0 : -1.0;
  const bool   has_rotation     = angular_distance >= 1e-6;

  const TrapezoidalProfile angular_profile = has_rotation
      ? build_profile(
          angular_distance, 0.0, 0.0,
          max_angular_velocity_,
          max_angular_acceleration_)
      : TrapezoidalProfile{};

  const double t_total = has_rotation
      ? std::max(linear_profile.t_total, angular_profile.t_total)
      : linear_profile.t_total;

  const double dt      = time_step_;
  const int    n_steps = static_cast<int>(t_total / dt);

  trajectory.reserve(static_cast<std::size_t>(n_steps) + 2);

  for (int i = 0; i <= n_steps; ++i) {
    const double t = i * dt;
    const auto [s, v, a] = evaluate_profile(linear_profile, t);

    double heading = travel_heading;
    double omega   = 0.0;
    if (has_rotation) {
      const auto ang = evaluate_profile(angular_profile, t);
      heading = from.heading + angular_dir * std::get<0>(ang);
      omega   = angular_dir * std::get<1>(ang);
    }

    trajectory.push_back({
      t,
      from.x + s * ux,
      from.y + s * uy,
      from.z + s * uz,
      v * ux,
      v * uy,
      v * uz,
      a * ux,
      a * uy,
      a * uz,
      heading,
      omega
    });
  }

  if (t_total - n_steps * dt > 1e-9) {
    trajectory.push_back({
      t_total,
      to.x,
      to.y,
      to.z,
      linear_profile.vf * ux,
      linear_profile.vf * uy,
      linear_profile.vf * uz,
      0.0,
      0.0,
      0.0,
      travel_heading,
      0.0
    });
  }

  return trajectory;
}

std::vector<vehicle::TrajectoryPoint> KinematicPointToPointLocalPlanner::plan_rotation(
    const vehicle::Kinematics& from,
    double target_heading) const
{
  const double delta_heading = normalize_angle(target_heading - from.heading);
  const double angular_distance = std::abs(delta_heading);
  const double direction = (delta_heading >= 0.0) ? 1.0 : -1.0;

  std::vector<vehicle::TrajectoryPoint> trajectory;

  if (angular_distance < 1e-6) {
    trajectory.push_back({0.0, from.x, from.y, from.z,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, from.heading, 0.0});
    return trajectory;
  }

  const TrapezoidalProfile profile = build_profile(
      angular_distance, 0.0, 0.0,
      max_angular_velocity_,
      max_angular_acceleration_);

  const double dt = time_step_;
  const int n_steps = static_cast<int>(profile.t_total / dt);

  trajectory.reserve(static_cast<std::size_t>(n_steps) + 2);

  for (int i = 0; i <= n_steps; ++i) {
    const double t = i * dt;
    const auto [s, v, a] = evaluate_profile(profile, t);

    trajectory.push_back({
      t,
      from.x,
      from.y,
      from.z,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      from.heading + direction * s,
      direction * v
    });
  }

  if (profile.t_total - n_steps * dt > 1e-9) {
    trajectory.push_back({
      profile.t_total,
      from.x,
      from.y,
      from.z,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      target_heading,
      0.0
    });
  }

  return trajectory;
}

std::vector<vehicle::TrajectoryPoint> KinematicPointToPointLocalPlanner::plan_vertical(
    const vehicle::Kinematics& from,
    double target_z) const
{
  const double dz       = target_z - from.z;
  const double distance = std::abs(dz);
  const double dir      = (dz >= 0.0) ? 1.0 : -1.0;

  std::vector<vehicle::TrajectoryPoint> trajectory;

  if (distance < 1e-6) {
    trajectory.push_back({0.0, from.x, from.y, from.z,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, from.heading, 0.0});
    return trajectory;
  }

  const double v0 = std::clamp(dir * from.vz, 0.0, max_vertical_velocity_);
  const TrapezoidalProfile profile = build_profile(
      distance, v0, 0.0,
      max_vertical_velocity_,
      max_vertical_acceleration_);

  const double dt      = time_step_;
  const int    n_steps = static_cast<int>(profile.t_total / dt);

  trajectory.reserve(static_cast<std::size_t>(n_steps) + 2);

  for (int i = 0; i <= n_steps; ++i) {
    const double t = i * dt;
    const auto [s, v, a] = evaluate_profile(profile, t);

    trajectory.push_back({
      t,
      from.x,
      from.y,
      from.z + dir * s,
      0.0,
      0.0,
      dir * v,
      0.0,
      0.0,
      dir * a,
      from.heading,
      0.0
    });
  }

  if (profile.t_total - n_steps * dt > 1e-9) {
    trajectory.push_back({
      profile.t_total,
      from.x,
      from.y,
      target_z,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      0.0,
      from.heading,
      0.0
    });
  }

  return trajectory;
}

}  // namespace arch_nav::planner
