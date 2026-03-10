#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "core/planner/kinematic_point_to_point_local_planner.hpp"

using arch_nav::planner::KinematicPointToPointLocalPlanner;
using arch_nav::vehicle::Kinematics;
using arch_nav::vehicle::TrajectoryPoint;

namespace {

constexpr double kMaxLinearVelocity     = 2.0;
constexpr double kMaxAngularVelocity    = 1.0;

KinematicPointToPointLocalPlanner make_planner()
{
  return KinematicPointToPointLocalPlanner(kMaxLinearVelocity, 1.0, kMaxAngularVelocity, 0.5, 0.5, 0.2, 0.1);
}

Kinematics make_state(
    double x, double y, double z, double heading,
    double vx = 0.0, double vy = 0.0, double vz = 0.0)
{
  return Kinematics(
      x, y, z, vx, vy, vz, 0.0, 0.0, 0.0,
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN(),
      heading);
}

}  // namespace

TEST(KinematicPointToPointLocalPlannerPlanTravel, SamePosition_ReturnsSinglePoint)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(1.0, 2.0, 0.0, 0.0);
  const auto trajectory = planner.plan_travel(initial, initial);
  ASSERT_EQ(trajectory.size(), 1u);
  EXPECT_DOUBLE_EQ(trajectory.front().t, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.front().x, 1.0);
  EXPECT_DOUBLE_EQ(trajectory.front().y, 2.0);
}

TEST(KinematicPointToPointLocalPlannerPlanTravel, FirstPoint_MatchesInitialState)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const Kinematics goal    = make_state(10.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_travel(initial, goal);
  ASSERT_FALSE(trajectory.empty());
  EXPECT_DOUBLE_EQ(trajectory.front().t,  0.0);
  EXPECT_DOUBLE_EQ(trajectory.front().x,  0.0);
  EXPECT_DOUBLE_EQ(trajectory.front().y,  0.0);
  EXPECT_DOUBLE_EQ(trajectory.front().z,  0.0);
  EXPECT_DOUBLE_EQ(trajectory.front().vx, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.front().vy, 0.0);
  EXPECT_DOUBLE_EQ(trajectory.front().vz, 0.0);
}

TEST(KinematicPointToPointLocalPlannerPlanTravel, LastPoint_MatchesGoalPosition)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const Kinematics goal    = make_state(10.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_travel(initial, goal);
  ASSERT_FALSE(trajectory.empty());
  EXPECT_NEAR(trajectory.back().x, 10.0, 1e-9);
  EXPECT_NEAR(trajectory.back().y,  0.0, 1e-9);
  EXPECT_NEAR(trajectory.back().z,  0.0, 1e-9);
}

TEST(KinematicPointToPointLocalPlannerPlanTravel, Timestamps_MonotonicallyIncreasing)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const Kinematics goal    = make_state(10.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_travel(initial, goal);
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_GT(trajectory[i].t, trajectory[i - 1].t);
  }
}

TEST(KinematicPointToPointLocalPlannerPlanTravel, FirstHeading_MatchesInitialHeading)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.5);
  const Kinematics goal    = make_state(3.0, 4.0, 0.0, 0.0);
  const auto trajectory = planner.plan_travel(initial, goal);
  ASSERT_FALSE(trajectory.empty());
  EXPECT_NEAR(trajectory.front().heading, 0.5, 1e-9);
}

TEST(KinematicPointToPointLocalPlannerPlanTravel, LastHeading_MatchesTravelDirection)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.5);
  const Kinematics goal    = make_state(3.0, 4.0, 0.0, 0.0);
  const double expected_heading = std::atan2(4.0, 3.0);
  const auto trajectory = planner.plan_travel(initial, goal);
  ASSERT_FALSE(trajectory.empty());
  EXPECT_NEAR(trajectory.back().heading, expected_heading, 1e-9);
}

TEST(KinematicPointToPointLocalPlannerPlanTravel, Omega_NonZeroWhenHeadingDiffers)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const Kinematics goal    = make_state(3.0, 4.0, 0.0, 0.0);
  const auto trajectory = planner.plan_travel(initial, goal);
  bool found_nonzero = false;
  for (const auto & point : trajectory) {
    if (std::abs(point.omega) > 1e-9) {
      found_nonzero = true;
      break;
    }
  }
  EXPECT_TRUE(found_nonzero);
}

TEST(KinematicPointToPointLocalPlannerPlanTravel, Omega_ZeroThroughout)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const Kinematics goal    = make_state(10.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_travel(initial, goal);
  for (const auto & point : trajectory) {
    EXPECT_DOUBLE_EQ(point.omega, 0.0);
  }
}

TEST(KinematicPointToPointLocalPlannerPlanTravel, MaxLinearVelocity_NotExceeded)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const Kinematics goal    = make_state(10.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_travel(initial, goal);
  for (const auto & point : trajectory) {
    const double speed = std::sqrt(
        point.vx * point.vx + point.vy * point.vy + point.vz * point.vz);
    EXPECT_LE(speed, kMaxLinearVelocity + 1e-9);
  }
}

TEST(KinematicPointToPointLocalPlannerPlanTravel, TrapezoidalProfile_ReachesMaxVelocity)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const Kinematics goal    = make_state(10.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_travel(initial, goal);
  bool reached_max = false;
  for (const auto & point : trajectory) {
    const double speed = std::sqrt(
        point.vx * point.vx + point.vy * point.vy + point.vz * point.vz);
    if (speed >= kMaxLinearVelocity - 1e-6) {
      reached_max = true;
      break;
    }
  }
  EXPECT_TRUE(reached_max);
}

TEST(KinematicPointToPointLocalPlannerPlanTravel, TriangularProfile_DoesNotReachMaxVelocity)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const Kinematics goal    = make_state(1.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_travel(initial, goal);
  for (const auto & point : trajectory) {
    const double speed = std::sqrt(
        point.vx * point.vx + point.vy * point.vy + point.vz * point.vz);
    EXPECT_LT(speed, kMaxLinearVelocity - 1e-6);
  }
}

TEST(KinematicPointToPointLocalPlannerPlanTravel, VelocityCoherentWithPosition)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const Kinematics goal    = make_state(10.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_travel(initial, goal);
  double integrated_distance = 0.0;
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    const double dt  = trajectory[i].t - trajectory[i - 1].t;
    const double vi  = std::sqrt(
        trajectory[i - 1].vx * trajectory[i - 1].vx +
        trajectory[i - 1].vy * trajectory[i - 1].vy +
        trajectory[i - 1].vz * trajectory[i - 1].vz);
    const double vi1 = std::sqrt(
        trajectory[i].vx * trajectory[i].vx +
        trajectory[i].vy * trajectory[i].vy +
        trajectory[i].vz * trajectory[i].vz);
    integrated_distance += (vi + vi1) / 2.0 * dt;
  }
  EXPECT_NEAR(integrated_distance, 10.0, 0.01);
}

TEST(KinematicPointToPointLocalPlannerPlanRotation, SameHeading_ReturnsSinglePoint)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 1.0);
  const auto trajectory = planner.plan_rotation(initial, 1.0);
  ASSERT_EQ(trajectory.size(), 1u);
  EXPECT_DOUBLE_EQ(trajectory.front().t,       0.0);
  EXPECT_DOUBLE_EQ(trajectory.front().heading, 1.0);
}

TEST(KinematicPointToPointLocalPlannerPlanRotation, Position_FixedThroughout)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(3.0, 4.0, 1.5, 0.0);
  const auto trajectory = planner.plan_rotation(initial, 1.5);
  for (const auto & point : trajectory) {
    EXPECT_DOUBLE_EQ(point.x, 3.0);
    EXPECT_DOUBLE_EQ(point.y, 4.0);
    EXPECT_DOUBLE_EQ(point.z, 1.5);
  }
}

TEST(KinematicPointToPointLocalPlannerPlanRotation, LinearVelocity_ZeroThroughout)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_rotation(initial, 1.5);
  for (const auto & point : trajectory) {
    EXPECT_DOUBLE_EQ(point.vx, 0.0);
    EXPECT_DOUBLE_EQ(point.vy, 0.0);
    EXPECT_DOUBLE_EQ(point.vz, 0.0);
  }
}

TEST(KinematicPointToPointLocalPlannerPlanRotation, Timestamps_MonotonicallyIncreasing)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_rotation(initial, 1.5);
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    EXPECT_GT(trajectory[i].t, trajectory[i - 1].t);
  }
}

TEST(KinematicPointToPointLocalPlannerPlanRotation, FirstAndLastHeading_Correct)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.5);
  const auto trajectory = planner.plan_rotation(initial, 2.0);
  ASSERT_FALSE(trajectory.empty());
  EXPECT_NEAR(trajectory.front().heading, 0.5, 1e-9);
  EXPECT_NEAR(trajectory.back().heading,  2.0, 1e-9);
}

TEST(KinematicPointToPointLocalPlannerPlanRotation, OmegaAtEndpoints_IsZero)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_rotation(initial, 1.5);
  ASSERT_FALSE(trajectory.empty());
  EXPECT_NEAR(trajectory.front().omega, 0.0, 1e-9);
  EXPECT_NEAR(trajectory.back().omega,  0.0, 1e-9);
}

TEST(KinematicPointToPointLocalPlannerPlanRotation, PositiveRotation_OmegaIsPositive)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_rotation(initial, 1.5);
  ASSERT_GT(trajectory.size(), 2u);
  EXPECT_GT(trajectory[trajectory.size() / 2].omega, 0.0);
}

TEST(KinematicPointToPointLocalPlannerPlanRotation, NegativeRotation_OmegaIsNegative)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 1.5);
  const auto trajectory = planner.plan_rotation(initial, 0.0);
  ASSERT_GT(trajectory.size(), 2u);
  EXPECT_LT(trajectory[trajectory.size() / 2].omega, 0.0);
}

TEST(KinematicPointToPointLocalPlannerPlanRotation, AngleWraparound_TakesShortestPath)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 3.0);
  const auto trajectory = planner.plan_rotation(initial, -3.0);
  ASSERT_GT(trajectory.size(), 2u);
  EXPECT_GT(trajectory[trajectory.size() / 2].omega, 0.0);
}

TEST(KinematicPointToPointLocalPlannerPlanRotation, MaxAngularVelocity_NotExceeded)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial = make_state(0.0, 0.0, 0.0, 0.0);
  const auto trajectory = planner.plan_rotation(initial, 1.5);
  for (const auto & point : trajectory) {
    EXPECT_LE(std::abs(point.omega), kMaxAngularVelocity + 1e-9);
  }
}

TEST(KinematicPointToPointLocalPlannerPlanRotation, OmegaCoherentWithHeading)
{
  KinematicPointToPointLocalPlanner planner = make_planner();
  const Kinematics initial       = make_state(0.0, 0.0, 0.0, 0.0);
  const double         target_heading = 1.5;
  const auto trajectory = planner.plan_rotation(initial, target_heading);
  double integrated_angle = 0.0;
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    const double dt = trajectory[i].t - trajectory[i - 1].t;
    integrated_angle += (trajectory[i - 1].omega + trajectory[i].omega) / 2.0 * dt;
  }
  EXPECT_NEAR(integrated_angle, target_heading, 0.01);
}
