#include <cmath>
#include <limits>
#include <gtest/gtest.h>

#include "core/context/vehicle_context.hpp"

using arch_nav::constants::VehicleStatusStates;
using arch_nav::context::VehicleContext;
using arch_nav::vehicle::Kinematics;
using arch_nav::vehicle::GlobalPosition;
using arch_nav::vehicle::VehicleStatus;

TEST(VehicleContext, DefaultsAreInvalid) {
  VehicleContext manager;
  EXPECT_FALSE(manager.get_global_position().is_valid());
  EXPECT_FALSE(manager.get_kinematic().is_valid());
  EXPECT_FALSE(manager.get_vehicle_status().is_valid());
}

TEST(VehicleContext, UpdateGlobalPositionSkipsNaN) {
  VehicleContext manager;
  GlobalPosition state;
  state.lat = 40.0;
  manager.update(state);
  auto gps = manager.get_global_position();
  EXPECT_DOUBLE_EQ(gps.lat, state.lat);
  EXPECT_TRUE(std::isnan(gps.lon));
  EXPECT_TRUE(std::isnan(gps.alt));
  EXPECT_FALSE(gps.is_valid());
}

TEST(VehicleContext, UpdateKinematicSkipsNaN) {
  VehicleContext manager;
  Kinematics state;
  state.vx = 1.5;
  state.heading = 0.7;
  manager.update(state);
  auto kin = manager.get_kinematic();
  EXPECT_DOUBLE_EQ(kin.vx, 1.5);
  EXPECT_DOUBLE_EQ(kin.heading, 0.7);
  EXPECT_TRUE(std::isnan(kin.x));
}

TEST(VehicleContext, UpdateVehicleStatus) {
  VehicleContext manager;
  VehicleStatus state(VehicleStatusStates::STATE_HOLD,
                           VehicleStatusStates::STATE_ARMED);
  manager.update(state);
  auto vs = manager.get_vehicle_status();
  EXPECT_EQ(vs.nav_state, VehicleStatusStates::STATE_HOLD);
  EXPECT_EQ(vs.arm_state, VehicleStatusStates::STATE_ARMED);

  VehicleStatus state2(VehicleStatusStates::STATE_UNKNOWN,
                            VehicleStatusStates::STATE_UNKNOWN);
  manager.update(state2);
  vs = manager.get_vehicle_status();
  EXPECT_EQ(vs.nav_state, VehicleStatusStates::STATE_UNKNOWN);
  EXPECT_EQ(vs.arm_state, VehicleStatusStates::STATE_UNKNOWN);
}
