#include <gtest/gtest.h>

#include "core/model/vehicle/vehicle_status.hpp"

using arch_nav::constants::VehicleStatusStates;
using arch_nav::vehicle::VehicleStatus;

TEST(VehicleStatus, InitDefaults) {
  VehicleStatus state;
  EXPECT_EQ(state.nav_state, VehicleStatusStates::STATE_UNKNOWN);
  EXPECT_EQ(state.arm_state, VehicleStatusStates::STATE_UNKNOWN);
  EXPECT_FALSE(state.is_valid());
}

TEST(VehicleStatus, InitWithStates) {
  VehicleStatus state(VehicleStatusStates::STATE_HOLD,
                           VehicleStatusStates::STATE_ARMED);
  EXPECT_EQ(state.nav_state, VehicleStatusStates::STATE_HOLD);
  EXPECT_EQ(state.arm_state, VehicleStatusStates::STATE_ARMED);
  EXPECT_TRUE(state.is_valid());
}

TEST(VehicleStatus, InitWithUnknownArmState) {
  VehicleStatus state(VehicleStatusStates::STATE_HOLD,
                           VehicleStatusStates::STATE_UNKNOWN);
  EXPECT_EQ(state.nav_state, VehicleStatusStates::STATE_HOLD);
  EXPECT_EQ(state.arm_state, VehicleStatusStates::STATE_UNKNOWN);
  EXPECT_FALSE(state.is_valid());
}
