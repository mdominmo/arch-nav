#include <gtest/gtest.h>

#include <array>
#include <utility>

#include "adapters/px4/vehicle_status_adapter.hpp"

using arch_nav::constants::VehicleStatusStates;
using arch_nav::adapters::px4::VehicleStatusAdapter;

TEST(VehicleStatusAdapter, MapAllSupportedNavigationStates) {
  const std::array<std::pair<uint8_t, VehicleStatusStates>, 6> nav_cases{{
      {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF,
       VehicleStatusStates::STATE_TAKEOFF},
      {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_LOITER,
       VehicleStatusStates::STATE_HOLD},
      {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_MISSION,
       VehicleStatusStates::STATE_GOING_TO_WAYPOINT},
      {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_POSCTL,
       VehicleStatusStates::STATE_ON_AIR},
      {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_RTL,
       VehicleStatusStates::STATE_RTL},
      {px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD,
       VehicleStatusStates::STATE_OFFBOARD},
  }};

  for (const auto& nav_case : nav_cases) {
    px4_msgs::msg::VehicleStatus origin{};
    origin.nav_state = nav_case.first;
    origin.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED;

    const auto state = VehicleStatusAdapter::map_from(origin);
    EXPECT_EQ(state.nav_state, nav_case.second);
  }
}

TEST(VehicleStatusAdapter, MapAllSupportedArmingStates) {
  const std::array<std::pair<uint8_t, VehicleStatusStates>, 2> arm_cases{{
      {px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED,
       VehicleStatusStates::STATE_DISARMED},
      {px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED,
       VehicleStatusStates::STATE_ARMED},
  }};

  for (const auto& arm_case : arm_cases) {
    px4_msgs::msg::VehicleStatus origin{};
    origin.arming_state = arm_case.first;
    origin.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

    const auto state = VehicleStatusAdapter::map_from(origin);
    EXPECT_EQ(state.arm_state, arm_case.second);
  }
}

TEST(VehicleStatusAdapter, MapUnknownStatesFallback) {
  px4_msgs::msg::VehicleStatus origin{};
  origin.arming_state = 99U;
  origin.nav_state = 99U;

  const auto state = VehicleStatusAdapter::map_from(origin);

  EXPECT_EQ(state.arm_state, VehicleStatusStates::STATE_UNKNOWN);
  EXPECT_EQ(state.nav_state, VehicleStatusStates::STATE_UNKNOWN);
}
