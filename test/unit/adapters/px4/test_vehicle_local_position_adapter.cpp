#include <cmath>
#include <gtest/gtest.h>

#include "adapters/px4/vehicle_local_position_adapter.hpp"

using arch_nav::adapters::px4::VehicleLocalPositionAdapter;

TEST(VehicleLocalPositionAdapter, MapFromVehicleLocalPosition) {
  px4_msgs::msg::VehicleLocalPosition origin{};
  origin.x = 1.0F;
  origin.y = 2.0F;
  origin.z = -3.0F;
  origin.vx = 0.1F;
  origin.vy = 0.2F;
  origin.vz = 0.3F;
  origin.ax = 0.01F;
  origin.ay = 0.02F;
  origin.az = 0.03F;
  origin.ref_lat = 40.4168;
  origin.ref_lon = -3.7038;
  origin.ref_alt = 650.5F;
  origin.heading = 1.57F;

  const auto state = VehicleLocalPositionAdapter::map_from(origin);

  EXPECT_DOUBLE_EQ(state.x, static_cast<double>(origin.x));
  EXPECT_DOUBLE_EQ(state.y, static_cast<double>(origin.y));
  EXPECT_DOUBLE_EQ(state.z, static_cast<double>(origin.z));
  EXPECT_DOUBLE_EQ(state.vx, static_cast<double>(origin.vx));
  EXPECT_DOUBLE_EQ(state.vy, static_cast<double>(origin.vy));
  EXPECT_DOUBLE_EQ(state.vz, static_cast<double>(origin.vz));
  EXPECT_DOUBLE_EQ(state.ax, static_cast<double>(origin.ax));
  EXPECT_DOUBLE_EQ(state.ay, static_cast<double>(origin.ay));
  EXPECT_DOUBLE_EQ(state.az, static_cast<double>(origin.az));
  EXPECT_DOUBLE_EQ(state.ref_lat, static_cast<double>(origin.ref_lat));
  EXPECT_DOUBLE_EQ(state.ref_lon, static_cast<double>(origin.ref_lon));
  EXPECT_DOUBLE_EQ(state.ref_alt, static_cast<double>(origin.ref_alt));
  EXPECT_DOUBLE_EQ(state.heading, static_cast<double>(origin.heading));
}
