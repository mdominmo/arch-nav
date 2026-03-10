#include <gtest/gtest.h>

#include "adapters/px4/vehicle_global_position_adapter.hpp"

using arch_nav::adapters::px4::VehicleGlobalPositionAdapter;

TEST(VehicleGlobalPositionAdapter, MapFromVehicleGlobalPosition) {
  px4_msgs::msg::VehicleGlobalPosition origin{};
  origin.lat = 40.4168;
  origin.lon = -3.7038;
  origin.alt = 650.5F;

  const auto state = VehicleGlobalPositionAdapter::map_from(origin);

  EXPECT_DOUBLE_EQ(state.lat, origin.lat);
  EXPECT_DOUBLE_EQ(state.lon, origin.lon);
  EXPECT_DOUBLE_EQ(state.alt, static_cast<double>(origin.alt));
}
