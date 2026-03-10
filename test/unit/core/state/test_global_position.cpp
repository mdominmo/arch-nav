#include <cmath>
#include <gtest/gtest.h>

#include "core/model/vehicle/global_position.hpp"

using arch_nav::vehicle::GlobalPosition;

TEST(GlobalPosition, InitDefaults) {
  GlobalPosition state;
  EXPECT_TRUE(std::isnan(state.lat));
  EXPECT_TRUE(std::isnan(state.lon));
  EXPECT_TRUE(std::isnan(state.alt));
  EXPECT_FALSE(state.is_valid());
}
