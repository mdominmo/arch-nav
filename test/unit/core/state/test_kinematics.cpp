#include <cmath>
#include <gtest/gtest.h>

#include "core/model/vehicle/kinematics.hpp"

using arch_nav::vehicle::Kinematics;

TEST(Kinematics, InitDefaults) {
  Kinematics state;
  EXPECT_TRUE(std::isnan(state.x));
  EXPECT_TRUE(std::isnan(state.y));
  EXPECT_TRUE(std::isnan(state.z));
  EXPECT_TRUE(std::isnan(state.vx));
  EXPECT_TRUE(std::isnan(state.vy));
  EXPECT_TRUE(std::isnan(state.vz));
  EXPECT_TRUE(std::isnan(state.ax));
  EXPECT_TRUE(std::isnan(state.ay));
  EXPECT_TRUE(std::isnan(state.az));
  EXPECT_TRUE(std::isnan(state.ref_lat));
  EXPECT_TRUE(std::isnan(state.ref_lon));
  EXPECT_TRUE(std::isnan(state.ref_alt));
  EXPECT_TRUE(std::isnan(state.heading));
  EXPECT_FALSE(state.is_valid());
}
