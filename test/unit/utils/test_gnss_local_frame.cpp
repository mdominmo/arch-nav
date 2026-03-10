#include <cmath>

#include <gtest/gtest.h>

#include "utils/gnss_local_frame.hpp"

using namespace arch_nav::utils::gnss_local_frame;

TEST(GnssLocalFrame, LatToNedX_ZeroAtReference) {
  EXPECT_DOUBLE_EQ(lat_to_ned_x(40.0, 40.0), 0.0);
}

TEST(GnssLocalFrame, LonToNedY_ZeroAtReference) {
  EXPECT_DOUBLE_EQ(lon_to_ned_y(-3.0, -3.0, 40.0), 0.0);
}

TEST(GnssLocalFrame, AltToNedZ_ZeroAtReference) {
  EXPECT_DOUBLE_EQ(alt_to_ned_z(650.0, 650.0), 0.0);
}

TEST(GnssLocalFrame, LatToNedX_NorthIsPositive) {
  EXPECT_GT(lat_to_ned_x(40.001, 40.0), 0.0);
}

TEST(GnssLocalFrame, LonToNedY_EastIsPositive) {
  EXPECT_GT(lon_to_ned_y(-2.999, -3.0, 40.0), 0.0);
}

TEST(GnssLocalFrame, AltToNedZ_AboveIsNegative) {
  EXPECT_LT(alt_to_ned_z(660.0, 650.0), 0.0);
}

TEST(GnssLocalFrame, LatToNedX_ApproximateDistance) {
  const double x = lat_to_ned_x(40.001, 40.0);
  EXPECT_NEAR(x, 111.2, 1.0);
}
