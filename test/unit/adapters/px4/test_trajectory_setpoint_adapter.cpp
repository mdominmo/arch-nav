#include <gtest/gtest.h>

#include "adapters/px4/trajectory_setpoint_adapter.hpp"
#include "core/model/vehicle/trajectory_point.hpp"

using arch_nav::adapters::px4::TrajectorySetpointAdapter;
using arch_nav::vehicle::TrajectoryPoint;

TEST(TrajectorySetpointAdapter, MapFromTrajectoryPoint) {
  TrajectoryPoint origin{};
  origin.x       = 1.0;
  origin.y       = 2.0;
  origin.z       = -3.0;
  origin.vx      = 0.1;
  origin.vy      = 0.2;
  origin.vz      = 0.3;
  origin.ax      = 0.01;
  origin.ay      = 0.02;
  origin.az      = 0.03;
  origin.heading = 1.57;
  origin.omega   = 0.5;

  const auto msg = TrajectorySetpointAdapter::map_from(origin);

  EXPECT_FLOAT_EQ(msg.position[0], static_cast<float>(origin.x));
  EXPECT_FLOAT_EQ(msg.position[1], static_cast<float>(origin.y));
  EXPECT_FLOAT_EQ(msg.position[2], static_cast<float>(origin.z));

  EXPECT_FLOAT_EQ(msg.velocity[0], static_cast<float>(origin.vx));
  EXPECT_FLOAT_EQ(msg.velocity[1], static_cast<float>(origin.vy));
  EXPECT_FLOAT_EQ(msg.velocity[2], static_cast<float>(origin.vz));

  EXPECT_FLOAT_EQ(msg.acceleration[0], static_cast<float>(origin.ax));
  EXPECT_FLOAT_EQ(msg.acceleration[1], static_cast<float>(origin.ay));
  EXPECT_FLOAT_EQ(msg.acceleration[2], static_cast<float>(origin.az));

  EXPECT_FLOAT_EQ(msg.yaw,      static_cast<float>(origin.heading));
  EXPECT_FLOAT_EQ(msg.yawspeed, static_cast<float>(origin.omega));
}
