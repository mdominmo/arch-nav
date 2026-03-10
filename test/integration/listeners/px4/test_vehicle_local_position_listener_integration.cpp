#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "core/context/vehicle_context.hpp"
#include "listeners/px4/vehicle_local_position_listener.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "rclcpp/rclcpp.hpp"

using arch_nav::context::VehicleContext;
using arch_nav::listeners::px4::VehicleLocalPositionListener;

class RclcppGuard {
 public:
  RclcppGuard() {
    std::filesystem::create_directories("/tmp/arch_nav_ros_home");
    std::filesystem::create_directories("/tmp/arch_nav_ros_logs");
    setenv("ROS_HOME", "/tmp/arch_nav_ros_home", 1);
    setenv("ROS_LOG_DIR", "/tmp/arch_nav_ros_logs", 1);

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      owns_init_ = true;
    }
  }

  ~RclcppGuard() {
    if (owns_init_ && rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

 private:
  bool owns_init_ = false;
};

TEST(VehicleLocalPositionListenerIntegration, UpdatesManagerFromTopic) {
  RclcppGuard rclcpp_guard;

  std::shared_ptr<rclcpp::Node> node;
  try {
    node = std::make_shared<rclcpp::Node>("vehicle_local_position_listener_test_node");
  } catch (const std::exception& exception) {
    GTEST_SKIP() << "Skipping integration test: ROS middleware unavailable ("
                 << exception.what() << ")";
  }

  VehicleContext manager;

  const std::string topic_name = "/test/fmu/out/vehicle_local_position";
  VehicleLocalPositionListener listener(*node, manager, topic_name);

  auto publisher =
      node->create_publisher<px4_msgs::msg::VehicleLocalPosition>(
          topic_name, rclcpp::QoS(10).best_effort());

  px4_msgs::msg::VehicleLocalPosition origin{};
  origin.x = 10.0F;
  origin.y = 20.0F;
  origin.z = -30.0F;
  origin.vx = 1.0F;
  origin.vy = 2.0F;
  origin.vz = 3.0F;
  origin.ax = 0.1F;
  origin.ay = 0.2F;
  origin.az = 0.3F;
  origin.ref_lat = 40.0;
  origin.ref_lon = -3.0;
  origin.ref_alt = 650.0F;
  origin.heading = 1.57F;

  bool received = false;
  for (int i = 0; i < 50 && !received; ++i) {
    publisher->publish(origin);
    rclcpp::spin_some(node);

    const auto state = manager.get_kinematic();
    received = !std::isnan(state.x);
    if (!received) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  const auto state = manager.get_kinematic();
  EXPECT_TRUE(received);
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
