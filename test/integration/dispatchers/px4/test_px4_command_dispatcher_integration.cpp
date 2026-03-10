#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "dispatchers/px4/px4_command_dispatcher.hpp"
#include "core/model/vehicle/trajectory_point.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "rclcpp/rclcpp.hpp"

using arch_nav::dispatchers::px4::Px4CommandDispatcher;
using arch_nav::vehicle::TrajectoryPoint;

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

TEST(Px4CommandDispatcherIntegration, ExecuteTrajectory_PublishesOnTrajectoryTopic) {
  RclcppGuard rclcpp_guard;

  std::shared_ptr<rclcpp::Node> node;
  try {
    node = std::make_shared<rclcpp::Node>("px4_command_dispatcher_trajectory_test_node");
  } catch (const std::exception& exception) {
    GTEST_SKIP() << "Skipping integration test: ROS middleware unavailable ("
                 << exception.what() << ")";
  }

  const std::string trajectory_topic = "/test/fmu/in/trajectory_setpoint";
  Px4CommandDispatcher dispatcher(*node, trajectory_topic);

  std::optional<px4_msgs::msg::TrajectorySetpoint> received_msg;
  auto subscriber = node->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      trajectory_topic,
      rclcpp::QoS(10).best_effort(),
      [&received_msg](const px4_msgs::msg::TrajectorySetpoint& msg) {
        received_msg = msg;
      });

  TrajectoryPoint point{};
  point.x       = 1.0;
  point.y       = 2.0;
  point.z       = -3.0;
  point.vx      = 0.1;
  point.vy      = 0.2;
  point.vz      = 0.3;
  point.ax      = 0.01;
  point.ay      = 0.02;
  point.az      = 0.03;
  point.heading = 1.57;

  bool completed = false;
  dispatcher.execute_trajectory({point}, [&completed]() { completed = true; });

  for (int i = 0; i < 50 && !received_msg; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  dispatcher.stop();

  ASSERT_TRUE(received_msg.has_value());
  EXPECT_GT(received_msg->timestamp, 0u);
  EXPECT_FLOAT_EQ(received_msg->position[0], static_cast<float>(point.x));
  EXPECT_FLOAT_EQ(received_msg->position[1], static_cast<float>(point.y));
  EXPECT_FLOAT_EQ(received_msg->position[2], static_cast<float>(point.z));
  EXPECT_FLOAT_EQ(received_msg->velocity[0], static_cast<float>(point.vx));
  EXPECT_FLOAT_EQ(received_msg->velocity[1], static_cast<float>(point.vy));
  EXPECT_FLOAT_EQ(received_msg->velocity[2], static_cast<float>(point.vz));
  EXPECT_FLOAT_EQ(received_msg->yaw,         static_cast<float>(point.heading));
}

TEST(Px4CommandDispatcherIntegration, ExecuteArm_PublishesVehicleCommandWithParam1_1) {
  RclcppGuard rclcpp_guard;

  std::shared_ptr<rclcpp::Node> node;
  try {
    node = std::make_shared<rclcpp::Node>("px4_command_dispatcher_arm_test_node");
  } catch (const std::exception& exception) {
    GTEST_SKIP() << "Skipping integration test: ROS middleware unavailable ("
                 << exception.what() << ")";
  }

  const std::string command_topic = "/test/fmu/in/vehicle_command_arm";
  Px4CommandDispatcher dispatcher(*node, "/test/fmu/in/trajectory_setpoint_arm", command_topic);

  std::optional<px4_msgs::msg::VehicleCommand> received_msg;
  auto subscriber = node->create_subscription<px4_msgs::msg::VehicleCommand>(
      command_topic,
      rclcpp::QoS(10).best_effort(),
      [&received_msg](const px4_msgs::msg::VehicleCommand& msg) {
        received_msg = msg;
      });

  dispatcher.execute_arm();

  for (int i = 0; i < 50 && !received_msg; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  dispatcher.stop();

  ASSERT_TRUE(received_msg.has_value());
  EXPECT_EQ(received_msg->command,
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM);
  EXPECT_FLOAT_EQ(received_msg->param1, 1.0f);
  EXPECT_TRUE(received_msg->from_external);
  EXPECT_GT(received_msg->timestamp, 0u);
}

TEST(Px4CommandDispatcherIntegration, ExecuteDisarm_PublishesVehicleCommandWithParam1_0) {
  RclcppGuard rclcpp_guard;

  std::shared_ptr<rclcpp::Node> node;
  try {
    node = std::make_shared<rclcpp::Node>("px4_command_dispatcher_disarm_test_node");
  } catch (const std::exception& exception) {
    GTEST_SKIP() << "Skipping integration test: ROS middleware unavailable ("
                 << exception.what() << ")";
  }

  const std::string command_topic = "/test/fmu/in/vehicle_command_disarm";
  Px4CommandDispatcher dispatcher(*node, "/test/fmu/in/trajectory_setpoint_disarm", command_topic);

  std::optional<px4_msgs::msg::VehicleCommand> received_msg;
  auto subscriber = node->create_subscription<px4_msgs::msg::VehicleCommand>(
      command_topic,
      rclcpp::QoS(10).best_effort(),
      [&received_msg](const px4_msgs::msg::VehicleCommand& msg) {
        received_msg = msg;
      });

  dispatcher.execute_disarm();

  for (int i = 0; i < 50 && !received_msg; ++i) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  ASSERT_TRUE(received_msg.has_value());
  EXPECT_EQ(received_msg->command,
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM);
  EXPECT_FLOAT_EQ(received_msg->param1, 0.0f);
  EXPECT_TRUE(received_msg->from_external);
  EXPECT_GT(received_msg->timestamp, 0u);
}
