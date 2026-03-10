#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "core/constants/vehicle_status_states.hpp"
#include "core/context/vehicle_context.hpp"
#include "listeners/px4/vehicle_status_listener.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "rclcpp/rclcpp.hpp"

using arch_nav::constants::VehicleStatusStates;
using arch_nav::context::VehicleContext;
using arch_nav::listeners::px4::VehicleStatusListener;

class RclcppGuard {
 public:
  RclcppGuard() {
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

TEST(VehicleStatusListenerIntegration, UpdatesManagerFromTopic) {
  RclcppGuard rclcpp_guard;

  std::shared_ptr<rclcpp::Node> node;
  try {
    node = std::make_shared<rclcpp::Node>("vehicle_status_listener_test_node");
  } catch (const std::exception& exception) {
    GTEST_SKIP() << "Skipping integration test: ROS middleware unavailable ("
                 << exception.what() << ")";
  }

  VehicleContext manager;

  const std::string topic_name = "/test/fmu/out/vehicle_status";
  VehicleStatusListener listener(*node, manager, topic_name);

  auto publisher = node->create_publisher<px4_msgs::msg::VehicleStatus>(
      topic_name, rclcpp::QoS(10).best_effort());

  px4_msgs::msg::VehicleStatus origin{};
  origin.nav_state = px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  origin.arming_state = px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;

  bool received = false;
  for (int i = 0; i < 50 && !received; ++i) {
    publisher->publish(origin);
    rclcpp::spin_some(node);

    const auto state = manager.get_vehicle_status();
    received = state.nav_state != VehicleStatusStates::STATE_UNKNOWN &&
               state.arm_state != VehicleStatusStates::STATE_UNKNOWN;
    if (!received) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  const auto state = manager.get_vehicle_status();
  EXPECT_TRUE(received);
  EXPECT_EQ(state.nav_state, VehicleStatusStates::STATE_OFFBOARD);
  EXPECT_EQ(state.arm_state, VehicleStatusStates::STATE_ARMED);
}
