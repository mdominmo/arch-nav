#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "core/context/vehicle_context.hpp"
#include "listeners/px4/vehicle_global_position_listener.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "rclcpp/rclcpp.hpp"

using arch_nav::context::VehicleContext;
using arch_nav::listeners::px4::VehicleGlobalPositionListener;

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

TEST(VehicleGlobalPositionListenerIntegration, UpdatesManagerFromTopic) {
  RclcppGuard rclcpp_guard;

  std::shared_ptr<rclcpp::Node> node;
  try {
    node = std::make_shared<rclcpp::Node>(
        "vehicle_global_position_listener_test_node");
  } catch (const std::exception& exception) {
    GTEST_SKIP() << "Skipping integration test: ROS middleware unavailable ("
                 << exception.what() << ")";
  }

  VehicleContext manager;

  const std::string topic_name = "/test/fmu/out/vehicle_global_position";
  VehicleGlobalPositionListener listener(*node, manager, topic_name);

  auto publisher = node->create_publisher<px4_msgs::msg::VehicleGlobalPosition>(
      topic_name, rclcpp::QoS(10).best_effort());

  px4_msgs::msg::VehicleGlobalPosition origin{};
  origin.lat = 40.4168;
  origin.lon = -3.7038;
  origin.alt = 650.5F;

  bool received = false;
  for (int i = 0; i < 50 && !received; ++i) {
    publisher->publish(origin);
    rclcpp::spin_some(node);

    const auto state = manager.get_global_position();
    received = !std::isnan(state.lat);
    if (!received) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  const auto state = manager.get_global_position();
  EXPECT_TRUE(received);
  EXPECT_DOUBLE_EQ(state.lat, origin.lat);
  EXPECT_DOUBLE_EQ(state.lon, origin.lon);
  EXPECT_DOUBLE_EQ(state.alt, static_cast<double>(origin.alt));
}
