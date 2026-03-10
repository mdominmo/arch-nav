#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include "geographic_msgs/msg/geo_pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "core/constants/operation_status.hpp"
#include "core/constants/vehicle_status_states.hpp"
#include "core/controller/operational_controller.hpp"
#include "core/context/vehicle_context.hpp"
#include "core/planner/kinematic_point_to_point_local_planner.hpp"
#include "core/model/vehicle/kinematics.hpp"
#include "core/model/vehicle/vehicle_status.hpp"
#include "dispatchers/px4/px4_command_dispatcher.hpp"

using arch_nav::constants::OperationStatus;
using arch_nav::constants::VehicleStatusStates;
using arch_nav::controller::OperationalController;
using arch_nav::context::VehicleContext;
using arch_nav::dispatchers::px4::Px4CommandDispatcher;
using arch_nav::planner::KinematicPointToPointLocalPlanner;
using arch_nav::vehicle::Kinematics;
using arch_nav::vehicle::VehicleStatus;

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
    if (owns_init_ && rclcpp::ok()) rclcpp::shutdown();
  }

 private:
  bool owns_init_ = false;
};

namespace {

constexpr double kRefLat  = 40.0;
constexpr double kRefLon  = 0.0;
constexpr double kRefAlt  = 0.0;
constexpr double kDroneAlt = 5.0;
constexpr double kDroneZ  = -5.0;

Kinematics make_kinematics(double heading = 0.0) {
  return Kinematics(
      0.0, 0.0, kDroneZ,
      0.0, 0.0, 0.0,
      0.0, 0.0, 0.0,
      kRefLat, kRefLon, kRefAlt,
      heading);
}

KinematicPointToPointLocalPlanner make_planner() {
  return KinematicPointToPointLocalPlanner(10.0, 20.0, 6.28, 6.28, 0.5, 0.2, 0.05);
}

geographic_msgs::msg::GeoPose make_waypoint(double lat, double lon, double alt) {
  geographic_msgs::msg::GeoPose p;
  p.position.latitude  = lat;
  p.position.longitude = lon;
  p.position.altitude  = alt;
  return p;
}

const double kLat5m = kRefLat + 5.0 / (M_PI / 180.0 * 6371000.0);
const double kLon5m = kRefLon + 5.0 / (M_PI / 180.0 * 6371000.0 * std::cos(kRefLat * M_PI / 180.0));

}  // namespace

class OperationalControllerIntegration : public ::testing::Test {
 protected:
  void SetUp() override {
    try {
      node_ = std::make_shared<rclcpp::Node>(
          "ctrl_integration_test_" + std::to_string(++counter_));
    } catch (const std::exception& e) {
      GTEST_SKIP() << "ROS middleware unavailable (" << e.what() << ")";
    }

    dispatcher_    = std::make_unique<Px4CommandDispatcher>(
        *node_,
        "/test/ctrl/trajectory_setpoint",
        "/test/ctrl/vehicle_command",
        "/test/ctrl/offboard_control_mode");
    planner_       = std::make_unique<KinematicPointToPointLocalPlanner>(make_planner());
    state_manager_ = std::make_unique<VehicleContext>();
    controller_    = std::make_unique<OperationalController>(
        *state_manager_, *planner_, *dispatcher_);

    state_manager_->update(make_kinematics());
    ASSERT_EQ(controller_->operation_status(), OperationStatus::HANDOVER);

    state_manager_->update(
        VehicleStatus(VehicleStatusStates::STATE_OFFBOARD,
                      VehicleStatusStates::STATE_ARMED));
    ASSERT_EQ(controller_->operation_status(), OperationStatus::IDDLE);
  }

  void TearDown() override {
    if (controller_ && controller_->operation_status() == OperationStatus::RUNNING) {
      controller_->stop();
    }
    controller_.reset();
    dispatcher_.reset();
    node_.reset();
  }

  void wait_for_status(OperationStatus expected, int max_iterations = 4000) {
    for (int i = 0;
         i < max_iterations && controller_->operation_status() != expected;
         ++i) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  RclcppGuard guard_;
  std::shared_ptr<rclcpp::Node>                     node_;
  std::unique_ptr<Px4CommandDispatcher>             dispatcher_;
  std::unique_ptr<KinematicPointToPointLocalPlanner> planner_;
  std::unique_ptr<VehicleContext>                   state_manager_;
  std::unique_ptr<OperationalController>            controller_;

  static int counter_;
};

int OperationalControllerIntegration::counter_ = 0;

TEST_F(OperationalControllerIntegration, Takeoff_CompletesAndReturnsToIddle) {
  controller_->takeoff(5.0);
  ASSERT_EQ(controller_->operation_status(), OperationStatus::RUNNING);

  wait_for_status(OperationStatus::IDDLE);

  EXPECT_EQ(controller_->operation_status(), OperationStatus::IDDLE);
}

TEST_F(OperationalControllerIntegration, Land_TransitionsToRunningThenDisarmed) {
  controller_->land();
  ASSERT_EQ(controller_->operation_status(), OperationStatus::RUNNING);

  state_manager_->update(
      VehicleStatus(VehicleStatusStates::STATE_OFFBOARD,
                    VehicleStatusStates::STATE_DISARMED));

  EXPECT_EQ(controller_->operation_status(), OperationStatus::DISARMED);
}

TEST_F(OperationalControllerIntegration,
       WaypointFollowing_HeadingAligned_CompletesAndReturnsToIddle) {
  std::vector<geographic_msgs::msg::GeoPose> wps;
  wps.push_back(make_waypoint(kLat5m, kRefLon, kDroneAlt));

  controller_->waypoint_following(std::move(wps));
  ASSERT_EQ(controller_->operation_status(), OperationStatus::RUNNING);

  wait_for_status(OperationStatus::IDDLE);

  EXPECT_EQ(controller_->operation_status(), OperationStatus::IDDLE);
}

TEST_F(OperationalControllerIntegration,
       WaypointFollowing_HeadingNotAligned_RotatesAndCompletes) {
  std::vector<geographic_msgs::msg::GeoPose> wps;
  wps.push_back(make_waypoint(kRefLat, kLon5m, kDroneAlt));

  controller_->waypoint_following(std::move(wps));
  ASSERT_EQ(controller_->operation_status(), OperationStatus::RUNNING);

  wait_for_status(OperationStatus::IDDLE, 4000);

  EXPECT_EQ(controller_->operation_status(), OperationStatus::IDDLE);
}

TEST_F(OperationalControllerIntegration,
       WaypointFollowing_MultipleWaypoints_CompletesAll) {
  std::vector<geographic_msgs::msg::GeoPose> wps;
  wps.push_back(make_waypoint(kLat5m, kRefLon, kDroneAlt));
  wps.push_back(make_waypoint(kLat5m * 2.0 - kRefLat, kRefLon, kDroneAlt));

  controller_->waypoint_following(std::move(wps));
  ASSERT_EQ(controller_->operation_status(), OperationStatus::RUNNING);

  wait_for_status(OperationStatus::IDDLE, 4000);

  EXPECT_EQ(controller_->operation_status(), OperationStatus::IDDLE);
}

TEST_F(OperationalControllerIntegration, Stop_WhileRunning_TransitionsToIddle) {
  controller_->takeoff(5.0);
  ASSERT_EQ(controller_->operation_status(), OperationStatus::RUNNING);

  controller_->stop();

  EXPECT_EQ(controller_->operation_status(), OperationStatus::IDDLE);
}
