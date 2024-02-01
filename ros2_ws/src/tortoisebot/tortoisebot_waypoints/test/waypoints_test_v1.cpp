#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"
#include "gtest/gtest.h"
#include <memory>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class GTEST_Class {
public:
  GTEST_Class() { rclcpp::init(0, nullptr); }
};

class WaypointTest : public ::testing::Test {
public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandle = rclcpp_action::ClientGoalHandle<WaypointAction>;

  WaypointTest() {

    test_node = rclcpp::Node::make_shared("test_node");
    odom_subscriber = test_node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&WaypointTest::odom_callback, this, _1));

    action_client = rclcpp_action::create_client<WaypointAction>(
        test_node, "tortoisebot_as");

    this->goal.x = 0.5;
    this->goal.y = 0.5;

    float goal_yaw = atan2(goal.y - curr.y, goal.x - curr.x);

    if (!action_client->wait_for_action_server(10s)) {
      RCLCPP_ERROR(test_node->get_logger(), "Action server not available");
    }

    auto goal_msg = WaypointAction::Goal();

    goal_msg.position = this->goal;
    auto goal_options =
        rclcpp_action::Client<WaypointAction>::SendGoalOptions();
    goal_options.goal_response_callback =
        std::bind(&WaypointTest::goal_response_callback, this, _1);
    goal_options.feedback_callback =
        std::bind(&WaypointTest::feedback_callback, this, _1, _2);
    goal_options.result_callback =
        std::bind(&WaypointTest::result_callback, this, _1);
    auto future = action_client->async_send_goal(goal_msg, goal_options);
  }

  bool PositionTest() {
    while (is_active) {
      rclcpp::spin_some(test_node);
    }

    float x_error = abs(goal.x - curr.x);
    float y_error = abs(goal.y - curr.y);

    if (x_error <= 0.4 && y_error <= 0.4) {
      return true;
    }

    return false;
  }

  bool AngleTest() {
    while (is_active) {
      rclcpp::spin_some(test_node);
    }

    float yaw_error = abs(goal_yaw - curr_yaw);

    if (yaw_error <= 7.0) {
      return true;
    }

    return false;
  }

private:
  std::shared_ptr<rclcpp::Node> test_node;

  void
  goal_response_callback(std::shared_future<GoalHandle::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(test_node->get_logger(), "Goal rejected!");
    } else {
      RCLCPP_INFO(test_node->get_logger(), "Goal accepted!");
    }
  }
  void feedback_callback(
      GoalHandle::SharedPtr,
      const std::shared_ptr<const WaypointAction::Feedback> feedback) {}

  
  void result_callback(const GoalHandle::WrappedResult &result) {
    is_active = false;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    curr = msg->pose.pose.position;
    tf2::Quaternion curr_angle;
    tf2::convert(msg->pose.pose.orientation, curr_angle);
    curr_yaw = tf2::getYaw(curr_angle);
  }

  float goal_yaw = 0.0;
  bool is_active = true;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  geometry_msgs::msg::Point goal;
  geometry_msgs::msg::Point curr;
  float curr_yaw;
  rclcpp_action::Client<WaypointAction>::SharedPtr action_client;
  rclcpp::TimerBase::SharedPtr timer;
};

GTEST_Class gtest;

TEST_F(WaypointTest, Position) { EXPECT_TRUE(PositionTest()); }

TEST_F(WaypointTest, Angle) { EXPECT_TRUE(AngleTest()); }

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
