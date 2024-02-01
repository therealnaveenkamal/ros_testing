#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tortoisebot_waypoints/action/waypoint_action.hpp"



using std::placeholders::_1;
using std::placeholders::_2;

class WaypointActionClass : public rclcpp::Node {
public:
  using WaypointAction = tortoisebot_waypoints::action::WaypointAction;
  using GoalHandleWaypointAction = rclcpp_action::ServerGoalHandle<WaypointAction>;

  WaypointActionClass(const rclcpp::NodeOptions &options = rclcpp::NodeOptions()): Node("tortoisebot_as", options) {

    action_server = rclcpp_action::create_server<WaypointAction>(
        this, "tortoisebot_as",
        std::bind(&WaypointActionClass::handle_goal, this, _1, _2),
        std::bind(&WaypointActionClass::handle_cancel, this, _1),
        std::bind(&WaypointActionClass::handle_accepted, this, _1));
        pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        sub = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&WaypointActionClass::odom_callback, this, _1));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    curr = msg->pose.pose.position;
    tf2::Quaternion temp_yaw;
    tf2::convert(msg->pose.pose.orientation, temp_yaw);
    __yaw = tf2::getYaw(temp_yaw);
  }

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const WaypointAction::Goal> goal) {
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {
    std::thread{std::bind(&WaypointActionClass::execute, this, _1), goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleWaypointAction> goal_handle) {

  const auto d_precision = 0.1;
  const auto y_precision = M_PI / 90;

    const auto goal = goal_handle->get_goal();

    auto feedback = std::make_shared<WaypointAction::Feedback>();
    auto result = std::make_shared<WaypointAction::Result>();

    rclcpp::Rate loop_rate(10);

    auto dPos = goal->position;

    bool is_active = true;

    while (is_active) {
      auto dYaw = atan2(dPos.y - curr.y, dPos.x - curr.x);
      auto ePos = sqrt(pow(dPos.y - curr.y, 2) + pow(dPos.x - curr.x, 2));
      auto eYaw = dYaw - __yaw;

      if (goal_handle->is_canceling()) {
        result->success = false;
              feedback->position = curr;

        feedback->state = "CANCELLED";
        goal_handle->publish_feedback(feedback);
        goal_handle->canceled(result);
        is_active = false;
        rclcpp::shutdown();
      }

      if (abs(eYaw) > y_precision) {
        cmd_vel_msg.angular.z = (eYaw > 0) ? 0.5 : -0.5;
        pub->publish(cmd_vel_msg);
      } else {
        cmd_vel_msg.linear.x = 0.6;
        cmd_vel_msg.angular.z = 0.0;
        pub->publish(cmd_vel_msg);
      }

      feedback->position = curr;
      feedback->state = "IN PROGRESS";
      goal_handle->publish_feedback(feedback);

      if (ePos < d_precision) {
        is_active = false;
        cmd_vel_msg.angular.z = 0.0;
        cmd_vel_msg.linear.x = 0.0;
        pub->publish(cmd_vel_msg);
        feedback->state = "SUCCESS";
        goal_handle->publish_feedback(feedback);
        result->success = true;
        goal_handle->succeed(result);
      }

      loop_rate.sleep();
    }
  }

private:
  float __yaw;
  rclcpp_action::Server<WaypointAction>::SharedPtr action_server;
  geometry_msgs::msg::Twist cmd_vel_msg;
  geometry_msgs::msg::Point curr;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto server = std::make_shared<WaypointActionClass>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(server);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}