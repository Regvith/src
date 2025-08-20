#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <cmath>
#include <memory>
#include <thread>

#include "fastbot_interfaces/action/waypoint.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using Waypoint = fastbot_interfaces::action::Waypoint;
using GoalHandleWaypoint = rclcpp_action::ServerGoalHandle<Waypoint>;

class WaypointActionServer : public rclcpp::Node {
public:
  WaypointActionServer() : Node("test_waypoint_node") {
    // Publishers / Subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot/cmd_vel", 10);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/fastbot/odom", 10,
        std::bind(&WaypointActionServer::odom_callback, this,
                  std::placeholders::_1));

    // Action server
    action_server_ = rclcpp_action::create_server<Waypoint>(
        this, "fastbot_as",
        std::bind(&WaypointActionServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&WaypointActionServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&WaypointActionServer::handle_accepted, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Fastbot Action Server Started");
  }

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  geometry_msgs::msg::Point current_pos_;
  double yaw_ = 0.0;

  std::shared_ptr<GoalHandleWaypoint> current_goal_handle_;

  const double yaw_precision_ = M_PI / 90.0; // 2 degrees
  const double dist_precision_ = 0.05;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_ = msg->pose.pose.position;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, yaw_);
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const Waypoint::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received relative goal: dx=%.2f, dy=%.2f",
                goal->position.x, goal->position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleWaypoint> /*goal_handle*/) {
    RCLCPP_INFO(this->get_logger(), "Goal canceled");
    stop_robot();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleWaypoint> goal_handle) {
    // Convert relative goal to absolute target based on current pose
    auto rel_goal = goal_handle->get_goal();
    geometry_msgs::msg::Point abs_target;
    abs_target.x = current_pos_.x + rel_goal->position.x;
    abs_target.y = current_pos_.y + rel_goal->position.y;

    current_goal_handle_ = goal_handle;

    std::thread{
        std::bind(&WaypointActionServer::control_loop, this, abs_target)}
        .detach();
  }

  void control_loop(geometry_msgs::msg::Point target) {
    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result = std::make_shared<Waypoint::Result>();

    bool success = true;
    while (rclcpp::ok() && current_goal_handle_->is_active()) {
      double dx = target.x - current_pos_.x;
      double dy = target.y - current_pos_.y;
      double err_pos = std::sqrt(dx * dx + dy * dy);
      double desired_yaw = std::atan2(dy, dx);
      double err_yaw = desired_yaw - yaw_;

      while (err_yaw > M_PI)
        err_yaw -= 2 * M_PI;
      while (err_yaw < -M_PI)
        err_yaw += 2 * M_PI;

      // Debugging
      RCLCPP_INFO(
          this->get_logger(),
          "Current pos: (%.2f, %.2f), Target: (%.2f, %.2f), err_pos=%.3f",
          current_pos_.x, current_pos_.y, target.x, target.y, err_pos);

      if (err_pos < dist_precision_)
        break;

      geometry_msgs::msg::Twist twist_msg;
      if (std::fabs(err_yaw) > yaw_precision_) {
        twist_msg.angular.z = (err_yaw > 0) ? 0.65 : -0.65;
      } else {
        twist_msg.linear.x = 0.6;
      }

      cmd_vel_pub_->publish(twist_msg);

      // Publish feedback
      feedback->position = current_pos_;
      feedback->state =
          (std::fabs(err_yaw) > yaw_precision_) ? "fix_yaw" : "go_to_point";
      current_goal_handle_->publish_feedback(feedback);
    }

    stop_robot();

    if (success && current_goal_handle_->is_active()) {
      result->success = true;
      current_goal_handle_->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
    }
  }

  void stop_robot() {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear.x = 0.0;
    twist_msg.angular.z = 0.0;
    cmd_vel_pub_->publish(twist_msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
