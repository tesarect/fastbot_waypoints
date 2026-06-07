// Copyright 2024 rosman
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "fastbot_waypoints/action/waypoint.hpp"

class WaypointActionServer : public rclcpp::Node
{
public:
  using Waypoint = fastbot_waypoints::action::Waypoint;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Waypoint>;

  explicit WaypointActionServer()
  : Node("fastbot_as")
  {
    // Declare tunable parameters so values can be changed without rebuilding
    declare_parameter("max_linear_velocity", 0.5);
    declare_parameter("max_angular_velocity", 0.65);
    declare_parameter("kp_linear", 0.5);
    declare_parameter("kp_angular", 2.0);
    declare_parameter("yaw_precision", 0.05);
    declare_parameter("dist_precision", 0.05);

    max_linear_vel_ = get_parameter("max_linear_velocity").as_double();
    max_angular_vel_ = get_parameter("max_angular_velocity").as_double();
    kp_linear_ = get_parameter("kp_linear").as_double();
    kp_angular_ = get_parameter("kp_angular").as_double();
    yaw_precision_ = get_parameter("yaw_precision").as_double();
    dist_precision_ = get_parameter("dist_precision").as_double();

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/fastbot/cmd_vel", 1);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fastbot/odom", 10,
      std::bind(&WaypointActionServer::odom_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<Waypoint>(
      this, "fastbot_as",
      std::bind(&WaypointActionServer::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&WaypointActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&WaypointActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Fastbot action server started");
  }

private:
  rclcpp_action::Server<Waypoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Current robot pose (updated by odom callback)
  geometry_msgs::msg::Point current_position_;
  double current_yaw_{0.0};

  // Control parameters (set from ROS params)
  double max_linear_vel_;
  double max_angular_vel_;
  double kp_linear_;
  double kp_angular_;
  double yaw_precision_;
  double dist_precision_;

  // Update pose from odometry; uses tf2::getYaw to avoid manual quaternion math
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_position_ = msg->pose.pose.position;
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    current_yaw_ = tf2::getYaw(q);
  }

  // Wrap angle to [-pi, pi] to avoid sign-flip jumps near +/-180 deg
  double normalize_angle(double a)
  {
    return std::atan2(std::sin(a), std::cos(a));
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const Waypoint::Goal> goal)
  {
    RCLCPP_INFO(
      get_logger(), "Received goal: x=%.2f y=%.2f",
      goal->position.x, goal->position.y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "Goal cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::thread{
      std::bind(&WaypointActionServer::execute, this, std::placeholders::_1),
      goal_handle}
    .detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto feedback = std::make_shared<Waypoint::Feedback>();
    auto result = std::make_shared<Waypoint::Result>();
    const auto & goal = goal_handle->get_goal();
    const double des_x = goal->position.x;
    const double des_y = goal->position.y;

    rclcpp::Rate rate(25);
    bool reached_goal = false;
    std::string state = "idle";

    while (rclcpp::ok() && !reached_goal) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        stop_robot();
        return;
      }

      const double dx = des_x - current_position_.x;
      const double dy = des_y - current_position_.y;
      const double dist_error = std::hypot(dx, dy);
      const double desired_yaw = std::atan2(dy, dx);
      const double yaw_error = normalize_angle(desired_yaw - current_yaw_);

      RCLCPP_INFO(get_logger(), "Distance error: %.2f, Yaw error: %.2f",
        dist_error, yaw_error);

      geometry_msgs::msg::Twist twist;

      if (std::fabs(yaw_error) > yaw_precision_) {
        // Phase 1: rotate in place until facing the goal
        RCLCPP_INFO(this->get_logger(), " 🌀 Rotating in place");
        state = "fix yaw";
        twist.angular.z = std::clamp(kp_angular_ * yaw_error,
          -max_angular_vel_, max_angular_vel_);
      } else if (dist_error > dist_precision_) {
        // Phase 2: drive straight — no angular correction to avoid arcing
        RCLCPP_INFO(this->get_logger(), " 🔼 Moving forward");
        state = "go to point";
        twist.linear.x = std::clamp(kp_linear_ * dist_error, 0.1, max_linear_vel_);
      } else {
        // Goal reached
        RCLCPP_INFO(this->get_logger(), " 👍 Goal reached");
        state = "goal reached";
        reached_goal = true;
      }

      cmd_vel_pub_->publish(twist);

      feedback->position = current_position_;
      feedback->state = state;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    stop_robot();
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  }

  void stop_robot() {cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaypointActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}