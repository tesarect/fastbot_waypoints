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
#include "fastbot_waypoints/action/waypoint_action.hpp"

class WaypointActionServer : public rclcpp::Node
{
public:
  using WaypointAction = fastbot_waypoints::action::WaypointAction;
  using GoalHandle = rclcpp_action::ServerGoalHandle<WaypointAction>;

  explicit WaypointActionServer()
  : Node("fastbot_as")
  {
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&WaypointActionServer::odom_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<WaypointAction>(
      this, "fastbot_as",
      std::bind(&WaypointActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&WaypointActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&WaypointActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Fastbot action server started");
  }

private:
  rclcpp_action::Server<WaypointAction>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  double pos_x_{0.0};
  double pos_y_{0.0};
  double yaw_{0.0};

  static constexpr double YAW_PRECISION = M_PI / 90.0;
  static constexpr double DIST_PRECISION = 0.05;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pos_x_ = msg->pose.pose.position.x;
    pos_y_ = msg->pose.pose.position.y;
    const auto & q = msg->pose.pose.orientation;
    yaw_ = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const WaypointAction::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal: x=%.2f y=%.2f",
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
      goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    auto feedback = std::make_shared<WaypointAction::Feedback>();
    auto result = std::make_shared<WaypointAction::Result>();
    const auto & goal = goal_handle->get_goal();
    const double des_x = goal->position.x;
    const double des_y = goal->position.y;

    rclcpp::Rate rate(25);
    double err_pos = std::hypot(des_y - pos_y_, des_x - pos_x_);

    while (err_pos > DIST_PRECISION && rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        result->success = false;
        goal_handle->canceled(result);
        stop_robot();
        return;
      }

      const double desired_yaw = std::atan2(des_y - pos_y_, des_x - pos_x_);
      const double err_yaw = desired_yaw - yaw_;
      err_pos = std::hypot(des_y - pos_y_, des_x - pos_x_);

      geometry_msgs::msg::Twist twist;
      std::string state;

      if (std::fabs(err_yaw) > YAW_PRECISION) {
        state = "fix yaw";
        twist.angular.z = err_yaw > 0 ? 0.65 : -0.65;
      } else {
        state = "go to point";
        twist.linear.x = 0.6;
      }

      cmd_vel_pub_->publish(twist);

      feedback->position.x = pos_x_;
      feedback->position.y = pos_y_;
      feedback->state = state;
      goal_handle->publish_feedback(feedback);

      rate.sleep();
    }

    stop_robot();
    result->success = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  }

  void stop_robot()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  }
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