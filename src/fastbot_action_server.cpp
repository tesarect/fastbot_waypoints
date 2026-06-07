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
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "fastbot_waypoints/action/waypoint_action.hpp"

class WaypointActionServer : public rclcpp::Node
{
public:
  using WaypointAction = fastbot_waypoints::action::WaypointAction;
  using GoalHandle = rclcpp_action::ServerGoalHandle<WaypointAction>;

  explicit WaypointActionServer()
  : Node("fastbot_as"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    // Declare tunable parameters so values can be changed without rebuilding
    declare_parameter("max_linear_velocity", 0.5);
    declare_parameter("max_angular_velocity", 0.65);
    declare_parameter("kp_linear", 0.5);
    declare_parameter("kp_angular", 2.0);
    declare_parameter("yaw_precision", 0.20);
    declare_parameter("dist_precision", 0.05);

    max_linear_vel_ = get_parameter("max_linear_velocity").as_double();
    max_angular_vel_ = get_parameter("max_angular_velocity").as_double();
    kp_linear_ = get_parameter("kp_linear").as_double();
    kp_angular_ = get_parameter("kp_angular").as_double();
    yaw_precision_ = get_parameter("yaw_precision").as_double();
    dist_precision_ = get_parameter("dist_precision").as_double();

    // Derive forward-axis offset from the camera TF: camera Z-axis points forward
    forward_offset_ = compute_forward_offset("fastbot_base_link", "fastbot_camera");

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/fastbot/cmd_vel", 1);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fastbot/odom", 10,
      std::bind(&WaypointActionServer::odom_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<WaypointAction>(
      this, "fastbot_as",
      std::bind(&WaypointActionServer::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&WaypointActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&WaypointActionServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Fastbot action server started (forward_offset=%.3f rad)", forward_offset_);
  }

private:
  rclcpp_action::Server<WaypointAction>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Current robot pose (updated by odom callback)
  geometry_msgs::msg::Point current_position_;
  double current_yaw_{0.0};

  // Angle offset between base_link X-axis and robot's physical forward direction,
  // derived from the camera frame at startup
  double forward_offset_{M_PI_2};

  // Control parameters (set from ROS params)
  double max_linear_vel_;
  double max_angular_vel_;
  double kp_linear_;
  double kp_angular_;
  double yaw_precision_;
  double dist_precision_;

  // Look up the transform from base_frame to camera_frame and extract the yaw offset
  // between base_link X-axis and camera's forward (Z) axis projected onto XY plane.
  // Falls back to pi/2 if TF is not available.
  double compute_forward_offset(const std::string & base_frame, const std::string & camera_frame)
  {
    // Wait up to 5 s for the TF tree to be populated
    for (int i = 0; i < 50; ++i) {
      if (tf_buffer_.canTransform(base_frame, camera_frame, tf2::TimePointZero)) {
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    try {
      auto tf = tf_buffer_.lookupTransform(base_frame, camera_frame, tf2::TimePointZero);
      tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      // Camera Z-axis (forward in camera convention) expressed in base_link frame
      tf2::Vector3 cam_fwd = m * tf2::Vector3(0.0, 0.0, 1.0);
      double offset = std::atan2(cam_fwd.y(), cam_fwd.x());
      RCLCPP_INFO(get_logger(), "Camera forward in base_link: (%.3f, %.3f) → offset=%.3f rad",
        cam_fwd.x(), cam_fwd.y(), offset);
      return offset;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(),
        "Could not look up %s→%s: %s. Falling back to pi/2.",
        base_frame.c_str(), camera_frame.c_str(), ex.what());
      return M_PI_2;
    }
  }

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
    std::shared_ptr<const WaypointAction::Goal> goal)
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
    auto feedback = std::make_shared<WaypointAction::Feedback>();
    auto result = std::make_shared<WaypointAction::Result>();
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
      // forward_offset_ aligns base_link X-axis with the robot's actual forward direction
      const double desired_yaw = std::atan2(dy, dx) + forward_offset_;
      const double yaw_error = normalize_angle(desired_yaw - current_yaw_);

      geometry_msgs::msg::Twist twist;

      if (std::fabs(yaw_error) > yaw_precision_) {
        // Phase 1: rotate in place until facing the goal
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                             1000,
                             " Rotating in place | pos=(%.3f, %.3f) | err=(%.3f, %.3f)",
                             current_position_.x, current_position_.y, dist_error, yaw_error);
        state = "fix yaw";
        twist.angular.z = std::clamp(kp_angular_ * yaw_error,
          -max_angular_vel_, max_angular_vel_);
      } else if (dist_error > dist_precision_) {
        // Phase 2: move forward with gentle angular correction to prevent lateral drift
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                             1000,
                             " Moving forward     | pos=(%.3f, %.3f) | err=(%.3f, %.3f)",
                             current_position_.x, current_position_.y, dist_error, yaw_error);
        state = "go to point";
        twist.linear.x = std::clamp(kp_linear_ * dist_error, 0.1, max_linear_vel_);
        twist.angular.z = std::clamp(0.5 * kp_angular_ * yaw_error,
          -max_angular_vel_, max_angular_vel_);
      } else {
        // Goal reached
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                             1000,
                             " Goal reached       | pos=(%.3f, %.3f) | err=(%.3f, %.3f)",
                             current_position_.x, current_position_.y, dist_error, yaw_error);
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