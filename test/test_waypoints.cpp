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

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "fastbot_waypoints/action/waypoint_action.hpp"

using WaypointAction = fastbot_waypoints::action::WaypointAction;

class WaypointTest : public ::testing::Test
{
protected:
  static constexpr double GOAL_X = 0.5;
  static constexpr double GOAL_Y = 0.5;
  static constexpr double DIST_PRECISION = 0.1;
  static constexpr double YAW_PRECISION = M_PI / 10.0;

  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = rclcpp::Node::make_shared("waypoint_test_node");

    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "/fastbot/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        pos_x_ = msg->pose.pose.position.x;
        pos_y_ = msg->pose.pose.position.y;
        const auto & q = msg->pose.pose.orientation;
        yaw_ = std::atan2(
          2.0 * (q.w * q.z + q.x * q.y),
          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        odom_received_ = true;
      });

    // Wait for first odom message
    while (!odom_received_) {
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    initial_x_ = pos_x_;
    initial_y_ = pos_y_;

    action_client_ = rclcpp_action::create_client<WaypointAction>(node_, "fastbot_as");
    ASSERT_TRUE(action_client_->wait_for_action_server(std::chrono::seconds(10))) <<
      "Action server not available";

    // Send goal and wait for completion
    auto goal_msg = WaypointAction::Goal();
    goal_msg.position.x = GOAL_X;
    goal_msg.position.y = GOAL_Y;
    goal_msg.position.z = 0.0;

    auto future_goal = action_client_->async_send_goal(goal_msg);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node_, future_goal, std::chrono::seconds(10)),
      rclcpp::FutureReturnCode::SUCCESS) << "Goal not accepted";

    auto goal_handle = future_goal.get();
    ASSERT_NE(goal_handle, nullptr) << "Goal was rejected";

    auto future_result = action_client_->async_get_result(goal_handle);
    ASSERT_EQ(
      rclcpp::spin_until_future_complete(node_, future_result, std::chrono::seconds(60)),
      rclcpp::FutureReturnCode::SUCCESS) << "Timed out waiting for result";

    goal_succeeded_ = future_result.get().result->success;

    // Spin briefly to get final odom
    rclcpp::spin_some(node_);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<WaypointAction>::SharedPtr action_client_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  double pos_x_{0.0};
  double pos_y_{0.0};
  double yaw_{0.0};
  double initial_x_{0.0};
  double initial_y_{0.0};
  bool odom_received_{false};
  bool goal_succeeded_{false};
};

TEST_F(WaypointTest, test_goal_position_reached)
{
  ASSERT_TRUE(goal_succeeded_) << "Action server reported failure";
  double dist = std::hypot(pos_x_ - GOAL_X, pos_y_ - GOAL_Y);
  EXPECT_LT(dist, DIST_PRECISION) <<
    "Position error " << dist << " m exceeds threshold " << DIST_PRECISION << " m";
}

TEST_F(WaypointTest, test_goal_yaw_reached)
{
  ASSERT_TRUE(goal_succeeded_) << "Action server reported failure";
  double desired_yaw = std::atan2(GOAL_Y - initial_y_, GOAL_X - initial_x_);
  double err_yaw = std::fabs(yaw_ - desired_yaw);
  if (err_yaw > M_PI) {
    err_yaw = 2.0 * M_PI - err_yaw;
  }
  EXPECT_LT(err_yaw, YAW_PRECISION) <<
    "Yaw error " << err_yaw << " rad exceeds threshold " << YAW_PRECISION << " rad";
}

// int main(int argc, char ** argv)
// {
//   testing::InitGoogleTest(&argc, argv);
//   return RUN_ALL_TESTS();
// }