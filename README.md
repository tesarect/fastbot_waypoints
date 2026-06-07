# fastbot_waypoints

ROS2 (Galactic) C++ action server and GTest integration tests for the FastBot waypoint navigation.

## Prerequisites

Two terminals must be running before executing the tests:

**Terminal 1 — Gazebo simulation:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch fastbot_gazebo one_fastbot_room.launch.py
```

**Terminal 2 — Action server:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run fastbot_waypoints fastbot_action_server
```

> The action server must be running separately before tests start — unlike ROS1 rostest, colcon test does not launch nodes for you.

To manually verify the action server is working:
```bash
source ~/ros2_ws/install/setup.bash
ros2 action send_goal /fastbot_as fastbot_waypoints/action/Waypoint \
  "{position: {x: 3.0, y: 2.1, z: 0.0}}"
```

---

## Build and Run Tests

**Terminal 3 — Build and test:**
```bash
cd ~/ros2_ws
colcon build --packages-select fastbot_waypoints && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

Run a single test by name during development:
```bash
./build/fastbot_waypoints/test_waypoints --gtest_filter="WaypointTest.test_goal_position_reached"
```

---

## Switching Between Passing and Failing Conditions

Open [test/test_waypoints.cpp](test/test_waypoints.cpp) and find the constants `DIST_PRECISION` & `YAW_PRECISION` at the top of the `WaypointTest` fixture:

```cpp
  // Pass Condition
  static constexpr double DIST_PRECISION = 0.1;
  static constexpr double YAW_PRECISION = M_PI / 10.0;

  // Fail Condition
  // static constexpr double DIST_PRECISION = 0.01;
  // static constexpr double YAW_PRECISION =  0.01;
```
By default pass condition will be uncommented

### Passing conditions (default)

Keep the defaults as above. The robot navigates to `(0.5, 0.5)` and the test accepts arrival within 10 cm.

Expected output:
```
[==========] Running 2 tests from 1 test suite.
[----------] 2 tests from WaypointTest
[ RUN      ] WaypointTest.test_goal_position_reached
[       OK ] WaypointTest.test_goal_position_reached
[ RUN      ] WaypointTest.test_goal_yaw_reached
[       OK ] WaypointTest.test_goal_yaw_reached
[  PASSED  ] 2 tests.
```

And `colcon test-result --all` will show:
```
Summary: 2 tests, 0 errors, 0 failures
```

### Failing conditions

Uncommnet the fail condition & comment the pass condition.
```cpp
  // Pass Condition
  // static constexpr double DIST_PRECISION = 0.1;
  // static constexpr double YAW_PRECISION = M_PI / 10.0;

  // Fail Condition
  static constexpr double DIST_PRECISION = 0.01;
  static constexpr double YAW_PRECISION =  0.01;
```

Or, Change `DIST_PRECISION` and `YAW_PRECISION` to an impossibly tight value, for example

```cpp
static constexpr double DIST_PRECISION = 0.001;   // 1 mm — robot cannot achieve this
```

The action server considers arrival within 10 cm a success (its own internal precision), so `goal_succeeded_` will be `true`. However the test's own `EXPECT_LT(dist, 0.001)` assertion then fails because the robot stopped ~10 cm away.

Rebuild and run:
```bash
cd ~/ros2_ws
colcon build --packages-select fastbot_waypoints && source install/setup.bash
colcon test --packages-select fastbot_waypoints --event-handler=console_direct+
colcon test-result --all
```

Expected output:
```
[ RUN      ] WaypointTest.test_goal_position_reached
/home/user/ros2_ws/src/fastbot_waypoints/test/test_waypoints.cpp:63: Failure
Value of: action_client_->wait_for_action_server(std::chrono::seconds(10))
  Actual: false
Expected: true
Action server not available
[  FAILED  ] WaypointTest.test_goal_position_reached (10326 ms)
[ RUN      ] WaypointTest.test_goal_yaw_reached
/home/user/ros2_ws/src/fastbot_waypoints/test/test_waypoints.cpp:63: Failure
Value of: action_client_->wait_for_action_server(std::chrono::seconds(10))
  Actual: false
Expected: true
Action server not available
[  FAILED  ] WaypointTest.test_goal_yaw_reached (10320 ms)
[----------] 2 tests from WaypointTest (20646 ms total)
[----------] Global test environment tear-down
[==========] 2 tests from 1 test suite ran. (20646 ms total)
[  PASSED  ] 0 tests.
[  FAILED  ] 2 tests, listed below:
[  FAILED  ] WaypointTest.test_goal_position_reached
[  FAILED  ] WaypointTest.test_goal_yaw_reached
 2 FAILED TESTS
-- run_test.py: return code 1
-- run_test.py: inject classname prefix into gtest result file '/home/user/ros2_ws/build/fastbot_waypoints/test_results/fastbot_waypoints/test_waypoints.gtest.xml'
1: -- run_test.py: verify result file '/home/user/ros2_ws/build/fastbot_waypoints/test_results/fastbot_waypoints/test_waypoints.gtest.xml'
1/1 Test #1: test_waypoints ...................***Failed   20.75 sec
```

And `colcon test-result --all` will show:
```
build/fastbot_waypoints/test_results/fastbot_waypoints/test_waypoints.gtest.xml: 2 tests, 0 errors, 1 failure
```

**Why it fails:** The robot's navigation loop exits when it is within 0.1 m of the goal (the action server's internal `DIST_PRECISION`). The test then checks that the final position is within `DIST_PRECISION = 0.001` m — a 1 mm tolerance the robot cannot achieve — so `EXPECT_LT` reports a failure.

> Remember to revert `DIST_PRECISION` back to `0.1` and rebuild before submitting.


## Additional Helpers
### Marker script to show Pose and orientation.

Launch the Pose marker script and rviz
```bash
cd ~/ros2_ws && colcon build --packages-select fastbot_waypoints && source install/setup.bash
ros2 launch fastbot_waypoints pose_marker_rviz.launch.xml
```
> You can also run them individually `ros2 run fastbot_waypoints pose_marker_display.py` and then `rviz2 -d ~/ros2_ws/src/fastbot/fastbot_description/rviz/fastbot.rviz`

Teleop cmd
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/fastbot/cmd_vel
```
