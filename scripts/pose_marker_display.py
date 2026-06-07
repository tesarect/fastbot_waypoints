#!/usr/bin/env python3
# Copyright 2026 fastbot_waypoints
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Display fastbot x, y, yaw as RViz markers.

Subscribes to both /odom and /fastbot/odom and publishes a MarkerArray on:
  /odom_markers        ← from /odom
  /fastbot_odom_markers ← from /fastbot/odom

Each MarkerArray contains:
  - Marker 0: green sphere  — robot position (x, y)
  - Marker 1: red arrow     — yaw direction
  - Marker 2: white text    — numeric x, y, yaw

Run:
  ros2 run fastbot_waypoints pose_marker_display.py

In RViz2:
  - Fixed Frame: odom
  - Add > MarkerArray > Topic: /odom_markers
  - Add > MarkerArray > Topic: /fastbot_odom_markers
"""

from transforms3d.euler import quat2euler

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion


def quat_to_yaw(q: Quaternion) -> float:
    """Return yaw (rad) from a geometry_msgs/Quaternion using transforms3d."""
    # transforms3d quat convention: (w, x, y, z)
    # quat2euler returns (roll, pitch, yaw) in 'sxyz' convention by default
    _, _, yaw = quat2euler([q.w, q.x, q.y, q.z])
    return yaw


class PoseMarkerDisplay(Node):

    def __init__(self):
        super().__init__('pose_marker_display')

        # Publisher pair for each odometry source
        self._pub_odom = self.create_publisher(
            MarkerArray, '/odom_markers', 10)
        self._pub_fastbot_odom = self.create_publisher(
            MarkerArray, '/fastbot_odom_markers', 10)

        self.create_subscription(
            Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(
            Odometry, '/fastbot/odom', self._fastbot_odom_cb, 10)

        self.get_logger().info(
            'pose_marker_display ready\n'
            '  /odom          -> /odom_markers\n'
            '  /fastbot/odom  -> /fastbot_odom_markers')

    # ------------------------------------------------------------------
    def _odom_cb(self, msg: Odometry) -> None:
        self._pub_odom.publish(self._build_array(msg, ns='odom'))

    def _fastbot_odom_cb(self, msg: Odometry) -> None:
        self._pub_fastbot_odom.publish(
            self._build_array(msg, ns='fastbot_odom'))

    # ------------------------------------------------------------------
    def _build_array(self, msg: Odometry, ns: str) -> MarkerArray:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)

        stamp = msg.header.stamp
        frame = msg.header.frame_id or 'odom'

        array = MarkerArray()
        # array.markers.append(self._sphere_marker(x, y, stamp, frame, ns))
        # array.markers.append(self._arrow_marker(x, y, yaw, stamp, frame, ns))
        array.markers.append(self._text_marker(x, y, yaw, stamp, frame, ns))
        return array

    # ------------------------------------------------------------------
    # Marker builders
    # ------------------------------------------------------------------

    def _base_marker(
            self, marker_id: int, stamp, frame: str, ns: str
    ) -> Marker:
        m = Marker()
        m.header.stamp = stamp
        m.header.frame_id = frame
        m.ns = ns
        m.id = marker_id
        m.action = Marker.ADD
        return m

    def _sphere_marker(
            self, x: float, y: float, stamp, frame: str, ns: str
    ) -> Marker:
        """Green sphere at the robot's (x, y) position."""
        m = self._base_marker(0, stamp, frame, ns)
        m.type = Marker.SPHERE

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1
        m.pose.orientation.w = 1.0

        m.scale.x = 0.12
        m.scale.y = 0.12
        m.scale.z = 0.12

        m.color.r = 0.0
        m.color.g = 0.9
        m.color.b = 0.2
        m.color.a = 0.9
        return m

    def _arrow_marker(
            self, x: float, y: float, yaw: float,
            stamp, frame: str, ns: str
    ) -> Marker:
        """Red arrow pointing in the yaw direction."""
        from transforms3d.euler import euler2quat  # (w,x,y,z) output
        m = self._base_marker(1, stamp, frame, ns)
        m.type = Marker.ARROW

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.1

        # Rotation about Z only
        w, ex, ey, ez = euler2quat(0.0, 0.0, yaw)
        m.pose.orientation.x = ex
        m.pose.orientation.y = ey
        m.pose.orientation.z = ez
        m.pose.orientation.w = w

        m.scale.x = 0.30   # shaft length
        m.scale.y = 0.05   # shaft diameter
        m.scale.z = 0.08   # head diameter

        m.color.r = 0.9
        m.color.g = 0.1
        m.color.b = 0.1
        m.color.a = 0.95
        return m

    def _text_marker(
            self, x: float, y: float, yaw: float,
            stamp, frame: str, ns: str
    ) -> Marker:
        """White floating label with numeric x, y, yaw."""
        import math
        m = self._base_marker(2, stamp, frame, ns)
        m.type = Marker.TEXT_VIEW_FACING

        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.35
        m.pose.orientation.w = 1.0

        m.scale.z = 0.30

        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0

        m.text = (
            f'x: {x:+.3f} m\n'
            f'y: {y:+.3f} m\n'
            f'yaw: {math.degrees(yaw):+.1f} deg'
        )
        return m


# --------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = PoseMarkerDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
