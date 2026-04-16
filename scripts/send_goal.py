#!/usr/bin/env python3
"""
Send a Nav2 goal to TidyBot.

Usage:
  # By name (preset goals)
  python3 scripts/send_goal.py room_b
  python3 scripts/send_goal.py collection
  python3 scripts/send_goal.py shelf
  python3 scripts/send_goal.py home

  # By explicit x y (in map frame)
  python3 scripts/send_goal.py 5.0 1.5
  python3 scripts/send_goal.py 5.0 1.5 0.0        # with yaw

Coordinate reference (map frame; origin = robot spawn pose in world):
    (0, 0)     robot spawn / collection box area
    (5, 1.5)   Room B table
    (7, 0)     shelf against east wall
    (1, -2)    Room A couch

Requires navigation.launch.py to already be running.
"""
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


# Named goals in map frame coordinates (x, y, yaw_deg).
# Pickup-object approaches stop 0.4 m west of the target so the robot
# faces the object without colliding with it.
PRESETS = {
    # Navigation waypoints
    'home':       (0.0, 0.0, 0.0),
    'collection': (0.0, -1.0, 0.0),
    'room_a':     (1.0, 1.0, 0.0),
    'room_b':     (5.0, 1.5, 0.0),
    'shelf':      (7.0, 0.5, 180.0),
    'couch':      (-0.5, -2.0, 0.0),

    # Pickup-object approaches (object pose - 0.4 m in x, facing +x)
    'red':        (0.6, 0.0, 0.0),
    'green':      (1.4, -0.7, 0.0),
    'blue':       (4.1, -1.2, 0.0),
    'yellow':     (4.6, 0.0, 0.0),
    'orange':     (6.1, 1.2, 0.0),
    'pink':       (5.4, -1.5, 0.0),   # alias for purple cube
    'purple':     (5.4, -1.5, 0.0),
}


def yaw_to_quat(yaw_rad):
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


class GoalSender(Node):
    def __init__(self):
        super().__init__('goal_sender')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send(self, x, y, yaw_deg=0.0):
        if not self.client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                'Nav2 action server /navigate_to_pose not available. '
                'Is navigation.launch.py running and Nav2 active?')
            return 1

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quat(math.radians(yaw_deg))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal.pose = pose

        self.get_logger().info(
            f'Sending goal: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.1f} deg')

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error('Goal rejected by Nav2.')
            return 2

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        status = result_future.result().status
        # nav2_msgs action status codes: 4 = SUCCEEDED
        if status == 4:
            self.get_logger().info('Goal reached.')
            return 0
        self.get_logger().warn(f'Goal finished with status {status}.')
        return 3


def parse_args(argv):
    if len(argv) == 1:
        name = argv[0]
        if name not in PRESETS:
            print(f'Unknown preset {name!r}. Choices: {list(PRESETS)}',
                  file=sys.stderr)
            sys.exit(2)
        return PRESETS[name]
    if len(argv) == 2:
        return float(argv[0]), float(argv[1]), 0.0
    if len(argv) == 3:
        return float(argv[0]), float(argv[1]), float(argv[2])
    print(__doc__, file=sys.stderr)
    sys.exit(2)


def main():
    argv = sys.argv[1:]
    if not argv:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    x, y, yaw = parse_args(argv)

    rclpy.init()
    node = GoalSender()
    try:
        rc = node.send(x, y, yaw)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
