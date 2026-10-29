
#!/usr/bin/env python3
"""Pure Pursuit waypoint following for the F1TENTH simulator."""

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker


LOOKAHEAD_DISTANCE = 1.2
WHEELBASE = 0.33
MAX_STEERING = math.radians(20.0)


class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, '/drive', 10
        )

        self.path_pub = self.create_publisher(
            Marker, '/env_viz', 10
        )

        self.target_pub = self.create_publisher(
            Marker, '/dynamic_viz', 10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.pose_callback,
            10
        )

        # Load the professor's CSV waypoints.
        csv_path = Path(__file__).resolve().parent / 'data.csv'

        if not csv_path.is_file():
            raise FileNotFoundError(
                'Waypoint file not found: {}'.format(csv_path)
            )

        self.waypoints = []

        with csv_path.open(newline='') as file:
            reader = csv.reader(file)

            for row in reader:
                if len(row) >= 2:
                    x = float(row[0])
                    y = float(row[1])
                    self.waypoints.append((x, y))

        if len(self.waypoints) < 2:
            raise ValueError('Not enough waypoints in data.csv')

        self.callback_count = 0

        self.get_logger().info(
            'Loaded {} waypoints'.format(len(self.waypoints))
        )

    def pose_callback(self, odom):

        # Simulator runs around 250 Hz.
        # Process every fifth message (about 50 Hz).
        self.callback_count += 1

        if self.callback_count % 5 != 0:
            return

        # Read car position.
        x = odom.pose.pose.position.x
        y = odom.pose.pose.position.y

        # Convert quaternion orientation to yaw.
        q = odom.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # Find the closest waypoint.
        nearest = min(
            range(len(self.waypoints)),
            key=lambda i: (
                (self.waypoints[i][0] - x) ** 2
                + (self.waypoints[i][1] - y) ** 2
            )
        )

        # Find a waypoint ahead of the car, at least
        # LOOKAHEAD_DISTANCE away.
        target_index = None
        target_dx = 0.0
        target_dy = 0.0

        for offset in range(1, len(self.waypoints) + 1):

            i = (nearest + offset) % len(self.waypoints)

            dx = self.waypoints[i][0] - x
            dy = self.waypoints[i][1] - y

            forward = cos_yaw * dx + sin_yaw * dy
            distance_squared = dx * dx + dy * dy

            if (
                forward > 0.0
                and distance_squared >= LOOKAHEAD_DISTANCE ** 2
            ):
                target_index = i
                target_dx = dx
                target_dy = dy
                break

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = odom.header.stamp

        # Stop safely if no valid target was found.
        if target_index is None:
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
            self.drive_pub.publish(drive_msg)
            return

        # Transform target into the vehicle's coordinates.
        local_y = (
            -sin_yaw * target_dx
            + cos_yaw * target_dy
        )

        real_distance_squared = (
            target_dx ** 2 + target_dy ** 2
        )

        # Calculate Pure Pursuit curvature.
        curvature = (
            2.0 * local_y / real_distance_squared
        )

        # Convert curvature to steering angle.
        steering = math.atan(WHEELBASE * curvature)

        # Limit steering to +/- 20 degrees.
        steering = max(
            -MAX_STEERING,
            min(MAX_STEERING, steering)
        )

        drive_msg.drive.steering_angle = steering

        # Slow down for sharper turns.
        if abs(steering) > math.radians(12.0):
            drive_msg.drive.speed = 0.8
        else:
            drive_msg.drive.speed = 1.2

        self.drive_pub.publish(drive_msg)

        # Publish visualization markers.
        frame = odom.header.frame_id or 'map'

        self.publish_target(
            frame, odom.header.stamp, target_index
        )

        if self.callback_count % 250 == 0:
            self.publish_path(frame, odom.header.stamp)

    def publish_target(self, frame, stamp, index):

        marker = Marker()

        marker.header.frame_id = frame
        marker.header.stamp = stamp

        marker.ns = 'pure_pursuit_goal'
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.waypoints[index][0]
        marker.pose.position.y = self.waypoints[index][1]
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3

        marker.color.r = 1.0
        marker.color.a = 1.0

        self.target_pub.publish(marker)

    def publish_path(self, frame, stamp):

        marker = Marker()

        marker.header.frame_id = frame
        marker.header.stamp = stamp

        marker.ns = 'pure_pursuit_path'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.07
        marker.scale.y = 0.07

        marker.color.b = 1.0
        marker.color.a = 1.0

        for x, y in self.waypoints:
            point = Point()
            point.x = x
            point.y = y
            marker.points.append(point)

        self.path_pub.publish(marker)


def main(args=None):

    rclpy.init(args=args)

    node = PurePursuit()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
