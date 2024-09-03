#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros
import csv

class RobotLocationListener(Node):
    def __init__(self):
        super().__init__('robot_location_listener')

        self.plan_publisher = self.create_publisher(Path, 'plan', 10)
        self.waypoints = self.read_waypoints_from_csv('/ros2_ws/maps/ki_3f_r_centerline.csv')

        # Create a TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.position = [0, 0]

        # Set the timer to periodically check the transform
        self.timer = self.create_timer(0.1, self.timer_callback)  # 1.0 seconds interval
    
    def read_waypoints_from_csv(self, filename):
        waypoints = []
        with open(filename, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            next(csvreader)  # Skip the header
            for row in csvreader:
                x, y = float(row[0]), float(row[1])
                waypoints.append((x, y))
        return waypoints

    def publish_plan(self):
        _, idx = self.get_closest_waypoint()

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        # Publish first 10 waypoints
        for i in range(10):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = self.waypoints[(idx + i) % len(self.waypoints)][0]
            pose.pose.position.y = self.waypoints[(idx + i) % len(self.waypoints)][1]
            path.poses.append(pose)
        self.plan_publisher.publish(path)

    def get_closest_waypoint(self):
        min_dist = float('inf')
        closest_waypoint = None
        closest_index = None
        for i, (x, y) in enumerate(self.waypoints):
            dist = (self.position[0] - x) ** 2 + (self.position[1] - y) ** 2
            if dist < min_dist:
                min_dist = dist
                closest_waypoint = (x, y)
                closest_index = i
        return closest_waypoint, closest_index

    def timer_callback(self):
        try:
            # Get the latest transform between 'map' and 'base_link'
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform('map', 'base_link', now)
            self.position = [transform.transform.translation.x, transform.transform.translation.y]

            # Print the translation and rotation of the robot in the map frame
            self.get_logger().info(
                f"Robot Location: Translation (x: {transform.transform.translation.x:.2f}, "
                f"y: {transform.transform.translation.y:.2f}, z: {transform.transform.translation.z:.2f}), "
                f"Rotation (x: {transform.transform.rotation.x:.2f}, "
                f"y: {transform.transform.rotation.y:.2f}, "
                f"z: {transform.transform.rotation.z:.2f}, "
                f"w: {transform.transform.rotation.w:.2f})"
            )
            self.publish_plan()
        except tf2_ros.LookupException:
            self.get_logger().warn('Transform not available yet.')
        except tf2_ros.ConnectivityException:
            self.get_logger().error('Connectivity problem occurred.')
        except tf2_ros.ExtrapolationException:
            self.get_logger().error('Extrapolation error occurred.')

def main(args=None):
    rclpy.init(args=args)
    node = RobotLocationListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
