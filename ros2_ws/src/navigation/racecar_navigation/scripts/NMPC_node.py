#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from ackermann_msgs.msg import AckermannDrive
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import numpy as np
from nav_msgs.msg import Path

# import csv
import NMPC

class NMPCNode(Node):
    def __init__(self):
        super().__init__('robot_location_listener')

        # Create a subscriber with best effort QoS
        self.odom_subscriber = self.create_subscription(Odometry, '/pf/pose/odom', self.odom_callback, qos_profile_sensor_data)
        self.ackermann_publisher = self.create_publisher(AckermannDrive, 'ackermann_cmd', 10)
        self.plan_publisher = self.create_publisher(Path, 'plan', 10)
        self.waypoints = np.loadtxt('/ros2_ws/maps/addis_min_time3.csv', delimiter=';')

        self.pose = None
        self.velocity = None

        self.state = NMPC.State(x=0, y=0, yaw=0, v=0)
        self.ds = 0.5
        self.target_ind = 0
        self.oa = None
        self.odelta = None
        self.dt = 0.05
        

        # # Create a TF buffer and listener
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)



        # Set the timer to periodically check the transform
        self.timer = self.create_timer(self.dt, self.timer_callback)  # 1.0 seconds interval


    # def publish_plan(self):
    #     _, idx = self.get_closest_waypoint()

    #     path = Path()
    #     path.header.frame_id = 'map'
    #     path.header.stamp = self.get_clock().now().to_msg()
    #     # Publish first 10 waypoints
    #     for i in range(10):
    #         pose = PoseStamped()
    #         pose.header.frame_id = 'map'
    #         pose.pose.position.x = self.waypoints[(idx + i) % len(self.waypoints)][0]
    #         pose.pose.position.y = self.waypoints[(idx + i) % len(self.waypoints)][1]
    #         path.poses.append(pose)
    #     self.plan_publisher.publish(path)

    # def get_closest_waypoint(self):
    #     min_dist = float('inf')
    #     closest_waypoint = None
    #     closest_index = None
    #     for i, (x, y) in enumerate(self.waypoints):
    #         dist = (self.position[0] - x) ** 2 + (self.position[1] - y) ** 2
    #         if dist < min_dist:
    #             min_dist = dist
    #             closest_waypoint = (x, y)
    #             closest_index = i
    #     return closest_waypoint, closest_index

    def odom_callback(self, msg):
        self.pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.velocity = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]
        self.state.x = msg.pose.pose.position.x
        self.state.y = msg.pose.pose.position.y
        rpy = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.state.yaw = rpy[2]
        self.state.v = msg.twist.twist.linear.x

    def timer_callback(self):
        # Extract the waypoint
        cx = self.waypoints[:, 1]
        cy = self.waypoints[:, 2]
        # Calculate the yaw by shifting the array
        cyaw = np.zeros_like(cx)
        cyaw[1:] = np.arctan2(np.diff(cy), np.diff(cx))
        cyaw[0] = cyaw[1]
        
        sp = self.waypoints[:, 5]
        ck = self.waypoints[:, 4]

        xref, self.target_ind, dref = NMPC.calc_ref_trajectory(self.state, cx, cy, cyaw, ck, sp, self.ds, self.target_ind)
        x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]

        self.oa, self.odelta, ox, oy, oyaw, ov = NMPC.iterative_linear_mpc_control(xref, x0, dref, self.oa, self.odelta)
        di, ai = 0.0, 0.0
        if self.odelta is not None:
            di, ai = self.odelta[0], self.oa[0]
            vel_c = self.state.v + ai * self.dt

            ackermann_drive = AckermannDrive()
            ackermann_drive.speed = vel_c
            ackermann_drive.steering_angle =  di
            self.ackermann_publisher.publish(ackermann_drive)
        
        # Publish the plan
        plan = Path()
        plan.header.frame_id = 'map'
        plan.header.stamp = self.get_clock().now().to_msg()
        for i in range(len(xref[0])):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = xref[0][i]
            pose.pose.position.y = xref[1][i]
            plan.poses.append(pose)
        self.plan_publisher.publish(plan)

        print('yaw_ref:', xref[3, 0],  'yaw:', self.state.yaw)



def main(args=None):
    rclpy.init(args=args)
    node = NMPCNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
