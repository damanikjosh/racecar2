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
import cubic_spline_planner

# import csvs
import NMPC

class NMPCNode(Node):
    def __init__(self):
        super().__init__('robot_location_listener')

        # Create a subscriber with best effort QoS
        self.odom_subscriber = self.create_subscription(Odometry, '/pf/pose/odom', self.odom_callback, qos_profile_sensor_data)
        self.ackermann_publisher = self.create_publisher(AckermannDrive, 'ackermann_cmd', 10)
        self.plan_publisher = self.create_publisher(Path, 'plan', 10)
        # self.waypoints = np.loadtxt('/ros2_ws/maps/track_optim.csv', delimiter=',', skiprows=1)
        self.waypoints = np.loadtxt('/ros2_ws/maps/track_optim.csv', delimiter=';', skiprows=1)
        for i in range(len(self.waypoints)):
            current_wpt = np.array([self.waypoints[i][1], self.waypoints[i][2]])
            next_wpt = np.array([self.waypoints[(i + 1),1 ], self.waypoints[(i + 1), 2]])
            diff = next_wpt - current_wpt
            yaw = np.arctan2(diff[1], diff[0])
            self.waypoints[i][3] = yaw
        current_wpt = np.array([self.waypoints[-1][1], self.waypoints[-1][2]])
        next_wpt = np.array([self.waypoints[0][1], self.waypoints[0][2]])
        diff = next_wpt - current_wpt
        yaw = np.arctan2(diff[1], diff[0])
        self.waypoints[-1][3] = yaw
        self.waypoints[:, 3] = self.smooth_yaw(self.waypoints[:, 3])

        self.pose = None
        self.velocity = None

        self.state = NMPC.State(x=0, y=0, yaw=0, v=0)
        self.ds = 0.5
        self.target_ind = 0
        self.oa = None
        self.odelta = None

        self.oa_array = []
        self.odelta_array = []
        self.dt = 0.02

        self.action = AckermannDrive()
        self.accels = []
        self.steering_angles = []

        # Set the timer to periodically check the transform
        self.control_timer = self.create_timer(self.dt, self.control_callback)  # 1.0 seconds interval
        # self.speed_timer = self.create_timer(0.1, self.publish_speed)

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

    def publish_speed(self):
        # Take out one odelta and oa
        if len(self.odelta_array) > 0:
            # Pop the numpy array
            di, ai = self.odelta_array.pop(0), self.oa_array.pop(0)
            vel_c = self.state.v + 2.5 * ai * self.dt

            action = AckermannDrive()
            action.speed = vel_c
            action.steering_angle =  0.75 * di
            self.ackermann_publisher.publish(action)
            print('Published speed:', vel_c, 'Published steering angle:', di)

    def odom_callback(self, msg):
        self.pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.velocity = [msg.twist.twist.linear.x, msg.twist.twist.linear.y]
        self.state.x = msg.pose.pose.position.x
        self.state.y = msg.pose.pose.position.y
        rpy = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.state.yaw = rpy[2]
        self.state.v = msg.twist.twist.linear.x

    def control_callback(self):
        # Extract the waypoint
        cx = self.waypoints[:, 1]
        cy = self.waypoints[:, 2]
        cyaw = self.waypoints[:, 3]
        # cyaw = self.waypoints[:, 3]
        # Calculate the yaw by shifting the array
        # cyaw = np.zeros_like(cx)
        # cyaw[1:] = np.arctan2(np.diff(cy), np.diff(cx))
        # cyaw[0] = cyaw[1]
        
        # sp = NMPC.calc_speed_profile(cx, cy, cyaw, target_speed=2.0)
        sp = self.waypoints[:, 5]
        ck = self.waypoints[:, 4]

        xref, self.target_ind, dref = NMPC.calc_ref_trajectory(self.state, cx, cy, cyaw, None, sp, self.ds, self.target_ind)
        x0 = [self.state.x, self.state.y, self.state.v, self.state.yaw]

        self.oa, self.odelta, ox, oy, oyaw, ov = NMPC.iterative_linear_mpc_control(xref, x0, dref, self.oa, self.odelta)
        self.oa_array = self.oa.tolist()
        self.odelta_array = self.odelta.tolist()
        self.publish_speed()
        
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

        print('yaw_ref:', xref[3, 0],  'yaw:', self.state.yaw, 'speed_ref:', xref[2, 0], 'speed:', self.state.v)



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
