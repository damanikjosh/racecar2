#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

class SteeringPublisher(Node):
    def __init__(self):
        super().__init__('steering_publisher')
        self.publisher_ = self.create_publisher(Float32, '/steering/position/command', 10)
        self.timer = self.create_timer(0.1, self.publish_command)
        self.counter = 0

    def publish_command(self):
        msg = Float32()

        if self.counter == 0:
            msg.data = 0.5
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: {msg.data}')
            time.sleep(0.2)
            self.counter += 1

        elif self.counter == 1:
            msg.data = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info(f'Publishing: {msg.data}')
            time.sleep(1.0)
            exit(0)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
