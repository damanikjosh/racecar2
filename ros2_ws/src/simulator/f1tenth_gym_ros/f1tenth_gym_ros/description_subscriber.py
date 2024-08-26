import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped

class LinkPositionListener(Node):

    def __init__(self):
        super().__init__('link_position_listener')
        
        # Create a TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Get the transform once during initialization
        transform = self.get_transform_once('left_front_wheel', 'base_link')
        self.handle_transform(transform)

    def get_transform_once(self, source_frame, target_frame):
        while True:
            try:
                # Lookup the transform between base_link and left_front_wheel with a timeout
                rclpy.spin_once(self)
                now = rclpy.time.Time()
                transform = self.tf_buffer.lookup_transform(
                    source_frame, 
                    target_frame,
                    now,
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                return transform
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass

    def handle_transform(self, transform: TransformStamped):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.get_logger().info(
            f"xyz: ({translation.x}, {translation.y}, {translation.z}), "
            f"orientation (quaternion): "
            f"({rotation.x}, {rotation.y}, {rotation.z}, {rotation.w})"
        )

def main(args=None):
    rclpy.init(args=args)
    link_position_listener = LinkPositionListener()
    rclpy.spin(link_position_listener)
    link_position_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()