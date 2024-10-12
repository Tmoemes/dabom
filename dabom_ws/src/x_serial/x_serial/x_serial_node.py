#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class XSerialNode(Node):
    """A ROS2 node that forwards /motor_vel topic to /arduino_vel topic."""

    def __init__(self):
        super().__init__('x_serial_node')

        # Create a subscriber to /motor_vel
        self.subscription = self.create_subscription(
            TwistStamped,
            '/motor_vel',
            self.motor_vel_callback,
            10
        )

        # Create a publisher to /arduino_vel
        self.publisher_ = self.create_publisher(TwistStamped, '/arduino_vel', 10)

        self.get_logger().info("x_serial_node started, forwarding /motor_vel to /arduino_vel.")

    def motor_vel_callback(self, msg: TwistStamped):
        """Callback that forwards the TwistStamped message from /motor_vel to /arduino_vel."""
        self.publisher_.publish(msg)
        self.get_logger().info(f"Forwarded /motor_vel message to /arduino_vel: {msg}")

def main(args=None):
    rclpy.init(args=args)
    node = XSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
