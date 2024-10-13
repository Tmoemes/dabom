#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class InvKinNode(Node):
    """Node for calculating inverse kinematics for a Mecanum-wheeled robot."""

    def __init__(self):
        super().__init__('inv_kin_node')

        # Declare ROS2 parameters for robot configuration with default values
        self.declare_parameter('lx', 0.3)  # Half of the wheelbase (meters)
        self.declare_parameter('ly', 0.3)  # Half of the track width (meters)
        self.declare_parameter('wheel_radius', 0.04)  # Wheel radius (meters)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motor_vel_topic', '/motor_vel')
        self.declare_parameter('frame_id', 'base_link')

        # Retrieve parameter values (these will respect any overrides from YAML)
        self.lx = self.get_parameter('lx').value
        self.ly = self.get_parameter('ly').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.motor_vel_topic = self.get_parameter('motor_vel_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # Log the parameters once at startup
        self.get_logger().info(f'Parameters: lx={self.lx}, ly={self.ly}, wheel_radius={self.wheel_radius}')

        # Subscribe to the desired velocity (geometry_msgs/Twist)
        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Publisher for individual wheel velocities (TwistStamped) with timestamp
        self.publisher = self.create_publisher(TwistStamped, self.motor_vel_topic, 10)

        # Transformation matrix for inverse kinematics (precomputed at initialization)
        self.T = (1 / self.wheel_radius) * np.array([
            [1, -1, -(self.lx + self.ly)],  # Wheel 1 (Front-Left)
            [1, 1, (self.lx + self.ly)],    # Wheel 2 (Front-Right)
            [1, 1, -(self.lx + self.ly)],   # Wheel 3 (Rear-Left)
            [1, -1, (self.lx + self.ly)]    # Wheel 4 (Rear-Right)
        ])

    def cmd_vel_callback(self, msg: Twist):
        """Callback function to process incoming velocity commands and compute wheel velocities."""
        try:
            vx = msg.linear.x
            vy = msg.linear.y
            omega = msg.angular.z

            velocities = np.array([vx, vy, omega])
            wheel_velocities = self.T @ velocities

            stamped_msg = TwistStamped()
            stamped_msg.header.stamp = self.get_clock().now().to_msg()
            stamped_msg.header.frame_id = self.frame_id

            stamped_msg.twist.linear.x = wheel_velocities[0]  # Wheel 1 velocity
            stamped_msg.twist.linear.y = wheel_velocities[1]  # Wheel 2 velocity
            stamped_msg.twist.linear.z = wheel_velocities[2]  # Wheel 3 velocity
            stamped_msg.twist.angular.x = wheel_velocities[3]  # Wheel 4 velocity

            self.publisher.publish(stamped_msg)
            self.get_logger().debug(f'Published wheel velocities: {wheel_velocities}')

        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = InvKinNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
