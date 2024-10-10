#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class MecanumKinematicsNode(Node):
    """Node for calculating inverse kinematics for a Mecanum-wheeled robot.

    Subscribes to a `cmd_vel` topic to receive desired robot velocities and 
    computes the corresponding wheel velocities using inverse kinematics. 
    Publishes the wheel velocities as a `TwistStamped` message.
    """

    def __init__(self):
        super().__init__('mecanum_kinematics_node')

        # Declare ROS2 parameters for robot configuration
        self.declare_parameter('lx', 0.3)  # Half of the wheelbase (meters)
        self.declare_parameter('ly', 0.3)  # Half of the track width (meters)
        self.declare_parameter('wheel_radius', 0.04)  # Wheel radius (meters)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('motor_vel_topic', '/motor_velocities')
        self.declare_parameter('frame_id', 'base_link')

        # Get parameter values
        self.lx = self.get_parameter('lx').get_parameter_value().double_value
        self.ly = self.get_parameter('ly').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self.motor_vel_topic = self.get_parameter('motor_vel_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

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
        """Callback function to process incoming velocity commands and compute wheel velocities.

        Args:
            msg (Twist): The incoming desired linear and angular velocity for the robot.
        """
        try:
            # Extract desired linear and angular velocities
            vx = msg.linear.x
            vy = msg.linear.y
            omega = msg.angular.z

            # Create velocity vector for the robot
            velocities = np.array([vx, vy, omega])

            # Calculate wheel velocities using inverse kinematics
            wheel_velocities = self.T @ velocities

            # Prepare and publish the motor velocity message with timestamp
            stamped_msg = TwistStamped()
            stamped_msg.header.stamp = self.get_clock().now().to_msg()  # Add the current timestamp
            stamped_msg.header.frame_id = self.frame_id  # Use parameterized frame ID

            # Set calculated wheel velocities
            stamped_msg.twist.linear.x = wheel_velocities[0]  # Wheel 1 velocity
            stamped_msg.twist.linear.y = wheel_velocities[1]  # Wheel 2 velocity
            stamped_msg.twist.linear.z = wheel_velocities[2]  # Wheel 3 velocity
            stamped_msg.twist.angular.x = wheel_velocities[3]  # Wheel 4 velocity

            # Publish the message with calculated wheel velocities
            self.publisher.publish(stamped_msg)

            # Log wheel velocities only at the debug level to avoid spamming logs
            self.get_logger().debug(f'Published wheel velocities: {wheel_velocities}')
        except Exception as e:
            self.get_logger().error(f'Error in cmd_vel_callback: {str(e)}')

    @staticmethod
    def wrap_angle(angle):
        """Wrap the given angle to the range [-pi, pi].

        Args:
            angle (float): The input angle in radians.

        Returns:
            float: The angle wrapped to the range [-pi, pi].
        """
        # return (angle + np.pi) % (2 * np.pi) - np.pi
        return np.mod(angle, 2*np.pi)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumKinematicsNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
