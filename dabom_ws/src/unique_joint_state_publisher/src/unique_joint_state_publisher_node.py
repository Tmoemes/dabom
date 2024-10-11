#!/usr/bin/env python3

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped


class UniqueJointStatePublisherNode(Node):
    """Node responsible for publishing JointState messages based on wheel velocities.
    
    This node subscribes to the /arduino_vel topic, processes wheel angular velocities, 
    and publishes JointState messages for four wheels.
    """

    def __init__(self):
        super().__init__('unique_joint_state_publisher_node')

        # Publisher for JointState messages
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Subscriber to /arduino_vel topic for motor velocities
        self.subscription = self.create_subscription(
            TwistStamped,
            '/arduino_vel',  # Topic for wheel velocities
            self.arduino_vel_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Robot parameters (wheel radius, ensure these match your URDF)
        self.wheel_radius = 0.04  # Wheel radius in meters

        # Joint states initialization
        self.joint_names = ['Wheel1', 'Wheel2', 'Wheel3', 'Wheel4']  # Must match URDF joint names
        self.joint_positions = np.zeros(4)    # Initialize joint positions to zero
        self.joint_velocities = np.zeros(4)   # Initialize joint velocities to zero
        self.joint_efforts = [0.0] * 4        # Placeholder for efforts (populate if available)

        # Time tracking for integration of velocities to positions
        self.last_time = self.get_clock().now()

        self.get_logger().info('UniqueJointStatePublisherNode has been started.')

    def get_delta_time(self, current_time):
        """Helper function to compute the time delta in seconds."""
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return None  # Return None to signal invalid delta time
        self.last_time = current_time
        return dt

    def arduino_vel_callback(self, msg: TwistStamped):
        """Callback that processes wheel velocities and updates joint states.
        
        This function integrates angular velocities over time to update joint positions and 
        then publishes the updated JointState message.
        """
        try:
            # Get the current time and delta time
            current_time = self.get_clock().now()
            dt = self.get_delta_time(current_time)
            if dt is None:
                return  # Skip if delta time is invalid

            # Extract wheel angular velocities from the received TwistStamped message
            wheel_angular_velocities = np.array([
                msg.twist.linear.x,     # Wheel1 (Front-Left)
                msg.twist.linear.y,     # Wheel2 (Front-Right)
                msg.twist.linear.z,     # Wheel3 (Rear-Left)
                msg.twist.angular.x     # Wheel4 (Rear-Right)
            ])

            # Update joint positions by integrating angular velocities over time
            self.joint_positions += wheel_angular_velocities * dt
            self.joint_velocities = wheel_angular_velocities

            # Create and populate JointState message
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = current_time.to_msg()
            joint_state_msg.name = self.joint_names
            joint_state_msg.position = self.joint_positions.tolist()
            joint_state_msg.velocity = self.joint_velocities.tolist()
            joint_state_msg.effort = self.joint_efforts  # Placeholder for future effort data

            # Publish the JointState message
            self.joint_state_publisher.publish(joint_state_msg)

        except Exception as e:
            self.get_logger().error(f'Error in arduino_vel_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = UniqueJointStatePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down UniqueJointStatePublisherNode.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
