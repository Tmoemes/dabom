#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import TransformStamped
import tf2_ros

def euler_to_quaternion(z):
    return [
        np.cos(z / 2),
        np.sin(z / 2) * 0,
        np.sin(z / 2) * 0,
        np.sin(z / 2) * 1
    ]

class OdomNode(Node):
    """Node responsible for calculating odometry for a Mecanum-wheeled robot."""

    def __init__(self):
        super().__init__('odom_node')

        # Declare ROS2 parameters
        self.declare_parameter('lx', 0.3)
        self.declare_parameter('ly', 0.3)
        self.declare_parameter('wheel_radius', 0.04)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # Retrieve parameters
        self.lx = self.get_parameter_or('lx', 0.3).value
        self.ly = self.get_parameter_or('ly', 0.3).value
        self.wheel_radius = self.get_parameter_or('wheel_radius', 0.04).value
        self.odom_frame = self.get_parameter_or('odom_frame', 'odom').value
        self.base_frame = self.get_parameter_or('base_frame', 'base_footprint').value

        # Log the parameters at startup
        self.get_logger().info(f'Parameters: lx={self.lx}, ly={self.ly}, wheel_radius={self.wheel_radius}')
        self.get_logger().info(f'Odom frame: {self.odom_frame}, Base frame: {self.base_frame}')

        # Publisher for odometry information
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Transform broadcaster to publish TF information (odom -> base_link)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber to motor velocities
        self.subscription = self.create_subscription(
            TwistStamped,
            '/arduino_vel',
            self.motor_velocities_callback,
            1
        )
        self.subscription  # Prevent unused variable warning

        # State variables for robot's pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Update transformation matrix used for forward kinematics
        self.update_transformation_matrix()

        self.get_logger().info('OdomNode has been started.')

    def update_transformation_matrix(self):
        """Precomputes the transformation matrix for forward kinematics."""
        self.T_fwd = (self.wheel_radius / 4) * np.array([
            [1, 1, 1, 1],  # Forward/backward component (vx)
            [-1, 1, 1, -1],  # Lateral component (vy)
            [-1 / (self.lx + self.ly), 1 / (self.lx + self.ly),
             -1 / (self.lx + self.ly), 1 / (self.lx + self.ly)]  # Rotational component (omega)
        ])

    def motor_velocities_callback(self, msg: TwistStamped):
        """Callback that processes motor velocities and updates robot odometry."""
        try:
            current_time = self.get_clock().now()
            dt_duration = current_time - self.last_time
            dt = self.duration_to_sec(dt_duration)
            if dt <= 0.0:
                return
            self.last_time = current_time

            wheel_angular_velocities = np.array([
                msg.twist.linear.x,  # Front-Left Wheel
                msg.twist.linear.y,  # Front-Right Wheel
                msg.twist.linear.z,  # Rear-Left Wheel
                msg.twist.angular.x  # Rear-Right Wheel
            ])

            robot_velocities = self.T_fwd @ wheel_angular_velocities
            vx, vy, omega = robot_velocities

            delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
            delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
            delta_theta = omega * dt

            self.x += delta_x
            self.y += delta_y
            self.theta = self.wrap_angle(self.theta + delta_theta)

            odom_msg = self.create_odometry_msg(current_time, vx, vy, omega)
            self.odom_publisher.publish(odom_msg)
            self.publish_transform(current_time, odom_msg)

        except Exception as e:
            self.get_logger().error(f'Error in motor_velocities_callback: {str(e)}')

    def create_odometry_msg(self, current_time, vx, vy, omega):
        """Creates and returns an Odometry message with the current pose and velocities."""
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        q = euler_to_quaternion(self.theta)
        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.y = q[2]
        odom_msg.pose.pose.orientation.z = q[3]
        odom_msg.pose.pose.orientation.w = q[0]

        odom_msg.pose.covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega

        odom_msg.twist.covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]

        return odom_msg

    def publish_transform(self, current_time, odom_msg):
        """Publishes a TF transform based on the current odometry."""
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def duration_to_sec(duration):
        """Converts a Duration object to seconds."""
        if hasattr(duration, 'to_sec'):
            return duration.to_sec()
        elif hasattr(duration, 'sec') and hasattr(duration, 'nanosec'):
            return duration.sec + duration.nanosec * 1e-9
        elif hasattr(duration, 'nanoseconds'):
            return duration.nanoseconds / 1e9
        else:
            raise AttributeError("Unknown Duration type. Cannot convert to seconds.")

    @staticmethod
    def wrap_angle(angle):
        """Wrap the angle to [0, 2*pi]."""
        return np.mod(angle, 2 * np.pi)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down odometry node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
