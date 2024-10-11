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
        np.cos(z/2),
        np.sin(z/2) * 0,
        np.sin(z/2) * 0,
        np.sin(z/2) * 1
    ]

class OdomNode(Node):
    """Node responsible for calculating odometry for a Mecanum-wheeled robot.
    
    Subscribes to motor velocities, computes robot velocities using forward kinematics,
    and publishes the computed odometry and corresponding TF transform. The odometry
    is published to the '/odom' topic according to ROS2 standard conventions.
    """

    def __init__(self):
        super().__init__('odom_node')

        # Publisher for odometry information
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Transform broadcaster to publish TF information (odom -> base_link)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscriber to motor_velocities from arduino
        self.subscription = self.create_subscription(
            TwistStamped,
            '/motor_velocities',  # Updated to '/arduino_vel'
            self.motor_velocities_callback,
            1
        )
        self.subscription  # Prevent unused variable warning

        # Robot parameters (geometry)
        self.lx = 0.3  # Half of the wheelbase (meters)
        self.ly = 0.3  # Half of the track width (meters)
        self.wheel_radius = 0.04  # Wheel radius (meters)

        # State variables for robot's pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Update transformation matrix used for forward kinematics
        self.update_transformation_matrix()

        self.get_logger().info('OdomNode has been started.')

    def update_transformation_matrix(self):
        """Precomputes the transformation matrix for forward kinematics.

        The matrix is based on the robot's geometry (lx, ly, wheel_radius) and is used
        to convert wheel angular velocities into robot velocities.
        """
        self.T_fwd = (self.wheel_radius / 4) * np.array([
            [1, 1, 1, 1],             # Forward/backward component (vx)
            [-1, 1, 1, -1],           # Lateral component (vy)
            [-1 / (self.lx + self.ly), 1 / (self.lx + self.ly),  # Rotational component (omega)
             -1 / (self.lx + self.ly), 1 / (self.lx + self.ly)]
        ])

    def motor_velocities_callback(self, msg: TwistStamped):
        """Callback that processes motor velocities and updates robot odometry.

        This function is triggered whenever a new message is received on the
        '/arduino_vel' topic. It uses the angular velocities of the wheels to
        calculate the robot's velocity and pose (x, y, theta).
        """
        try:
            # Get the current time and compute time delta (dt)
            current_time = self.get_clock().now()
            dt_duration = current_time - self.last_time
            dt = self.duration_to_sec(dt_duration)
            if dt <= 0.0:
                return  # Removed redundant logging
            self.last_time = current_time

            # Extract wheel angular velocities from TwistStamped message
            wheel_angular_velocities = np.array([
                msg.twist.linear.x,     # Front-Left Wheel
                msg.twist.linear.y,     # Front-Right Wheel
                msg.twist.linear.z,     # Rear-Left Wheel
                msg.twist.angular.x     # Rear-Right Wheel
            ])

            # Convert angular velocities to linear velocities at the wheel circumference
            #wheel_velocities = wheel_angular_velocities * self.wheel_radius

            # Compute robot velocities (vx, vy, omega) using forward kinematics
            robot_velocities = self.T_fwd @ wheel_angular_velocities 
            vx, vy, omega = robot_velocities

            # Update robot's pose based on velocities and time delta
            delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
            delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
            delta_theta = omega * dt

            self.x += delta_x
            self.y += delta_y
            self.theta = self.wrap_angle(self.theta + delta_theta)

            # Publish updated odometry message
            odom_msg = self.create_odometry_msg(current_time, vx, vy, omega)
            self.odom_publisher.publish(odom_msg)

            # Publish corresponding TF transform
            self.publish_transform(current_time, odom_msg)

        except Exception as e:
            self.get_logger().error(f'Error in motor_velocities_callback: {str(e)}')

    @staticmethod
    def duration_to_sec(duration):
        """
        Converts a Duration object to seconds.
        Handles both rclpy.duration.Duration and builtin_interfaces.msg.Duration.
        """
        # Check for rclpy.duration.Duration
        if hasattr(duration, 'to_sec'):
            return duration.to_sec()
        # Check for builtin_interfaces.msg.Duration
        elif hasattr(duration, 'sec') and hasattr(duration, 'nanosec'):
            return duration.sec + duration.nanosec * 1e-9
        # Check for rclpy.duration.Duration with 'nanoseconds' attribute
        elif hasattr(duration, 'nanoseconds'):
            return duration.nanoseconds / 1e9
        else:
            raise AttributeError("Unknown Duration type. Cannot convert to seconds.")

    def create_odometry_msg(self, current_time, vx, vy, omega):
        """Creates and returns an Odometry message with the current pose and velocities."""
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation (queuleraternion)
        half_theta = self.theta
        q = euler_to_quaternion(self.theta + np.pi / 2)
        odom_msg.pose.pose.orientation.x = q[1]
        odom_msg.pose.pose.orientation.y = q[2]
        odom_msg.pose.pose.orientation.z = q[3]
        odom_msg.pose.pose.orientation.w = q[0]

        # Pose covariance
        odom_msg.pose.covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]

        # Velocities
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega

        # Twist covariance
        odom_msg.twist.covariance = [0.01 if i % 7 == 0 else 0.0 for i in range(36)]

        return odom_msg

    def publish_transform(self, current_time, odom_msg):
        """Publishes a TF transform based on the current odometry."""
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def wrap_angle(angle):
        """Wrap the angle to [0, 2*pi]."""
        # return (angle + math.pi) % (2 * math.pi) - math.pi
        return np.mod(angle, 2*np.pi)




def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()  # Node name updated to 'odom_node'
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down odometry node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
