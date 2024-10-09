import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros
import numpy as np
import math

class MecanumOdometryNode(Node):
    def __init__(self):
        super().__init__('mecanum_odometry_node')

        # Publisher for odometry
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        # Publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, '/joint_states', 10)

        # TF broadcaster for odometry to base_link
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscription to the motor encoder data (expected Float32MultiArray: [wheel1, wheel2, wheel3, wheel4])
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/motor_encoders',
            self.encoder_callback,
            10)

        # Robot parameters
        self.lx = 0.3  # Half of the wheelbase (distance between front and back wheels)
        self.ly = 0.3  # Half of the track width (distance between left and right wheels)
        self.wheel_radius = 0.04  # Wheel radius in meters (40 mm)

        # State variables for odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # Transformation matrix for forward kinematics
        self.T_fwd = self.wheel_radius / 4 * np.array([
            [1, 1, 1, 1],
            [-1, 1, 1, -1],
            [-1 / (self.lx + self.ly), 1 / (self.lx + self.ly), -1 / (self.lx + self.ly), 1 / (self.lx + self.ly)]
        ])

    def encoder_callback(self, msg: Float32MultiArray):
        # Get current time and compute time delta
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        # Extract wheel velocities from encoder data
        wheel_velocities = np.array(msg.data[:4])

        # Calculate robot velocities in the base frame (vx, vy, omega)
        robot_velocities = self.T_fwd @ wheel_velocities

        # Update the pose using the velocities
        vx, vy, omega = robot_velocities
        delta_x = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * dt
        delta_y = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * dt
        delta_theta = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Wrap the angle theta to stay within [-pi, pi]
        self.theta = self.wrap_angle(self.theta)

        # Create and publish the Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = omega

        self.odom_publisher.publish(odom_msg)

        # Create and publish the TF transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = odom_msg.pose.pose.orientation.z
        t.transform.rotation.w = odom_msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def wrap_angle(angle):
        """Wrap the given angle to the range [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

def main(args=None):
    rclpy.init(args=args)
    mecanum_odometry_node = MecanumOdometryNode()

    try:
        rclpy.spin(mecanum_odometry_node)
    except KeyboardInterrupt:
        pass

    mecanum_odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
