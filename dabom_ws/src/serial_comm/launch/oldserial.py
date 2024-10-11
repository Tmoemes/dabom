from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
import numpy as np
import tf2_ros
import serial 
import math
import rclpy

from math_utils import euler_to_quaternion, z_rotation_matrix
from kinematics_mechanum_wheel import KinematicMechanumWheel
from variance_calculations import var_gearbox_backlash, var_resolution
from general_utils import convert_serial_data_to_angular_velocities, fill_odometry_message
import time

class KinOdomProcessing(Node):

    def __init__(self) -> None:
        super().__init__('kin_odom_publisher')
        # Setup publishing and subscriptions
        timer_period = 0.1
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.kinematics_callback, 10)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(timer_period, self.odom_timer_callback)
        # Setup for wheel object instance
        y_to_wheel = (15/100)
        x_to_wheel =  (15/100)
        self.x = 0
        self.y = 0
        self.theta = 0
        radius = ((8/2)/100) 
        self.radius = radius
        self.baud = 115200
        self.max_angular_velocities = 60
        self.max_output_angular_velocities = 11
        angle_from_wheels = np.pi/2
        self.wheel = KinematicMechanumWheel(y_to_wheel, x_to_wheel, radius, angle_from_wheels)
        self.old_ang_velocity = np.array([0, 0, 0, 0])
        # Setup serial port
        self.serial = serial.Serial("/dev/serial0", self.baud, timeout=0.01, write_timeout=0.01)
        # Setup timestamps for delta time calculations
        self.last_time = self.get_clock().now()
        # Calculate constnat variances for covariance matrix for odometry message
        self.var_encoding = var_resolution(self.radius, 1440)

    def tf_publish(self, current_time):
        print('publishing tf')
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.y
        t.transform.translation.y = self.x
        t.transform.translation.z = 0.0

        orientation =  euler_to_quaternion(0, 0, self.theta)
        t.transform.rotation.x = orientation[1] 
        t.transform.rotation.y = orientation[2] 
        t.transform.rotation.z = orientation[3] 
        t.transform.rotation.w = orientation[0] 

        self.tf_broadcaster.sendTransform(t)      

    def odom_timer_callback(self):
        serial_read_back = None
        try:
            serial_read_back = self.serial.readline().strip()
        except Exception as e:
            self.serial = serial.Serial("/dev/serial0", self.baud)
            print("Exception:",e)
        if serial_read_back:
            potential_ang_velocities = convert_serial_data_to_angular_velocities(serial_read_back, self.get_logger())
            if potential_ang_velocities is not None:
                # only update time when we are sure we have received a valid message. Otherwise we would be missing distance, 
                # Since we did not receive the correect velocities we didnt update the position of the robot
                current_time = self.get_clock().now()
                delta_time = (current_time - self.last_time).nanoseconds / 1e9
                # it's not potential anguluar velocity it is angular velocity
                ang_velocities = (2*np.pi*((potential_ang_velocities-self.old_ang_velocity)) / 1440) / delta_time
                self.old_ang_velocity = potential_ang_velocities
                #self.get_logger().info(f"encoder ang velocities {ang_velocities}")
                robot_velocities = self.wheel.calculate_robot_velocities(ang_velocities)
                #self.get_logger().info(f"robot velocities: {robot_velocities}")

                self.update_position_with_odometry(delta_time, robot_velocities)
                odom = fill_odometry_message(self.x, self.y, self.theta, current_time, robot_velocities)
                #var_gearbox_backlash = var_gearbox_backlash(ang_velocities, np.radians(0.5), self.wheel)

                ##variances = self.var_encoding + var_gearbox_backlash
                # odom.pose.covariance = [variances[0], 0, 0, 0, 0, 0,
                #                         0, variances[1], 0, 0, 0, 0,
                #                     0, 0, 99999, 0, 0, 0,
                #                     0, 0, 0, 99999, 0, 0,
                #                     0, 0, 0, 0, 99999, 0,
                #                     0, 0, 0, 0, 0, variances[2]]  

                # odom.twist.covariance = [variances[0], 0, 0, 0, 0, 0,
                #                      0, variances[1], 0, 0, 0, 0,
                #                      0, 0, 99999, 0, 0, 0,
                #                      0, 0, 0, 99999, 0, 0,
                #                      0, 0, 0, 0, 99999, 0,
                #                      0, 0, 0, 0, 0, variances[2]]  

                self.odom_publisher.publish(odom)
                self.tf_publish(current_time)
                self.last_time = current_time

    def update_position_with_odometry(self, delta_time, robot_velocities):
        # delta_x = robot_velocities[0] * delta_time
        # delta_y = robot_velocities[1] * delta_time
        # delta_theta = robot_velocities[2] * delta_time
        # self.theta += delta_theta
        # self.x += delta_y * math.cos(self.theta) - delta_x * math.sin(self.theta)
        # self.y += delta_y * math.sin(self.theta) + delta_x * math.cos(self.theta)
        vx = robot_velocities[0]
        vy = robot_velocities[1]
        vtheta = robot_velocities[2]
        delta_y = (vx * math.cos(self.theta) - vy * math.sin(self.theta)) * delta_time
        delta_x = (vx * math.sin(self.theta) + vy * math.cos(self.theta)) * delta_time
        # Optional quadrant-based sign adjustments
        print(f'theta: {np.rad2deg(self.theta)}')
        if np.pi / 2 <= self.theta < np.pi:  # Second quadrant (90 to 180 degrees)
            pass
            #delta_x = -delta_x
  #          tmp = delta_x
 #           delta_x = -delta_y  # Cos is negative in second quadrant
#            delta_y = tmp 
        elif np.pi <= self.theta < 3*np.pi / 2:  # Third quadrant (180 to 270 degrees)
            print("DEBUG")
            delta_x = delta_x  # Cos is negative
            delta_y = delta_y  # Sin is negative in the third quadrant
        elif np.pi / 2 <= self.theta < 0:  # Fourth quadrant (270 to 360 degrees or -90 to 0 degrees)
            delta_y = -delta_y  # Sin is negative
        
        # Continue for other quadrants if necessary
        self.x += delta_x
        print(delta_x)
        self.y += delta_y
        print(delta_y)
        self.theta += vtheta*delta_time
        self.theta = np.mod(self.theta, 2 * np.pi)



    def kinematics_callback(self, msg):
        start = time.time()
        velocities = np.array(
            [msg.linear.x, msg.linear.y, 3*msg.angular.z]
            )
        wheel_ang_velocities = self.wheel.calculate_wheel_velocities(velocities)
        wheel_ang_velocities = wheel_ang_velocities / (self.max_angular_velocities/self.max_output_angular_velocities)
        # 1st motor front left first value 
        # 2nd motor front right second
        # 3rd motor back left third
        # 4th motor back right fourth
        for i in range(4):
            serial_message = f'm {i} {wheel_ang_velocities[i]} \n'.encode()
            try:
                self.serial.write(serial_message)
            except Exception as e:
                self.serial = serial.Serial("/dev/serial0", self.baud)
                print(f"{e}")
                break
        end = time.time()

def main(args=None):
    rclpy.init(args=args)
    node = KinOdomProcessing()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()