#! /usr/bin/env python3

"""Xbox controller to ROS2 cmd_vel publisher."""

import pygame
import math
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class XboxController:
    """Handles Xbox controller input using Pygame."""

    DEADZONE = 0.15

    def __init__(self):
        """Initialize the Xbox controller and start the monitoring thread."""
        pygame.init()
        pygame.joystick.init()
        self.joystick = pygame.joystick.Joystick(0)
        self.LeftJoystickY = 0
        self.LeftJoystickX = 0
        self.RightJoystickX = 0

        self._monitor_thread = threading.Thread(target=self._monitor_controller, daemon=True)
        self._monitor_thread.start()

    def read(self):
        """Read and return joystick values with deadzone applied.

        Returns:
            list: A list of joystick values [x, y, right_x] with deadzone applied.
        """
        y = self.apply_deadzone(self.LeftJoystickY)  # Swapped x and y axes
        x = self.apply_deadzone(self.LeftJoystickX)  # Swapped x and y axes
        right_x = self.apply_deadzone(self.RightJoystickX)
        return [x, y, right_x]

    def apply_deadzone(self, value):
        """Apply deadzone to joystick value.

        Args:
            value (float): The joystick value to apply the deadzone to.

        Returns:
            float: The value after applying the deadzone threshold.
        """
        return value if abs(value) >= XboxController.DEADZONE else 0.0

    def _monitor_controller(self):
        """Monitor and update joystick values."""
        while True:
            pygame.event.pump()
            self.LeftJoystickY = self.joystick.get_axis(0)
            self.LeftJoystickX = self.joystick.get_axis(1)
            self.RightJoystickX = self.joystick.get_axis(3)


class MinimalPublisher(Node):
    """Publishes joystick input as geometry_msgs/Twist messages."""

    def __init__(self):
        """Initialize the publisher, Xbox controller, and set maximum values."""
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Set to 5 Hz (0.2 seconds)

        max_rpm = 100
        wheel_diameter = 0.08
        L = 0.3
        W = 0.3

        max_motor_speed_rad_s = (max_rpm * 2 * math.pi) / 60
        self.max_speed = max_motor_speed_rad_s * (wheel_diameter / 2)
        self.max_theta = self.max_speed / (L + W)

        self.joystick = XboxController()

    def timer_callback(self):
        """Read joystick input and publish corresponding Twist message."""
        x, y, theta = self.joystick.read()
        x = round(x * self.max_speed, 4)
        y = round(y * self.max_speed, 4)
        theta = round(theta * self.max_theta, 4)

        msg = Twist()
        msg.linear.x = -x
        msg.linear.y = -y
        msg.angular.z = -theta
        self.publisher_.publish(msg)


def main(args=None):
    """Initialize the ROS2 node and start spinning."""
    print("Xbox control ACTIVE!")
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
