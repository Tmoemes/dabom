from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='serial_comm',
            executable='serial_talker',
            name='serial_talker',
            output='screen',
            parameters=[{
                'port': '/dev/serial0', # /dev/ttyUSB0 ch340 seral usb port # '/dev/serial0',  # Raspberry Pi 4 built-in serial port
                'baudrate': 115200,
                'timeout': 0.01,
                'pulses_per_rev': 1440,
                'wheel_radius': 0.08,
                'timer_period': 0.5
            }]
        )
    ])