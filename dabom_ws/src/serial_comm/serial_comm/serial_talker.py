import numpy as np
import struct
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
import serial


class SerialTalker(Node):
    def __init__(self):
        super().__init__('serial_talker')
        
        # Declare parameters
        self.declare_parameter('timer_period', 0.008)  # 125 Hz
        self.declare_parameter('port', '/dev/ttyArduinoMega')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.02)
        self.declare_parameter('pulses_per_rev', 1440)
        self.declare_parameter('wheel_radius', 0.04)

        # Setup publisher and subscriber
        self.publisher_ = self.create_publisher(TwistStamped, '/arduino_vel', 1)
        self.subscription = self.create_subscription(
            TwistStamped, '/motor_vel', self.motor_vel_callback, 1)

        # Timer for periodic checks
        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.timer = self.create_timer(timer_period, self.read_serial_data)

        # Serial port configuration
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Initialize serial communication
        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            write_timeout=0
        )

        # Velocities and encoder calculations
        self.pulses_per_rev = self.get_parameter('pulses_per_rev').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.PI = np.pi  # Use numpy's pi
        self.last_encoder_counts = [0, 0, 0, 0]
        self.last_time = self.get_clock().now()

    def read_serial_data(self):
        try:
            # Calculate the total number of bytes to read (4 longs x 4 bytes each)
            bytes_to_read = 4 * 4  # 4 encoders * 4 bytes per long

            # Check if enough bytes are available
            if self.serial_port.in_waiting >= bytes_to_read:
                # Read the bytes
                encoder_bytes = self.serial_port.read(bytes_to_read)

                # Unpack the bytes into 4 long integers (little-endian)
                encoder_counts = struct.unpack('<llll', encoder_bytes)

                # Calculate angular velocities
                angular_velocities = self.calculate_angular_velocities(encoder_counts)

                # Publish the velocities
                self.publish_velocities(angular_velocities)
        except serial.SerialException:
            self.get_logger().error('Serial communication error. Attempting to reconnect.')
            self.reconnect_serial()

    def calculate_angular_velocities(self, encoder_counts):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        angular_velocities = []
        for i in range(4):
            delta_counts = encoder_counts[i] - self.last_encoder_counts[i]
            self.last_encoder_counts[i] = encoder_counts[i]

            # Calculate angular velocity (rad/s)
            angular_velocity = (delta_counts / self.pulses_per_rev) * 2 * self.PI / delta_time
            angular_velocities.append(angular_velocity)

        return angular_velocities

    def publish_velocities(self, angular_velocities):
        msg = TwistStamped()
        msg.twist.linear.x = angular_velocities[0]
        msg.twist.linear.y = angular_velocities[1]
        msg.twist.linear.z = angular_velocities[2]
        msg.twist.angular.x = angular_velocities[3]
        self.publisher_.publish(msg)

    def motor_vel_callback(self, msg):
        motor_velocities = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z, msg.twist.angular.x]
        # Send motor velocities to Arduino as binary data
        self.send_binary_data(motor_velocities)

    def send_binary_data(self, motor_velocities):
        if self.serial_port.is_open:
            try:
                START_MARKER = 0x02  # STX
                END_MARKER = 0x03    # ETX

                # Pack the motor velocities as 4 little-endian float values
                data_bytes = bytearray()
                for v in motor_velocities:
                    data_bytes.extend(struct.pack('<f', v))  # '<f' for little-endian float

                # Calculate checksum (XOR of all data bytes)
                checksum = 0
                for b in data_bytes:
                    checksum ^= b

                # Construct the packet
                packet = bytearray()
                packet.append(START_MARKER)
                packet.extend(data_bytes)
                packet.append(checksum)
                packet.append(END_MARKER)

                # Send the packet
                self.serial_port.write(packet)
            except serial.SerialException:
                self.get_logger().error('Failed to send binary data over serial.')
                self.reconnect_serial()

    def reconnect_serial(self):
        try:
            self.serial_port.close()
            self.serial_port.open()
        except serial.SerialException:
            self.get_logger().error('Reconnection failed. Please check the connection.')


def main(args=None):
    rclpy.init(args=args)
    serial_talker = SerialTalker()
    rclpy.spin(serial_talker)
    serial_talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
