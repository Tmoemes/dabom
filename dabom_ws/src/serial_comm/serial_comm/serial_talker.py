import numpy as np
import struct
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
import serial

# Set to True to enable debug messages, False to disable
DEBUG_ENABLED = False

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

        # Setup publishers
        self.velocity_publisher_ = self.create_publisher(TwistStamped, '/arduino_vel', 1)
        self.encoder_publisher_ = self.create_publisher(JointState, '/enc_cont', 1)
        self.motor_state_publisher_ = self.create_publisher(JointState, '/joint_states', 1)
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
        self.position = [0.0, 0.0, 0.0, 0.0]  # Initialize positions for the motors
        self.last_time = self.get_clock().now()

    def read_serial_data(self):
        try:
            while self.serial_port.in_waiting > 0:
                start_marker = self.serial_port.read(1)
                if start_marker == b'\x02':  # START_MARKER
                    self.read_binary_data()
                elif start_marker == b'\x04':  # DEBUG_MARKER
                    self.read_debug_message()
                else:
                    # Unknown marker, discard or handle as needed
                    pass
        except serial.SerialException:
            self.get_logger().error('Serial communication error. Attempting to reconnect.')
            self.reconnect_serial()

    def read_binary_data(self):
        data_length = 16 + 1 + 1  # 16 bytes data + 1 byte checksum + 1 byte END_MARKER
        packet = self.serial_port.read(data_length)

        if len(packet) != data_length:
            self.get_logger().error('Incomplete binary data packet received.')
            return

        # Verify END_MARKER
        if packet[-1] != 0x03:
            self.get_logger().error('Invalid END_MARKER in binary data packet.')
            return

        # Extract data and checksum
        data_bytes = packet[:16]
        received_checksum = packet[16]

        # Calculate checksum
        calculated_checksum = 0
        for b in data_bytes:
            calculated_checksum ^= b

        if calculated_checksum != received_checksum:
            self.get_logger().error('Checksum mismatch in binary data packet.')
            return

        # Unpack the data
        encoder_counts = struct.unpack('<llll', data_bytes)

        # Store the encoder counts for publishing
        self.last_encoder_counts = list(encoder_counts)

        # Calculate angular velocities and positions
        angular_velocities = self.calculate_angular_velocities(encoder_counts)
        self.calculate_positions(encoder_counts)

        # Publish the velocities and encoder counts
        self.publish_velocities(angular_velocities)

        # Publish the motor state (position only)
        self.publish_motor_state()

    def read_debug_message(self):
        length_byte = self.serial_port.read(1)
        if not length_byte:
            self.get_logger().error('Failed to read debug message length.')
            return

        length = length_byte[0]

        # Read the debug message and END_MARKER
        message_bytes = self.serial_port.read(length + 1)
        if len(message_bytes) != length + 1:
            self.get_logger().error('Incomplete debug message received.')
            return

        # Verify END_MARKER
        if message_bytes[-1] != 0x03:
            self.get_logger().error('Invalid END_MARKER in debug message.')
            return

        # Extract the debug message
        debug_message = message_bytes[:-1].decode('utf-8', errors='replace')
        if DEBUG_ENABLED:
            self.get_logger().info(f'Arduino Debug: {debug_message}')

    def calculate_angular_velocities(self, encoder_counts):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds
        self.last_time = current_time

        # Ensure delta_time is non-zero to avoid division by zero
        if delta_time == 0:
            self.get_logger().warn('Delta time is zero, skipping velocity calculation.')
            return [0.0, 0.0, 0.0, 0.0]

        angular_velocities = []
        for i in range(4):
            delta_counts = encoder_counts[i] - self.last_encoder_counts[i]
            self.last_encoder_counts[i] = encoder_counts[i]

            # Calculate angular velocity (rad/s)
            angular_velocity = (delta_counts / self.pulses_per_rev) * 2 * self.PI / delta_time
            angular_velocities.append(angular_velocity)

        return angular_velocities

    def calculate_positions(self, encoder_counts):
        # Calculate position for each motor (in radians)
        for i in range(4):
            self.position[i] = (encoder_counts[i] / self.pulses_per_rev) * 2 * self.PI

    def publish_velocities(self, angular_velocities):
        # Publish angular velocities
        msg = TwistStamped()
        msg.twist.linear.x = angular_velocities[0]
        msg.twist.linear.y = angular_velocities[1]
        msg.twist.linear.z = angular_velocities[2]
        msg.twist.angular.x = angular_velocities[3]
        self.velocity_publisher_.publish(msg)

        # Publish encoder counts with timestamp
        encoder_msg = JointState()
        encoder_msg.header.stamp = self.get_clock().now().to_msg()  # Add timestamp
        encoder_msg.header.frame_id = "encoders"
        encoder_msg.name = ['encoder_1', 'encoder_2', 'encoder_3', 'encoder_4']  # Optional: Add names for each encoder
        encoder_msg.position = [float(count) for count in self.last_encoder_counts]  # Store encoder counts as positions
        self.encoder_publisher_.publish(encoder_msg)

    def publish_motor_state(self):
        # Publish motor state (position only) using JointState
        motor_state_msg = JointState()
        motor_state_msg.header.stamp = self.get_clock().now().to_msg()
        motor_state_msg.header.frame_id = "base_link"  # Set the frame of reference
        motor_state_msg.name = ['Wheel1_1', 'Wheel2_1', 'Wheel3_1', 'Wheel4_1']
        motor_state_msg.position = self.position
        self.motor_state_publisher_.publish(motor_state_msg)

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

                if DEBUG_ENABLED:
                    self.get_logger().info(f'Sent motor velocities: {motor_velocities}')
            except serial.SerialException:
                self.get_logger().error('Failed to send binary data over serial.')
                self.reconnect_serial()

    def reconnect_serial(self):
        try:
            self.serial_port.close()
            self.serial_port.open()
            self.get_logger().info('Serial port reconnected.')
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
