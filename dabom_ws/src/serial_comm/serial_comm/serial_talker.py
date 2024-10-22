import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import serial
import struct


class SerialTalker(Node):
    def __init__(self):
        super().__init__('serial_talker')
        
        # Declare parameters
        self.declare_parameter('timer_period', 0.008)  # 125 Hz
        self.declare_parameter('port', '/dev/ttyArduinoMega')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.02)
        self.declare_parameter('debug_enabled', False)  # Debug output configuration

        # Setup publisher
        self.encoder_publisher = self.create_publisher(TwistStamped, '/ard_enc', 1)

        # Timer for periodic checks
        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.timer = self.create_timer(timer_period, self.read_serial_data)

        # Serial port configuration
        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Debug configuration
        self.debug_enabled = self.get_parameter('debug_enabled').get_parameter_value().bool_value

        # Initialize serial communication
        self.serial_port = self.setup_serial()

    def setup_serial(self):
        """Initialize the serial communication with the Arduino."""
        try:
            serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                write_timeout=0
            )
            self.get_logger().info(f'Serial connection established on {self.port}')
            return serial_port
        except serial.SerialException:
            self.get_logger().error('Failed to connect to the serial port. Check the connection.')
            return None

    def read_serial_data(self):
        """Read data from the serial port and process incoming packets."""
        if not self.serial_port or not self.serial_port.is_open:
            self.get_logger().error('Serial port is not open.')
            return

        try:
            while self.serial_port.in_waiting > 0:
                start_marker = self.serial_port.read(1)
                if start_marker == b'\x02':  # START_MARKER for encoder data
                    self.process_encoder_data()
                elif start_marker == b'\x04' and self.debug_enabled:  # DEBUG_MARKER for debug messages
                    self.process_debug_message()
                else:
                    # Unknown marker, discard or handle as needed
                    pass
        except serial.SerialException:
            self.get_logger().error('Serial communication error. Attempting to reconnect.')
            self.reconnect_serial()

    def process_encoder_data(self):
        """Read and publish encoder data received from the serial port."""
        data_length = 16 + 1 + 1  # 16 bytes for encoder data + 1 byte checksum + 1 byte END_MARKER
        packet = self.serial_port.read(data_length)

        if len(packet) != data_length:
            self.get_logger().error('Incomplete encoder data packet received.')
            return

        if packet[-1] != 0x03:  # Verify END_MARKER
            self.get_logger().error('Invalid END_MARKER in encoder data packet.')
            return

        # Extract data and checksum
        data_bytes = packet[:16]
        received_checksum = packet[16]

        if not self.validate_checksum(data_bytes, received_checksum):
            self.get_logger().error('Checksum mismatch in encoder data packet.')
            return

        # Unpack the encoder data
        encoder_counts = struct.unpack('<llll', data_bytes)
        self.publish_encoder_data(encoder_counts)

    def process_debug_message(self):
        """Process debug messages from the serial port."""
        length_byte = self.serial_port.read(1)
        if not length_byte:
            self.get_logger().error('Failed to read debug message length.')
            return

        length = length_byte[0]
        message_bytes = self.serial_port.read(length + 1)

        if len(message_bytes) != length + 1:
            self.get_logger().error('Incomplete debug message received.')
            return

        if message_bytes[-1] != 0x03:  # Verify END_MARKER
            self.get_logger().error('Invalid END_MARKER in debug message.')
            return

        # Extract the debug message
        debug_message = message_bytes[:-1].decode('utf-8', errors='replace')
        self.get_logger().info(f'Arduino Debug: {debug_message}')

    def validate_checksum(self, data_bytes, received_checksum):
        """Validate the checksum of the data packet."""
        calculated_checksum = 0
        for b in data_bytes:
            calculated_checksum ^= b
        return calculated_checksum == received_checksum

    def publish_encoder_data(self, encoder_counts):
        """Publish the encoder data as a TwistStamped message."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = encoder_counts[0]
        msg.twist.linear.y = encoder_counts[1]
        msg.twist.linear.z = encoder_counts[2]
        msg.twist.angular.x = encoder_counts[3]
        self.encoder_publisher.publish(msg)

    def reconnect_serial(self):
        """Attempt to reconnect the serial port."""
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
