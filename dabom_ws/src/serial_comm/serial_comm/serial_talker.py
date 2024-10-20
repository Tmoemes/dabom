from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
import serial


class Serial_Talker(Node):

    def __init__(self):
        super().__init__('Serial_Talker')

        # Declare parameters with default values
        self.declare_parameter('timer_period', 0.2)
        self.declare_parameter('port', '/dev/ttyS0')
        self.declare_parameter('baudrate', 38400)
        self.declare_parameter('timeout', 0.01)
        self.declare_parameter('pulses_per_rev', 1440)
        self.declare_parameter('wheel_radius', 0.08)

        # Create a subscription to the /motor_vel topic
        self.subscription = self.create_subscription(TwistStamped, '/motor_vel', self.motor_vel_callback, 1)

        # Create a publisher to the /arduino_vel topic
        self.publisher_ = self.create_publisher(TwistStamped, '/arduino_vel', 1)
        
        # Get the timer period parameter and create a timer
        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        self.timer = self.create_timer(timer_period, self.arduino_vel_callback)

        # Get serial communication parameters
        self.port = self.get_parameter('port').get_parameter_value().string_value   
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value


        # Initialize serial communication
        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )

        # Initialize variables for velocity calculation
        self.last_time = self.get_clock().now()
        self.pulses_per_rev = self.get_parameter('pulses_per_rev').get_parameter_value().integer_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.last_encoder_values = [0, 0, 0, 0]

    def arduino_vel_callback(self):
        serial_read = None
        try:
            serial_read = self.serial_port.readline().strip()
            # self.get_logger().info('receiving: "%s"' % serial_read)
            
        except serial.SerialException:
            # Log an error if there is an issue reading from the serial port
            self.get_logger().error('Error reading from serial port')
            # Reinitialize the serial port
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
        if serial_read:
            try:
                # Parse the encoder values from the serial data
                encoder_values = str(serial_read)[2:-1].split(',')
                if len(encoder_values) != 4:
                    raise Exception('Incorrect number of encoder values')
                for i in range(4):
                    encoder_values[i] = float(encoder_values[i])
            except Exception as e:
                self.get_logger().error('Error parsing serial data')
                return
            # self.get_logger().info('read encoder val: "%s"' % encoder_values)
            calculated_velocities = self.calculate_velocities(encoder_values)
            # self.get_logger().info('calculated vel: "%s"' % calculated_velocities)
            msg = TwistStamped()
            msg.twist.linear.x = calculated_velocities[0]
            msg.twist.linear.y = calculated_velocities[1]
            msg.twist.linear.z = calculated_velocities[2]
            msg.twist.angular.x = calculated_velocities[3]
            self.publisher_.publish(msg)
            

    def calculate_velocities(self, encoder_values):
        current_time = self.get_clock().now()
        delta_time = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        angular_velocities = [0, 0, 0, 0]
        for i in range(4):
            encoder_diff = encoder_values[i] - self.last_encoder_values[i]
            angular_velocities[i] = (encoder_diff / self.pulses_per_rev) * 2 * 3.14159 / delta_time
        self.last_encoder_values = encoder_values
        return angular_velocities
        
    def send_serial_data(self, data):
        if self.serial_port.is_open:
            try:
                # self.serial_port.reset_output_buffer()
                # self.get_logger().info('sending: "%s"' % data.encode())
                self.serial_port.write(data.encode())
            except serial.SerialException:
                self.get_logger().error('Error writing to serial port')
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.timeout
                )

    def motor_vel_callback(self, msg):
        motor_vels = [msg.twist.linear.x, msg.twist.linear.y,msg.twist.linear.z, msg.twist.angular.x]
        # Send the message over the serial port
        for i in range(4):
            serial_message = f'm {i} {str(round(motor_vels[i],3))}\n'
            # self.get_logger().info('sending: "%s"' % serial_message)
            self.send_serial_data(serial_message)
        
def main(args=None):
    rclpy.init(args=args)

    serial_talker = Serial_Talker()

    rclpy.spin(serial_talker)

    serial_talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()