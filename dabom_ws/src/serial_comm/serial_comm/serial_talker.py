from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
import serial


class Serial_Talker(Node):

    def __init__(self):
        super().__init__('Serial_Talker')
        self.subscription = self.create_subscription(TwistStamped, '/arduino_vel', self.motor_vel_callback, 10)
        # self.subscription = self.create_subscription(TwistStamped, '/motor_velocities', self.motor_vel_callback, 10)
        self.publisher_ = self.create_publisher(TwistStamped, '/arduino_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.arduino_vel_callback)

        # port='/dev/serial0',  # Raspberry Pi 4 built-in serial port
        self.port='/dev/ttyUSB0'  # ch340 seral usb port
        self.baudrate=115200
        self.timeout = 0.01

        # Initialize serial communication
        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=self.timeout
        )

    def arduino_vel_callback(self):
        serial_read = None
        try:
            serial_read = self.serial_port.readline().decode()
        except serial.SerialException:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
        if serial_read:
            try:
                encoder_values = serial_read.split(',')
                for i in range(4):
                    encoder_values[i] = float(encoder_values[i])
            except ValueError:
                self.get_logger().error('Error parsing serial data')
                return
            msg = TwistStamped()
            msg.twist.linear.x = encoder_values[0]
            msg.twist.linear.y = encoder_values[1]
            msg.twist.linear.z = encoder_values[2]
            msg.twist.angular.x = encoder_values[3]
            self.publisher_.publish(msg)
        
            # Send the message over the serial port
            # self.send_serial_data(msg.data)

    def send_serial_data(self, data):
        if self.serial_port.is_open:
            try:
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
        self.get_logger().info('Received: "%s"' % msg.twist.linear.x)
        motor_vels = [msg.twist.linear.x, msg.twist.linear.y,msg.twist.linear.z, msg.twist.angular.x]
        # Send the message over the serial port
        for i in range(4):
            serial_message = F'm {i} {str(motor_vels[i])}\n'
            self.send_serial_data(serial_message)
        


def main(args=None):
    rclpy.init(args=args)

    serial_talker = Serial_Talker()

    rclpy.spin(serial_talker)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    serial_talker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()