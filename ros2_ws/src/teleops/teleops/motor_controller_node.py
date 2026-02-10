import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial


class MotorControllerNode(Node):
    def __init__(self):
        super().__init__('motor_controller_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.3)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('timeout', 0.5)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.timeout = self.get_parameter('timeout').value

        self.ser = None
        self._open_serial()

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        self.timer = self.create_timer(self.timeout, self.timeout_callback)
        self.last_cmd_time = self.get_clock().now()

        self.get_logger().info(
            f'Motor controller started: port={self.serial_port}, '
            f'baud={self.baud_rate}, wheel_base={self.wheel_base}, '
            f'max_speed={self.max_speed}'
        )

    def _open_serial(self):
        try:
            self.ser = serial.Serial(
                self.serial_port, self.baud_rate, timeout=1
            )
            self.get_logger().info(f'Opened serial port {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().warn(
                f'Could not open serial port {self.serial_port}: {e}. '
                'Commands will be logged only.'
            )
            self.ser = None

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive kinematics
        left_vel = linear - (angular * self.wheel_base / 2.0)
        right_vel = linear + (angular * self.wheel_base / 2.0)

        # Map m/s to PWM range [-255, 255]
        left_pwm = int(max(-255, min(255, (left_vel / self.max_speed) * 255)))
        right_pwm = int(max(-255, min(255, (right_vel / self.max_speed) * 255)))

        self._send_motor_command(left_pwm, right_pwm)

    def timeout_callback(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed >= self.timeout:
            self._send_motor_command(0, 0)

    def _send_motor_command(self, left: int, right: int):
        command = f'M {left} {right}\n'

        if self.ser is not None:
            try:
                self.ser.write(command.encode('ascii'))
            except serial.SerialException as e:
                self.get_logger().error(f'Serial write failed: {e}')
        else:
            self.get_logger().debug(f'Serial command: {command.strip()}')

    def destroy_node(self):
        if self.ser is not None:
            self._send_motor_command(0, 0)
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
