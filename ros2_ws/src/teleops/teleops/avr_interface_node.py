import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32MultiArray
import serial


class AvrInterfaceNode(Node):
    def __init__(self):
        super().__init__('avr_interface_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.3)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('telemetry_timeout', 2.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.max_speed = self.get_parameter('max_speed').value
        self.timeout = self.get_parameter('timeout').value
        self.telemetry_timeout = self.get_parameter('telemetry_timeout').value

        self.ser = None
        self._open_serial()

        self.battery_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        self.encoder_pub = self.create_publisher(Int32MultiArray, 'encoder_ticks', 10)

        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )

        self.timer = self.create_timer(self.timeout, self.timeout_callback)
        self.last_cmd_time = self.get_clock().now()

        self.last_telemetry_time = self.get_clock().now()
        self.telemetry_timer = self.create_timer(1.0, self.telemetry_watchdog)

        self._reader_running = True
        self._reader_thread = threading.Thread(
            target=self._serial_reader, daemon=True
        )
        self._reader_thread.start()

        self.get_logger().info(
            f'AVR interface started: port={self.serial_port}, '
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

    def _serial_reader(self):
        while self._reader_running:
            if self.ser is None:
                break
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
                if not line:
                    continue
                self._process_incoming(line)
            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                break

    def _process_incoming(self, line: str):
        if line == 'READY':
            self.get_logger().info('AVR reports READY')
            self.last_telemetry_time = self.get_clock().now()
            return

        if line.startswith('S '):
            parts = line.split()
            if len(parts) == 4:
                try:
                    battery_mv = int(parts[1])
                    enc_left = int(parts[2])
                    enc_right = int(parts[3])
                except ValueError:
                    self.get_logger().warn(f'Bad telemetry values: {line}')
                    return

                self.last_telemetry_time = self.get_clock().now()

                voltage_msg = Float32()
                voltage_msg.data = battery_mv / 1000.0
                self.battery_pub.publish(voltage_msg)

                encoder_msg = Int32MultiArray()
                encoder_msg.data = [enc_left, enc_right]
                self.encoder_pub.publish(encoder_msg)
            else:
                self.get_logger().warn(f'Malformed telemetry: {line}')
            return

        self.get_logger().debug(f'Unknown AVR message: {line}')

    def telemetry_watchdog(self):
        if self.ser is None:
            return
        elapsed = (
            self.get_clock().now() - self.last_telemetry_time
        ).nanoseconds / 1e9
        if elapsed >= self.telemetry_timeout:
            self.get_logger().warn(
                f'No telemetry from AVR for {elapsed:.1f}s'
            )

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        left_vel = linear - (angular * self.wheel_base / 2.0)
        right_vel = linear + (angular * self.wheel_base / 2.0)

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
            self.get_logger().info(f'Serial command: {command.strip()}')

    def destroy_node(self):
        self._reader_running = False
        if self.ser is not None:
            self._send_motor_command(0, 0)
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AvrInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
