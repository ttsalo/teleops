import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32MultiArray
from prometheus_client import Gauge, Counter, start_http_server


class MetricsNode(Node):
    def __init__(self):
        super().__init__('metrics_node')

        self.declare_parameter('metrics_port', 9101)
        self.declare_parameter('telemetry_timeout', 2.0)
        port = self.get_parameter('metrics_port').value
        self._telemetry_timeout = self.get_parameter('telemetry_timeout').value

        self._battery_gauge = Gauge(
            'teleops_battery_voltage_volts', 'Battery voltage in volts')
        self._enc_counter = Counter(
            'teleops_encoder_ticks_total', 'Cumulative encoder ticks', ['side'])
        self._cmd_linear = Gauge(
            'teleops_cmd_vel_linear_mps', 'cmd_vel linear.x in m/s')
        self._cmd_angular = Gauge(
            'teleops_cmd_vel_angular_radps', 'cmd_vel angular.z in rad/s')
        self._telemetry_age = Gauge(
            'teleops_telemetry_age_seconds', 'Seconds since last AVR telemetry')
        self._avr_ready = Gauge(
            'teleops_avr_ready', '1 if AVR telemetry is recent, else 0')

        # Pre-register label sets so they appear in /metrics before first message
        self._enc_counter.labels(side='left')
        self._enc_counter.labels(side='right')

        self._prev_enc = [None, None]
        self._last_battery_time = self.get_clock().now()

        start_http_server(port)
        self.get_logger().info(f'Prometheus metrics on :{port}/metrics')

        self.create_subscription(Float32, 'battery_voltage', self._battery_cb, 10)
        self.create_subscription(Int32MultiArray, 'encoder_ticks', self._encoder_cb, 10)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self.create_timer(1.0, self._update_staleness)

    def _battery_cb(self, msg):
        self._battery_gauge.set(msg.data)
        self._last_battery_time = self.get_clock().now()

    def _encoder_cb(self, msg):
        sides = ['left', 'right']
        for i, side in enumerate(sides):
            current = msg.data[i]
            prev = self._prev_enc[i]
            if prev is not None:
                delta = current - prev
                if delta > 0:
                    self._enc_counter.labels(side=side).inc(delta)
                # negative delta = AVR reboot/reset, skip silently
            self._prev_enc[i] = current

    def _cmd_vel_cb(self, msg):
        self._cmd_linear.set(msg.linear.x)
        self._cmd_angular.set(msg.angular.z)

    def _update_staleness(self):
        age = (self.get_clock().now() - self._last_battery_time).nanoseconds / 1e9
        self._telemetry_age.set(age)
        self._avr_ready.set(1.0 if age < self._telemetry_timeout else 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = MetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
