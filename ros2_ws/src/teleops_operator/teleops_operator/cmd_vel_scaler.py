import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class CmdVelScaler(Node):
    def __init__(self):
        super().__init__('cmd_vel_scaler')
        self.declare_parameter('scale_axis', 2)
        self._scale_axis = self.get_parameter('scale_axis').value
        self._scale = 1.0

        self._pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Twist, 'cmd_vel_raw', self._cmd_vel_callback, 10)
        self.create_subscription(Joy, 'joy', self._joy_callback, 10)

    def _joy_callback(self, msg: Joy):
        if self._scale_axis < len(msg.axes):
            self._scale = (msg.axes[self._scale_axis] + 1.0) / 2.0

    def _cmd_vel_callback(self, msg: Twist):
        out = Twist()
        out.linear.x = msg.linear.x * self._scale
        out.angular.z = msg.angular.z * self._scale
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelScaler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
