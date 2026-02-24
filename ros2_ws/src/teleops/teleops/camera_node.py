import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import rclpy
from rclpy.node import Node


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('receiver_host', '192.168.1.100')
        self.declare_parameter('receiver_port', 5600)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('bitrate', 2000)

        host = self.get_parameter('receiver_host').value
        port = self.get_parameter('receiver_port').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        framerate = self.get_parameter('framerate').value
        bitrate = self.get_parameter('bitrate').value

        Gst.init(None)

        pipeline_str = (
            f'libcamerasrc ! '
            f'video/x-raw,width={width},height={height},'
            f'framerate={framerate}/1 ! '
            f'videoconvert ! '
            f'x264enc tune=zerolatency bitrate={bitrate} '
            f'speed-preset=superfast ! '
            f'h264parse ! '
            f'rtph264pay config-interval=1 ! '
            f'udpsink host={host} port={port} sync=false'
        )

        self.get_logger().info(f'Pipeline: {pipeline_str}')
        self.pipeline = Gst.parse_launch(pipeline_str)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error('Failed to start GStreamer pipeline')
        else:
            self.get_logger().info(
                f'Camera streaming started: {width}x{height}@{framerate}fps '
                f'bitrate={bitrate}kbps → udp://{host}:{port}'
            )

        self.bus_timer = self.create_timer(1.0, self._poll_bus)

    def _poll_bus(self):
        bus = self.pipeline.get_bus()
        while True:
            msg = bus.pop()
            if msg is None:
                break
            t = msg.type
            if t == Gst.MessageType.ERROR:
                err, debug = msg.parse_error()
                self.get_logger().error(f'GStreamer error: {err.message}')
                self.get_logger().debug(f'GStreamer debug: {debug}')
            elif t == Gst.MessageType.WARNING:
                warn, debug = msg.parse_warning()
                self.get_logger().warn(f'GStreamer warning: {warn.message}')
            elif t == Gst.MessageType.EOS:
                self.get_logger().info('GStreamer end-of-stream')
            elif t == Gst.MessageType.STATE_CHANGED:
                if msg.src == self.pipeline:
                    old, new, pending = msg.parse_state_changed()
                    self.get_logger().debug(
                        f'Pipeline state: {old.value_nick} → {new.value_nick}'
                    )

    def destroy_node(self):
        self.get_logger().info('Shutting down camera pipeline')
        self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
