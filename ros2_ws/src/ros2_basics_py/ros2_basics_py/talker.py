"""Publisher with an explicit QoS profile."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)
from std_msgs.msg import String

CHATTER_QOS = QoSProfile(
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)


class Talker(Node):

    def __init__(self):
        super().__init__('talker_py')
        rate_hz = self.declare_parameter('publish_rate_hz', 1.0).value
        self._pub = self.create_publisher(String, 'chatter', CHATTER_QOS)
        self._count = 0
        self._timer = self.create_timer(1.0 / rate_hz, self._on_timer)

    def _on_timer(self):
        msg = String()
        msg.data = f'Hello from Python: {self._count}'
        self._pub.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self._count += 1


def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
