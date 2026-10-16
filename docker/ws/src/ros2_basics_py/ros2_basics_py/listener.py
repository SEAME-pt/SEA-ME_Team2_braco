"""Subscriber with an explicit QoS profile."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros2_basics_py.talker import CHATTER_QOS
from std_msgs.msg import String


class Listener(Node):

    def __init__(self):
        super().__init__('listener_py')
        self._sub = self.create_subscription(String, 'chatter', self._on_message, CHATTER_QOS)

    def _on_message(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = Listener()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
