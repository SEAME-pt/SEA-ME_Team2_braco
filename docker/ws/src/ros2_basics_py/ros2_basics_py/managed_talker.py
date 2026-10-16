"""Lifecycle (managed) talker: publishes on lifecycle_chatter only while Active."""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.lifecycle import Node, Publisher, State, TransitionCallbackReturn
from ros2_basics_py.talker import CHATTER_QOS
from std_msgs.msg import String


class ManagedTalker(Node):

    def __init__(self):
        super().__init__('managed_talker_py')
        self._pub: Publisher | None = None
        self._timer = None
        self._count = 0
        self.get_logger().info('created (Unconfigured)')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self._pub = self.create_lifecycle_publisher(String, 'lifecycle_chatter', CHATTER_QOS)
        self._timer = self.create_timer(1.0, self._on_timer)
        self.get_logger().info('on_configure: publisher and timer created')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_activate: publishing')
        return super().on_activate(state)       # activates the lifecycle publisher

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info('on_deactivate: paused')
        return super().on_deactivate(state)     # deactivates the lifecycle publisher

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._release()
        self.get_logger().info('on_cleanup: resources released')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._release()
        self.get_logger().info('on_shutdown')
        return TransitionCallbackReturn.SUCCESS

    def _release(self):
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._pub is not None:
            self.destroy_publisher(self._pub)
            self._pub = None

    def _on_timer(self):
        if self._pub is None or not self._pub.is_activated:
            return
        msg = String()
        msg.data = f'Lifecycle Python: {self._count}'
        self._count += 1
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ManagedTalker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
