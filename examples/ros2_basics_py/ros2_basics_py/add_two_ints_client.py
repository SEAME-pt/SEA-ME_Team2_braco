"""Service client, usage: ros2 run ros2_basics_py add_two_ints_client 2 3."""
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from ros2_basics_interfaces.srv import AddTwoInts


class AddTwoIntsClient(Node):

    def __init__(self):
        super().__init__('add_two_ints_client_py')
        self._client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for service /add_two_ints...')

    def send_request(self, a, b):
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        return self._client.call_async(request)


def main(args=None):
    argv = remove_ros_args(args if args is not None else sys.argv)
    if len(argv) != 3:
        print('usage: add_two_ints_client A B')
        return 1
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    future = node.send_request(int(argv[1]), int(argv[2]))
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info(f'Result: {future.result().sum}')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
