"""Action client, usage: ros2 run ros2_basics_py fibonacci_client 10."""
import sys

from action_msgs.msg import GoalStatus
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from seame_interfaces.action import Fibonacci


class FibonacciClient(Node):

    def __init__(self):
        super().__init__('fibonacci_client_py')
        self._client = ActionClient(self, Fibonacci, 'fibonacci')
        self.done = False

    def send_goal(self, order):
        self.get_logger().info('waiting for action server /fibonacci...')
        self._client.wait_for_server()
        goal = Fibonacci.Goal()
        goal.order = order
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected')
            self.done = True
            return
        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {list(feedback_msg.feedback.partial_sequence)}')

    def _on_result(self, future):
        response = future.result()
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Result: {list(response.result.sequence)}')
        else:
            self.get_logger().warning(f'Goal finished with status {response.status}')
        self.done = True


def main(args=None):
    argv = remove_ros_args(args if args is not None else sys.argv)
    order = int(argv[1]) if len(argv) > 1 else 10
    rclpy.init(args=args)
    node = FibonacciClient()
    node.send_goal(order)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
