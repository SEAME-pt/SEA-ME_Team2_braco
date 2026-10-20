"""Action server: computes a Fibonacci sequence with feedback and cancellation."""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node

from seame_interfaces.action import Fibonacci

MAX_ORDER = 50


class FibonacciServer(Node):

    def __init__(self):
        super().__init__('fibonacci_server_py')
        self._server = ActionServer(
            self, Fibonacci, 'fibonacci',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    def _on_goal(self, goal_request):
        if not 1 <= goal_request.order <= MAX_ORDER:
            self.get_logger().warning(f'Rejecting goal: order={goal_request.order}')
            return GoalResponse.REJECT
        self.get_logger().info(f'Accepting goal: order={goal_request.order}')
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().info('Cancel requested')
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        feedback = Fibonacci.Feedback()
        feedback.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result(sequence=feedback.partial_sequence)

            feedback.partial_sequence.append(
                feedback.partial_sequence[i] + feedback.partial_sequence[i - 1])
            self.get_logger().info(f'Feedback: {list(feedback.partial_sequence)}')
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0)

        goal_handle.succeed()
        return Fibonacci.Result(sequence=feedback.partial_sequence)


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciServer()
    try:
        # Multi-threaded, so cancel requests are processed while a goal executes.
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
