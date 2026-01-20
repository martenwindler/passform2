import rclpy
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor

from passform_skills.nodes import PrimitiveLifecycle

import passform_msgs.msg
from passform_msgs.action import Passform

class ExamplePrimitive(PrimitiveLifecycle):
    """Minimal action server that processes one goal at a time."""

    def __init__(self):
        super().__init__('skill_driver')

    def execute_skill(self, goal_handle) -> Passform.Result:
        """Execute the goal."""

        # EXECUTION
        while rclpy.ok():
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                self.get_logger().warn('Skill execution aborted')
                return Passform.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Skill execution canceled')
                return Passform.Result()

            feedback_msg = Passform.Feedback()
            goal_handle.publish_feedback(feedback_msg)
            break

        # SUCCESS
        goal_handle.succeed()
        result = Passform.Result()
        self.get_logger().info('Skill execution succeeded')
        return result

    def estimate_duration(self, task:passform_msgs.msg.Task) -> Duration:
        """Provides an estimate for skill performance duration"""
        return Duration(seconds=5, nanoseconds=0)

    def is_reachable(self, location) ->  bool:
        return True

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()  # to handle multiple callback_groups
    node = ExamplePrimitive()

    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
