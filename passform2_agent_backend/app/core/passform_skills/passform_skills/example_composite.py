import rclpy
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor

from passform_skills.nodes import CompositeLifecycle

import passform_msgs.msg
from passform_msgs.action import Passform

class ExampleComposite(CompositeLifecycle):
    """Minimal action server that processes one goal at a time."""

    def __init__(self):
        super().__init__('skill_driver')

    def estimate_duration(self, task:passform_msgs.msg.Task) -> Duration:
        """Provides an estimate for skill performance duration"""
        return Duration(seconds=5, nanoseconds=0)

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()  # to handle multiple callback_groups
    node = ExampleComposite()

    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
