# see https://github.com/ros2/demos/blob/rolling/lifecycle_py/lifecycle_py/talker.py for demo

import rclpy
from pathlib import Path

from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import State, TransitionCallbackReturn

from passform_util.types import Skill
from passform_skills.nodes import PrimitiveLifecycle
from passform_skills.mixins import canRequestSkills
from passform_skills.action_relay import ActionRelay

from passform_msgs.action import Passform
import passform_msgs.msg

class CompositeLifecycle(canRequestSkills, PrimitiveLifecycle):
    """General skill interface to PassForM System. Used by all modules."""

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.primitives : list[Skill] = []
        self.relay: ActionRelay = None

    def create_primitives(self) -> None:
        """Create Skill Submodels for all required primitives (only for composite skills)
        Will erase previously stored primitives beforehand.
        """
        # initial cleanup most likely overkill, since this function is only used at beginning.
        self.primitives = list() # reset list
        param_file = Path(self.get_parameter('skill_file').value)

        for primitive in self.skill.get_primitives():
            primitive_file = param_file.parent / f'{str(primitive).lower()}.yaml'

            self.get_logger().debug(f'Looking for primitive file "{primitive_file}"')
            if not primitive_file.is_file():
                raise RuntimeError(f'Primitive data file "{primitive_file}" for {str(self.skill)} missing ')

            self.primitives.append( Skill.from_file(primitive_file) )

    def create_action_clients(self):
        """Create the action clients for all stored primitives
        """
        # initial cleanup most likely overkill, since this function is only used at beginning..
        self.relay = ActionRelay(self) # remove all old clients
        for primitive in self.primitives:
            self.relay.create_action_client(
                key = primitive.id_short,
                action_type = Passform,
                action_name = primitive.get_driver(),
            )

    def _calculate_cost(self, task:passform_msgs.msg.Task) -> passform_msgs.msg.Cost:
        """Calculate some key characteristics for a task exection
        """
        return self.request_cost(task)  # provided by canRequestCost class

    def _task_reachable(self, task:passform_msgs.msg.Task) -> bool:
        """Evaluates the reachability of all points provided in a task condition
        """
        condition = task.condition
        locations = [condition.start, *condition.waypoint, condition.end] # list of passform Location
        return all([self.is_reachable(loc) for loc in locations])

    def _remove_skill(self, logging:bool=False) -> None:
        """Removes a skill from basyx and local storage
        """
        super()._remove_skill(logging)
        self.primitives = []
        self.relay = None

    # ACTION SERVER
    def execute_skill(self, goal_handle) -> Passform.Result:
        """Goal exection by relaying the goal_handle to the individual action_clients.

        The order of relaying is specified by the order of primitives stored in the description.
        No parallel execution supported.
        Feedback is sent by the individual clients.
        """
        self.get_logger().warn(f'Using naïve primitive calls. Add execute_skill(self, goal_handle) -> Passform.Result')
        # EXECUTION
        for primitive in self.primitives:
        # for cost_response in self.task_request:
        # TODO: replace primitive walkthrough by cost-walkthrough
            self.relay.relay_goal(goal_handle, primitive.id_short)
            while rclpy.ok() and self.relay.is_active():
                # If goal is flagged as no longer active (ie. another goal was accepted),
                # then stop executing
                if not goal_handle.is_active:
                    self.relay.cancel()
                    self.get_logger().warn('Skill execution aborted')
                    return Passform.Result()

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.relay.cancel()
                    self.get_logger().info('Skill execution canceled')
                    return Passform.Result()

            # fetch intermediate result
            primitive_result = self.relay.get_result()

        # SUCCESS
        self.get_logger().info('Composite skill execution succeeded.')
        goal_handle.succeed()
        result = primitive_result
        return result

    ## LIFECYCLE
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Perform super().on_configure and append primitive creation
        """
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE
        try:
            self.create_primitives()
            self.create_action_clients()
            return TransitionCallbackReturn.SUCCESS
        except Exception as err:
            self.get_logger().error(f"on_configure() failed: {err}.")
            return TransitionCallbackReturn.FAILURE

def main():
    rclpy.init()

    executor = MultiThreadedExecutor()  # to handle multiple callback_groups
    node = CompositeLifecycle('composite_lifecycle')

    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
