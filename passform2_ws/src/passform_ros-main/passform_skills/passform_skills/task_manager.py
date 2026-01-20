"""PassForM centralized task manager

This node serves as the global receiver for passform skills. Once a skill is received,
the node sends a cost request as broad and waits for offers.
After a fixed amount of time, the optimal offers is picked and the skill relayed to the
corresponding ActionServer.

Attributes:
    request_topic (str): topic on which skill requests will be sent
    cost_topic (str): topic on which cost offers are received

Todo:
    * add proper cost comparison
"""

import threading
import uuid
import copy

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Duration

from passform_skills.mixins import canRequestSkills
from passform_skills.action_relay import ActionRelay

import passform_msgs.msg
import passform_msgs.srv
import passform_msgs.action
import std_msgs.msg

class TaskManager(canRequestSkills, Node):
    """Simple skill requester to relay passform actions to matching nodes
    """
    def __init__(self):
        super().__init__('task_manager')
        self.declare_parameter('request_topic', '/skill_request')
        self.declare_parameter('cost_topic', '/skill_response')
        self.declare_parameter('feedback_topic', '/task_feedback')
        self.declare_parameter('response_wait_time', 3.0)

        self.active_id : int = 0
        self.available_skills : list[passform_msgs.msg.Cost] = list()
        self.cost_search_active : bool = False
        self._request_msg : passform_msgs.msg.Task() = None

        self._goal_handle = None
        self._goal_lock = threading.Lock()

        # interfaces
        self._action_server = ActionServer(
            self,
            passform_msgs.action.Passform,
            '/perform_action',
            execute_callback=self.execute_skill,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self._request_publisher = self.create_publisher(
            msg_type = passform_msgs.msg.Task,
            topic = self.get_parameter('request_topic').value,
            qos_profile = rclpy.qos.qos_profile_services_default,
        )
        self._cost_listener = self.create_subscription(
            msg_type = passform_msgs.msg.Cost,
            topic = self.get_parameter('cost_topic').value,
            callback = self._cost_cb,
            qos_profile = rclpy.qos.qos_profile_services_default,
        )

        self._feedback_publisher = self.create_publisher(
            msg_type = std_msgs.msg.String,
            topic = self.get_parameter('feedback_topic').value,
            qos_profile = rclpy.qos.qos_profile_system_default,
        )
        self._feedback_timer = self.create_timer(
            timer_period_sec = 0.5,
            callback = self._publish_feedback,
            callback_group = ReentrantCallbackGroup()
        )

        self.get_logger().info('TaskManager active')

    def _cost_cb(self, msg: passform_msgs.msg.Cost):
        if not self.cost_search_active:
            return
        if self.response_is_valid(msg):
            self.available_skills.append(msg)

    def response_is_valid(self, msg: passform_msgs.msg.Cost):
        """Verify cost responses before adding them to local storage."""
        # response fits request
        if msg.request_id != self._request_msg.request_id:
            return False
        # response action is valid
        # TODO: add more complex topic availability verification
        if msg.action_topic == '':
            return False
        self.get_logger().info(f'Received valid offer for skill {msg.skill_uuid}')
        return True

    def request_cost(self, task: passform_msgs.msg.Task):
        """Publishes a global cost request and waits a while to allow modules to calculate their cost.
        
        The cost responses will be stored in a class variable, since the subscription works with a 
        callback.
        The uuid will be randomized to prevent confusion with the original skill
        :param task: task to be published with randomized uid.
        :param wait_sec: wait time to allow for some cost calculation in models
        """
        self.available_skills = list() # reset responses
        # copy request and set unique request ID
        self._request_msg = copy.deepcopy(task) # deep copy to prevent request_id clash
        self._request_msg.request_id = str(uuid.uuid4())
        # activate search and publish
        self.cost_search_active = True
        self.get_logger().info(f'Sending cost request: {self._request_msg.request_id}')
        self._request_publisher.publish(self._request_msg)

    def pick_optimal_skill(self, cost_list: list[passform_msgs.msg.Cost]) -> passform_msgs.msg.Cost:
        """Deactivates the search and picks the optimal skill from all cost msgs.
        Raises RuntimeError if no valid skill is received.
        """
        self.cost_search_active = False
        if len(self.available_skills) == 0:
            raise RuntimeError("No valid cost responses")
        return cost_list[0]

    # ACTION SERVER
    def goal_callback(self, goal_request):
        """Accept every request."""
        self.get_logger().debug('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Abort previous skill and start new skill"""
        self.abort_execution(goal_handle)
        goal_handle.execute()

    def abort_execution(self, goal_handle=None):
        """Abort the currently executed skill.
        Guarded to prevent cancelation while skill search.
        """
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().warn('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action.
        Guarded to prevent cancelation while skill search.
        """
        self.get_logger().info('Received cancel request')
        with self._goal_lock:
            return CancelResponse.ACCEPT

    def execute_skill(self, goal_handle) -> passform_msgs.action.Passform.Result:
        """Executes a passform skill by relaying the skill to available nodes.
        
        1. the task gets rebranded (new uuid) and broadcasted on a global request topic
        1b. other nodes calculate their availability and cost and respond on another global topic
        2. the most suitable skill offer gets picked
        3. the skill gets relayed to the winning node
        4. waiting for the external node to finish
        5. return external result
        """
        # 1. COST REQUEST
        self.request_cost(goal_handle.request.task)
        # 2. WAIT FOR RESPONSE AN PICK SKILL
        with self._goal_lock:
            # guard while waiting to prevent race condition in case of abrupt re-calls
            # async wait to allow for response            
            wait_time = self.get_parameter('response_wait_time').value
            self.get_logger().info(f'Waiting {wait_time} sec for cost responses')
            rclpy.clock.Clock().sleep_for(Duration(seconds=wait_time))    # async sleep and wait for cost responses
            try:
                winning_skill = self.pick_optimal_skill(self.available_skills)
            except RuntimeError as err:
                # pickin skill failed -> task fails, since no skill available
                self.get_logger().warn(f'Failed to pick skill: {err}. Task cancelled.')
                goal_handle.abort()
                return passform_msgs.action.Passform.Result()

        # 3. RELAY SKILL
        self.relay = ActionRelay(self)
        # local variabe to prevent destruction after completion, which might raise due to incomming feedback
        self.relay.create_action_client(
            key = winning_skill.skill_uuid,
            action_type = passform_msgs.action.Passform,
            action_name = winning_skill.action_topic,
        )
        self.relay.relay_goal(goal_handle, winning_skill.skill_uuid)

        self.active_id = 0
        for param in goal_handle.request.task.optional_parameter:
            if param.name == 'task_id':
                self.active_id = param.value.integer_value
                break
        
        # 4. WAIT WHILE EXECUTION
        while rclpy.ok() and self.relay.is_active():
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                self.relay.cancel()
                self.get_logger().warn('Skill execution aborted')
                return passform_msgs.action.Passform.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.relay.cancel()
                self.get_logger().info('Skill execution canceled')
                return passform_msgs.action.Passform.Result()

        # 5 RETURN
        self._set_success()
        result = self.relay.get_result()
        goal_handle.succeed()
        return result

    def _publish_feedback(self):
        if self._goal_handle is not None and self._goal_handle.is_active:
            self._feedback_publisher.publish(
                std_msgs.msg.String(data=str(self.active_id))
            )

    def _set_success(self):
        """if active_id is a counter, increase by 1"""
        if str(self.active_id).isdigit():
            self.active_id +=1

def main(args=None):
    rclpy.init(args=args)
    node = TaskManager()
    executor = MultiThreadedExecutor()  # to handle multiple callback_groups
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
