import rclpy
import uuid
import copy

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_services_default
from rclpy.time import Time, Duration
from collections import OrderedDict

from rclpy.action import ActionClient
from rclpy.action.server import ActionServer, GoalStatus

from passform_util.types.skills import SkillType
import passform_msgs.msg
from passform_msgs.action import Passform

class canRequestSkills:
    """ROS2 Mixin to allow a node to request tasks.

    Needs a ROS Node as child, since node features are used.
    Due to some problems with super() usage of rclpy.node.Node (at least it seems to me) this Mixin
    only works as a first parent of a Node-child class.
    e.g. :
        class NewNode(canRequestSkills, Node):
            def __init__(self, node_name, **kwargs):
                super().__init__(node_name, **kwargs)
    """

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.task_request: OrderedDict[str, passform_msgs.msg.Cost]  # task_uuid, cost

    def task_request_uuid_callback(self, cost: passform_msgs.msg.Cost):
        """Callback for cost responses. This direct answers turns two topics into
        a service-like behavior.
        call: passform_msgs.msg.Task
        response: passform_msgs.msg.Cost (this function)

        :param cost: passform_msgs.msg.Cost
        """
        if len(self.task_request) == 0:
            return
        if cost.request_id in list(self.task_request.keys()):
            self.task_request[cost.request_id] = cost

    def get_final_cost(self) -> passform_msgs.msg.Cost:
        """Calculates the final cost based on all responses.

        Assumes the first element in self.task_request is the first task to be performed.
        """
        cost_msg = passform_msgs.msg.Cost()
        # initial data
        first_task_id = next(iter(self.task_request))
        earliest_start = Time.from_msg(self.task_request[first_task_id].earliest_start)
        duration_ns = 0
        energy = 0.0
        # gather response data
        for id, cost in self.task_request.items():
            earliest_start = min(earliest_start, Time.from_msg(cost.earliest_start))
            duration_ns += Duration.from_msg(cost.duration).nanoseconds
            # TODO: check duration while comparing earliest_start
            energy += cost.energy
        # data to message
        cost_msg.earliest_start = earliest_start.to_msg()
        cost_msg.duration = Duration(nanoseconds=duration_ns).to_msg()
        cost_msg.energy = energy
        return cost_msg

    def request_cost(self, task: passform_msgs.msg.Task) -> passform_msgs.msg.Cost:
        """Creates services to relay the cost request to all primitives.

        Will respond without id if request fails.
        The requests will be stored in order of their call. The first element in request has to have
        the earlist start time.
        """
        self.task_request = OrderedDict() # reset list
        # topics
        request_publisher = self.create_publisher(
            msg_type = passform_msgs.msg.Task,
            topic = self.get_parameter('request_topic').value,
            qos_profile = qos_profile_services_default,
        )
        cost_listener = self.create_subscription(
            msg_type = passform_msgs.msg.Cost,
            topic = self.get_parameter('cost_topic').value,
            qos_profile = qos_profile_services_default,
            callback_group = ReentrantCallbackGroup(),
            callback = self.task_request_uuid_callback,
        )
        try:
            # create list of all skill_types to be requested
            if hasattr(self, 'primitives'):
                # true for composite skills
                required_skills = [int(p.get_skilltype()) for p in self.primitives]
            else:
                required_skills = [SkillType(task.type.skill_type)]
            # create a request based on all required skill_types
            for skill in required_skills:
                request = copy.deepcopy(task)   # deep copy to prevent request_id clash
                request.request_id = str(uuid.uuid4())
                request.type.skill_type = int(skill)
                request_publisher.publish(request)
                self.get_logger().info(f'Sub-cost request: {request.request_id}')
                self.task_request[request.request_id] = None

            # wait while response are coming in
            clock = rclpy.clock.Clock()
            while rclpy.ok() and list(self.task_request.values()).count(None) > 0:
                clock.sleep_for(Duration(seconds=0.2))
                # TODO: Add timeout

            # fixed values
            cost_msg = self.get_final_cost()
            cost_msg.request_id = task.request_id
            cost_msg.skill_uuid = self.skill.identification.id
            cost_msg.action_topic = f'{self.get_namespace()}/{self.skill.id_short}'.replace('//','/')
        except Exception as err:
            self.get_logger().warn(f'Failed to request cost: {err}')
            cost_msg.request_id = ''
        finally:
            return cost_msg
