# see https://github.com/ros2/demos/blob/rolling/lifecycle_py/lifecycle_py/talker.py for demo

import rclpy
import threading

from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_services_default
from rclpy.time import Time, Duration
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from passform_util.registration import DiscoverPassform
from passform_util.types.skills import Skill, Operation
from passform_util.basyx.connector import RestServerConnector

from passform_msgs.action import Passform
from tf2_geometry_msgs import PointStamped
import passform_msgs.msg
import geometry_msgs.msg

class PrimitiveLifecycle(Node):
    """General skill interface to PassForM System. Used by all modules."""

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

        self.declare_parameter('skill_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter('uuid', rclpy.Parameter.Type.STRING)
        self.declare_parameter('request_topic', '/skill_request')   # defaults to global topic
        self.declare_parameter('cost_topic', '/skill_response')   # defaults to global topic
        self.declare_parameter('pub_rate', 5)   # rate of skill publisher in sec
        self.declare_parameter('number_of_limit_points', 4)   # number of points limiting the reach

        self.basyx : RestServerConnector = None
        self.skill : Skill = None
        self.hardware_id : str = None

        self._action_server : ActionServer = None
        self._request_listener : rclpy.subscription.Subscription = None
        self._cost_publisher : rclpy.publisher.Publisher = None

        self._goal_handle = None
        self._goal_lock = threading.Lock()

        # transform buffer and reach limits for reachability analysis
        self.tf_buffer = Buffer(cache_time=Duration(seconds = 10.))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.no_of_reach_limits = self.get_parameter('number_of_limit_points').value

    ## LOGIC
    def create_skill(self) -> None:
        """ Create Skill Submodels for each skill in the skill file
        """
        param_file = self.get_parameter('skill_file').value
        self.skill = Skill.from_file(param_file)
        # basyx
        connected = False
        while not connected:
            try:
                self.basyx.add(self.skill, self.hardware_id)
                connected = True
            except Exception: # TODO: specify exception type
                delay = Duration(seconds=1)
                self.get_logger().warn(f'Problem reading AAS {self.hardware_id}. Retry in {delay}.')
                rclpy.clock.Clock().sleep_for(delay)
        tmp = self.basyx.get_submodel(self.hardware_id, self.skill.id_short) # confirm creation
        self.skill.parent = tmp.parent # update local copy to server side information
        # logging and internal storage
        self.get_logger().info(f'Created skill {str(self.skill)}.')

    def analyse_operations(self) -> None:
        """Checks if all operations are of type ROS. Prints a warning if not.
        """
        # TODO: could be way nicer
        self.get_logger().info(f'Validate skill {str(self.skill)}')
        try:
            # ensure all operations utilize ROS messages
            for operation in self.skill.get_operations():
                if not operation.is_ros():
                    self.get_logger().warn(f'operation "{operation.id_short}" not of ROS type')
                    # TODO: can we remove individual operations in basyx?
        except Exception as err:
            self.get_logger().warn(f'Failed to validate skill {str(self.skill)}: {err}')
            self._remove_skill(logging=True)

    def _destroy_interfaces(self) -> None:
        for interface in [
            self._action_server,
            self._request_listener,
            self._cost_publisher,
            self.basyx
        ]:
            try:
                interface.destroy()
            except AttributeError:
                # if node was never configured, the interface are still None
                pass

    def _create_interfaces(self) -> None:
        self._action_server = ActionServer(
            self,
            Passform,
            self.skill.get_driver(),
            execute_callback=self.execute_skill,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )
        self._request_listener = self.create_subscription(
            msg_type = passform_msgs.msg.Task,
            topic = self.get_parameter('request_topic').value,
            callback = self.skill_request_callback,
            qos_profile = qos_profile_services_default,
        )
        self._cost_publisher = self.create_publisher(
            msg_type = passform_msgs.msg.Cost,
            topic = self.get_parameter('cost_topic').value,
            qos_profile = qos_profile_services_default,
        )

    def skill_request_callback(self, msg:passform_msgs.msg.Task):
        """Handle the request to perform a specific task.
        """
        self.get_logger().debug(f'Received skill request for task {msg.type.skill_type} ({msg.type.name})')
        try:
            if not self._is_task_fulfillable(msg):
                return
            resp = self._calculate_cost(msg)
            self.get_logger().info(f'Offering the skill {str(self.skill)}')
            self._cost_publisher.publish(resp)
        except Exception as err:
            self.get_logger().warn(f'Failed to calculate skill cost. {err}')

    def _is_task_fulfillable(self, task:passform_msgs.msg.Task) -> bool:
        try:
            if task.type.skill_type != int(self.skill.get_skilltype()):
                self.get_logger().debug('Declined task due to unmatching type.')
                return False
            if not self._task_reachable(task):
                self.get_logger().debug('Declined task due to unreachable conditions.')
                return False
            if len(self.task_to_operation(task)) == 0:
                self.get_logger().debug('Declined task due no available operations.')
                return False
            return True
        except Exception as err:
            self.get_logger().warn(f'Error while calculating fulfillability: {err}.')
            return False

    def task_to_operation(self, task:passform_msgs.msg.Task) -> list[Operation]:
        """Prepares a list of potential operations to perform the requested task.

        List might be empty, if no operations fit the task.
        """
        eligible_operations = list()
        for operation in self.skill.get_operations():
            self.get_logger().info(f'Test operation {operation.id_short}')
            if operation.can_perform(task):
                eligible_operations.append(operation)
        return eligible_operations

    def _calculate_cost(self, task:passform_msgs.msg.Task) -> passform_msgs.msg.Cost:
        """Calculate some key characteristics for a task exection
        """
        cost_msg = passform_msgs.msg.Cost()
        # dynamic values
        cost_msg.duration = self.estimate_duration(task).to_msg()
        cost_msg.earliest_start = self.estimate_earliest_start(task).to_msg()
        cost_msg.energy = self.estimate_energy_in_watt(task)
        # fixed values
        cost_msg.skill_uuid = self.skill.identification.id
        cost_msg.action_topic = f'{self.get_namespace()}/{self.skill.id_short}'.replace('//','/')
        cost_msg.request_id = task.request_id
        return cost_msg

    def estimate_duration(self, task:passform_msgs.msg.Task) -> Duration:
        """Provides an estimate for skill performance duration
        This function needs an override.
        Raises as default.
        """
        raise RuntimeError('Skill duration esimation not implemented. Add estimate_duration().')

    def estimate_earliest_start(self, task:passform_msgs.msg.Task) -> Time:
        """Provides the earliest start time
        Defaults to now.
        """
        return self.get_clock().now()

    def estimate_energy_in_watt(self, task:passform_msgs.msg.Task) -> float:
        """Provides an estimate for skill energy consumption in watt
        This function needs an override.
        Defaults to 0 and a warning.
        """
        self.get_logger().warn('Skill energy consumption esimation not implemented.')
        return 0.

    def is_reachable(self, location:passform_msgs.msg.Location) -> bool:
        """Evaluates the reachability of a location
        This function needs an override.
        Defaults to True and a warning.
        """
        self.get_logger().warn('Reachability esimation not implemented.')
        # The following code is a rough idea for an implementation using shapely
        # Warning: The transforms are not unified to the same base-coordinates
        # import shapely.geometry
        # poly_points = list()
        # for tf in self.get_limits():
        #     poly_points.append( (tf.transform.translation.x, tf.transform.translation.y) )
        # point = shapely.geometry.Point(location.pose.position.x, location.pose.position.y)
        # polygon = shapely.geometry.Polygon(poly_points)
        # reachable = polygon.contains(point)
        # self.get_logger().info(f'Reach check: {point} in {polygon} = {reachable}')
        # return reachable
        return True

    def get_limits(self, limit_list:list[str]=None) -> list[geometry_msgs.msg.TransformStamped]:
        """Gets transform from world to reach limits.
        This function relies on the number of reach limits specified.
        :param limit_list: list of point names limiting the reach. Must span a proper polygon.
        """
        if limit_list is None:
            limit_list = [(f'reach_{idx}') for idx in range(1, self.no_of_reach_limits+1)]           
        return [self.get_local_transform(point) for point in limit_list]

    def get_local_transform(self, link: str, source_frame = 'world') -> geometry_msgs.msg.TransformStamped:
        """Getter for local transform buffer from world to /module_uuid/{link}.
        :param link: local name of link without module namespace.
        :param source_frame: source frame for transform calculation. default=world
        """
        uuid = self.get_parameter('uuid').value
        return self.get_transform(f'{uuid}/{link}',source_frame)

    def get_transform(self, link: str, source_frame = 'world') -> geometry_msgs.msg.TransformStamped:
        """Getter for local transform buffer from world {link}.
        :param link: name of link.
        :param source_frame: source frame for transform calculation. default=world
        """
        return self.tf_buffer.lookup_transform(
            target_frame = source_frame,
            source_frame = link,
            time = rclpy.time.Time(),
            timeout=Duration(seconds=1.0)
        )

    def location_to_transform(self, location: passform_msgs.msg.Location, source_frame = 'world') -> geometry_msgs.msg.TransformStamped:
        """Turn a location message into a global frame Transform.
        """
        # TODO: Doesnt support Quaternions and is na unreliable implementation
        tf = geometry_msgs.msg.TransformStamped()
        if location.aoi.uuid == '':
            # no uuid -> localization by stored pose
            loc_pose = geometry_msgs.msg.PointStamped()
            loc_pose.header = location.header
            loc_pose.point = location.pose.position
            pose_stamped =  self.tf_buffer.transform(source_frame,loc_pose)
            tf.header = pose_stamped.header
            for coord in ['x','y','z']:
                # lazy way to copy coordinates from PoseStamped to TransformStamped
                setattr(tf.transform.translation, coord, getattr(pose_stamped.point, coord))
            return tf
        return self.get_transform(location.aoi.uuid)

    def _task_reachable(self, task:passform_msgs.msg.Task) -> bool:
        """Evaluates the reachability of all points provided in a task condition
        """
        condition = task.condition
        locations = [condition.start, *condition.waypoint, condition.end] # list of passform Location
        return all([self.is_reachable(loc) for loc in locations])

    def _remove_skill(self, logging:bool=False) -> None:
        """Removes a skill from basyx and local storage
        """
        if self.skill is not None:
            self.basyx.discard(self.hardware_id, self.skill.id_short)
            if logging: self.get_logger().info(f'Removed skill {str(self.skill)}')
            self.skill = None

    # ACTION SERVER
    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        if not self.is_active():
            self.get_logger().info('Rejected goal because not active.')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """Abort previous skill and start new skill"""
        self.abort_execution(goal_handle)
        goal_handle.execute()

    def abort_execution(self, goal_handle=None):
        """Abort the currently executed skill"""
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().warn('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_skill(self, goal_handle) -> Passform.Result:
        """Execute the new skill goal.
        
        The implementation provided presents a basic sceleton to create a skill.
        Due to the included abort() this implementation will always fail though.
        Look at the code example and implement your own driver handling accordingly.
        """
        self.get_logger().warn(
            'No skill action implemented. '
            'Add execute_skill(self, goal_handle) -> Passform.Result'
        )
        goal_handle.abort() # this will fail the rest of the skill
        # 1. prepare
        # 2. execution loop
        # 2.1 interruption handling
        # 2.2 perform step
        # 2.3 send feedback
        # 3 setting succes
        # 4 return result

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

        # SUCCESS
        goal_handle.succeed()
        result = Passform.Result()
        return result

    ## LIFECYCLE
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """ unconfigured -> inactive
        """
        self.get_logger().debug("on_configure() is called.")
        try:
            self.hardware_id = self.get_parameter('uuid').value
            self.basyx = DiscoverPassform(self).start_basyx()
            self.create_skill()
            self.analyse_operations()
            self._create_interfaces()
            self.get_logger().debug("on_configure() succeeded.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as err:
            self.get_logger().error(f"on_configure() failed: {err}.")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """ inactive -> active
        """
        self.get_logger().debug('Skill activation succeeded.')
        return super().on_activate(state) # parent class needs a call

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """ active -> inactive
        """
        self.abort_execution()
        return super().on_deactivate(state) # parent class needs a call

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """ inactive -> unconfigured
        """
        self.get_logger().debug('on_cleanup() is called.')
        self._remove_skill()
        # self._destroy_interfaces() # this throws badly
        # exception: "rclpy._rclpy_pybind11.InvalidHandle: cannot use Destroyable because destruction was requested"
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """ unconfigured / inactive / active -> finalized
        """
        self.get_logger().debug('on_shutdown() is called.')
        self._remove_skill()
        self.abort_execution()
        return TransitionCallbackReturn.SUCCESS

    def destroy_node(self):
        """Perform all shutdown procedures and call parent destroy_node()
        """
        self.abort_execution()
        self._remove_skill()
        self._destroy_interfaces()
        # can we combine this with on_shutdown()?
        super().destroy_node()

    def is_active(self):
        """True, if lifecycle node is in status 3: active"""
        # TODO: use LifecycleState or other library variables for comparison
        # currently it is either
        # self._state_machine.current_state[0] == 3
        # self._state_machine.current_state[1] == active
        return self._state_machine.current_state[1] == 'active'

def main():
    rclpy.init()

    executor = MultiThreadedExecutor()  # to handle multiple callback_groups
    node = PrimitiveLifecycle('primitive_lifecycle')

    try:
        rclpy.spin(node, executor=executor)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
