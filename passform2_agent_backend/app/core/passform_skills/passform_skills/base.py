import rclpy
import threading
from pathlib import Path

from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import qos_profile_services_default
from rclpy.time import Duration

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Importe aus deinem neuen Paket-Struktur
from .relay import ActionRelay
from .mixins import SkillRequestMixin  # Umbenannt von canRequestSkills

# Nachrichten-Typen
from passform_msgs.action import Passform
import passform_msgs.msg
import geometry_msgs.msg

# Utilities (Pfade müssen ggf. an deine neue Struktur angepasst werden)
from passform_util.registration import DiscoverPassform
from passform_util.types.skills import Skill

class PrimitiveLifecycle(Node):
    """Basis-Klasse für alle Single-Skills (Primitive)."""

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

        # Parameter
        self.declare_parameter('skill_file', rclpy.Parameter.Type.STRING)
        self.declare_parameter('uuid', rclpy.Parameter.Type.STRING)
        self.declare_parameter('request_topic', '/skill_request')
        self.declare_parameter('cost_topic', '/skill_response')
        self.declare_parameter('number_of_limit_points', 4)

        # Interne States
        self.basyx = None
        self.skill = None
        self.hardware_id = None
        self._goal_handle = None
        self._goal_lock = threading.Lock()

        # Interfaces
        self._action_server = None
        self._request_listener = None
        self._cost_publisher = None

        # Transformationen
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.no_of_reach_limits = self.get_parameter('number_of_limit_points').value

    def _create_interfaces(self) -> None:
        """Initialisiert ActionServer und Topic-Interfaces."""
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
            passform_msgs.msg.Task,
            self.get_parameter('request_topic').value,
            self.skill_request_callback,
            qos_profile_services_default
        )
        self._cost_publisher = self.create_publisher(
            passform_msgs.msg.Cost,
            self.get_parameter('cost_topic').value,
            qos_profile_services_default
        )

    # --- Action Server Callbacks ---

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT if self.is_active() else GoalResponse.REJECT

    def handle_accepted_callback(self, goal_handle):
        self.abort_execution(goal_handle)
        goal_handle.execute()

    def cancel_callback(self, goal):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def abort_execution(self, goal_handle=None):
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().warn('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle

    def execute_skill(self, goal_handle) -> Passform.Result:
        """Muss von der Kind-Klasse überschrieben werden."""
        self.get_logger().error('execute_skill() not implemented!')
        goal_handle.abort()
        return Passform.Result()

    # --- Skill Logik & Cost ---

    def skill_request_callback(self, msg: passform_msgs.msg.Task):
        try:
            if self._is_task_fulfillable(msg):
                cost_msg = self._calculate_cost(msg)
                self._cost_publisher.publish(cost_msg)
        except Exception as err:
            self.get_logger().warn(f'Cost calculation failed: {err}')

    def _is_task_fulfillable(self, task: passform_msgs.msg.Task) -> bool:
        if task.type.skill_type != int(self.skill.get_skilltype()):
            return False
        return self._task_reachable(task)

    def _task_reachable(self, task: passform_msgs.msg.Task) -> bool:
        locs = [task.condition.start, *task.condition.waypoint, task.condition.end]
        return all([self.is_reachable(loc) for loc in locs])

    def is_reachable(self, location) -> bool:
        """Standardmäßig True, sollte überschrieben werden."""
        return True

    def _calculate_cost(self, task: passform_msgs.msg.Task) -> passform_msgs.msg.Cost:
        cost = passform_msgs.msg.Cost()
        cost.duration = self.estimate_duration(task).to_msg()
        cost.earliest_start = self.get_clock().now().to_msg()
        cost.skill_uuid = self.skill.identification.id
        cost.action_topic = f'{self.get_namespace()}/{self.skill.id_short}'.replace('//','/')
        cost.request_id = task.request_id
        return cost

    def estimate_duration(self, task) -> Duration:
        raise NotImplementedError("Skills must implement estimate_duration")

    # --- Lifecycle Callbacks ---

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        try:
            self.hardware_id = self.get_parameter('uuid').value
            self.basyx = DiscoverPassform(self).start_basyx()
            
            # Skill laden
            param_file = self.get_parameter('skill_file').value
            self.skill = Skill.from_file(param_file)
            
            # AAS Registrierung
            self.basyx.add(self.skill, self.hardware_id)
            
            self._create_interfaces()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Configuration failed: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        if self.skill:
            self.basyx.discard(self.hardware_id, self.skill.id_short)
        return TransitionCallbackReturn.SUCCESS

    def is_active(self):
        return self._state_machine.current_state[1] == 'active'


class CompositeLifecycle(SkillRequestMixin, PrimitiveLifecycle):
    """Basis-Klasse für zusammengesetzte Skills (Orchestrierung)."""

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)
        self.primitives: list[Skill] = []
        self.relay: ActionRelay = None

    def create_primitives(self) -> None:
        self.primitives = []
        param_file = Path(self.get_parameter('skill_file').value)

        for primitive_name in self.skill.get_primitives():
            primitive_file = param_file.parent / f'{str(primitive_name).lower()}.yaml'
            if not primitive_file.is_file():
                raise RuntimeError(f'Primitive file {primitive_file} missing')
            self.primitives.append(Skill.from_file(primitive_file))

    def create_action_clients(self):
        self.relay = ActionRelay(self)
        for p in self.primitives:
            self.relay.create_action_client(
                key=p.id_short,
                action_type=Passform,
                action_name=p.get_driver()
            )

    def execute_skill(self, goal_handle) -> Passform.Result:
        """Führt alle Sub-Skills nacheinander aus (Relay-Logik)."""
        last_result = Passform.Result()
        
        for primitive in self.primitives:
            self.relay.relay_goal(goal_handle, primitive.id_short)
            
            while rclpy.ok() and self.relay.is_active():
                if not goal_handle.is_active or goal_handle.is_cancel_requested:
                    self.relay.cancel()
                    self.get_logger().warn('Composite execution aborted/canceled')
                    return Passform.Result()
            
            last_result = self.relay.get_result()

        goal_handle.succeed()
        return last_result

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        # Erst die Basis (Primitive) konfigurieren
        if super().on_configure(state) != TransitionCallbackReturn.SUCCESS:
            return TransitionCallbackReturn.FAILURE
        
        try:
            self.create_primitives()
            self.create_action_clients()
            return TransitionCallbackReturn.SUCCESS
        except Exception as e:
            self.get_logger().error(f"Composite config failed: {e}")
            return TransitionCallbackReturn.FAILURE