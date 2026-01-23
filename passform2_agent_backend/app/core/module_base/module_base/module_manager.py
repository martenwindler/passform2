# see https://github.com/ros2/demos/blob/rolling/lifecycle_py/lifecycle_py/talker.py for demo

from typing import Optional
import yaml

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.lifecycle import Node, State, TransitionCallbackReturn

from passform_util.types import Module
from passform_util.registration import DiscoverPassform
from passform_util.basyx.connector import RestServerConnector
import passform_util.network as nw

from nav2_msgs.srv import ManageLifecycleNodes
import passform_msgs.msg
from passform_msgs.srv import RegisterModule

class ModuleManager(Node):
    """General registration interface to PassForM System. Used by all modules."""

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

        self.client_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_static_parameter()

        self._status_publisher : Optional[rclpy.publisher.Publisher] = None
        self._inventory_subscription : Optional[rclpy.subscription.Subscription] = None
        self._registration_client : Optional[rclpy.client.Client] = None
        self._status_timer : Optional[rclpy.timer.Timer] = None
        self.basyx : Optional[RestServerConnector] = None
        self.base_services = list()
        self._inventory : list[passform_msgs.msg.Item] = list()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """
        unconfigured -> inactive
        :return:
          TransitionCallbackReturn.SUCCESS -> "inactive".
          TransitionCallbackReturn.FAILURE -> "unconfigured".
          TransitionCallbackReturn.ERROR or any uncaught exceptions -> "errorprocessing"
        """
        try:
            # DISCOVER PASSFORM (blocks by waiting for base discovery service)
            discoverer = DiscoverPassform(self)
            self.basyx = discoverer.start_basyx()
            self.base_services = discoverer.get_services()
            self.basyx.add_aas(self.create_module_aas())

            # create clients
            self.create_interfaces()

            return TransitionCallbackReturn.SUCCESS
        except Exception as err:
            self.get_logger().error(f"on_configure() failed: {err}.")
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """
        Activation registers the module at the master (basyx and hardware main) as
        well as starting the lc_managers.
        """
        # The default LifecycleNode callback is the one transitioning
        # LifecyclePublisher entities from inactive to enabled.
        # If you override on_activate(), don't forget to call the parent class method as well!!
        super().on_activate(state)
        try:
            resp = self.register_module()
            if not resp.success:
                raise RuntimeError(resp.message)
            self.manage_lifecycle_nodes(ManageLifecycleNodes.Request.STARTUP)
            return TransitionCallbackReturn.SUCCESS
        except Exception as err:
            self.get_logger().error(f"on_activate() failed: {err}.")
            return TransitionCallbackReturn.FAILURE


    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """
        Deactivation results in unregistration from master (basyx and hardware main)
        as well as stopping all connected lifecycles by resetting the lc_manager.
        """
        # Same reasong here that for on_activate().
        # These are the two only cases where you need to call the parent method.
        super().on_deactivate(state)
        try:
            self.manage_lifecycle_nodes(ManageLifecycleNodes.Request.RESET)
            resp = self.unregister_module()
            if not resp.success:
                self.get_logger().warn(f"on_deactivate() problem: {resp.message}.")
            return TransitionCallbackReturn.SUCCESS
        except Exception as err:
            self.get_logger().error(f"on_deactivate() failed: {err}.")
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """ inactive -> unconfigured
        """
        self.get_logger().debug('on_cleanup() is called.')
        self.destroy_interfaces()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """ unconfigured / inactive / active -> finalized
        Will shutdown the skill_manager, since skills might be running comming from active.
        """
        self.get_logger().debug('on_shutdown() is called.')
        self.manage_lifecycle_nodes(ManageLifecycleNodes.Request.SHUTDOWN)
        self.destroy_interfaces()
        return TransitionCallbackReturn.SUCCESS

    def register_module(self) -> RegisterModule.Response:
        """Calls registration service. Returns reponse."""
        self.get_logger().debug('Registering Module')
        return self.perform_registration_request(RegisterModule.Request.REGISTER)

    def unregister_module(self) -> RegisterModule.Response:
        """Calls unregistration service. Returns reponse."""
        self.get_logger().debug('Unregistering Module')
        return self.perform_registration_request(RegisterModule.Request.UNREGISTER)

    def registration_msg_body(
        self,
        registration_type: int
    ) -> RegisterModule.Request:
        """
        Create a ROS message for the registration request.
        """
        assert registration_type in [
            RegisterModule.Request.REGISTER,
            RegisterModule.Request.UNREGISTER
        ], f'Unknown registration type "{registration_type}".'
        req = RegisterModule.Request()
        req.reg_type = registration_type
        # module data
        req.module.uuid = self.get_parameter('uuid').value # FIXME
        req.module.name = self.get_parameter('name').value # FIXME
        req.module.description = self.get_parameter('description').value # FIXME
        req.module.supplier_name = self.get_parameter('supplier_name').value # FIXME
        req.module.serial_number = self.get_parameter('serial_number').value # FIXME
        # bay data
        bay_id = self.get_parameter('bay_id').value
        if self.get_parameter('virtual').value:
            req.id_type = 'VIRTUAL'
        elif bay_id > 0 :
            req.id = str(bay_id)
            req.id_type = 'FIXED'
        else:
            req.id = nw.get_ip()
            req.id_type = 'IP'
        self.get_logger().debug(f'Registration request: {req}.')
        return req

    def manage_lifecycle_nodes(self, transition: int ):
        """Call to start the connected lifecycle nodes via a ManageLifecycleNodes request.
        All lc_manager are assumed to be in the local namespace of the module_manager.
        Raises, if startup fails.

        :param transition: ManageLifecycleNodes command in [0..4]. At best, use with predefined values
            from request message (e.g. ManageLifecycleNodes.Request.STARTUP)
        """
        assert transition in range(0,5), 'ManageLifecycleNodes command must be in [0..4]'
        for lc_name in self.get_parameter('lc_manager_names').value:
            lc_manager_topic = (
                self.get_namespace() +'/'+lc_name+'/manage_nodes'
            )
            lc_manager_topic = lc_manager_topic.replace('//','/')
            lc_manager_service = self.create_client(
                srv_type = ManageLifecycleNodes,
                srv_name = lc_manager_topic,
                callback_group=ReentrantCallbackGroup()
            )
            if not lc_manager_service.wait_for_service(timeout_sec=2):
                raise RuntimeError(f'Local lc_manager on "{lc_manager_topic}" unavailable')
            req = ManageLifecycleNodes.Request()
            req.command = transition
            future = lc_manager_service.call_async(req)
            result = None # if rclpy.ok() fails result is already defined
            while rclpy.ok():
                if future.done():
                    result = future.result()
                    break
            if result is None or not result.success:
                raise RuntimeError('Failed to transition lc_manager')
        self.get_logger().debug('Performed lc_manager transition.')
        return

    def perform_registration_request(
        self,
        registration_type
    ) -> RegisterModule.Response:
        """
        Generic registration service call. Raises at failure.

        The message is created internally based on local data.
        :registration_type: RegisterModule.Request.TYPE
        """
        self.get_logger().debug('perform_registration_request() is called.')
        req = self.registration_msg_body(registration_type)
        result = RegisterModule.Response()
        self.get_logger().debug(f'Registration request: {req}')

        future = self._registration_client.call_async(req)
        self.get_logger().info('Waiting for registration response.')
        
        if not self._registration_client.wait_for_service(timeout_sec=3):
            result.message = 'Registration service not available'
            return result

        while rclpy.ok():
            if future.done():
                result = future.result()
                break

        self.get_logger().debug(f'Registration result: {result.message}')
        return result

    def create_interfaces(self) -> None:
        self._registration_client = self.create_client(
            RegisterModule,
            self.base_services['registration'],
            callback_group=self.client_cb_group
        )
        self._inventory_subscription = self.create_subscription(
            msg_type = passform_msgs.msg.Inventory,
            topic = 'inventory',
            qos_profile = 10,
            callback = self.set_inventory
        )
        self._status_publisher = self.create_publisher(
            passform_msgs.msg.ModuleStatus,
            '/module/status',
            10
        )
        self._status_timer = self.create_timer(
            timer_period_sec = 1,
            callback = self.publish_status,
        )

    def set_inventory(self, msg : passform_msgs.msg.Inventory):
        """Blindly sets the local inventory to data
        Use to relay the decentral inventory to /module/status
        """
        self._inventory = msg.data

    def publish_status(self) -> None:
        msg = passform_msgs.msg.ModuleStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.uuid = self.get_parameter('uuid').value
        msg.inventory = self._inventory
        self._status_publisher.publish(msg)

    def destroy_interfaces(self) -> None:
        """Destroys all 'interfaces' like subscriber, services, ..."""
        for interface in [
            self._registration_client,
            self._status_publisher,
            self._status_timer,
            self.basyx,
        ]:
            try:
                interface.destroy()
            except AttributeError:
                # if node was never configured, the interface are still None
                pass

    def create_module_aas(self) -> Module:
        properties_path = self.get_parameter('properties_file').value
        thumbnail = self.get_parameter('thumbnail').value
        # try to read read properties file
        try:
            with open(properties_path, 'r') as file:
                properties = yaml.safe_load(file)
        except Exception as err:
            self.get_logger().warn(f'Failed to read properties file "{properties_path}". {err}')
            properties = None

        module_aas = Module(
            unique_id = self.get_parameter('uuid').value,
            name = self.get_parameter('name').value,
            properties = properties,
            thumbnail = thumbnail
            )
        return module_aas

    def create_static_parameter(self):
        self.declare_parameter('uuid', rclpy.Parameter.Type.STRING)
        self.declare_parameter('bay_id', 0)
        self.declare_parameter('virtual', False)
        self.declare_parameter('name', 'missing-name')
        self.declare_parameter('description', 'missing-description')
        self.declare_parameter('supplier_name', 'missing-supplier')
        self.declare_parameter('serial_number', 'missing-serialno')
        self.declare_parameter('base_topic', '/base/base_manager')

        self.declare_parameter('properties_file', '')
        self.declare_parameter('thumbnail', '')
        self.declare_parameter('lc_manager_names', ['action/skill_manager/'])

def main():
    rclpy.init()

    executor = MultiThreadedExecutor()  # to handle multiple callback_groups
    node = ModuleManager('module_manager')
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
