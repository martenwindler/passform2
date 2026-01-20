import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import sw_watchdog_msgs.msg
import passform_msgs.msg
import passform_msgs.srv
import diagnostic_msgs.msg

import passform_base.data_manager
from passform_base import RegistrationRequest
from passform_util.types import Module
import passform_util.network as nw

class BaseManager(Node):
    """
    Node that performs general base management functions.
    Main purpose is handling of registration requests.
    """
    def __init__(self):
        super().__init__('base_manager')

        # multiple callback_groups are necessary to nest services
        watchdog_cb_group = MutuallyExclusiveCallbackGroup()
        service_cb_group = MutuallyExclusiveCallbackGroup()

        self.declare_parameter('status_topic', 'status')
        self.declare_parameter('bay_id', Parameter.Type.INTEGER_ARRAY)
        self.declare_basyxdb_parameter()

        self.service_names = {
            'registration': '/passform/register_module',
            'get_modules': '/passform/get_modules'
        }
        self.create_subscription(sw_watchdog_msgs.msg.Heartbeat,
                                '/module_heartbeat', self.heartbeat_cb, 10,
                                callback_group = watchdog_cb_group)

        self.create_service(passform_msgs.srv.Discover,
                                '/passform/discover', self.discover_cb,
                                callback_group = service_cb_group)
        self.create_service(passform_msgs.srv.RegisterModule,
                                self.service_names['registration'], self.registration_cb,
                                callback_group = service_cb_group)
        self.create_service(passform_msgs.srv.GetModules,
                                self.service_names['get_modules'], self.get_modules_cb,
                                callback_group = service_cb_group)
        self.create_data_manager()
        self.create_status_publisher()

    def create_data_manager(self) -> None:
        """Create local data storage with an uplink to basyx"""
        self.data_manager = passform_base.data_manager.DataManager(
            registry_host = self.get_parameter('basyx.registry_host').value
            )
        self.data_manager.init_bays(self.get_parameter('bay_id').value)
        self.get_logger().info('DataManager created.')

    def create_status_publisher(self) -> None:
        """Create all publisher"""
        topic = self.get_parameter('status_topic').value
        self.get_logger().info(f'Publishing base status to: {self.get_namespace()}/{topic}')
        self.status_msg = passform_msgs.msg.BaseStatus()
        self.bay_publisher = self.create_publisher(passform_msgs.msg.BaseStatus, topic, 10)
        self.timer_ = self.create_timer(2.0, self.publish_status_cb)

    def publish_status_cb(self) -> None:
        """Callback to publish the status message for base"""
        status_msg = passform_msgs.msg.BaseStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        for b in self.data_manager.get_physical_bays():
            bay_msg = passform_msgs.msg.Bay()
            bay_msg.bay_id = b.get_id()
            bay_msg.connection_status = b.get_connection_status()
            bay_msg.uuid = b.identification.id  # bay uuid
            bay_msg.module_uuid = b.get_module() # UUID of the module currently present in the bay
            bay_msg.supply = b.get_supply() # currently active supply (power, vacuum, ...)
            status_msg.bay_data.append(bay_msg)
        # msg.e_stop = True  # INCLUDE E_STOP DRIVER
        self.bay_publisher.publish(status_msg)

    def discover_cb(self,req,resp) -> passform_msgs.srv.Discover.Response:
        """Callback for discovery messages from modules.        
        Handles passform_msgs.srv.Discover service requests.
        """
        self.get_logger().debug(f'discover_cb() called with {req}')
        try:
            resp.server_description.append(
                diagnostic_msgs.msg.DiagnosticStatus(
                    name='services',
                    message='Available services.',
                    values = [diagnostic_msgs.msg.KeyValue(key=k,value=v) for k,v in self.service_names.items()]
                )
            )
            resp.server_description.append(
                diagnostic_msgs.msg.DiagnosticStatus(
                    name='basyx',
                    message='BaSyx configuration',
                    values = [
                        diagnostic_msgs.msg.KeyValue(
                            key='registry_host',
                            value=self.get_parameter('basyx.registry_host').value
                            ),
                        diagnostic_msgs.msg.KeyValue(
                            key='registry_type',
                            value=self.get_parameter('basyx.registry_type').value
                            )
                        ]
                )
            )
        except Exception as err:
            self.get_logger().warn(f'discover_cb() error: {err}')
        finally:
            return resp

    def registration_cb(self,req,resp) -> passform_msgs.srv.RegisterModule.Response:
        """Callback for (un-)registration requests from modules.
        Handles passform_msgs.srv.RegisterModule service requests and relays them to
        the corresponding callbacks.
        """
        self.get_logger().debug(f'registration_cb() called with {req}')
        try:
            resp.success = True
            if req.reg_type == passform_msgs.srv.RegisterModule.Request.REGISTER:
                resp = self.register_module_cb(req,resp)
            elif req.reg_type == passform_msgs.srv.RegisterModule.Request.UNREGISTER:
                resp = self.delete_module_cb(req,resp)
            else:
                raise KeyError(f'Unknown registration request type "{req.reg_type}".')
        except Exception as err:
            resp.success = False
            resp.message = str(err)
        finally:
            self.get_logger().debug(f'registration_cb() response: {resp}')
            return resp

    def register_module_cb(self, req, resp) -> passform_msgs.srv.RegisterModule.Response:
        """Callback for registration requests from modules -> REGISTRATION.
        Handles passform_msgs.srv.RegisterModule service requests
        """
        resp.success = True
        try:
            M = message_to_module(req.module)
            request = RegistrationRequest(req)
            resp.bay_id = self.data_manager.add_module(
                module = M,
                request = request,
                timeout_handler = lambda uuid: self.module_timeout_handler(uuid)
            )
            if request.is_virtual:
                self.get_logger().info(f'Added virtual module "{M.uuid}".')
            else:
                self.get_logger().info(f'Module "{M.uuid}" registered in bay {resp.bay_id}.')
        except Exception as err:
            self.get_logger().warn(f'Module "{req.module.uuid}" registration failed. {err}')
            resp.success = False
            resp.message = str(err)
        finally:
            self._print_info_log()
            return resp

    def delete_module_cb(self, req, resp) -> passform_msgs.srv.RegisterModule.Response:
        """Callback for registration requests from modules -> DELETION.
        Handles passform_msgs.srv.RegisterModule service requests
        """
        [resp.success, resp.message] = self.delete_module(req.module.uuid)
        return resp

    def get_modules_cb(self, req, resp) -> passform_msgs.srv.GetModules.Response:
        """GetModules service callback"""
        resp.uuid_list = self.data_manager.get_active_modules()
        return resp

    def heartbeat_cb(self, msg:sw_watchdog_msgs.msg.Heartbeat) -> None:
        """Reset watchdog for individual module"""
        self.get_logger().debug(f'Module "{msg.uuid}" heartbeat received.')
        self.data_manager.reset_watchdog(msg.uuid)

    def module_timeout_handler(self, uuid:str) -> None:
        """Handle module timeout. Will delete the module."""
        self.get_logger().warn(f'Module "{uuid}" timed out.')
        self.delete_module(uuid)

    def delete_module(self, uuid:str) -> tuple[bool, str]:
        """Delete module and remove information from datamanager"""
        success = True
        message = ''
        try:
            self.data_manager.delete_module(uuid)
            self.get_logger().info(f'Module "{uuid}" deleted.')
        except Exception as err:
            self.get_logger().warn(f'Module "{uuid}" deletion failed. {err}')
            success = False
            message = str(err)
        finally:
            self._print_info_log()
            return success, message

    def declare_basyxdb_parameter(self):
        """Set local ROS-node parameter relevant for basyx"""
        self.declare_parameter('basyx.registry_port', Parameter.Type.INTEGER)
        self.declare_parameter('basyx.registry_type', Parameter.Type.STRING)
        self.declare_parameter(
            'basyx.registry_host',
            nw.get_ip()+':'+str(self.get_parameter('basyx.registry_port').value)
        )

    def _print_info_log(self):
        self.get_logger().info(f'{self.data_manager.get_module_count()} modules registered.')
        self.get_logger().info(f'{self.data_manager.get_occuopied_bay_count()}/{self.data_manager.get_bay_count()} bays occupied.')

def message_to_module(msg:passform_msgs.msg.Module) -> Module:
    # description = msg.description
    # supplier_name = msg.supplier_name
    # serial_number = msg.serial_number
    return Module(
        unique_id = msg.uuid,
        name = msg.name,
    )

def main(args=None):
    rclpy.init(args=args)
    node = BaseManager()
    executor = MultiThreadedExecutor()  # to handle multiple callback_groups
    executor.add_node(node)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()
