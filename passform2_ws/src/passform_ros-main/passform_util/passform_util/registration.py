import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import rcl_interfaces.srv
from passform_msgs.srv import Discover

from passform_util.basyx.connector import RestServerConnector, ConnectorFactory

basyx_rest_parameter = [
    'registry_host'
]

class DiscoverPassform():

    def __init__(
        self,
        node : Node
    ):
        self._node = node
        self._discovery_performed = False

        self._response = Discover.Response()
        self._base_services = dict() # {service:topic}

        self.discover_passform()

    def start_basyx(self) -> RestServerConnector:
        """Starts a BasyxServer with the received information
        """
        if not self._discovery_performed:
            raise RuntimeError('PassForM discovery not performed.')

        basyx_data = self.get_basyx_data()
        if basyx_data['registry_type'].lower() != 'rest':
            raise RuntimeError(f'Only REST BaSyx supported, but registry_type is {basyx_data["registry_type"]}')
        return ConnectorFactory(basyx_data['registry_host'], node=self._node)

    def get_services(self) -> dict:
        return self._base_services

    def discover_passform(self) -> None:
        """Async call to the global DISCOVERY server
        """
        discover_topic = '/passform/discover'
        cb_group = None     # parallel tasks with inidiviual callback_groups
        active_spin = False # keeps the parent node spinning while waiting for a service response

        # differ between lifecylce nodes and normal nodes based on the state_machine of LC nodes
        if hasattr(self._node, '_state_machine'):
            # LC nodes need a specific cb group (dont know why)
            self._node.get_logger().debug(f'discover_passform() started for a lifecycle node.')
            cb_group = MutuallyExclusiveCallbackGroup()
        else:
            # normal nodes need to be kept spinning
            self._node.get_logger().debug(f'discover_passform() started for a normal node.')
            active_spin = True

        cli = self._node.create_client(Discover, discover_topic, callback_group=cb_group)
        self._node.get_logger().debug(f'discover_passform() client created on topic {discover_topic}.')
        while not cli.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise RuntimeError('Interrupted while waiting for base.')
            self._node.get_logger().info(f'PassForM Discovery Service "{discover_topic}" not available. Waiting...')

        req = Discover.Request()
        future = cli.call_async(req)
        while rclpy.ok():
            if active_spin: rclpy.spin_once(self._node)
            if future.done():
                self._response = future.result()
                break

        if len(self._response.server_description) == 0:
            raise RuntimeError('Received empty server description.')

        for ds in self._response.server_description:
            if ds.name.lower() == 'services':
                self._base_services = {kv.key:kv.value for kv in ds.values}

        self._node.get_logger().info('Discovery sucessful.')
        self._node.get_logger().debug(f'discovery response: {self._response}')
        self._discovery_performed = True

    def get_basyx_data(self) -> dict:
        for description in self._response.server_description:
            self._node.get_logger().debug(f'response: {self._response.server_description}.')
            if description.name.lower() == 'basyx':
                basyx_data = {kv.key:kv.value for kv in description.values}
                return basyx_data
        raise RuntimeError('Discovery message does not contain BaSyx information.')
