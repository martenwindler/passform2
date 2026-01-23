# see https://github.com/ros2/demos/blob/rolling/lifecycle_py/lifecycle_py/talker.py for demo

from typing import Optional
import yaml

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import Node, State, TransitionCallbackReturn

import passform_msgs.msg
import passform_msgs.srv
from module_base.inventory import Item, ModuleInventory
import passform_util.types.inventory
from passform_util.basyx.connector import RestServerConnector
from passform_util.registration import DiscoverPassform


class InventoryManager(Node):
    """General registration interface to PassForM System. Used by all modules."""

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

        self.create_static_parameter()
        self._inventory_publisher : Optional[rclpy.publisher.Publisher] = None
        self._add_service : Optional[rclpy.service.Service] = None
        self._use_service : Optional[rclpy.service.Service] = None
        self._status_timer : Optional[rclpy.timer.Timer] = None
        self.basyx : Optional[RestServerConnector] = None

        self.base_services : list[str] = list()
        self.inventory = ModuleInventory()

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """ unconfigured -> inactive
        """
        try:
            # DISCOVER PASSFORM (blocks by waiting for base discovery service)
            discoverer = DiscoverPassform(self)
            self.basyx = discoverer.start_basyx()
            self.base_services = discoverer.get_services()
            self.add_storage_location()
            # create clients
            self.create_interfaces()

            return TransitionCallbackReturn.SUCCESS
        except Exception as err:
            self.get_logger().error(f"on_configure() failed: {err}.")
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """ inactive -> unconfigured
        """
        self.get_logger().debug('on_cleanup() is called.')
        self.destroy_interfaces()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        """ unconfigured / inactive / active -> finalized
        """
        self.get_logger().debug('on_shutdown() is called.')
        self.destroy_interfaces()
        return TransitionCallbackReturn.SUCCESS

    def publish_status(self) -> None:
        """Publish inventory"""
        self._inventory_publisher.publish(self.inventory.to_msg())

    def create_interfaces(self) -> None:
        """Create services, publisher, etc"""
        # services
        self._add_service = self.create_service(
            srv_type = passform_msgs.srv.UpdateInventory,
            srv_name = self.get_name()+'/add',
            callback = self._add_cb,
        )
        self._use_service = self.create_service(
            srv_type = passform_msgs.srv.UpdateInventory,
            srv_name = self.get_name()+'/use',
            callback = self._use_cb,
        )
        self._use_service = self.create_service(
            srv_type = passform_msgs.srv.UpdateInventory,
            srv_name = self.get_name()+'/get',
            callback = self._get_cb,
        )
        # publisher
        self._inventory_publisher = self.create_publisher(
            passform_msgs.msg.Inventory,
            'inventory',
            qos_profile = 10,
        )
        self._status_timer = self.create_timer(
            timer_period_sec = 1,
            callback = self.publish_status,
        )

    def _add_cb(self, req:passform_msgs.srv.UpdateInventory.Request, resp) -> None:
        self.get_logger().debug(str(req))
        try:
            req_item = Item.from_msg(req.item)
            new_item = self.inventory.add(req_item)
            # update basyx stock
            self.basyx.add(
                passform_util.types.inventory.Item.from_msg(
                    new_item.to_msg()),
                self.get_parameter('uuid').value,
                'inventory'
            )
            # create response
            resp.item = new_item.to_msg()
            resp.success = True
        except Exception as err:
            # TODO: a basyx fail will return False, but the local inventory will be updated.
            resp.message = str(err)
        finally:
            return resp

    def _get_cb(self, req:passform_msgs.srv.UpdateInventory.Request, resp) -> None:
        self.get_logger().debug(str(req))
        try:
            item = Item.from_msg(req.item)
            available_item = self.inventory.available(item.get_uid(), item.get_quantity(), partial=False)
            if available_item.get_quantity() != item.get_quantity():
                raise ValueError('Not enough items in stock')
            # create response
            resp.item = available_item.to_msg()
            resp.success = True
        except Exception as err:
            resp.message = str(err)
        finally:
            return resp

    def _use_cb(self, req:passform_msgs.srv.UpdateInventory.Request, resp) -> None:
        self.get_logger().debug(str(req))
        try:
            item = Item.from_msg(req.item)
            item_used = self.inventory.use(item.get_uid(), item.get_quantity(), partial=False)
            if item_used.get_quantity() != item.get_quantity():
                raise ValueError('Not enough items in stock')
            # update basyx stock
            self.basyx.add(
                passform_util.types.inventory.Item.from_msg(
                    self.inventory.get(item.get_uid()).to_msg()),
                self.get_parameter('uuid').value,
                'inventory'
            )
            # create response
            resp.item = item_used.to_msg()
            resp.success = True
        except Exception as err:
            resp.message = str(err)
        finally:
            return resp

    def add_storage_location(self):
        """Read a yaml file and add locations to storage manager"""
        try:
            param_file = self.get_parameter('storage_file').value
            with open(param_file, 'r') as file:
                locations = yaml.safe_load(file)
        except rclpy.exceptions.ParameterUninitializedException:
            self.get_logger().info( 'Storage file not provided. '
                'Using default values without driver-support.')
            locations = [{
                'aoi': {
                    'label': self.get_namespace(),
                    'uuid':self.get_parameter('uuid').value
                },
                'driver_topic': 'DRIVER_NOT_SUPPORTED'
            }]
        for loc in locations:
            storage = passform_msgs.msg.Location(
                aoi = passform_msgs.msg.AreaOfInterest(
                    label = loc['aoi']['label'],
                    uuid = loc['aoi']['uuid']
                ))
            driver = loc['driver_topic']
            self.get_logger().debug(f'Adding storage {storage} @ {driver}')
            self.inventory.add_storage(
                location=storage,
                driver_topic=driver
            )

    def destroy_interfaces(self) -> None:
        """Destroys all 'interfaces' like subscriber, services, ..."""
        for interface in [
            self._inventory_publisher,
            self._status_timer,
            self._add_service,
            self._use_service,
            self.basyx
        ]:
            try:
                interface.destroy()
            except AttributeError:
                # if node was never configured, the interface are still None
                pass

    def create_static_parameter(self):
        self.declare_parameter('uuid', rclpy.Parameter.Type.STRING)
        self.declare_parameter('storage_file', rclpy.Parameter.Type.STRING)

def main():
    rclpy.init()

    executor = MultiThreadedExecutor()  # to handle multiple callback_groups
    node = InventoryManager('inventory')
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
