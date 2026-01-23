import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tf2_geometry_msgs.tf2_geometry_msgs import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from passform_util.registration import DiscoverPassform
from passform_util.types import Item

import passform_msgs.msg

class ProductPublisher(Node):
    """General registration interface to PassForM System. Used by all modules.
    
    Parameter:
        self.declare_parameter('name', rclpy.Parameter.Type.STRING)
        self.declare_parameter('uuid', rclpy.Parameter.Type.STRING)
        self.declare_parameter('source_frame', rclpy.Parameter.Type.STRING)
        self.declare_parameter('quantity', 1)
    Optional Parameter:
        translation.x (DOUBLE):
        translation.y (DOUBLE):
        translation.z (DOUBLE):
    """

    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs, allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True)
        
        # basyx
        self.basyx = DiscoverPassform(self).start_basyx()
        self.product = self.create_product()
        # self.basyx.add_aas( self.product )    # fails, because self.product: Item, which is not an AAS
        # TODO: add AAS for products to be added or make Item an AAS itself

        # status publisher
        self.status_publisher = self.create_publisher(passform_msgs.msg.Item, '/status/product', 10)
        self.pub_timer = self.create_timer(1, self._pub_status)

    def _pub_status(self):
        msg = self.product.to_msg()
        self.status_publisher.publish(msg)

    def create_product(self) -> Item:
        """Creates an Item Basyx class with some values set from ParameterValues.
        Uses the parameter 'name', 'uuid' and 'quantity' (defaults to 1).
        """
        return Item(
            name=self.get_parameter('name').value,
            uid=self.get_parameter('uuid').value,
            quantity=self.get_parameter_or('quantity', rclpy.Parameter('qty', rclpy.Parameter.Type.INTEGER, 1)).value,
            location=self.get_init_location()
        )

    def set_location(self, location: passform_msgs.msg.Location, source_frame: str=None):
        """Set locally stored location to new value.
        Teleports the item.
        """
        if isinstance(source_frame, str):
            location.header.frame_id = source_frame
        self.get_logger().debug(f'Set item location to {location}')
        self.product.set_location(location)

    def get_init_location(self) -> passform_msgs.msg.Location:
        """Returns a default location based on ParameterValues provided.
        Uses the parameter 'source_frame' and 'translation.x|y|z'.
        """
        location = passform_msgs.msg.Location()
        location.header.frame_id = self.get_parameter_or('source_frame',
            rclpy.Parameter('source_frame', rclpy.Parameter.Type.STRING, 'world')).value
        for coord in ['x', 'y', 'z']:
            try:
                setattr(location.pose.position, coord,
                    float(self.get_parameter('translation.'+coord).value)
                )
            except TypeError:
                # happens if a parameter is not declared. coordinate will remain at 0
                pass
        return location
        
def main():
    rclpy.init()
    node = ProductPublisher('product_publisher')

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
