import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from tf2_geometry_msgs import PointStamped
from geometry_msgs.msg import Quaternion, Vector3
from passform_msgs.msg import Item, Location
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker

class ProductRelay(Node):
    """
    Publishes the static transforms of all products to the specified base_link.

    The transforms are created based on the global 'product/status' topic.
    Warning:
    - Items to not time out.
    - Quaternions are not supported.
    - Areas of Interest are not supported (raise NotImplementedError)
    """
    def __init__(self):
        super().__init__('product_relay')

        self.broadcaster = TransformBroadcaster(self)

        self.declare_parameter('status_topic', '/status/product')
        self.declare_parameter('base_link', 'world')
        self.declare_parameter('pub_rate', 1)
        self.create_subscription(Item, self.get_parameter('status_topic').value,
                                self.update_cache, 10)

        self.item_cache : dict[str, TransformStamped] = dict()
        self.marker_cache: dict[str, Marker] = dict()
        
        self.pub_timer = self.create_timer(1./self.get_parameter('pub_rate').value, self._send_transforms)
        self.tf_buffer = Buffer(cache_time=rclpy.time.Duration(seconds = 10.))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.source_frame = self.get_parameter('base_link').value

        self.marker_pub = self.create_publisher(MarkerArray, '/viz/product', 10)

    def update_cache(self, msg: Item) -> None:
        """Adds the position of a received item message to the cache.
        Recalculates the location of the item to world coordinate system.
        """
        try:
            tf = self.location_to_transform(msg.location)
            tf.child_frame_id = msg.uuid
            self.item_cache[msg.uuid] = tf
            self.marker_cache[msg.uuid] = self.tf_to_marker(tf)
        except TransformException as err:
            self.get_logger().info(f'Transformation for item {msg.uuid} failed. {err}')
        except NotImplementedError as err:
            self.get_logger().warn(f'Transformation for item {msg.uuid} failed. {err}')
        except Exception as err:
            self.get_logger().error(f'Transformation for item {msg.uuid} failed. {err}')

    def location_to_transform(self, location: Location) -> TransformStamped:
        """Turn a location message into a global frame Transform.
        """
        # TODO: Doesnt support Quaternions and is na unreliable implementation
        tf = TransformStamped()
        if location.aoi.uuid == '':
            # no uuid -> localization by stored pose
            loc_pose = PointStamped()
            loc_pose.header = location.header
            loc_pose.point = location.pose.position
            pose_stamped =  self.tf_buffer.transform(loc_pose, self.source_frame)
            tf.header = pose_stamped.header
            for coord in ['x','y','z']:
                # lazy way to copy coordinates from PoseStamped to TransformStamped
                setattr(tf.transform.translation, coord, getattr(pose_stamped.point, coord))
            return tf
        # TODO: add AOI transform
        # WARNING: the following code is not tested. Just here as an idea
        # return self.tf_buffer.lookup_transform(
        #     target_frame = location.aoi.uuid,
        #     source_frame = self.source_frame,
        #     time = rclpy.time.Time(),
        #     timeout=rclpy.time.Duration(seconds=1.0)
        # )
        raise NotImplementedError('Localization by AOI not implemented.')


    def _send_transforms(self):
        self.broadcaster.sendTransform(self.item_cache.values())
        self.marker_pub.publish(MarkerArray(markers=list(self.marker_cache.values())))

    def tf_to_marker(self, tf: TransformStamped) -> Marker:
        msg = Marker()
        msg.header = tf.header
        msg.ns = tf.child_frame_id
        msg.id = 1
        msg.type = Marker.TEXT_VIEW_FACING
        msg.pose.position.x = tf.transform.translation.x
        msg.pose.position.y = tf.transform.translation.y
        msg.pose.position.z = tf.transform.translation.z
        # msg.pose.rotation = tf.transform.orientation
        msg.lifetime = rclpy.time.Duration(seconds=5).to_msg()
        sc = .05    # length of side in 
        msg.scale = Vector3(x=sc,y=sc,z=sc)
        msg.color = ColorRGBA(r=1.0,a=.5)
        msg.text = tf.child_frame_id
        return msg

def main():
    rclpy.init()
    node = ProductRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
