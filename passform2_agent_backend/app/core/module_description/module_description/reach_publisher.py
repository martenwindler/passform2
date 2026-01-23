import rclpy
import yaml
from rclpy.node import Node
from tf2_geometry_msgs.tf2_geometry_msgs import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class ModuleReachPublisher(Node):
    """
    Publishes the static transforms of points limiting the reach of the module
    """
    def __init__(self):
        super().__init__('reach_publisher')
        self.declare_parameter('uuid', rclpy.Parameter.Type.STRING)
        self.declare_parameter('point_file', rclpy.Parameter.Type.STRING)


        self.broadcaster = StaticTransformBroadcaster(self)
        self.base_link = self.get_parameter('uuid').value+'/base_top'

        self.make_transforms() # publish once at startup

    def make_transforms(self):
        """Read point file (yaml) and publish all points as static transform"""
        with open(self.get_parameter('point_file').value, 'r') as file:
            points = yaml.safe_load(file)

        transforms = []
        for point, coord in points.items():
            tf = TransformStamped()

            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self.base_link
            tf.child_frame_id = self.get_parameter('uuid').value+'/'+point

            tf.transform.translation.x = float(coord['x'])
            tf.transform.translation.y = float(coord['y'])
            tf.transform.translation.z = float(coord['z'])

            tf.transform.rotation.w = 1.0

            transforms.append(tf)

        self.broadcaster.sendTransform(transforms)

def main():
    rclpy.init()
    node = ModuleReachPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
