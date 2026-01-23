import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf_transformations import quaternion_from_euler

from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Quaternion
from passform_msgs.msg import BaseStatus

class BayStatePublisher(Node):
    """
    Publishes the static transforms of all bays to their corresponding module base_link

    The transforms are created based on the global 'base/status' topic.
    The list of transforms only contains information for occupied bays.
    """
    def __init__(self):
        super().__init__('bay_state_publisher')

        self.broadcaster = TransformBroadcaster(self, qos=QoSProfile(depth=10))

        self.declare_parameter('status_topic', '/base/status')
        self.create_subscription(BaseStatus, self.get_parameter('status_topic').value,
                                self.bay_status_cb, 10)

        self.get_logger().info("{0} started".format(self.get_name()))
        standard_rot = quaternion_from_euler(0,0,math.pi/2)
        self.q = Quaternion(x = standard_rot[0],
                            y = standard_rot[1],
                            z = standard_rot[2],
                            w = standard_rot[3],
                            )

    def bay_status_cb(self, msg:BaseStatus) -> None:
        """
        Create the tf_list for each occupied bay, linking it to the module uuid
        """
        tf_list = list()
        now = self.get_clock().now().to_msg()
        for b in msg.bay_data:
            if b.module_uuid != '':
                """only append occupied bays"""
                tf = TransformStamped()
                tf.header.stamp = now
                tf.header.frame_id = 'bay_'+str(b.bay_id)+'_dock'
                tf.child_frame_id = b.module_uuid+'/base_link'
                quat_tf = [0.0, 1.0, 0.0, 0.0]
                tf.transform.rotation = self.q
                tf_list.append(tf)
        self.broadcaster.sendTransform(tf_list)

def main():
    rclpy.init()
    node = BayStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
