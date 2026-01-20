from passform_msgs.srv import RegisterModule
from passform_msgs.msg import Module
import uuid
import rclpy


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('minimal_client')
    cli = node.create_client(RegisterModule, '/base/base_manager/register_module')

    req = RegisterModule.Request()
    req.module = Module()
    req.module.uuid = str(uuid.uuid4())
    req.ip = ['192.168.8.1']
    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    result = future.result()
    if result.success:
        node.get_logger().info(f'Registered in {result.bay_id}')
    else:
        node.get_logger().info(f'Registration failed: {result.message}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
