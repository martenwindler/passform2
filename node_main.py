class PassformManager(Node):
    def __init__(self):
        super().__init__('passform_manager')
        self.get_logger().info('Projekt passform2: Manager Node gestartet.')

def main(args=None):
    rclpy.init(args=args)
    node = PassformManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
