import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from rpi_i2c.sensors import UltrasonicSensor
from sensor_msgs.msg import Range

class RpiGpioUs(Node):
    def __init__(self):
        super().__init__('rpi_gpio_us')

        self.declare_parameter('trigger_pin', value = 20,
            descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('echo_pin', value = 21,
            descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('period', value = 1,
            descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('topic', value = 'us_data',
            descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING))

        self.sensor = UltrasonicSensor(
            trigger_GPIO = self.get_parameter('trigger_pin').value,
            echo_GPIO = self.get_parameter('echo_pin').value
        )
        self.get_logger().info('Created US-Sensor on Pins [ %i , %i ]' %(self.get_parameter('trigger_pin').value,
        self.get_parameter('echo_pin').value))

        self.pub = self.create_publisher(Range, self.get_parameter('topic').value, 10)
        self.timer = self.create_timer(self.get_parameter('period').value, self.publish)

        self.get_logger().info('Created %s' %str(self))

    def __str__(self):
        return f'{self.__class__.__name__}'

    def publish(self):
        msg = Range(
            range=self.sensor.get_distance()
        )
        self.get_logger().info(f'US Sensor distance: {msg.range}')
        self.pub.publish(msg)

def main(args=None):
    rclpy.init()
    node = RpiGpioUs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
