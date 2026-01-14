import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from rpi_i2c_msgs.msg import Analog, AnalogArray
from rpi_i2c.i2c_types import AnalogIn

class RpiI2cAi(Node):
    def __init__(self):
        super().__init__('rpi_i2c_ai', automatically_declare_parameters_from_overrides=True)

        # Parameter declaration
        # self.declare_parameter('address',
        #     descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        # self.declare_parameter('period', value = 1,
        #     descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        # self.declare_parameter('length', value = 5,
        #     descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        # self.declare_parameter('topic',value = 'ai_data',
        #     descriptor = ParameterDescriptor(type=ParameterType.PARAMTER_STRING))

        self.address = self.get_parameter('address').value
        self.period = self.get_parameter('period').value
        self.get_logger().info('Create I2C AnalogIn on address {:#04x}'.format(self.address))

        self.bus = AnalogIn(self.address, self.get_parameter('length').value)

        self.pub = self.create_publisher(AnalogArray, self.get_parameter('topic').value, 10)
        self.timer = self.create_timer(self.period, self.publish)

        # Logging success
        self.get_logger().info('Created %s' %str(self))


    def __str__(self):
        return "{0}".format(
            self.__class__.__name__, )

    def publish(self):
        msg = AnalogArray()
        data = self.bus.read_all()
        for idx, b in enumerate(list(data)):
            m = Analog()
            m.pin = idx+1
            m.data = b
            msg.data.append(m)
        self.get_logger().info('Sending from I2C@{:#04x}: {}'.format(self.address,str(msg)))
        self.pub.publish(msg)

def main(args=None):
    rclpy.init()
    node = RpiI2cAi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
