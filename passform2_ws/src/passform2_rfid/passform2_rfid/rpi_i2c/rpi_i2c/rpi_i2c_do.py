import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from rpi_i2c.i2c_types import DigitalOut
from rpi_i2c_msgs.msg import Digital, DigitalArray

class RpiI2cDo(Node):
    def __init__(self):
        super().__init__('rpi_i2c_do', automatically_declare_parameters_from_overrides=True)

        # Parameter declaration
        # self.declare_parameter('address',
        #     descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        # self.declare_parameter('period', value = 1,
        #     descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        # self.declare_parameter('length', value = 8,
        #     descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        # self.declare_parameter('topic', value = 'do_data',
        #     descriptor = ParameterDescriptor(type=ParameterType.PARAMTER_STRING))

        self.address = self.get_parameter('address').value
        self.get_logger().info('Create I2C DigitalOut on address {:#04x}'.format(self.address))

        self.bus = DigitalOut(self.address, self.get_parameter('length').value)

        self.create_subscription(Digital, self.get_parameter('topic').value, self.callback_write, 10)
        self.create_subscription(DigitalArray, self.get_parameter('topic').value+'_array', self.callback_write, 10)

        # Logging success
        self.get_logger().info('Created %s' %str(self))


    def __str__(self):
        return f'{self.__class__.__name__}'

    def callback_write(self, msg:Digital) -> None:
        '''Callback for Digital messages.'''
        self.get_logger().info('ROS to I2C@{:#04x}: {0}'.format(self.address,str(msg)))
        self._write_pin(pin=msg.pin, val=msg.data)

    def callback_write_array(self, msg:DigitalArray) -> None:
        '''Callback for DigitalArray messages. Writes each Digital value individually.'''
        self.get_logger().info('ROS to I2C@{:#04x}: {0}'.format(self.address,str(msg)))
        for m in msg.data:
            self._write_pin(pin=m.pin, val=m.data)

    def _write_pin(self, pin:int, val:bool) -> None:
        '''
        Calls the DigitalOut object to write value at pin. Includes logger for error handling.
        '''
        try:
            self.bus.write(pin=pin, val=val)
        except Exception as e:
            self.get_logger().warn('DigitalOut failed: %s' % e)

def main(args=None):
    rclpy.init()
    node = RpiI2cDo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy()
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
