import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from rpi_i2c.i2c_types import AnalogOut
from rpi_i2c_msgs.msg import Analog, AnalogArray

class RpiI2cAo(Node):
    def __init__(self):
        super().__init__('rpi_i2c_ao', automatically_declare_parameters_from_overrides=True)

        # Parameter declaration
        # self.declare_parameter('address', value = 0x58,
        #     descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        # self.declare_parameter('period', value = 1,
        #     descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        # self.declare_parameter('length', value = 4,
        #     descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        # self.declare_parameter('topic',value = 'ao_data',
        #     descriptor = ParameterDescriptor(type=ParameterType.PARAMTER_STRING))

        self.address = self.get_parameter('address').value
        self.get_logger().info('Create I2C AnalogOut on address {:#04x}'.format(self.address))

        self.bus = AnalogOut(self.address, self.get_parameter('length').value)

        self.create_subscription(Analog, self.get_parameter('topic').value, self.callback_write, 10)
        self.create_subscription(AnalogArray, self.get_parameter('topic').value+'_array', self.callback_write_array, 10)

        # Logging success
        self.get_logger().info('Created %s' %str(self))


    def __str__(self):
        return f'{self.__class__.__name__}'

    def callback_write(self, msg:Analog):
        '''Callback for Analog messages.'''
        self.get_logger().info('Received for RPi I2C@{:#04x}: {0}'.format(self.address,str(msg)))
        self._write_pin(pin=msg.pin, val=msg.data)

    def callback_write_array(self, msg:AnalogArray):
        '''
        Callback for AnalogArray messages.

        Performs single write, if length does not match full length of the analog out.
        '''
        self.get_logger().info('Received for RPi I2C@{:#04x}: {}'.format(self.address,str(msg)))
        self._write_array(msg)

    def _write_pin(self, pin:int, val:float) -> None:
        '''
        Calls the AnalogOut object to write value at pin.
        Includes logger for error handling.
        '''
        try:
            self.bus.write(pin=pin, val=val)
        except Exception as e:
            self.get_logger().warn('AnalogOut failed: %s' % e)

    def _write_array(self, msg:AnalogArray) -> None:
        '''
        Calls the AnalogOut object to write each AnalogArray element individually.
        Includes logger for error handling.
        '''

        for m in msg.data:
            self._write_pin(pin=m.pin, val=m.data)

    # def _write_array_blind(self, val:list[float]) -> None:
    #     '''
    #     Calls the AnalogOut object to write a full array in the exact order as received.
    #     Includes logger for error handling.
    #     '''
    #     try:
    #         self.bus.write(val)
    #     except Exception as e:
    #         self.get_logger().warn('AnalogOut array operation failed: %s' % e)

def main(args=None):
    rclpy.init()
    node = RpiI2cAo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
