#!/usr/bin/python
import time
import rclpy
import rclpy.parameter
from rclpy.node import Node
import rpi_i2c_msgs.msg
from rpi_piio.piio_boards import Adio

class RpiPiio(Node):
    def __init__(self):
        super().__init__('rpi_piio',allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)

        self.piio_board = Adio()
        # parameter
        self.period = 1.#self.get_parameter('rate').value

        # fixed rate publisher (everything as array)
        self.pub_digital = self.create_publisher(
            rpi_i2c_msgs.msg.DigitalArray, 'DI', 10)
        self.pub_analog = self.create_publisher(
            rpi_i2c_msgs.msg.AnalogArray, 'AI', 10)
        self.timer = self.create_timer(self.period, self.publish)

        # subscriber (both single value and array)
        #self.sub_DO = self.create_subscription(rpi_i2c_msgs.msg.Digital,'DO',self.write_digital,10)
        self.sub_DO_array = self.create_subscription(rpi_i2c_msgs.msg.DigitalArray,'DO_array',self.write_digital,10)
        #self.sub_AO = self.create_subscription(rpi_i2c_msgs.msg.Analog, 'AO', self.write_analog,10)
        self.sub_AO_array = self.create_subscription(rpi_i2c_msgs.msg.AnalogArray, 'AO_array', self.write_analog,10)
        #self.create_subscriber('DO', rpi_i2c_msgs.msg.Digital, self.write_digital)
        #self.create_subscriber('DO_array', rpi_i2c_msgs.msg.DigitalArray, self.write_digital)
        #self.create_subscriber('AO', rpi_i2c_msgs.msg.Analog, self.write_analog)
        #self.create_subscriber('AO_array', rpi_i2c_msgs.msg.AnalogArray, self.write_analog)

        # Logging success
        self.get_logger().info('Created %s' %str(self))


    def __str__(self):
        return "{0}".format(
            self.__class__.__name__, )

    def read_digital(self):
        '''Reads all dgital pins of the connected board'''
        msg = rpi_i2c_msgs.msg.DigitalArray()
        for pin in self.piio_board.get_DI_pins():
            m = rpi_i2c_msgs.msg.Digital()
            m.pin = pin
            m.data = self.piio_board.get_digital(m.pin)
            msg.data.append(m)
        return msg

    def publish(self):
        '''Publishes all digital and analog pins as array'''
        # digital
        msg = self.read_digital()
        self.get_logger().info('Reading DO: {}'.format(str(msg)))
        self.pub_digital.publish(msg)
        # analog
        msg = self.read_analog()
        #self.get_logger().info('Reading AO: {}'.format(str(msg)))
        self.pub_analog.publish(msg)

    def read_analog(self):
        '''Reads all analog pins of the connected board'''
        msg = rpi_i2c_msgs.msg.AnalogArray()
        for pin in self.piio_board.get_AI_pins():
            m = rpi_i2c_msgs.msg.Analog()
            m.pin = pin
            m.data = self.piio_board.get_analog(m.pin)
            msg.data.append(m)
        return msg

    def write_digital(self, msg):
        '''
        Sets the content of an Digital or DigitalArray message pin by pin.
        Failures in setting will result only in error messages.
        '''
        #self.get_logger().info('Callback DO: {}'.format(str(msg)))
        for d in msg.data:
            try:
                self.piio_board.set_digital(d.pin, d.data)
            except Exception as e:
                self.get_logger().error('Error setting digital output: %s' %e)

    def write_analog(self, msg):
        '''
        Sets the content of an Analog or AnalogArray message pin by pin.
        Failures in setting will result only in error messages.
        '''
        #self.get_logger().info('Callback received AO: {}'.format(str(msg)))
        for d in msg.data:
            try:
                self.piio_board.set_analog(d.pin, d.data)
            except Exception as e:
                self.get_logger().error('Error setting analog output: %s' %e)

def main(args=None):
    rclpy.init()
    node = RpiPiio()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy()
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
