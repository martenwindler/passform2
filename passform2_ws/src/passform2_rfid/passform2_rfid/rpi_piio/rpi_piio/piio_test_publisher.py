import rclpy
from rclpy.node import Node
import time
import rclpy.parameter
import rpi_i2c_msgs.msg
from rpi_piio.piio_boards import Adio


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.piio_board = Adio()
        timer_period = 0.5  # seconds

        self.publisher_DO = self.create_publisher(rpi_i2c_msgs.msg.DigitalArray, 'DO_array', 10)
        self.timer_DO = self.create_timer(timer_period, self.timer_callback_DO)

        self.publisher_AO = self.create_publisher(rpi_i2c_msgs.msg.AnalogArray, 'AO_array', 10)
        self.timer_AO = self.create_timer(timer_period, self.timer_callback_AO)

    def timer_callback_DO(self):
        msg = rpi_i2c_msgs.msg.DigitalArray()
        for pin in self.piio_board.get_DO_pins():
            m = rpi_i2c_msgs.msg.Digital()
            #self.get_logger().info('the pin: {}'.format(str(pin)))
            m.pin = pin
            m.data = 1
            msg.data.append(m)
        #self.get_logger().info('Sending DO: {}'.format(str(msg)))
        self.publisher_DO.publish(msg)

    def timer_callback_AO(self):
        msg = rpi_i2c_msgs.msg.AnalogArray()
        for pin in self.piio_board.get_AO_pins():
            m = rpi_i2c_msgs.msg.Analog()
            m.pin = pin
            m.data = 0.3
            msg.data.append(m)
        #self.get_logger().info('Sending AO: {}'.format(str(msg)))
        self.publisher_AO.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
