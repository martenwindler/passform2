import threading
import time
import rclpy
import rclpy.parameter
from rclpy.node import Node
from rpi_i2c_msgs.msg import Analog, AnalogArray
from rpi_i2c.ultrasonic_sensor import UltrasonicSensor

class RpiGpioUs(Node):
    def __init__(self):
        super().__init__('rpi_gpio_us',allow_undeclared_parameters=True,
                        automatically_declare_parameters_from_overrides=True)

        self.trigger_GPIO_1 = self.get_parameter('trigger_GPIO_1').value
        self.echo_GPIO_1 = self.get_parameter('echo_GPIO_1').value
        self.trigger_GPIO_2 = self.get_parameter('trigger_GPIO_2').value
        self.echo_GPIO_2 = self.get_parameter('echo_GPIO_2').value
        self.period = self.get_parameter('period').value

        self.get_logger().info('Create US-Sensor on Pins [ %i , %i ]' %(self.trigger_GPIO_1, self.echo_GPIO_1))
        self.sensor1 = UltrasonicSensor("linker US-Sensor", self.trigger_GPIO_1, self.echo_GPIO_1)
        self.get_logger().info('Create US-Sensor on Pins [ %i , %i ]' %(self.trigger_GPIO_2, self.echo_GPIO_2))
        self.sensor2 = UltrasonicSensor("rechter US-Sensor", self.trigger_GPIO_2, self.echo_GPIO_2)

        self.pub = self.create_publisher(AnalogArray, self.get_parameter('topic').value, 10)
        self.timer = self.create_timer(self.period, self.publish)

        # Logging success
        self.get_logger().info('Created %s' %str(self))

    def __str__(self):
        return "{0}".format(
            self.__class__.__name__, )

    def publish(self):
        msg = AnalogArray()
        data = [0.0,0.0]
        data[0] = self.sensor1.getDistance()
        data[1] = self.sensor2.getDistance()
        for idx, b in enumerate(list(data)):
            m = Analog()
            m.pin = idx+1
            m.data = float(b)
            msg.data.append(m)
        self.get_logger().info('US-Sensors to ROS: ' + str(msg))
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
