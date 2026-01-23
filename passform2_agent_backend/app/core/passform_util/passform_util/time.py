import rclpy
import rclpy.qos
import rclpy.duration
import time
from threading import Timer

from std_msgs.msg import Header

class Watchdog:
    def __init__(self, timeout, timeout_status):
        self.timeout_status = timeout_status
        self.timer = TimerRestart(timeout, self.handle_timeout)

    def on_heartbeat(self, status):
        """Update last_seen time to now() and resets heartbeat"""
        object.__setattr__(self, 'last_seen', int(time.time()) )
        self.set_status(status)
        self.timer.start()

    def set_status(self, status):
        """Update module status"""
        object.__setattr__(self, 'status', status)

    def handle_timeout(self):
        """Set module to stale if heartbeat fails"""
        self.set_status(self.timeout_status)
        raise RuntimeError('Module timed out.')

    def stop(self):
        """Stops the heartbeat timeout"""
        self.timer.cancel()

class Heartbeat:
    def __init__(self, period):
        lease_delta = 0.02 # 20 ms
        node = rclpy.create_node('heartbeat')

        qos_profile = rclpy.qos.qos_profile_system_default
        qos_profile.liveliness=rclpy.qos.LivelinessPolicy.MANUAL_BY_TOPIC
        qos_profile.liveliness_lease_duration=rclpy.duration.Duration(seconds=period + lease_delta)
        qos_profile.deadline=rclpy.duration.Duration(seconds=period + lease_delta)

        self.publisher = node.create_publisher(
                                    Header,
                                    'ymodule_broadcast', qos_profile)
        self.timer = node.create_timer(period, self.timer_callback)

    def timer_callback(self):
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg);

class TimerRestart:
    def __init__(self, timeout, handle=None):
        self.timeout = timeout
        self.handle = handle
        self.timer = None

    def start(self):
        if self.timer is not None:
            self.timer.cancel()
        self.timer = Timer(self.timeout, self.handle)
        self.timer.start()

    def cancel(self):
        if self.timer is not None:
            self.timer.cancel()
        self.timer = None
