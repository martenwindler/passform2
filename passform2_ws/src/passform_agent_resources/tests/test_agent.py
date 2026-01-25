#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from passform_agent_resources.msg import AgentAnnounce, AgentInfo, GridPosition

class MultiAgentTestNode(Node):
    def __init__(self):
        super().__init__('multi_agent_test_node')
        
        # QoS auf Transient Local setzen, damit die Bridge die Daten sicher empfängt
        qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        self.publisher = self.create_publisher(AgentAnnounce, 'agent_announce', qos)
        self.timer = self.create_timer(1.0, self.publish_agents)
        self.get_logger().info('🚀 Test-Agent aktiv...')

    def publish_agents(self):
        msg = AgentAnnounce()
        
        # Diese Felder existieren erst nach dem erfolgreichen 'colcon build'!
        msg.active = True
        
        info = AgentInfo()
        info.agent_id = "Robot-007"
        info.module_type = "ftf"
        
        pos = GridPosition()
        pos.x = 1
        pos.y = 1
        info.position = pos
        info.orientation = 0.0
        
        msg.agent_info = info
        
        self.publisher.publish(msg)
        self.get_logger().info('✅ Paket gesendet')

def main():
    rclpy.init()
    node = MultiAgentTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()