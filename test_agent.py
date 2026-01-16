#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

# Importiere die neuen Nachrichtentypen
from passform_agent_msgs.msg import AgentInfo, GridPosition

class MultiAgentTestNode(Node):
    def __init__(self):
        super().__init__('multi_agent_test_node')
        # Wir publizieren auf 'agent_info', worauf dein Backend hört
        self.publisher = self.create_publisher(AgentInfo, 'agent_info', 10)
        self.get_logger().info('🚀 Starte Simulation mit verschachtelter GridPosition...')
        
        # Sende-Intervall: 1 Sekunde
        self.timer = self.create_timer(1.0, self.publish_agents)

    def publish_agents(self):
        # --- AGENT 1: Robot-007 ---
        msg1 = AgentInfo()
        msg1.agent_id = "Robot-007"
        msg1.module_type = "tisch"
        
        # WICHTIG: Erstellung des Unter-Objekts GridPosition
        pos1 = GridPosition()
        pos1.x = 1
        pos1.y = 1
        msg1.position = pos1 # Zuweisung an das Feld 'position'
        
        msg1.orientation = 0.0
        self.publisher.publish(msg1)

        # --- AGENT 2: Robot-008 ---
        msg2 = AgentInfo()
        msg2.agent_id = "Robot-008"
        msg2.module_type = "greifer"
        
        pos2 = GridPosition()
        pos2.x = 3
        pos2.y = 1
        msg2.position = pos2
        
        msg2.orientation = 90.0
        self.publisher.publish(msg2)

        self.get_logger().info('✅ Heartbeats für Robot-007 (1,1) und Robot-008 (3,1) gesendet.')

def main(args=None):
    rclpy.init(args=args)
    node = MultiAgentTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Simulation gestoppt.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()