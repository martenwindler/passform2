import rclpy
from rclpy.node import Node
from passform_agent_msgs.msg import AgentInfo
import time

def main():
    rclpy.init()
    node = Node('mock_agent_node')
    publisher = node.create_publisher(AgentInfo, 'agent_info', 10)
    
    msg = AgentInfo()
    msg.agent_id = "Robot-007"
    msg.module_type = "tisch" # oder "greifer", "conveyeur"
    msg.x = 2
    msg.y = 2
    msg.orientation = 0.0
    msg.status = "online"

    print("Sending mock heartbeats... Press Ctrl+C to stop.")
    try:
        while rclpy.ok():
            publisher.publish(msg)
            time.sleep(1) # Jede Sekunde ein Heartbeat
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()