from typing import Dict, List
from passform_agent_msgs.msg import AgentInfo
from rosidl_runtime_py import message_to_ordereddict
from app.socket.socket_io_manager import socket_manager

class AgentManager:
    """Verwaltet aktive Agenten und sendet Updates an WebSocket-Clients"""
    
    def __init__(self):
        self.active_agents: Dict[str, AgentInfo] = {}

    def add_agent(self, agent_info: AgentInfo):
        """Fügt einen neuen Agenten hinzu oder aktualisiert einen bestehenden"""
        agent_id = agent_info.agent_id
        self.active_agents[agent_id] = agent_info
        self.send_agent_list()

    def remove_agent(self, agent_id: str):
        """Entfernt einen Agenten und sendet ein Update"""
        if agent_id in self.active_agents:
            self.active_agents.pop(agent_id)
            self.send_agent_list()

    def clear_all_agents(self):
        """Löscht alle Agenten (z.B. beim Mode-Wechsel)"""
        if self.active_agents:
            print(f"Lösche {len(self.active_agents)} Agenten beim Mode-Wechsel")
            self.active_agents.clear()
            self.send_agent_list()

    def send_update(self, event_name: str, data: Dict):
        """Sendet ein Update an alle WebSocket-Clients"""
        socket_manager.emit_event_sync(event_name, data)

    def send_agent_list(self):
        """Sendet die Liste aller aktiven Agenten an alle WebSocket-Clients"""
        self.send_update('active_agents', {
            'agents': [message_to_ordereddict(agent) for agent in self.active_agents.values()]
        })

    def get_all_agents(self) -> List[Dict]:
        """Gibt alle Agenten als Dict-Liste zurück"""
        return [message_to_ordereddict(agent) for agent in self.active_agents.values()]

    def get_agent(self, agent_id: str) -> Dict:
        """Gibt einen spezifischen Agenten zurück"""
        if agent_id in self.active_agents:
            return message_to_ordereddict(self.active_agents[agent_id])
        return None


agent_manager = AgentManager()
