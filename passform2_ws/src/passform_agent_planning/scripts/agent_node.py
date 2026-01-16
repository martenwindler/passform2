#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from collections import defaultdict
from passform_agent_msgs.msg import (
    AgentAnnounce, GridPosition, CostPropagation,
    PathRequest, AgentInfo, CFP, Proposal, Award, PathComplete
)

class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')
        # 1. Parameter
        self.agent_id = self.declare_parameter('agent_id', 'A_unknown').value
        self.module_type = self.declare_parameter('module_type', 'static').value
        pos = self.declare_parameter('position', [0, 0]).value
        self.x, self.y = pos[0], pos[1]
        
        self.agent_info = AgentInfo(agent_id=self.agent_id, module_type=self.module_type,
                                    position=GridPosition(x=self.x, y=self.y))

        # 2. State & Fail-Safe Speicher
        self.known_agents = {}      # id -> {'info': AgentInfo, 'timer': Timer}
        self.neighbors = []
        self.valid_moves = []
        self.cost_to_goal = defaultdict(lambda: float('inf'))
        self.proposals = defaultdict(list)
        self.active_negotiations = {}
        
        # Konstanten für Robustheit
        self.HEARTBEAT_TIMEOUT = 3.5  # Sekunden bis ein Agent als "tot" gilt
        self.AUCTION_TIMEOUT = 0.5    # Maximale Wartezeit auf Gebote

        # 3. ROS Setup
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.ann_pub = self.create_publisher(AgentAnnounce, 'agent_announce', 1)
        self.cost_pub = self.create_publisher(CostPropagation, 'cost_propagation', qos)
        self.cfp_pub = self.create_publisher(CFP, 'cfp', qos)
        self.prop_pub = self.create_publisher(Proposal, 'proposal', qos)
        self.award_pub = self.create_publisher(Award, 'award', qos)
        self.comp_pub = self.create_publisher(PathComplete, 'path_complete', qos)

        # Subscriptions
        for topic, msg, cb in [('agent_announce', AgentAnnounce, self.on_announce),
                               ('cost_propagation', CostPropagation, self.on_cost),
                               ('path_request', PathRequest, self.on_path_request),
                               ('cfp', CFP, self.on_cfp), ('proposal', Proposal, self.on_proposal),
                               ('award', Award, self.on_award), ('path_complete', PathComplete, self.on_path_complete)]:
            self.create_subscription(msg, topic, cb, qos if 'announce' not in topic else 1)

        self.create_timer(1.0, self.send_announce)
        self.get_logger().info(f"🚀 {self.agent_id} ready with Fail-Safe logic.")

    # --- FAIL-SAFE: HEARTBEAT MONITORING ---

    def send_announce(self):
        self.ann_pub.publish(AgentAnnounce(agent_info=self.agent_info, active=True))

    def on_announce(self, msg):
        aid = msg.agent_info.agent_id
        if aid == self.agent_id: return

        # Falls Agent neu oder bereits bekannt: Timer zurücksetzen (Watchdog)
        if aid in self.known_agents:
            self.destroy_timer(self.known_agents[aid]['timer'])
        
        # Erstelle Timeout-Timer für diesen spezifischen Agenten
        watchdog = self.create_timer(self.HEARTBEAT_TIMEOUT, lambda: self.remove_agent(aid))
        self.known_agents[aid] = {'info': msg.agent_info, 'timer': watchdog}
        self.update_topology()

    def remove_agent(self, aid):
        """Wird aufgerufen, wenn ein Heartbeat ausbleibt."""
        if aid in self.known_agents:
            self.get_logger().error(f"⚠️ Agent {aid} timed out! Removing from topology.")
            self.destroy_timer(self.known_agents[aid]['timer'])
            del self.known_agents[aid]
            self.update_topology()
            # Optional: Alle laufenden Verhandlungen abbrechen, da Pfad ungültig sein könnte
            # self.abort_all_negotiations()

    def update_topology(self):
        self.neighbors = [a['info'] for a in self.known_agents.values() 
                          if abs(a['info'].position.x - self.x) + abs(a['info'].position.y - self.y) == 1]
        dirs = {'greifer': [(0,1),(1,0),(0,-1),(-1,0)], 'mensch': [(0,1),(1,0),(0,-1),(-1,0)],
                'rollen_ns': [(0,1),(0,-1)], 'rollen_ow': [(1,0),(-1,0)]}.get(self.module_type, [])
        allowed = {(self.x+dx, self.y+dy) for dx, dy in dirs}
        self.valid_moves = [n for n in self.neighbors if (n.position.x, n.position.y) in allowed]

    # --- FAIL-SAFE: AUCTION TIMEOUT ---

    def start_auction(self, rid):
        if rid not in self.active_negotiations: return
        self.destroy_timer(self.active_negotiations[rid].pop('timer', None))
        
        goal = self.active_negotiations[rid]['goal']
        aw = Award(request_id=rid, winner=self.agent_info, cost=self.cost_to_goal[rid], goal=goal)
        aw.path.append(self.agent_info)
        self.active_negotiations[rid]['award'] = aw
        
        if not self.valid_moves:
            self.comp_pub.publish(PathComplete(request_id=rid, status=1))
            return
            
        # Sende CFP
        self.cfp_pub.publish(CFP(request_id=rid, current=self.agent_info, neighbors=self.valid_moves, goal=goal))

        # FAIL-SAFE: Starte einen Timer, der die Verhandlung nach X ms erzwingt, 
        # auch wenn nicht alle Nachbarn geantwortet haben.
        self.active_negotiations[rid]['auction_watchdog'] = self.create_timer(
            self.AUCTION_TIMEOUT, lambda: self.evaluate_proposals(rid, forced=True)
        )

    def on_proposal(self, msg):
        if msg.origin.agent_id != self.agent_id: return
        rid = msg.request_id
        self.proposals[rid].append(msg)
        
        # Wenn alle geantwortet haben, sofort auswerten
        if len(self.proposals[rid]) >= len(self.valid_moves):
            self.evaluate_proposals(rid)

    def evaluate_proposals(self, rid, forced=False):
        if rid not in self.active_negotiations: return
        
        # Stoppe den Watchdog-Timer
        timer = self.active_negotiations[rid].pop('auction_watchdog', None)
        if timer: self.destroy_timer(timer)

        if forced:
            self.get_logger().warning(f"⏰ Auction Timeout for {rid}! Evaluating with {len(self.proposals[rid])} bids.")

        valid = [p for p in self.proposals.pop(rid, []) if math.isfinite(p.bid)]
        if not valid:
            self.get_logger().error(f"❌ No valid bids for {rid}. Planning failed.")
            self.comp_pub.publish(PathComplete(request_id=rid, status=1))
            return

        best = min(valid, key=lambda p: p.bid)
        aw = self.active_negotiations[rid].pop('award')
        aw.winner, aw.path = best.proposer, aw.path + [best.proposer]
        self.award_pub.publish(aw)

    # --- RESTLICHE LOGIK (KOMPAKT) ---

    def calculate_move_cost(self, from_agent):
        cost = {'greifer': 3.5, 'mensch': 3.5, 'rollen_ns': 1.0, 'rollen_ow': 1.0}.get(self.module_type, 1.0)
        return float(cost + (2.0 if self.module_type == 'mensch' else 0.0))

    def on_cost(self, msg):
        if msg.origin.agent_id == self.agent_id: return
        if msg.target.agent_id == self.agent_id:
            new_cost = msg.cost + self.calculate_move_cost(msg.origin)
            if new_cost < self.cost_to_goal[msg.request_id]:
                self.cost_to_goal[msg.request_id] = new_cost
                for n in self.neighbors:
                    self.cost_pub.publish(CostPropagation(request_id=msg.request_id, origin=self.agent_info, target=n, cost=new_cost))

    def on_path_request(self, msg):
        rid = msg.request_id
        if (msg.goal.x, msg.goal.y) == (self.x, self.y):
            self.cost_to_goal[rid] = 0.0
            for n in self.neighbors: self.cost_pub.publish(CostPropagation(request_id=rid, origin=self.agent_info, target=n, cost=0.0))
        if (msg.start.x, msg.start.y) == (self.x, self.y):
            self.active_negotiations[rid] = {'goal': msg.goal}
            self.active_negotiations[rid]['timer'] = self.create_timer(0.2, lambda: self.start_auction(rid))

    def on_cfp(self, msg):
        if msg.current.agent_id != self.agent_id and self.agent_info in msg.neighbors:
            self.prop_pub.publish(Proposal(request_id=msg.request_id, origin=msg.current, proposer=self.agent_info, bid=self.cost_to_goal[msg.request_id]))

    def on_award(self, msg):
        if msg.winner.agent_id != self.agent_id: return
        if (self.x, self.y) == (msg.goal.x, msg.goal.y):
            self.comp_pub.publish(PathComplete(request_id=msg.request_id, status=0, path=msg.path, cost=msg.cost))
        else:
            self.active_negotiations[msg.request_id] = {'goal': msg.goal, 'award': msg}
            self.start_auction(msg.request_id)

    def on_path_complete(self, msg):
        rid = msg.request_id
        self.proposals.pop(rid, None)
        neg = self.active_negotiations.pop(rid, None)
        if neg and 'auction_watchdog' in neg: self.destroy_timer(neg['auction_watchdog'])

def main():
    rclpy.init()
    node = AgentNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()