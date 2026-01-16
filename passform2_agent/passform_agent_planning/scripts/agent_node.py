#!/usr/bin/env python3
from typing import List
import rclpy
import signal
import math          # ganz oben einfügen
from rclpy.signals import SignalHandlerOptions 
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from passform_agent_msgs.msg import (
    AgentAnnounce,
    GridPosition,
    CostPropagation,
    PathRequest,
    AgentInfo,
    CFP,
    Proposal,
    Award,
    PathComplete
)
from collections import defaultdict
from builtin_interfaces.msg import Duration  

_shutdown_requested = False  

class AgentNode(Node):
    def __init__(self):
        super().__init__('agent_node')

        # --- Parameter laden ---
        pos = self.declare_parameter('position', [0,0]) \
                  .get_parameter_value().integer_array_value
        self.x, self.y = pos
        self.agent_id = f"A_{self.x}_{self.y}"
        self.module_type = self.declare_parameter('module_type','greifer') \
                               .get_parameter_value().string_value
        
        self.agent_info: AgentInfo = AgentInfo(agent_id=self.agent_id,
                                               position=GridPosition( x=self.x, y=self.y), module_type=self.module_type)

        self.get_logger().info(
            f"Agent {self.agent_id} initialized at ({self.x},{self.y}), "
            f"type={self.module_type}"
        )

        self.heartbeat_freq   = 1
        self.heartbeat_period = 1.0 / self.heartbeat_freq
        self.timeout_factor   = 3
        self.heartbeat_timers = {}


        # --- State ---
        self.known_agents :dict[str, AgentInfo]       = defaultdict(AgentInfo)                # id -> (x,y,type)
        self.neighbors: list[AgentInfo]            = []          # Nachbarn
        self.valid_moves: list[AgentInfo]       = []              # Nachbarn wo das bauteil hin bewegt werden kann
        self.cost_to_goal  :dict[str, float]      = defaultdict(lambda: float('inf'))
        self.proposals           = defaultdict(list)  # req -> Liste der Proposals
        self.expected_neighbors  = {}                 # req -> wie viele Bids erwartet
        self.awards              = {}                 # req -> Award-Stub

        self.req_t0: dict[str, rclpy.time.Time] = {}

        # für convergence‐via‐timer
        self.convergence_delay   = 0.1                # Zeit in Sekunden
        self.convergence_timers  = {}                 # req -> Timer-Handle
        self.start_goals         = {}                 # req -> (goal_x, goal_y)


        # --- Publishers / Subscribers ---
        
        self.ann_pub = self.create_publisher(
            AgentAnnounce, 'agent_announce', 1)
        self.ann_sub = self.create_subscription(
            AgentAnnounce, 'agent_announce',
            self.on_announce, 1)
        
        qos_reliable = QoSProfile(depth=10)
        qos_reliable.reliability = ReliabilityPolicy.RELIABLE

        self.cost_pub = self.create_publisher(
            CostPropagation, 'cost_propagation', qos_reliable)
        self.cost_sub = self.create_subscription(
            CostPropagation, 'cost_propagation',
            self.on_cost, qos_reliable)

        self.req_sub = self.create_subscription(
            PathRequest, 'path_request',
            self.on_path_request, qos_reliable)

        self.cfp_pub = self.create_publisher(CFP, 'cfp', qos_reliable)
        self.cfp_sub = self.create_subscription(CFP, 'cfp',
                                                self.on_cfp, qos_reliable)

        self.prop_pub = self.create_publisher(Proposal, 'proposal', qos_reliable)
        self.prop_sub = self.create_subscription(Proposal, 'proposal',
                                                 self.on_proposal, qos_reliable)

        self.award_pub = self.create_publisher(Award, 'award', qos_reliable)
        self.award_sub = self.create_subscription(Award, 'award',
                                                  self.on_award, qos_reliable)

        self.comp_pub = self.create_publisher(PathComplete, 'path_complete', qos_reliable)
        self.comp_sub = self.create_subscription(PathComplete, 'path_complete',
                                                  self.on_path_complete, qos_reliable)

        # Einmaliges Announce nach Start
        # self.announce_timer = self.create_timer(0.1, self.send_announce)
        self.announce_timer = self.create_timer(
            self.heartbeat_period, self.send_announce
        )

    def _reset_heartbeat_timer(self, agent_info: AgentInfo):
        agent_id = agent_info.agent_id
        old = self.heartbeat_timers.pop(agent_id, None)
        if old:
            self.destroy_timer(old)
        else:
            #neu entdeckter Agent → speichern & Nachbarschaft updaten
            self.known_agents[agent_id] = AgentInfo(
                agent_id=agent_info.agent_id,
                position=GridPosition(
                    x=agent_info.position.x,
                    y=agent_info.position.y
                ),
                module_type=agent_info.module_type
            )
            self.update_neighbors()
            self.get_logger().info(f'New agent discovered: {agent_id}')

        def on_timeout():
            self.get_logger().warning(
                f'Heartbeat von {agent_id} abgelaufen – entferne Agent'
            )
            if agent_id in self.known_agents:
                del self.known_agents[agent_id]
                self.update_neighbors()
            # eigenen Timer-Handle entfernen
            t = self.heartbeat_timers.pop(agent_id, None)
            if t:
                self.destroy_timer(t)

        timeout = self.timeout_factor * self.heartbeat_period
        self.heartbeat_timers[agent_id] = self.create_timer(timeout, on_timeout)


    def send_announce(self):
        msg = AgentAnnounce()
        msg.agent_info = self.agent_info
        msg.active       = True
        self.ann_pub.publish(msg)

    def send_unannounce(self):
        msg = AgentAnnounce()
        msg.agent_info = self.agent_info
        msg.active       = False
        self.ann_pub.publish(msg)
        self.get_logger().info(f"Sent AgentAnnounce (inactive) for {self.agent_id}")

    def on_announce(self, msg: AgentAnnounce):
        if msg.agent_info.agent_id == self.agent_id:
            return

        self._reset_heartbeat_timer(msg.agent_info)

        # if not msg.active:
        #     # Abmeldung
        #     if msg.agent_info.agent_id in self.known_agents:
        #         self.get_logger().info(f"Agent {msg.agent_info.agent_id} offline")
        #         del self.known_agents[msg.agent_info.agent_id]
        #         self.update_neighbors()
        #     return

        # # sonst ganz normal Anmeldung
        # self.get_logger().info(
        #     f"Received announce: id={msg.agent_info.agent_id} pos=({msg.agent_info.position.x},{msg.agent_info.position.y}) "
        #     f"type={msg.agent_info.module_type}"
        # )
        # self.known_agents[msg.agent_info.agent_id] = AgentInfo(
        #     agent_id=msg.agent_info.agent_id,
        #     position=GridPosition(x=msg.agent_info.position.x, y=msg.agent_info.position.y),
        #     module_type=msg.agent_info.module_type
        # )
        # self.update_neighbors()


    def update_neighbors(self):

        # sammle alle physisch angrenzenden Agents
        phys = []
        for agent_info in self.known_agents.values():
            if abs(agent_info.position.x - self.x) + abs(agent_info.position.y - self.y) == 1:
                phys.append(agent_info)
        self.neighbors = phys



        if self.module_type in ('greifer','mensch'):
            dirs = [(0,1),(1,0),(0,-1),(-1,0)]
        elif self.module_type == 'rollen_ns':
            dirs = [(0,1),(0,-1)]
        elif self.module_type == 'rollen_ow':
            dirs = [(1,0),(-1,0)]
        else:
            dirs = []
        allowed = {(self.x+dx, self.y+dy) for dx,dy in dirs}
        self.valid_moves = [
            agent_info for agent_info in self.neighbors
            if (agent_info.position.x,agent_info.position.y) in allowed
        ]
        self.get_logger().debug(f"Neighbors updated: {self.neighbors}")

    def on_cost(self, msg: CostPropagation):

        if msg.origin == self.agent_info:
            return
        

        req = msg.request_id
        self.get_logger().debug(
            f"on_cost: req={req} origin=({msg.origin.position.x},{msg.origin.position.y}) "
            f"target=({msg.target.position.x},{msg.target.position.y}) cost={msg.cost}"
        )

        # Reset convergence timer
        if req in self.start_goals:
            self.get_logger().debug(
                f"Resetting convergence timer for req={req}"
            )
            self.reset_convergence_timer(req)
        
        # Update cost_to_goal
        if (msg.target== self.agent_info):
            if msg.origin in self.valid_moves:
                newc = msg.cost + self.move_cost(msg.origin)
                if newc < self.cost_to_goal[req]:
                    self.cost_to_goal[req] = newc
                    self.get_logger().info(
                        f"Updated cost_to_goal[{req}] = {newc}"
                    )
                    self.propagate_cost(req)
            

    def propagate_cost(self, req: str):
        for target_agent_info in self.neighbors:
            m = CostPropagation()
            m.request_id = req
            m.origin = self.agent_info
            m.target = target_agent_info
            m.cost = self.cost_to_goal[req]
            self.cost_pub.publish(m)
            self.get_logger().debug(
                f"Propagated cost req={req} to ({target_agent_info.position.x},{target_agent_info.position.y}) cost={m.cost}"
            )


    def i_am_designated_fail_publisher(self, req: str) -> bool:
        """
        Make the election *deterministic*:
        the agent whose id has the smallest lexical order
        among all currently known agents takes the job.
        """
        lowest = min([self.agent_id] + list(self.known_agents.keys()))
        return self.agent_id == lowest  # True for exactly one process
    
    def node_exists(self, x: int, y: int) -> bool:
        """True ⇢ an announced agent occupies (x,y)."""
        if (x, y) == (self.x, self.y):
            return True
        return any((agent_info.position.x, agent_info.position.y) == (x, y) for agent_info in self.known_agents.values())



    def on_path_request(self, msg: PathRequest):
        req = msg.request_id
        self.get_logger().info(
            f"Received PathRequest req={req} "
            f"start=({msg.start.x},{msg.start.y}) "
            f"goal=({msg.goal.x},{msg.goal.y})"
        )

        start_missing = not self.node_exists(msg.start.x, msg.start.y)
        goal_missing  = not self.node_exists(msg.goal.x,  msg.goal.y)

        if (start_missing or goal_missing) and self.i_am_designated_fail_publisher(req):
            reason = ("start" if start_missing else "") + (" & " if start_missing and goal_missing else "") + ("goal" if goal_missing else "")
            self.get_logger().warning(
                f"{reason.capitalize()} node missing for req={req} – immediate FAIL")
            self.pub_failure(req)
            return

        # Rückwärts-Propagation
        if (msg.goal.x, msg.goal.y) == (self.x, self.y):
            self.get_logger().info(f"Node is goal for req={req}, init costs")
            self.req_t0[req] = self.get_clock().now() 
            self.cost_to_goal[req] = 0.0
            self.propagate_cost(req)

        # Start-Knoten: nur Ziel merken
        if (msg.start.x, msg.start.y) == (self.x, self.y):
            if req in self.convergence_timers:
                self.destroy_timer(self.convergence_timers.pop(req))
                self.get_logger().debug(f"Cleared old timer for req={req}")
            self.start_goals[req] = (msg.goal.x, msg.goal.y)
            self.get_logger().info(
                f"Node is start for req={req}, "
                f"will start auction after convergence"
            )
            

    def reset_convergence_timer(self, req: str):
        if req in self.convergence_timers:
            self.destroy_timer(self.convergence_timers.pop(req))
        self.get_logger().debug(
            f"Setting convergence timer ({self.convergence_delay}s) for req={req}"
        )

        def on_timeout():
            self.get_logger().info(
                f"Convergence period ended for req={req}, launching auction"
            )
            if req in self.start_goals:
                self.start_auction(req)
                self.start_goals.pop(req, None)
            t = self.convergence_timers.pop(req, None)
            if t:
                self.destroy_timer(t)

        timer_handle = self.create_timer(self.convergence_delay, on_timeout)
        self.convergence_timers[req] = timer_handle

    def start_auction(self, req: str):
        goal_x, goal_y = self.start_goals.pop(req)
        self.get_logger().info(
            f"Starting auction for req={req}, goal=({goal_x},{goal_y}), "
            f"neighbors={self.neighbors}"
        )
        aw = Award()
        aw.request_id = req
        aw.winner = self.agent_info
        aw.goal.x,    aw.goal.y    = goal_x, goal_y
        aw.path.append(aw.winner)
        aw.cost = self.cost_to_goal[req]
        self.awards[req] = aw
        self.expected_neighbors[req] = len(self.valid_moves)

        cfp = CFP()
        cfp.request_id = req
        cfp.current = self.agent_info
        cfp.goal.x,    cfp.goal.y    = goal_x, goal_y
        cfp.neighbors = [
            agent_info for agent_info in self.valid_moves
        ]
        if not cfp.neighbors:
            self.pub_failure(req)
        
        self.cfp_pub.publish(cfp)
        self.get_logger().debug(f"Published CFP for req={req}")

    def on_cfp(self, msg: CFP):
        if msg.current == self.agent_info:
            return
        req = msg.request_id
        
        if self.agent_info in msg.neighbors:
            bid = self.cost_to_goal[req]
            p = Proposal()
            p.request_id = req
            p.origin = msg.current
            p.proposer = self.agent_info
            p.bid = bid
            self.prop_pub.publish(p)
            self.get_logger().info(
                f"Received CFP req={req}, sent Proposal bid={bid}"
            )

    def on_proposal(self, msg: Proposal):
        if msg.origin != self.agent_info:
            return
        
        req = msg.request_id
        self.proposals[req].append(msg)
        count = len(self.proposals[req])
        expected = self.expected_neighbors.get(req, 0)
        self.get_logger().info(
            f"Received Proposal req={req} from "
            f"({msg.proposer.position.x},{msg.proposer.position.y}) bid={msg.bid} "
            f"({count}/{expected})"
        )
        if count >= expected:
            self.evaluate_proposals(req)

    def evaluate_proposals(self, req: str):
        allp: list[Proposal] = self.proposals.pop(req, [])
        if not allp:
            return

        # ▸ nur endliche Gebote berücksichtigen
        finite = [p for p in allp if math.isfinite(p.bid)]
        if not finite:
            self.get_logger().info(
                f"No finite bids for req={req} → declaring FAILURE")
            self.awards.pop(req, None)           # Pfad-Stub holen
            
            self.pub_failure(req)
            self.get_logger().info(f"Published PathComplete req={req}")

            self.expected_neighbors.pop(req, None)
            return

        best = min(finite, key=lambda p: p.bid)
        aw: Award = self.awards.pop(req, Award())
        self.get_logger().info(
            f"Evaluating proposals req={req}, best bid={best.bid} "
            f"from ({best.proposer.position.x},{best.proposer.position.y})"
        )
        aw.winner = best.proposer
        aw.path.append(aw.winner)

        self.award_pub.publish(aw)
        self.expected_neighbors.pop(req, None)

    def pub_failure(self, req: str):
        comp = PathComplete()
        comp.request_id = req
        comp.status = 1
        self.comp_pub.publish(comp)
        

    def on_award(self, msg: Award):

        if msg.winner != self.agent_info:
            return

        req = msg.request_id
        self.get_logger().info(
            f"Received Award req={req}, winner=({msg.winner.position.x},{msg.winner.position.y})"
        )

        if any(gp == self.agent_info for gp in msg.path[:-1] ):
            self.pub_failure(req)
            return
        
        
        if msg.goal == self.agent_info.position:
            dt = self.get_clock().now() - self.req_t0.pop(req)
            comp = PathComplete()
            comp.request_id = req
            comp.status = 0
            comp.path = msg.path
            comp.cost = msg.cost
            self.get_logger().info(f"Published PathComplete req={req}")
            dt_ns = dt.nanoseconds                    # Gesamtdauer [ns]
            comp.planning_time = Duration()
            comp.planning_time.sec     = dt_ns // 1_000_000_000 
            comp.planning_time.nanosec = dt_ns % 1_000_000_000
            self.comp_pub.publish(comp)
            return
        self.expected_neighbors[req] = len(self.valid_moves)
        self.awards[req] = msg
        self.get_logger().info(
            f"Propagating Award to next round for req={req}, "
            f"neighbors={self.valid_moves}"
        )
        cfp = CFP()
        cfp.request_id = req
        cfp.current = self.agent_info
        cfp.goal    = msg.goal
        cfp.neighbors = [
            agent_info for agent_info in self.valid_moves
        ]
        self.cfp_pub.publish(cfp)

    
    def on_path_complete(self, msg: PathComplete):
        req = msg.request_id

        # Egal ob SUCCESS oder FAILURE → Zustand vollständig bereinigen
        self.get_logger().info(f"Cleaning up all state for request {req}")

        # Timer ggf. stoppen
        timer = self.convergence_timers.pop(req, None)
        if timer:
            self.destroy_timer(timer)

        # Alle bekannten Daten zur Request-ID entfernen
        self.proposals.pop(req, None)
        self.expected_neighbors.pop(req, None)
        self.awards.pop(req, None)
        self.cost_to_goal.pop(req, None)
        self.start_goals.pop(req, None)

        
    def move_cost(self, target_module_type: AgentInfo) -> float:

        is_human = self.module_type == 'mensch'
        is_human_weight = 1.0 if is_human else 0.0

        gripper_close_to_human_weight = 0.5 if (target_module_type.module_type == 'mensch' and self.module_type == 'greifer') else 0.0

        execution_time_map ={
            'greifer':   3.5,
            'mensch':    3.5,
            'rollen_ns': 1.0,
            'rollen_ow': 1.0
        }

        execution_time_weight = execution_time_map.get(self.module_type, 1.0)

        return execution_time_weight + is_human_weight + gripper_close_to_human_weight


def main():
    # rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    rclpy.init()
    node = AgentNode()

    # ---------- Signal-Handler ----------
    # def _on_signal(signum, frame):
    #     global _shutdown_requested
    #     if _shutdown_requested:      # nur einmal reagieren
    #         return
    #     _shutdown_requested = True
    #     node.get_logger().info(
    #         f'{signal.strsignal(signum)} empfangen – sende Unannounce …')
    #     node.send_unannounce()
    #     # einmal pumpen → Nachricht verlässt Prozess
    #     rclpy.spin_once(node, timeout_sec=0.1)

    # signal.signal(signal.SIGINT,  _on_signal)
    # signal.signal(signal.SIGTERM, _on_signal)
    # ------------------------------------

    try:
        while rclpy.ok() and not _shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
