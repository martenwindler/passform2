#!/usr/bin/env python3
# central_planner_node.py
from __future__ import annotations
import heapq
import math
from typing import Dict, Tuple, List, Optional

import rclpy
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.timer import Timer
from builtin_interfaces.msg import Duration

from passform_agent_msgs.msg import (
    AgentAnnounce,
    AgentInfo,
    PathRequest,
    PathComplete,
)

# ====================================================================== #
#                         Hilfsfunktionen                                #
# ====================================================================== #
def allowed_dirs(module_type: str) -> List[Tuple[int, int]]:
    if module_type in ("greifer", "mensch"):
        return [(0, 1), (1, 0), (0, -1), (-1, 0)]
    if module_type == "rollen_ns":
        return [(0, 1), (0, -1)]
    if module_type == "rollen_ow":
        return [(1, 0), (-1, 0)]
    return []

def move_cost(from_mod: AgentInfo, to_mod: AgentInfo) -> float:
    is_human_weight               = 1.0 if from_mod.module_type == "mensch"  else 0.0
    gripper_close_to_human_weight = 0.5 if (
        to_mod.module_type == "mensch" and from_mod.module_type == "greifer"
    ) else 0.0

    exec_weight = {
        "greifer":   3.5,
        "mensch":    3.5,
        "rollen_ns": 1.0,
        "rollen_ow": 1.0,
    }.get(from_mod.module_type, 1.0)

    return exec_weight + is_human_weight + gripper_close_to_human_weight


# ====================================================================== #
#                         Central Planner Node                           #
# ====================================================================== #
class CentralPlannerNode(Node):
    """
    Zentraler A*-Planer:
    * übernimmt Agenten-Heartbeats (10 Hz) und hält Live-Weltmodell
    * beantwortet `PathRequest`s mit einem Pfad + Kosten
    """

    # ------------------------------------------------------------------ #
    def __init__(self) -> None:
        os.environ["ROS_DOMAIN_ID"] = str(1)
        super().__init__("central_planner_node")

        # -------------------- Heartbeat-Parameter -------------------- #
        self.heartbeat_period: float          = 1      # 10 Hz
        self.timeout_factor:    int           = 3        # 0,3 s Timeout
        self.heartbeat_timers: Dict[str,Timer] = {}      # ★
        self.agent_positions:  Dict[str,Tuple[int,int]] = {}  # id → (x,y) ★
        # ------------------------------------------------------------- #

        # (x,y) → AgentInfo   ⇒ Basis fürs A*
        self.grid: Dict[Tuple[int, int], AgentInfo] = {}

        self.create_subscription(
            AgentAnnounce, "agent_announce", self.on_agent_announce,1
        )

        self.create_subscription(
            PathRequest, "path_request_central", self.on_path_request, 10
        )
        self.comp_pub = self.create_publisher(
            PathComplete, "path_complete", 10
        )

        self.get_logger().info("CentralPlannerNode gestartet, warte auf Agenten…")


    # ================================================================= #
    #                        Agent / Heartbeats                          #
    # ================================================================= #
    def _reset_heartbeat_timer(self, agent_info: AgentInfo) -> None:   # ★
        """(Re)initialisiert den Timeout-Timer eines Agenten."""
        agent_id = agent_info.agent_id

        # ▸ vorherigen Timer stoppen, falls vorhanden
        old = self.heartbeat_timers.pop(agent_id, None)
        if old:
            self.destroy_timer(old)

        # ▸ Position in `grid` aktualisieren
        new_pos = (agent_info.position.x, agent_info.position.y)
        prev_pos: Optional[Tuple[int,int]] = self.agent_positions.get(agent_id)
        if prev_pos and prev_pos in self.grid:
            self.grid.pop(prev_pos, None)                  # alte Zelle räumen
        self.grid[new_pos]          = agent_info
        self.agent_positions[agent_id] = new_pos

        # ▸ neuen Timeout-Timer anlegen
        def _on_timeout() -> None:
            # Agent gilt als offline → aus Modellen entfernen
            if agent_id in self.agent_positions:
                pos = self.agent_positions.pop(agent_id)
                self.grid.pop(pos, None)
                self.get_logger().warning(f"Agent offline (Timeout): {agent_id}")
            t = self.heartbeat_timers.pop(agent_id, None)
            if t:
                self.destroy_timer(t)

        timeout = self.timeout_factor * self.heartbeat_period
        self.heartbeat_timers[agent_id] = self.create_timer(timeout, _on_timeout)

    # ------------------------------------------------------------------ #
    def on_agent_announce(self, msg: AgentAnnounce) -> None:
        """
        Jeder eingehende AgentAnnounce ist ein Heartbeat.
        `msg.active` wird ignoriert – Timeout-Mechanismus ist maßgeblich.
        """
        try:
            self._reset_heartbeat_timer(msg.agent_info)
        except Exception as exc:     # pragma: no-cover
            self.get_logger().error(f"Fehler in on_agent_announce: {exc}")

    # ================================================================= #
    #                       Behandlung PathRequest                       #
    # ================================================================= #
    def on_path_request(self, msg: PathRequest) -> None:
        req_id = msg.request_id
        t0     = self.get_clock().now()
        start  = (msg.start.x, msg.start.y)
        goal   = (msg.goal.x,  msg.goal.y)

        self.get_logger().info(f"PathRequest {req_id}: start={start} → goal={goal}")

        # ▸ Existenzprüfung
        if start not in self.grid or goal not in self.grid:
            self._publish_complete(req_id, status=1)
            self.get_logger().warning(f"Start/Goal fehlt für req={req_id} – abgebrochen")
            return

        # ▸ A*-Suche
        success, path_ids = self._a_star_path(start, goal)
        if not success:
            self._publish_complete(req_id, status=1)
            self.get_logger().info(f"Kein Pfad gefunden für req={req_id}")
            return

        # ▸ Kosten summieren ★
        total_cost = 0.0
        for i in range(len(path_ids) - 1):
            a, b = path_ids[i], path_ids[i + 1]
            total_cost += move_cost(self.grid[a], self.grid[b])

        # ▸ Nachricht füllen & publizieren
        dt = self.get_clock().now() - t0
        comp = PathComplete()
        comp.request_id = req_id
        comp.status     = 0
        comp.path       = [self.grid[p] for p in path_ids]

        comp.planning_time          = Duration()
        comp.planning_time.sec      = dt.nanoseconds // 1_000_000_000
        comp.planning_time.nanosec  = dt.nanoseconds % 1_000_000_000
        comp.cost = float(total_cost)                             # ★

        self.comp_pub.publish(comp)
        self.get_logger().info(
            f"Pfad publiziert (req={req_id}, Länge={len(path_ids)}, "
            f"Kosten={total_cost:.2f})"
        )

    # ================================================================= #
    #                         A*-Suche                                   #
    # ================================================================= #
    def _a_star_path(
        self, start: Tuple[int, int], goal: Tuple[int, int]
    ) -> Tuple[bool, List[Tuple[int, int]]]:

        open_q: List[Tuple[float, float, Tuple[int, int]]] = []
        heapq.heappush(open_q, (0.0, 0.0, start))

        g_cost:   Dict[Tuple[int, int], float]            = {start: 0.0}
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}

        while open_q:
            _, g, current = heapq.heappop(open_q)
            if current == goal:
                return True, self._reconstruct(came_from, current)

            cur_info = self.grid[current]
            for dx, dy in allowed_dirs(cur_info.module_type):
                nxt = (current[0] + dx, current[1] + dy)
                if nxt not in self.grid:
                    continue
                tentative_g = g + move_cost(cur_info, self.grid[nxt])
                if tentative_g < g_cost.get(nxt, math.inf):
                    g_cost[nxt] = tentative_g
                    came_from[nxt] = current
                    f = tentative_g + self._heuristic(nxt, goal)
                    heapq.heappush(open_q, (f, tentative_g, nxt))

        return False, []          # Kein Pfad

    @staticmethod
    def _heuristic(a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    @staticmethod
    def _reconstruct(
        came_from: Dict[Tuple[int, int], Tuple[int, int]], current: Tuple[int, int]
    ) -> List[Tuple[int, int]]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    # ================================================================= #
    #                         Helper                                     #
    # ================================================================= #
    def _publish_complete(self, req_id: str, status: int) -> None:
        comp = PathComplete()
        comp.request_id = req_id
        comp.status     = status
        comp.cost       = 0.0          # ★ immer initialisiert
        self.comp_pub.publish(comp)


# ====================================================================== #
#                                 main                                   #
# ====================================================================== #
def main() -> None:
    rclpy.init()
    node = CentralPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
