import heapq
import math
import logging
from typing import Dict, Tuple, List, Optional, Any

# Wir importieren den config_manager, um auf die SSoT zuzugreifen
from app.managers.config_manager import config_manager

logger = logging.getLogger("planner")

class PathPlanner:
    """
    Zentraler A*-Planer, der seine Gewichte aus der config.json bezieht.
    """

    def get_planning_weights(self) -> Dict[str, float]:
        """Holt die aktuellen Gewichte aus der Single Source of Truth."""
        return config_manager.current_data.get("config", {}).get("planning_weights", {
            "execution_time_default": 1.0,
            "complex_module_time": 3.5,
            "human_extra_weight": 1.0,
            "proximity_penalty": 0.5
        })

    def get_allowed_dirs(self, module_type: str, travel_mode: str = "chain") -> List[Tuple[int, int]]:
        """Definiert die Bewegungsfreiheit basierend auf dem Modultyp."""
        if travel_mode == "ftf" or module_type == "ftf":
            return [(0, 1), (1, 0), (0, -1), (-1, 0)]
            
        if module_type in ("greifer", "mensch", "tisch", "conveyeur"):
            return [(0, 1), (1, 0), (0, -1), (-1, 0)]
        if module_type == "rollen_ns":
            return [(0, 1), (0, -1)]
        if module_type == "rollen_ow":
            return [(1, 0), (-1, 0)]
        return []

    def calculate_move_cost(self, current_pos: Tuple[int, int], neighbor_pos: Tuple[int, int], 
                            grid: dict, travel_mode: str) -> float:
        """
        Berechnet die Kosten basierend auf der SSoT-Formel:
        $Cost = T_{exec} + W_{human} + W_{proximity}$
        """
        weights = self.get_planning_weights()
        
        if travel_mode == "ftf":
            return weights.get("execution_time_default", 1.0)
        
        # --- Modus KETTE (Chain) ---
        from_agent = grid.get(current_pos, {})
        to_agent = grid.get(neighbor_pos, {})
        
        m_type_from = from_agent.get("module_type", "unknown")
        m_type_to = to_agent.get("module_type", "unknown")

        # 1. Basis-Zeit (T_exec)
        if m_type_from in ["greifer", "mensch"]:
            cost = weights.get("complex_module_time", 3.5)
        else:
            cost = weights.get("execution_time_default", 1.0)

        # 2. Mensch-Zuschlag (W_human)
        if m_type_from == "mensch":
            cost += weights.get("human_extra_weight", 1.0)

        # 3. Nähe-Strafe (W_proximity): Greifer gibt Bauteil an Mensch
        if m_type_from == "greifer" and m_type_to == "mensch":
            cost += weights.get("proximity_penalty", 0.5)
        
        return cost

    def a_star(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int], 
               grid: Dict[Tuple[int, int], dict], travel_mode: str = "chain"):
        """Berechnet den optimalen Pfad unter Einbeziehung der SSoT-Gewichte."""
        if start_pos == goal_pos:
            return [grid.get(start_pos, {"x": start_pos[0], "y": start_pos[1]})], 0.0, "Identisch."

        open_q = []
        heapq.heappush(open_q, (0.0, 0.0, start_pos))
        
        g_score = {start_pos: 0.0}
        came_from = {}

        while open_q:
            _, current_g, current = heapq.heappop(open_q)

            if current == goal_pos:
                path = self._reconstruct_path(came_from, current, grid)
                return path, g_score[current], f"Pfad gefunden ({travel_mode})."

            current_agent = grid.get(current, {"module_type": "ftf" if travel_mode == "ftf" else "unknown"})
            directions = self.get_allowed_dirs(current_agent.get("module_type", ""), travel_mode)

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Validierung
                if travel_mode == "chain":
                    if neighbor not in grid: continue
                else:
                    if neighbor in grid and neighbor != goal_pos and neighbor != start_pos:
                        if not grid[neighbor].get("is_dynamic", False): continue

                tentative_g = current_g + self.calculate_move_cost(current, neighbor, grid, travel_mode)
                
                if tentative_g < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal_pos)
                    heapq.heappush(open_q, (f_score, tentative_g, neighbor))

        return None, 0.0, "Kein Pfad möglich."

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int], grid: dict) -> List[dict]:
        path = []
        while current in came_from:
            node = grid.get(current, {"x": current[0], "y": current[1], "module_type": "empty"})
            path.append(node)
            current = came_from[current]
        
        start_node = grid.get(current, {"x": current[0], "y": current[1], "module_type": "empty"})
        path.append(start_node)
        path.reverse()
        return path

# Singleton
planner = PathPlanner()