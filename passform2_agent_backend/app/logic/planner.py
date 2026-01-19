import heapq
import math
import logging
from typing import Dict, Tuple, List, Optional, Any

# Wir importieren den config_manager, um auf die SSoT zuzugreifen
from app.managers.config_manager import config_manager

logger = logging.getLogger("planner")

class PathPlanner:
    """
    Zentraler A*-Planer, der das Contract Net Protocol simuliert.
    Berechnet Pfade basierend auf dynamischen Geboten (Weights) der Agenten.
    """

    def get_planning_weights(self) -> Dict[str, float]:
        """Holt die aktuellen Gewichte aus der Single Source of Truth."""
        return config_manager.current_data.get("config", {}).get("planning_weights", {
            "execution_time_default": 1.0,
            "complex_module_time": 3.5,
            "human_extra_weight": 1.0,
            "proximity_penalty": 0.5,
            "hardware_safety_factor": 1.2
        })

    def get_allowed_dirs(self, module_type: str) -> List[Tuple[int, int]]:
        """Definiert die Durchlassrichtung basierend auf dem Modultyp."""
        m_type = str(module_type).lower()
        
        # Omnidirektionale Module
        if m_type in ("greifer", "mensch", "tisch", "conveyeur", "ftf"):
            return [(0, 1), (1, 0), (0, -1), (-1, 0)]
        
        # Gerichtete Rollenmodule
        if m_type == "rollen_ns":
            return [(0, 1), (0, -1)]
        if m_type == "rollen_ow":
            return [(1, 0), (-1, 0)]
            
        return [(0, 1), (1, 0), (0, -1), (-1, 0)] # Fallback

    def calculate_move_cost(self, current_pos: Tuple[int, int], neighbor_pos: Tuple[int, int], 
                            grid: dict, weights: Dict[str, float]) -> float:
        """
        Kostenberechnung (Gebots-Logik). Hier fließen die Frontend-Parameter ein.
        """
        from_agent = grid.get(current_pos, {})
        to_agent = grid.get(neighbor_pos, {})
        
        m_type_from = str(from_agent.get("module_type", "unknown")).lower()
        m_type_to = str(to_agent.get("module_type", "unknown")).lower()

        # 1. Basis-Zeit (T_exec) bestimmen
        if m_type_from == "mensch":
            # Mensch nutzt komplexe Zeit + spezifisches Extra-Gewicht
            base_cost = weights.get("complex_module_time", 3.5)
            cost = base_cost + weights.get("human_extra_weight", 1.0)
        
        elif m_type_from == "greifer":
            # Greifer nutzt nur komplexe Zeit
            cost = weights.get("complex_module_time", 3.5)
            
        elif m_type_from in ["conveyeur", "rollen_ns", "rollen_ow", "tisch"]:
            # Standard-Module nutzen die Basis-Transportzeit
            cost = weights.get("execution_time_default", 1.0)
            
        else:
            cost = weights.get("execution_time_default", 1.0)

        # 2. Nähe-Strafe (W_proximity) - Interaktion Mensch/Maschine
        if m_type_from == "greifer" and m_type_to == "mensch":
            cost += weights.get("proximity_penalty", 0.5)
        
        # 3. Hardware Sicherheitsfaktor (Globaler Multiplikator)
        cost *= weights.get("hardware_safety_factor", 1.0)
        
        return cost

    def a_star(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int], 
               grid: Dict[Tuple[int, int], dict], travel_mode: str = "chain", 
               weights: Optional[Dict[str, float]] = None):
        """
        Kern-Algorithmus für die Pfadsuche.
        """
        if weights is None:
            weights = self.get_planning_weights()

        # Validierung
        if start_pos not in grid:
            return None, 0.0, f"Start {start_pos} ist kein Agent!"
        if goal_pos not in grid:
            return None, 0.0, f"Ziel {goal_pos} ist kein Agent!"

        if start_pos == goal_pos:
            return [grid.get(start_pos)], 0.0, "Identisch."

        logger.info(f"A* Suche: {start_pos} -> {goal_pos} | Mode: {travel_mode}")

        open_q = []
        # (f_score, g_score, position)
        heapq.heappush(open_q, (0.0, 0.0, start_pos))
        
        g_score = {start_pos: 0.0}
        came_from = {}

        while open_q:
            _, current_g, current = heapq.heappop(open_q)

            if current == goal_pos:
                path = self._reconstruct_path(came_from, current, grid)
                return path, g_score[current], "Erfolg"

            current_agent = grid.get(current, {})
            directions = self.get_allowed_dirs(current_agent.get("module_type", "unknown"))

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Im Chain-Modus nur über Agenten wandern
                if travel_mode == "chain" and neighbor not in grid:
                    continue

                step_cost = self.calculate_move_cost(current, neighbor, grid, weights)
                tentative_g = current_g + step_cost
                
                if tentative_g < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal_pos)
                    heapq.heappush(open_q, (f_score, tentative_g, neighbor))

        return None, 0.0, "Keine Pfadverbindung über Agenten gefunden."

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int], grid: dict) -> List[dict]:
        path = []
        while current in came_from:
            agent = grid.get(current, {"agent_id": f"node_{current[0]}_{current[1]}", "x": current[0], "y": current[1]})
            path.append(agent)
            current = came_from[current]
        
        # Start-Knoten hinzufügen
        start_agent = grid.get(current, {"agent_id": "start", "x": current[0], "y": current[1]})
        path.append(start_agent)
        path.reverse()
        return path

# Singleton
planner = PathPlanner()