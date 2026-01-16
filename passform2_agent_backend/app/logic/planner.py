import heapq
import math
import logging
from typing import Dict, Tuple, List, Optional

logger = logging.getLogger("planner")

class PathPlanner:
    """
    Zentraler A*-Planer. 
    Gibt (path, cost, message) zurück, um dem Benutzer Feedback zu geben.
    """
    
    @staticmethod
    def get_allowed_dirs(module_type: str) -> List[Tuple[int, int]]:
        """Definiert die Bewegungsfreiheit basierend auf dem Modultyp."""
        if module_type in ("greifer", "mensch", "tisch"):
            return [(0, 1), (1, 0), (0, -1), (-1, 0)]
        if module_type == "rollen_ns":
            return [(0, 1), (0, -1)]
        if module_type == "rollen_ow":
            return [(1, 0), (-1, 0)]
        if module_type == "conveyeur":
            return [(0, 1), (1, 0), (0, -1), (-1, 0)]
        return []

    @staticmethod
    def calculate_move_cost(from_agent: dict, to_agent: dict) -> float:
        """Berechnet die gewichteten Kosten für einen Schritt."""
        m_type = from_agent.get("module_type", "unknown")
        is_human_weight = 1.0 if m_type == "mensch" else 0.0
        
        exec_weight = {
            "greifer": 3.5, 
            "mensch": 3.5, 
            "tisch": 3.5,
            "rollen_ns": 1.0, 
            "rollen_ow": 1.0,
            "conveyeur": 1.5
        }.get(m_type, 1.0)
        
        return exec_weight + is_human_weight

    def a_star(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int], grid: Dict[Tuple[int, int], dict]):
        """
        Berechnet den Pfad.
        Heuristik: Manhattan-Distanz $d(a, b) = |a_x - b_x| + |a_y - b_y|$
        """
        # --- 1. VORAB-CHECKS (Teile & Herrsche) ---
        if start_pos == goal_pos:
            return [grid.get(start_pos)], 0.0, "Start und Ziel sind identisch."

        if start_pos not in grid:
            return None, 0.0, f"Startpunkt {start_pos} ist nicht im Gitter (kein Agent)."

        if goal_pos not in grid:
            return None, 0.0, f"Zielpunkt {goal_pos} ist nicht im Gitter (kein Agent)."

        # --- 2. INITIALISIERUNG ---
        open_q = []
        # (priority, current_g_score, position)
        heapq.heappush(open_q, (0.0, 0.0, start_pos))
        
        g_score = {start_pos: 0.0}
        came_from = {}
        
        # Zur Diagnose: Wie viele Zellen haben wir untersucht?
        cells_visited = 0

        # --- 3. HAUPTSCHLEIFE ---
        while open_q:
            _, current_g, current = heapq.heappop(open_q)
            cells_visited += 1

            # Ziel erreicht?
            if current == goal_pos:
                path = self._reconstruct_path(came_from, current, grid)
                return path, g_score[current], f"Pfad mit {len(path)} Modulen gefunden."

            agent_info = grid.get(current)
            if not agent_info:
                continue

            # Nachbarn prüfen
            for dx, dy in self.get_allowed_dirs(agent_info.get("module_type", "")):
                neighbor = (current[0] + dx, current[1] + dy)
                
                # Ist dort überhaupt ein Agent? (Die "Lücken"-Prüfung)
                if neighbor not in grid:
                    continue
                
                neighbor_agent = grid[neighbor]
                tentative_g = current_g + self.calculate_move_cost(agent_info, neighbor_agent)
                
                if tentative_g < g_score.get(neighbor, math.inf):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self._heuristic(neighbor, goal_pos)
                    heapq.heappush(open_q, (f_score, tentative_g, neighbor))

        # --- 4. FEHLER-DIAGNOSE ---
        # Wenn wir hier landen, wurde die Queue leer, ohne das Ziel zu finden.
        return None, 0.0, "Keine Verbindung möglich. Prüfe auf Lücken oder falsche Modul-Ausrichtungen!"

    def _heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan-Distanz."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def _reconstruct_path(self, came_from: dict, current: Tuple[int, int], grid: dict) -> List[dict]:
        """Baut die Liste der Agenten-Daten vom Ziel zum Start auf."""
        path = []
        while current in came_from:
            path.append(grid[current])
            current = came_from[current]
        path.append(grid[current])
        path.reverse()
        return path

# Singleton Instanz
planner = PathPlanner()