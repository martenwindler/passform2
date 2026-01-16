import heapq
import math
import logging
from typing import Dict, Tuple, List, Optional

logger = logging.getLogger("planner")

class PathPlanner:
    """
    Zentraler A*-Planer. 
    Unterstützt jetzt zwei Modi:
    1. 'chain': Pfad nur über existierende Module (statisch).
    2. 'ftf': Pfad über leere Felder, statische Module sind Hindernisse.
    """
    
    @staticmethod
    def get_allowed_dirs(module_type: str, travel_mode: str = "chain") -> List[Tuple[int, int]]:
        """Definiert die Bewegungsfreiheit."""
        # Das FTF ist omnidirektional auf dem Gitter
        if travel_mode == "ftf" or module_type == "ftf":
            return [(0, 1), (1, 0), (0, -1), (-1, 0)]
            
        # Statische Module
        if module_type in ("greifer", "mensch", "tisch", "conveyeur"):
            return [(0, 1), (1, 0), (0, -1), (-1, 0)]
        if module_type == "rollen_ns":
            return [(0, 1), (0, -1)]
        if module_type == "rollen_ow":
            return [(1, 0), (-1, 0)]
        return []

    @staticmethod
    def calculate_move_cost(current_pos: Tuple[int, int], neighbor_pos: Tuple[int, int], grid: dict, travel_mode: str) -> float:
        """Berechnet die Kosten für einen Schritt."""
        if travel_mode == "ftf":
            # Das FTF hat konstante Kosten auf freiem Feld
            return 1.0
        
        # Klassische Kosten für Modul-Ketten
        from_agent = grid.get(current_pos, {})
        m_type = from_agent.get("module_type", "unknown")
        
        exec_weight = {
            "greifer": 3.0, 
            "mensch": 5.0, 
            "tisch": 1.0,
            "rollen_ns": 1.0, 
            "rollen_ow": 1.0,
            "conveyeur": 1.2
        }.get(m_type, 1.0)
        
        return exec_weight

    def a_star(self, start_pos: Tuple[int, int], goal_pos: Tuple[int, int], grid: Dict[Tuple[int, int], dict], travel_mode: str = "chain"):
        """
        Berechnet den optimalen Pfad.
        travel_mode="chain": Bewegung nur auf Agenten in 'grid'.
        travel_mode="ftf": Bewegung auf freien Feldern erlaubt, Agenten in 'grid' sind Hindernisse.
        """
        if start_pos == goal_pos:
            return [grid.get(start_pos, {"x": start_pos[0], "y": start_pos[1]})], 0.0, "Start und Ziel identisch."

        # Open Set: (priority, current_g_score, position)
        open_q = []
        heapq.heappush(open_q, (0.0, 0.0, start_pos))
        
        g_score = {start_pos: 0.0}
        came_from = {}

        while open_q:
            _, current_g, current = heapq.heappop(open_q)

            if current == goal_pos:
                path = self._reconstruct_path(came_from, current, grid)
                return path, g_score[current], f"Pfad gefunden ({travel_mode})."

            # Richtungen bestimmen
            current_agent = grid.get(current, {"module_type": "ftf" if travel_mode == "ftf" else "unknown"})
            directions = self.get_allowed_dirs(current_agent.get("module_type", ""), travel_mode)

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # --- VALIDIERUNG DER BEWEGUNG ---
                if travel_mode == "chain":
                    # Modus KETTE: Nur dort lang, wo ein Agent steht
                    if neighbor not in grid:
                        continue
                else:
                    # Modus FTF: Darf überall hin, ABER statische Agenten sind Hindernisse
                    # Ausnahme: Das Ziel selbst darf betreten werden (zum Aufnehmen/Absetzen)
                    if neighbor in grid and neighbor != goal_pos and neighbor != start_pos:
                        # Prüfen ob der Agent dort statisch ist
                        occ_agent = grid[neighbor]
                        if not occ_agent.get("is_dynamic", False):
                            continue # Hindernis!

                # Kosten berechnen
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
            # Wenn kein Agent im Grid ist (FTF auf freiem Feld), erzeugen wir ein temporäres Objekt
            node = grid.get(current, {"x": current[0], "y": current[1], "module_type": "empty"})
            path.append(node)
            current = came_from[current]
        
        start_node = grid.get(current, {"x": current[0], "y": current[1], "module_type": "empty"})
        path.append(start_node)
        path.reverse()
        return path

# Singleton Instanz
planner = PathPlanner()