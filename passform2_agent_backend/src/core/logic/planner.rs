use std::collections::{BinaryHeap, HashMap};
use std::cmp::Ordering;

// --- HILFSSTRUKTUREN FÜR A* ---

#[derive(Copy, Clone, PartialEq)]
struct Node {
    f_score: f64,
    g_score: f64,
    pos: (i32, i32),
}

// BinaryHeap in Rust ist ein Max-Heap. Für A* brauchen wir einen Min-Heap.
// Daher drehen wir die Logik bei Ord/PartialOrd um.
impl Eq for Node {}
impl Ord for Node {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_score.partial_cmp(&self.f_score).unwrap_or(Ordering::Equal)
    }
}
impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

// --- DER PLANNER ---

pub struct PathPlanner {
    // Standard-Gewichte (können später aus dem ConfigManager kommen)
    pub default_weights: HashMap<String, f64>,
}

impl PathPlanner {
    pub fn new() -> Self {
        let mut weights = HashMap::new();
        weights.insert("execution_time_default".into(), 1.0);
        weights.insert("complex_module_time".into(), 3.5);
        weights.insert("human_extra_weight".into(), 1.0);
        weights.insert("proximity_penalty".into(), 0.5);
        weights.insert("hardware_safety_factor".into(), 1.2);

        Self { default_weights: weights }
    }

    /// Bestimmt erlaubte Richtungen basierend auf dem Modultyp
    fn get_allowed_dirs(&self, module_type: &str) -> Vec<(i32, i32)> {
        match module_type.to_lowercase().as_str() {
            "rollen_ns" => vec![(0, 1), (0, -1)],
            "rollen_ow" => vec![(1, 0), (-1, 0)],
            _ => vec![(0, 1), (1, 0), (0, -1), (-1, 0)], // Omnidirektional
        }
    }

    /// Die Kostenberechnung (deine Gebots-Logik)
    fn calculate_move_cost(
        &self,
        from_type: &str,
        to_type: &str,
        weights: &HashMap<String, f64>,
    ) -> f64 {
        let mut cost = match from_type.to_lowercase().as_str() {
            "mensch" => weights.get("complex_module_time").unwrap_or(&3.5) + weights.get("human_extra_weight").unwrap_or(&1.0),
            "greifer" => *weights.get("complex_module_time").unwrap_or(&3.5),
            _ => *weights.get("execution_time_default").unwrap_or(&1.0),
        };

        // Nähe-Strafe
        if from_type == "greifer" && to_type == "mensch" {
            cost += weights.get("proximity_penalty").unwrap_or(&0.5);
        }

        // Globaler Sicherheitsfaktor
        cost * weights.get("hardware_safety_factor").unwrap_or(&1.0)
    }

    /// Kern-Algorithmus: A*
    pub fn a_star(
        &self,
        start: (i32, i32),
        goal: (i32, i32),
        grid: &HashMap<(i32, i32), String>, // (Pos) -> (ModuleType)
        weights_override: Option<HashMap<String, f64>>,
    ) -> Option<(Vec<(i32, i32)>, f64)> {
        let weights = weights_override.unwrap_or(self.default_weights.clone());

        let mut open_q = BinaryHeap::new();
        let mut g_score = HashMap::new();
        let mut came_from = HashMap::new();

        g_score.insert(start, 0.0);
        open_q.push(Node { f_score: 0.0, g_score: 0.0, pos: start });

        while let Some(Node { pos: current, g_score: current_g, .. }) = open_q.pop() {
            if current == goal {
                return Some((self.reconstruct_path(came_from, current), current_g));
            }

            let module_type = grid.get(&current).map(|s| s.as_str()).unwrap_or("unknown");
            
            for (dx, dy) in self.get_allowed_dirs(module_type) {
                let neighbor = (current.0 + dx, current.1 + dy);
                
                if let Some(to_type) = grid.get(&neighbor) {
                    let step_cost = self.calculate_move_cost(module_type, to_type, &weights);
                    let tentative_g = current_g + step_cost;

                    if tentative_g < *g_score.get(&neighbor).unwrap_or(&f64::INFINITY) {
                        came_from.insert(neighbor, current);
                        g_score.insert(neighbor, tentative_g);
                        let f_score = tentative_g + self.heuristic(neighbor, goal);
                        open_q.push(Node { f_score, g_score: tentative_g, pos: neighbor });
                    }
                }
            }
        }
        None
    }

    fn heuristic(&self, a: (i32, i32), b: (i32, i32)) -> f64 {
        ((a.0 - b.0).abs() + (a.1 - b.1).abs()) as f64
    }

    fn reconstruct_path(&self, came_from: HashMap<(i32, i32), (i32, i32)>, mut current: (i32, i32)) -> Vec<(i32, i32)> {
        let mut path = vec![current];
        while let Some(&prev) = came_from.get(&current) {
            path.push(prev);
            current = prev;
        }
        path.reverse();
        path
    }
}