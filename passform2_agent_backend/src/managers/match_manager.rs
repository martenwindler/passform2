use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{info, warn, error};
use serde::{Deserialize, Serialize};
use crate::AppState;
use crate::core::logic::planner::PathPlanner;
use passform_agent_resources::msg::{MatchCFP, MatchAward, NavPathRequest};
use std::collections::HashMap;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MatchTask {
    pub id: String,
    pub task_type: String,
    pub priority: i32,
    pub status: String,
}

pub struct MatchManager {
    state: Arc<AppState>,
    planner: PathPlanner,
    // Aktive Verhandlungen: Task-ID -> Status
    pub active_matches: RwLock<HashMap<String, MatchTask>>,
}

impl MatchManager {
    pub fn new(state: Arc<AppState>) -> Self {
        Self {
            state,
            planner: PathPlanner::new(),
            active_matches: RwLock::new(HashMap::new()),
        }
    }

    /// Der Haupt-Loop des MatchManagers
    pub async fn run(&self) {
        info!("⚖️ MatchManager (CNP-Engine) gestartet.");
        // Hier könnte ein Intervall-Loop laufen, der nach neuen Aufgaben in einer Queue sucht
    }

    /// Startet eine Verhandlung (Contract Net Protocol)
    pub async fn start_negotiation(&self, task_id: String, start_pos: (i32, i32), goal_pos: (i32, i32)) {
        info!("📢 Starte Verhandlung für Task: {}", task_id);

        // 1. Weltmodell aus dem AgentManager holen
        let agents_guard = self.state.agent_manager.agents.read().await;
        
        // 2. Erstelle ein Grid-Snapshot für den A*-Planner
        let mut grid_snapshot = HashMap::new();
        for agent in agents_guard.values() {
            // Nur Agenten berücksichtigen, die nicht 'stale' (Watchdog) oder 'offline' sind
            if agent.status != "stale" && agent.status != "offline" {
                grid_snapshot.insert((agent.x, agent.y), agent.module_type.clone());
            }
        }

        // 3. Den günstigsten Pfad via A* (planner.rs) berechnen
        // Das ersetzt die langsame ROS-Cost-Propagation der Python-Nodes
        if let Some((path, total_cost)) = self.planner.a_star(start_pos, goal_pos, &grid_snapshot, None) {
            info!("✅ Pfad gefunden! Kosten: {:.2}, Agenten im Pfad: {}", total_cost, path.len());

            // 4. Den Zuschlag (MatchAward) erteilen
            // In deinem Fall: Wir finden den ersten fähigen Agenten am Startpunkt
            if let Some(winner_id) = self.find_agent_at_pos(start_pos, &agents_guard) {
                self.assign_task(&task_id, &winner_id, path, total_cost).await;
            } else {
                warn!("❌ Kein Agent an Startposition {:?} gefunden.", start_pos);
            }
        } else {
            error!("❌ Planung fehlgeschlagen: Kein Pfad von {:?} nach {:?} möglich.", start_pos, goal_pos);
        }
    }

    fn find_agent_at_pos(&self, pos: (i32, i32), agents: &HashMap<String, crate::managers::agent_manager::AgentEntry>) -> Option<String> {
        agents.values()
            .find(|a| (a.x, a.y) == pos && a.status == "active")
            .map(|a| a.agent_id.clone())
    }

    async fn assign_task(&self, task_id: &str, agent_id: &str, _path: Vec<(i32, i32)>, cost: f64) {
        info!("🏆 Task {} an Agent {} vergeben (Kosten: {})", task_id, agent_id, cost);
        
        // Hier nutzen wir MatchAward und NavPathRequest später, 
        // für jetzt unterdrücken wir die Warnung durch das Präfix '_'
        let _unused_award = MatchAward::default(); 
        
        // Update Dashboard via Socket.io
        let _ = self.state.socket_manager.io.emit("match_awarded", serde_json::json!({
            "task_id": task_id,
            "agent_id": agent_id,
            "cost": cost
        }));
    }
}