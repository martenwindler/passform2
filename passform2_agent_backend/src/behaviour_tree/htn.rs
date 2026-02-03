use std::collections::{HashSet, HashMap};

#[derive(Debug, Clone, PartialEq, Eq)] 
pub struct WorldState(pub HashSet<String>);

pub struct Operator {
    pub pre: HashSet<String>,
    pub eff: HashSet<String>,
}

pub struct HTNPlanner {
    pub operators: HashMap<String, Operator>,
    pub methods: HashMap<String, Vec<Vec<String>>>,
}

impl HTNPlanner {
    pub fn plan(&self, mut state: WorldState, tasks: Vec<String>) -> Result<Vec<String>, String> {
        let mut final_plan = Vec::new();

        for task in tasks {
            if let Some(op) = self.operators.get(&task) {
                // Check Preconditions
                if op.pre.is_subset(&state.0) {
                    // Apply Effects
                    for p in &op.pre { state.0.remove(p); }
                    for e in &op.eff { state.0.insert(e.clone()); }
                    final_plan.push(task);
                } else {
                    return Err(format!("Preconditions failed for: {}", task));
                }
            } else if let Some(methods) = self.methods.get(&task) {
                // HTN findet hier die erste valide Methode
                for subtasks in methods {
                    match self.plan(state.clone(), subtasks.clone()) {
                        Ok(subplan) => {
                            // Update State basierend auf dem Subplan
                            for step in &subplan {
                                let op = &self.operators[step];
                                for p in &op.pre { state.0.remove(p); }
                                for e in &op.eff { state.0.insert(e.clone()); }
                            }
                            final_plan.extend(subplan);
                            break; // Methode gefunden
                        }
                        Err(_) => continue, // Nächste Methode probieren
                    }
                }
            }
        }
        Ok(final_plan)
    }
}