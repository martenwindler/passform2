import { type StateCreator } from "zustand";
import { type Store } from "./index"; // wichtig für Typen

export type RosDuration = {
  sec: number; // Sekunden
  nanosec: number; // Nanosekunden
};

export type Path = {
  request_id: string; // eindeutige ID für den Pfad
  status: number;
  planning_time: RosDuration;
  cost: number;
  path: AgentModule[]; // Array von Agenten, die den Pfad repräsentieren
};

export type AgentModule = {
  agent_id?: string; // optional, falls nicht immer gesetzt
  module_type: string; // z.B. "rollen_ns", "rollen_ow", "greifer", "mensch"
  position: GridCell; // Position im Grid
};

export type GridCell = {
  x: number;
  y: number;
};

export type GridSceneSlice = {
  dreiD: boolean;
  setDreiD: (dreiD: boolean) => void;

  editing: boolean;
  setEditing: (editing: boolean) => void;

  isPanning: boolean;
  setIsPanning: (editing: boolean) => void;

  hoveredCell: null | GridCell;
  setHoveredCell: (pos: GridCell | null) => void;

  cellInteractionDisabled: boolean;
  setCellInteractionDisabled: (open: boolean) => void;

  agentMap: Map<string, AgentModule>;
  setAgentMap: (map: Map<string, AgentModule>) => void;
  setAgentAt: (x: number, y: number, agent: AgentModule) => void;
  removeAgentAt: (x: number, y: number) => void;

  pathStart: null | GridCell;
  setPathStart: (pos: GridCell | null) => void;
  pathGoal: null | GridCell;
  setPathGoal: (pos: GridCell | null) => void;

  currentPath: null | Path;
  setCurrentPath: (path: Path | null) => void;
};

// Vollständig typisierte Factory-Funktion
export const createGridSceneSlice: StateCreator<
  Store,
  [],
  [],
  GridSceneSlice
> = (set, get, store) => ({
  dreiD: false,
  setDreiD: (mode) => set({ dreiD: mode }),

  editing: true,
  setEditing: (editing) => set({ editing }),

  isPanning: false,
  setIsPanning: (editing) => set({ isPanning: editing }),

  hoveredCell: null,
  setHoveredCell: (pos) => set({ hoveredCell: pos }),

  cellInteractionDisabled: false,
  setCellInteractionDisabled: (open) => set({ cellInteractionDisabled: open }),

  agentMap: new Map(),
  setAgentMap: (map) => {
    console.log("AgentMap", map);
    set({ agentMap: map });
  },
  setAgentAt: (x, y, agent) => {
    const map = new Map(get().agentMap);
    map.set(`${x}_${y}`, agent);
    set({ agentMap: map });
  },
  removeAgentAt: (x, y) => {
    const map = new Map(get().agentMap);
    map.delete(`${x}_${y}`);
    set({ agentMap: map });
  },

  pathStart: null,
  setPathStart: (pos) => set({ pathStart: pos }),
  pathGoal: null,
  setPathGoal: (pos) => set({ pathGoal: pos }),

  currentPath: null,
  setCurrentPath: (path) => {
    set({ currentPath: path });
  },
});
