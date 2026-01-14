import { type StateCreator } from "zustand";
import { type Store } from "./index"; // wichtig für Typen

export type Mode = "simulation" | "hardware";

export type ModeSlice = {
  mode: Mode;
  setMode: (mode: Mode) => void;
};

// Vollständig typisierte Factory-Funktion
export const createModeSlice: StateCreator<Store, [], [], ModeSlice> = (
  set,
  get,
  store
) => ({
  mode: "simulation",
  setMode: (mode) => set({ mode }),
});
