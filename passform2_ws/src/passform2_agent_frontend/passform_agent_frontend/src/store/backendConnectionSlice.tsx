import { type StateCreator } from "zustand";
import { type Store } from "./index"; // wichtig für Typen

export type Mode = "simulation" | "hardware";

export type BackendConnectionSlice = {
  connected: boolean;
  setConnected: (connected: boolean) => void;

  backendIP: string;
  setBackendIP: (ip: string) => void;
};

// Vollständig typisierte Factory-Funktion
export const createBackendConnectionSlice: StateCreator<
  Store,
  [],
  [],
  BackendConnectionSlice
> = (set, get, store) => ({
  connected: false,
  setConnected: (connected) => set({ connected }),

  backendIP: "localhost",
  setBackendIP: (ip) => set({ backendIP: ip }),
});
