import { create } from "zustand";
import { createModeSlice, type ModeSlice } from "./modeSlice";
import { createGridSceneSlice, type GridSceneSlice } from "./gridSceneSlice";
import {
  createBackendConnectionSlice,
  type BackendConnectionSlice,
} from "./backendConnectionSlice";

// Hier alle Slices einfügen
export type Store = ModeSlice & GridSceneSlice & BackendConnectionSlice;

export const useStore = create<Store>()((set, get, store) => ({
  ...createModeSlice(set, get, store),
  ...createGridSceneSlice(set, get, store),
  ...createBackendConnectionSlice(set, get, store),
}));
