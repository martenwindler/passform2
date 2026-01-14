// socket/SocketManager.ts
import { useStore } from "@/store";
import type { AgentModule } from "@/store/gridSceneSlice";
import { io, Socket } from "socket.io-client";

class SocketManager {
  private static instance: SocketManager;
  private socket: Socket | null = null;
  private currentBackendUrl: string = "";
  private unsub: () => void = () => {};

  private constructor() {
    const state = useStore.getState();
    this.currentBackendUrl = state.backendIP; // z. B. "http://localhost:8000"
    this.connect(this.currentBackendUrl);

    // Listen for changes to backendUrl in the store
    this.unsub = useStore.subscribe((state) => {
      if (state.backendIP !== this.currentBackendUrl) {
        console.log("🔄 Backend-URL hat sich geändert:", state.backendIP);
        this.reconnect(state.backendIP);
      }
    });
  }

  public static getInstance(): SocketManager {
    if (!SocketManager.instance) {
      SocketManager.instance = new SocketManager();
    }
    return SocketManager.instance;
  }

  private connect(ip: string) {
    this.socket = io("http://" + ip + ":8000", {
      transports: ["websocket"],
      reconnection: true,
    });

    this.setupListeners();
  }

  private reconnect(newUrl: string) {
    if (this.socket) {
      this.socket.disconnect();
    }
    this.currentBackendUrl = newUrl;
    this.connect(newUrl);
  }

  private setupListeners() {
    if (!this.socket) return;

    this.socket.on("connect", () => {
      console.log("✅ Socket.IO verbunden mit", this.currentBackendUrl);
      useStore.getState().setConnected(true);
    });

    this.socket.on("disconnect", () => {
      console.warn("❌ Socket.IO getrennt");
      useStore.getState().setConnected(false);
    });

    this.socket.on("connect_error", (error) => {
      console.error("❌ Socket.IO Verbindungsfehler:", error);
      useStore.getState().setConnected(false);
    });

    this.socket.on("connect_timeout", (timeout) => {
      console.error("❌ Socket.IO Verbindungszeitüberschreitung:", timeout);
      useStore.getState().setConnected(false);
    });

    this.socket.on("error", (attempt) => {
      console.log("🔄 Versuche erneut zu verbinden:", attempt);
      useStore.getState().setConnected(false);
    });

    this.socket.on("active_agents", (data) => {
      const setAgentMap = useStore.getState().setAgentMap;
      const agents = new Map<string, AgentModule>();
      console.log("Aktive Agenten:", data);

      data.agents.forEach((agent: any) => {
        agents.set(`${agent.position.x}_${agent.position.y}`, agent);
      });
      setAgentMap(agents);
    });

    this.socket.on("path_complete", (data) => {
      console.log("Pfadplanung abgeschlossen:", data);
      const setCurrentPath = useStore.getState().setCurrentPath;
      setCurrentPath(data);
    });

    this.socket.on("mode", (data) => {
      const mode = useStore.getState().mode;
      const setMode = useStore.getState().setMode;
      const setCurrentPath = useStore.getState().setCurrentPath;
      const setPathStart = useStore.getState().setPathStart;
      const setPathGoal = useStore.getState().setPathGoal;
      const setEditing = useStore.getState().setEditing;
      console.log("Mode update:", data);

      if (mode !== data) {
        console.log("Modus geändert:", data);
        setMode(data);
        setCurrentPath(null); // Reset current path when mode changes
        setPathStart(null); // Reset start point when mode changes
        setPathGoal(null); // Reset goal point when mode changes
        if (data !== "simulation") {
          setEditing(false);
        }
      }
    });
  }

  public getSocket(): Socket | null {
    return this.socket;
  }

  public destroy() {
    this.unsub(); // Stop listening for changes
    this.socket?.disconnect();
    this.socket = null;
  }
}

export const socketManager = SocketManager.getInstance();
