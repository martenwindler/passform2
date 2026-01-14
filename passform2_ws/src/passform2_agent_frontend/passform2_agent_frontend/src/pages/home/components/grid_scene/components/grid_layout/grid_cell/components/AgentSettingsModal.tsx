import React from "react";
import { Html } from "@react-three/drei";
import { useStore } from "@/store";
import { FaPlay, FaMapMarkerAlt, FaTrash, FaTimes } from "react-icons/fa";

type PathSelectionMenuProps = {
  position: [number, number, number];
  x: number;
  y: number;
  onClose: () => void;
  setLoading: (loading: boolean) => void;
};

const PathSelectionMenu: React.FC<PathSelectionMenuProps> = ({
  position,
  x,
  y,
  onClose,
  setLoading,
}) => {
  const mode = useStore((state) => state.mode);

  const handleSet = (target: "start" | "end") => {
    if (target === "start") {
      useStore.setState((state) => ({
        ...state,
        pathStart: { x, y },
        currentPath: null, // Reset current path when setting new start
      }));
    }
    if (target === "end") {
      useStore.setState((state) => ({
        ...state,
        pathGoal: { x, y },
        currentPath: null,
      }));
    }
    onClose();
  };

  const handleRemove = async () => {
    const backendIP = useStore.getState().backendIP;

    fetch(`http://${backendIP}:8000/api/nodes/kill_agent_node`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ node_id: `A_${x}_${y}` }),
    });
    setLoading(true);
    const agentMap = useStore.getState().agentMap;
    if (agentMap.get(`${x}_${y}`)?.module_type === "mensch_model") {
      const neighbors = [
        { x: x - 1, y }, // links
        { x: x + 1, y }, // rechts
        { x, y: y - 1 }, // oben
        { x, y: y + 1 }, // unten
      ];
      const workingAreasToRemove = neighbors.filter((pos) => {
        const key = `${pos.x}_${pos.y}`;
        return agentMap.get(key)?.module_type === "mensch";
      });
      for (const area of workingAreasToRemove) {
        fetch(`http://${backendIP}:8000/api/nodes/kill_agent_node`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ node_id: `A_${area.x}_${area.y}` }),
        });
      }
    }

    onClose();
  };

  const htmlPosition: [number, number, number] = [
    position[0] + 0.5,
    position[1] + 1,
    position[2] + 0.5,
  ];

  return (
    <Html position={htmlPosition} center>
      <div
        style={{
          background: "#ffffff",
          padding: "8px 12px",
          borderRadius: "6px",
          boxShadow: "0 4px 12px rgba(0, 0, 0, 0.15)",
          display: "flex",
          alignItems: "center",
          gap: "10px",
          zIndex: 1000,
          fontFamily: "sans-serif",
        }}
      >
        <button
          style={stripButton("#4caf50")}
          onClick={() => handleSet("start")}
        >
          <FaPlay style={{ marginRight: "6px" }} />
          Start
        </button>
        <button style={stripButton("#f44336")} onClick={() => handleSet("end")}>
          <FaMapMarkerAlt style={{ marginRight: "6px" }} />
          Ziel
        </button>
        {mode === "simulation" && (
          <button style={stripButton("#999", "#777")} onClick={handleRemove}>
            <FaTrash style={{ marginRight: "6px" }} />
            Entfernen
          </button>
        )}
        <div style={{ flexGrow: 1 }} />
        <button
          onClick={onClose}
          style={{
            ...stripButton("#ddd", "#aaa", "#333"),
            fontWeight: "bold",
            width: "32px",
            height: "32px",
            fontSize: "16px",
            padding: 0,
            display: "flex",
            justifyContent: "center",
            alignItems: "center",
          }}
        >
          <FaTimes />
        </button>
      </div>
    </Html>
  );
};

const stripButton = (
  bg: string,
  hoverBg: string = "",
  color: string = "white"
): React.CSSProperties => ({
  padding: "6px 12px",
  background: bg,
  color,
  border: "none",
  borderRadius: "4px",
  fontWeight: 500,
  cursor: "pointer",
  transition: "background 0.2s",
  fontSize: "14px",
  minWidth: "70px",
  textAlign: "center",
  display: "flex",
  alignItems: "center",
  ...(hoverBg
    ? {
        onMouseOver: (e: any) => (e.currentTarget.style.background = hoverBg),
        onMouseOut: (e: any) => (e.currentTarget.style.background = bg),
      }
    : {}),
});

export default PathSelectionMenu;
