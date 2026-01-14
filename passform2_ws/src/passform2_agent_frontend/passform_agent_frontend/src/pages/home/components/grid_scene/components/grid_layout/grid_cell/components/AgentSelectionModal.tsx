import React from "react";
import { useStore } from "@/store";
import { Html } from "@react-three/drei";
import * as THREE from "three";

type AgentSelectionModalProps = {
  position: [number, number, number];
  x: number;
  y: number;
  onClose: (type: string) => void;
};

const AgentSelectionModal: React.FC<AgentSelectionModalProps> = ({
  position,
  x,
  y,
  onClose,
}) => {
  const agentTypes = [
    { id: "rollen_ow", name: "Rollen OW" },
    { id: "rollen_ns", name: "Rollen NS" },
    { id: "greifer", name: "Greifer" },
    { id: "mensch", name: "Arbeitsplatte" },
    { id: "mensch_model", name: "Mensch" },
  ];

  const handleSelection = (type: string) => {
    onClose(type);
  };

  // Position leicht über der Zelle
  const htmlPosition: [number, number, number] = [
    position[0],
    position[1] + 0.5,
    position[2],
  ];

  return (
    <Html position={htmlPosition} center>
      <div
        style={{
          background: "white",
          padding: "15px",
          borderRadius: "8px",
          boxShadow: "0 4px 8px rgba(0, 0, 0, 0.2)",
          width: "200px",
          zIndex: 1000,
        }}
      >
        <h3
          style={{ margin: "0 0 10px 0", textAlign: "center", color: "black" }}
        >
          Agent auswählen
        </h3>
        <div style={{ display: "flex", flexDirection: "column", gap: "8px" }}>
          {" "}
          {agentTypes.map((type) => (
            <button
              key={type.id}
              onClick={() => handleSelection(type.id)}
              style={{
                padding: "8px 12px",
                cursor: "pointer",
                border: "1px solid #ccc",
                borderRadius: "4px",
                background: "#f0f0f0",
                color: "black",
                transition: "background 0.2s",
              }}
              onMouseOver={(e) =>
                (e.currentTarget.style.background = "#e0e0e0")
              }
              onMouseOut={(e) => (e.currentTarget.style.background = "#f0f0f0")}
            >
              {type.name}
            </button>
          ))}
          <button
            onClick={() => onClose("cancel")}
            style={{
              padding: "8px 12px",
              cursor: "pointer",
              border: "1px solid #ccc",
              borderRadius: "4px",
              background: "#ff6b6b",
              color: "white",
              marginTop: "5px",
              transition: "background 0.2s",
            }}
            onMouseOver={(e) => (e.currentTarget.style.background = "#ff5252")}
            onMouseOut={(e) => (e.currentTarget.style.background = "#ff6b6b")}
          >
            Abbrechen
          </button>
        </div>
      </div>
    </Html>
  );
};

export default AgentSelectionModal;
