import React, { useCallback, useEffect, useRef, useState } from "react";
import { useStore } from "@/store";
import { Conveyor } from "./components/agent_modules/conveyor/Conveyor";
import { Gripper } from "./components/agent_modules/gripper/Gripper";
import AgentSelectionModal from "./components/AgentSelectionModal";
import type { ThreeEvent } from "@react-three/fiber";

type GridCellProps = {
  x: number;
  y: number;
};

const GridCell = React.memo(({ x, y }: GridCellProps) => {
  const [showAgentSelectionMenu, setShowAgentSelectionMenu] = useState(false);
  const [showAgentSettings, setShowAgentSettings] = useState(false);
  const [loading, setLoading] = useState(false);

  const timeoutRef = useRef<number | null>(null);
  const previousAgentRef = useRef<any>(null);

  const setCellInteractionDisabled = useStore(
    (s) => s.setCellInteractionDisabled
  );
  const setHoveredCell = useStore((s) => s.setHoveredCell);
  const agent = useStore((s) => s.agentMap.get(`${x}_${y}`));

  const backendIP = useStore((s) => s.backendIP);

  const killAgentNode = async () => {
    const response = await fetch(
      "http://" + backendIP + ":8000/api/nodes/kill_agent_node",
      {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ node_id: `A_${x}_${y}` }),
      }
    );

    const data = await response.json();
    if (!response.ok) {
      throw new Error(`Fehler: ${data.detail || response.statusText}`);
    }
  };

  const startAgentNode = async (type: string) => {
    const response = await fetch(
      "http://" + backendIP + ":8000/api/nodes/start_agent_node",
      {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          node_id: `A_${x}_${y}`,
          position: [x, y],
          module_type: type,
        }),
      }
    );

    const data = await response.json();

    if (!response.ok) {
      throw new Error(`Fehler: ${data.detail || response.statusText}`);
    }
  };

  const handleClick = (e: ThreeEvent<MouseEvent>) => {
    e.stopPropagation();
    const { editing, isPanning, cellInteractionDisabled } = useStore.getState();
    if (isPanning || cellInteractionDisabled) return;

    previousAgentRef.current = agent;

    if (agent) {
      setShowAgentSettings(true);
    } else {
      if (!editing) {
        return;
      }
      setShowAgentSelectionMenu(true);
      setLoading(true);

      // Timeout-Schutz nach 15 Sekunden
      if (timeoutRef.current) clearTimeout(timeoutRef.current);
      timeoutRef.current = setTimeout(() => {
        setLoading(false);
      }, 15000);
    }
    setCellInteractionDisabled(true);
  };

  const addHumenAgent = () => {
    try {
      // 2. Freie Nachbarzelle für Mensch-Figur finden
      const neighbors = [
        { x: x - 1, y }, // links
        { x: x + 1, y }, // rechts
        { x, y: y - 1 }, // oben
        { x, y: y + 1 }, // unten
      ];
      const agentMap = useStore.getState().agentMap;

      const freeNeighbors = neighbors.filter((pos) => {
        const key = `${pos.x}_${pos.y}`;
        return agentMap.get(key)?.module_type === "mensch_model";
      });

      if (freeNeighbors.length === 0) {
        alert(
          "Die Arbeitsplatte konnte nicht hinzugefügt werden, da sie direkt neben einem Mensch platziert werden muss."
        );
        setLoading(false);
        return;
      }

      // 4. Mensch-Figur als separaten Agent hinzufügen
      startAgentNode("mensch");
    } catch (error) {
      console.error("❌ Fehler beim Hinzufügen des Menschen:", error);
      alert(`Fehler beim Hinzufügen des Menschen: ${error}`);
    }
  };

  // ⏱ Automatisch Beenden des Loading-Zustands, wenn Agent hinzugefügt oder entfernt wurde
  useEffect(() => {
    if (!loading) return;

    const prev = previousAgentRef.current;

    if (prev && !agent) {
      // Agent war vorher da, jetzt entfernt → fertig
      setLoading(false);
      previousAgentRef.current = null;
    }

    if (!prev && agent) {
      // Agent war vorher nicht da, jetzt da → fertig
      setLoading(false);
      previousAgentRef.current = null;
    }
  }, [agent, loading]);

  return (
    <>
      <group position={[x + 0.5, 0.01, y + 0.5]}>
        <mesh
          onPointerOver={() => {
            setHoveredCell({ x, y });
          }}
          onPointerOut={() => {
            setHoveredCell(null);
          }}
          onClick={handleClick}
          position={[0, 0, 0]}
          rotation={[-Math.PI / 2, 0, 0]}
        >
          <planeGeometry args={[1, 1]} />
          <meshBasicMaterial visible={false} />
        </mesh>
        {/* Ladeanzeige */}
        {loading && <LoadingSpinner />} {/* Agenten-Auswahlmenü */}
        {showAgentSelectionMenu && (
          <AgentSelectionModal
            position={[0, 0, 0]}
            x={x}
            y={y}
            onClose={(type: string) => {
              if (type !== "cancel") {
                if (type === "mensch") {
                  addHumenAgent();
                } else {
                  startAgentNode(type);
                }
              } else {
                setLoading(false);
              }
              setShowAgentSelectionMenu(false);
              setTimeout(() => {
                setCellInteractionDisabled(false);
              }, 50);
            }}
          />
        )}
      </group>

      {/* Agenten-Visualisierung */}
      {agent && !loading && (
        <>
          {agent.module_type === "rollen_ns" && (
            <Conveyor position={[x, y]} direction={1} scale={0.00165} />
          )}
          {agent.module_type === "rollen_ow" && (
            <Conveyor position={[x, y]} direction={0} scale={0.00165} />
          )}
          {agent.module_type === "greifer" && <Gripper position={[x, y]} />}{" "}
          {agent.module_type === "mensch" && (
            <group position={[x, 0.0, y]} castShadow>
              <Mensch position={[0, 0]} />
            </group>
          )}
          {agent.module_type === "mensch_model" && (
            <group position={[x, 0.0, y]} castShadow>
              {/* Nur die Mensch-Figur ohne Arbeitsplatte */}
              <mesh position={[0.5, 0.4, 0.5]} castShadow>
                <capsuleGeometry args={[0.3, 0.8, 8, 16]} />
                <meshStandardMaterial color="#2d6a9f" />
              </mesh>
            </group>
          )}
        </>
      )}

      {showAgentSettings && agent && (
        <group position={[x, 0.05, y]}>
          <AgentSettingsModal
            position={[0, 0, 0]}
            x={x}
            y={y}
            onClose={() => {
              setShowAgentSettings(false);
              setTimeout(() => {
                setCellInteractionDisabled(false);
              }, 50);
            }}
            setLoading={setLoading}
          ></AgentSettingsModal>
        </group>
      )}
    </>
  );
});

export default GridCell;

import { useFrame } from "@react-three/fiber";
import * as THREE from "three";
import AgentSettingsModal from "./components/AgentSettingsModal";
import { Mensch } from "./components/agent_modules/mensch/Mensch";

// 🔄 Rotierender Ladekreis als Komponente
const LoadingSpinner = () => {
  const ref = useRef<THREE.Mesh>(null);

  useFrame(() => {
    if (ref.current) {
      ref.current.rotation.z += 0.1; // dreht sich um Z-Achse
    }
  });

  return (
    <mesh ref={ref} rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.01, 0]}>
      <ringGeometry
        args={[0.35, 0.45, 32, 1, 0, Math.PI * 1.6]} // 1.6 Pi ≈ offener Bereich
      />
      <meshBasicMaterial color="orange" />
    </mesh>
  );
};
