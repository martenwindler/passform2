import { Canvas } from "@react-three/fiber";
import { Grid } from "@react-three/drei";

import React from "react";
import { Box, Button, VStack } from "@chakra-ui/react";
import { useStore } from "@/store";
import { CameraController } from "./components/CamaraController";
import { HoverOverlay } from "./components/HoverOverlay";
import { GridLayout } from "./components/grid_layout/GridLayout";
import { StartEndOverlay } from "./components/StartEndOverlay";
import { FlowingArrowPath } from "./components/AnimatedPath";

export const GridScene: React.FC = () => {
  const dreiD = useStore((state) => state.dreiD);
  const editing = useStore((state) => state.editing);
  const mode = useStore((state) => state.mode);
  const setEditing = useStore((state) => state.setEditing);
  const setDreiD = useStore((state) => state.setDreiD);
  const backendIP = useStore((state) => state.backendIP);

  const killAllNodes = async () => {
    const response = await fetch(
      "http://" + backendIP + ":8000/api/nodes/kill_all_nodes",
      {
        method: "POST",
        headers: { "Content-Type": "application/json" },
      }
    );

    const data = await response.json();
    if (!response.ok) {
      throw new Error(`Fehler: ${data.detail || response.statusText}`);
    }
  };

  const testPath = [
    { x: 0, y: 0 },
    { x: 1, y: 0 },
    { x: 1, y: 1 },
    { x: 2, y: 1 },
  ];

  return (
    <Box
      position="relative"
      width="100%"
      height="70%"
      border="1px solid white"
      bg="black"
    >
      {/* 👉 Overlay-Button oben links */}
      <Button
        size="sm"
        position="absolute"
        top="10px"
        left="10px"
        zIndex={10}
        onClick={() => setDreiD(!dreiD)}
        colorScheme="teal"
        variant={dreiD ? "solid" : "outline"}
      >
        {dreiD ? "3D-Ansicht" : "2D-Ansicht"}
      </Button>

      <VStack
        position="absolute"
        top="10px"
        right="10px"
        alignItems={"flex-end"}
      >
        {mode === "simulation" && (
          <Button
            size="sm"
            zIndex={10}
            onClick={() => setEditing(!editing)}
            colorScheme="teal"
            variant={editing ? "solid" : "outline"}
          >
            {editing ? "Bearbeiten" : "Bearbeiten"}
          </Button>
        )}
        {mode === "simulation" && editing && (
          <Button
            mt={2}
            size="sm"
            zIndex={10}
            onClick={() => killAllNodes()}
            colorScheme="teal"
            variant={editing ? "solid" : "outline"}
          >
            {"Reset"}
          </Button>
        )}
      </VStack>

      {/* 🎥 3D Szene */}
      <Canvas style={{ width: "100%", height: "100%", display: "block" }}>
        <Grid
          cellSize={1.0}
          position={[0, 0, 0]}
          cellColor={"#6f6f6f"}
          cellThickness={1}
          args={[32, 32]}
          sectionSize={100000}
          sectionThickness={0}
        />
        <FlowingArrowPath arrowSize={0.1} arrowSpeed={0.5} arrowSpacing={0.3} />
        <GridLayout></GridLayout>
        <HoverOverlay></HoverOverlay>
        <StartEndOverlay></StartEndOverlay>
        <directionalLight
          castShadow
          position={[0, 20, 0]}
          intensity={5.0}
          shadow-mapSize-width={1024}
          shadow-mapSize-height={1024}
        />
        <ambientLight intensity={5.0} />
        <CameraController />
      </Canvas>
    </Box>
  );
};
