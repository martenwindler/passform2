import { useStore } from "@/store";
import { Text } from "@react-three/drei";

export const HoverOverlay = () => {
  const hovered = useStore((s) => s.hoveredCell);
  const editing = useStore((s) => s.editing);
  const agent = useStore((s) => s.agentMap.get(`${hovered?.x}_${hovered?.y}`));
  const isPanning = useStore((s) => s.isPanning);
  const cellInteractionDisabled = useStore((s) => s.cellInteractionDisabled);

  if (!hovered || !editing || agent || isPanning || cellInteractionDisabled)
    return null;

  return (
    <group position={[hovered.x + 0.5, 0.1, hovered.y + 0.5]}>
      <Text
        fontSize={0.4}
        color="lime"
        anchorX="center"
        anchorY="middle"
        position={[0, 0.03, 0]}
        rotation={[-Math.PI / 2, 0, 0]}
      >
        +
      </Text>
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.01, 0]}>
        <ringGeometry args={[0.35, 0.45, 32]} />
        <meshBasicMaterial color="lime" opacity={0.6} transparent />
      </mesh>
    </group>
  );
};
