import React from "react";
import { Html } from "@react-three/drei";
import { useStore } from "@/store";
import { FaPlay, FaFlagCheckered } from "react-icons/fa";

export const StartEndOverlay = () => {
  const start = useStore((state) => state.pathStart);
  const end = useStore((state) => state.pathGoal);

  return (
    <>
      {" "}
      {start && (
        <MarkerHtml
          position={[start.x + 0.5, 0.2, start.y + 0.5]}
          color="#4caf50"
        >
          <FaPlay size={20} />
        </MarkerHtml>
      )}
      {end && (
        <MarkerHtml position={[end.x + 0.5, 0.2, end.y + 0.5]} color="#f44336">
          <FaFlagCheckered size={20} />
        </MarkerHtml>
      )}
    </>
  );
};

type MarkerHtmlProps = {
  position: [number, number, number];
  color: string;
  children: React.ReactNode;
};

const MarkerHtml: React.FC<MarkerHtmlProps> = ({
  position,
  color,
  children,
}) => {
  return (
    <Html position={position} center distanceFactor={8}>
      <div style={{ pointerEvents: "none" }}>
        <div style={getDropStyle(color)}>
          <div style={iconWrapperStyle}>{children}</div>
        </div>
      </div>
    </Html>
  );
};

const getDropStyle = (color: string): React.CSSProperties => ({
  width: "48px",
  height: "48px",
  background: color,
  borderRadius: "50% 50% 50% 50%",
  position: "relative",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
  color: "white",
  boxShadow: "0 2px 6px rgba(0,0,0,0.3)",
});

const iconWrapperStyle: React.CSSProperties = {
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
  width: "32px",
  height: "32px",
};
