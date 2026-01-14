import { useFrame } from "@react-three/fiber";
import * as THREE from "three";
import React, { useMemo, useRef, useState } from "react";
import { useStore } from "@/store";

interface FlowingArrowPathProps {
  arrowSpacing?: number;
  arrowSpeed?: number;
  arrowSize?: number;
  color?: string;
  cornerRadius?: number;
}

export const FlowingArrowPath: React.FC<FlowingArrowPathProps> = ({
  arrowSpacing = 1,
  arrowSpeed = 1,
  arrowSize = 0.4,
  color = "#00BFFF",
  cornerRadius = 0.15,
}) => {
  const groupRef = useRef<THREE.Group>(null);
  const [offset, setOffset] = useState(0);
  const currentPath = useStore((s) => s.currentPath);

  /* ---------- 1. KURVE AUFBAUEN (mit Entschärfungen) ---------- */
  const curve = useMemo<THREE.CurvePath<THREE.Vector3>>(() => {
    const path = new THREE.CurvePath<THREE.Vector3>();
    if (!currentPath) return path;

    /* 1-a) Rohpunkte laden und Duplikate entfernen */
    const raw = currentPath.path.map(
      (p) => new THREE.Vector3(p.position.x + 0.5, 0.1, p.position.y + 0.5)
    );
    const verts = raw.filter(
      (v, i) => i === 0 || v.distanceToSquared(raw[i - 1]) > 1e-10
    );

    if (verts.length < 2) return path;

    /* Helfer: nur Segmente > EPS Länge übernehmen */
    const pushIfLonger = (c: THREE.Curve<THREE.Vector3>, eps = 1e-6) => {
      if (c.getLength() > eps) path.add(c);
    };

    let prev = verts[0].clone();

    for (let i = 1; i < verts.length; i++) {
      const curr = verts[i].clone();

      /* Letztes Segment oder kein Radius → gerade Linie */
      if (i === verts.length - 1 || cornerRadius === 0) {
        pushIfLonger(new THREE.LineCurve3(prev, curr));
        prev = curr;
        continue;
      }

      const next = verts[i + 1].clone();
      const v1 = curr.clone().sub(prev).normalize();
      const v2 = next.clone().sub(curr).normalize();

      /* Winkel & Geraden-Check */
      let dot = THREE.MathUtils.clamp(v1.dot(v2), -1, 1);
      const angle = Math.acos(dot);
      if (angle < 1e-3 || Math.PI - angle < 1e-3) {
        pushIfLonger(new THREE.LineCurve3(prev, curr));
        prev = curr;
        continue;
      }

      const half = angle * 0.5;
      const maxRadius =
        Math.min(curr.distanceTo(prev), next.distanceTo(curr)) * 0.5;
      const r = Math.min(cornerRadius, maxRadius);

      const denom = Math.tan(half);
      const safeDen = Math.abs(denom) < 1e-6 ? 1e-6 : denom;

      const start = curr.clone().sub(v1.multiplyScalar(r / safeDen));
      const end = curr.clone().add(v2.multiplyScalar(r / safeDen));

      pushIfLonger(new THREE.LineCurve3(prev, start));
      pushIfLonger(new THREE.QuadraticBezierCurve3(start, curr, end));

      prev = end;
    }

    return path;
  }, [cornerRadius, currentPath]);
  /* ---------- 2. LÄNGE & ANIMATION ---------- */
  const totalLength = useMemo(() => {
    try {
      if (curve.curves.length === 0) {
        return 0;
      }

      const length = curve.getLength();

      // Fallback bei NaN oder 0
      if (isNaN(length) || length <= 0) {
        let manualLength = 0;
        for (const segment of curve.curves) {
          const segLength = segment.getLength();
          if (!isNaN(segLength) && segLength > 0) {
            manualLength += segLength;
          }
        }
        return manualLength;
      }

      return length;
    } catch (error) {
      console.warn("Error calculating curve length:", error);
      return 0;
    }
  }, [curve]);

  const arrowCount =
    totalLength > 0 ? Math.max(1, Math.floor(totalLength / arrowSpacing)) : 0;
  const effectiveSpacing = arrowCount > 0 ? totalLength / arrowCount : 0;

  const arrowShape = useMemo(() => {
    const s = new THREE.Shape();
    const l = arrowSize * 2,
      w = arrowSize * 2;
    s.moveTo(0, -w);
    s.lineTo(w, 0);
    s.lineTo(0, w);
    s.lineTo(-l, w);
    s.lineTo(-l + w, 0);
    s.lineTo(-l, -w);
    s.closePath();
    return s;
  }, [arrowSize]);
  useFrame((_, delta) => {
    if (totalLength > 0 && !isNaN(totalLength)) {
      setOffset((o) => (o + arrowSpeed * delta) % totalLength);
    }
  });
  /* ---------- 3. RENDER ---------- */
  return (
    <group ref={groupRef}>
      {currentPath &&
        totalLength > 0 &&
        !isNaN(totalLength) &&
        Array.from({ length: arrowCount }).map((_, i) => {
          const s = (i * effectiveSpacing + offset) % totalLength;
          const u = s / totalLength;

          // Sicherheitscheck für u
          if (isNaN(u) || u < 0 || u > 1) {
            return null;
          }

          try {
            const pos = curve.getPointAt(u);
            const tangent = curve.getTangentAt(u);

            // Null-Check für tangent
            if (!pos || !tangent) {
              return null;
            }

            const normalizedTangent = tangent.normalize();
            const angle = Math.atan2(-normalizedTangent.z, normalizedTangent.x);

            return (
              <mesh
                key={i}
                position={pos.toArray()}
                rotation={[-Math.PI / 2, 0, angle]}
              >
                <shapeGeometry args={[arrowShape]} />
                <meshBasicMaterial color={color} />
              </mesh>
            );
          } catch (error) {
            console.warn(`Error rendering arrow ${i}:`, error);
            return null;
          }
        })}
    </group>
  );
};
