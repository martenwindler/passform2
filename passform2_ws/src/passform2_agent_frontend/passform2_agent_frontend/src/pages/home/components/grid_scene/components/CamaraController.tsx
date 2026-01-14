import { useFrame } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import { useRef, useEffect } from "react";
import * as THREE from "three";
import { useStore } from "@/store";

export const CameraController = () => {
  const controlsRef = useRef<OrbitControlsImpl>(null!);
  const dreiD = useStore((state) => state.dreiD);
  const setIsPanning = useStore((state) => state.setIsPanning);

  const isAnimatingRef = useRef(false);
  const animationTarget = useRef({
    position: new THREE.Vector3(),
    target: new THREE.Vector3(),
    quaternion: new THREE.Quaternion(),
  });

  const prevDreiDRef = useRef(dreiD);

  // Letzter Zustand pro Modus
  const last3DStateRef = useRef<{
    position: THREE.Vector3;
    target: THREE.Vector3;
    quaternion: THREE.Quaternion;
  }>({
    position: new THREE.Vector3(10, 10, 10),
    target: new THREE.Vector3(0, 0, 0),
    quaternion: new THREE.Quaternion(),
  });

  const last2DStateRef = useRef<{
    position: THREE.Vector3;
    target: THREE.Vector3;
    quaternion: THREE.Quaternion;
  }>({
    position: new THREE.Vector3(0, 20, 0),
    target: new THREE.Vector3(0, 0, 0),
    quaternion: new THREE.Quaternion(),
  });

  const hasEntered3D = useRef(false);
  const hasEntered2D = useRef(false);
  const isUserControllingRef = useRef(false);

  useEffect(() => {
    const controls = controlsRef.current;
    if (!controls) return;

    const cam = controls.object;
    const tgt = controls.target;

    const getQuaternion = (
      position: THREE.Vector3,
      target: THREE.Vector3,
      up = new THREE.Vector3(0, 1, 0)
    ) => {
      const dummy = new THREE.PerspectiveCamera();
      dummy.up.copy(up);
      dummy.position.copy(position);
      dummy.lookAt(target);
      return dummy.quaternion.clone();
    };

    // Initialstart
    if (!hasEntered3D.current && !hasEntered2D.current) {
      const initialPos = dreiD
        ? new THREE.Vector3(10, 10, 10)
        : new THREE.Vector3(0, 20, 0);
      const initialTarget = new THREE.Vector3(0, 0, 0);
      const upVector = dreiD
        ? new THREE.Vector3(0, 1, 0)
        : new THREE.Vector3(1, 0, 0);

      // Direkt setzen statt animieren beim ersten Start
      cam.position.copy(initialPos);
      tgt.copy(initialTarget);
      cam.up.copy(upVector);
      cam.lookAt(initialTarget);

      if (dreiD) {
        hasEntered3D.current = true;
        last3DStateRef.current.position.copy(initialPos);
        last3DStateRef.current.target.copy(initialTarget);
        last3DStateRef.current.quaternion.copy(cam.quaternion);
      } else {
        hasEntered2D.current = true;
        last2DStateRef.current.position.copy(initialPos);
        last2DStateRef.current.target.copy(initialTarget);
        last2DStateRef.current.quaternion.copy(cam.quaternion);
      }

      controls.update();
      return;
    }

    // 3D → 2D
    if (prevDreiDRef.current && !dreiD) {
      // Aktuellen 3D-Zustand speichern
      last3DStateRef.current.position.copy(cam.position);
      last3DStateRef.current.target.copy(tgt);
      last3DStateRef.current.quaternion.copy(cam.quaternion);

      const usePos = hasEntered2D.current
        ? last2DStateRef.current.position.clone()
        : new THREE.Vector3(0, 20, 0);
      const useTgt = hasEntered2D.current
        ? last2DStateRef.current.target.clone()
        : new THREE.Vector3(0, 0, 0);

      animationTarget.current.position.copy(usePos);
      animationTarget.current.target.copy(useTgt);
      animationTarget.current.quaternion.copy(
        getQuaternion(usePos, useTgt, new THREE.Vector3(1, 0, 0))
      );

      isAnimatingRef.current = true;
      hasEntered2D.current = true;
    }

    // 2D → 3D
    if (!prevDreiDRef.current && dreiD) {
      // Aktuellen 2D-Zustand speichern
      last2DStateRef.current.position.copy(cam.position);
      last2DStateRef.current.target.copy(tgt);
      last2DStateRef.current.quaternion.copy(cam.quaternion);

      const usePos = hasEntered3D.current
        ? last3DStateRef.current.position.clone()
        : new THREE.Vector3(10, 10, 10);
      const useTgt = hasEntered3D.current
        ? last3DStateRef.current.target.clone()
        : new THREE.Vector3(0, 0, 0);

      animationTarget.current.position.copy(usePos);
      animationTarget.current.target.copy(useTgt);
      animationTarget.current.quaternion.copy(
        getQuaternion(usePos, useTgt, new THREE.Vector3(0, 1, 0))
      );

      isAnimatingRef.current = true;
      hasEntered3D.current = true;
    }

    prevDreiDRef.current = dreiD;
  }, [dreiD]);

  useFrame(() => {
    const controls = controlsRef.current;
    if (!controls) return;

    // Wenn Nutzer kontrolliert, keine Animation ausführen
    if (isUserControllingRef.current) {
      return;
    }

    if (isAnimatingRef.current) {
      const cam = controls.object;
      const tgt = controls.target;
      const toPos = animationTarget.current.position;
      const toTgt = animationTarget.current.target;
      const toQuat = animationTarget.current.quaternion;

      // Sanftere Animation mit angepasstem Faktor
      const animSpeed = dreiD ? 0.07 : 0.05;
      cam.position.lerp(toPos, animSpeed);
      tgt.lerp(toTgt, animSpeed);
      cam.quaternion.slerp(toQuat, animSpeed);

      // Up-Vektor frühzeitig setzen für bessere Animation
      cam.up.set(dreiD ? 0 : 1, dreiD ? 1 : 0, 0);

      const positionDone = cam.position.distanceTo(toPos) < 0.05;
      const targetDone = tgt.distanceTo(toTgt) < 0.05;
      const quaternionDone = 1 - cam.quaternion.dot(toQuat) < 0.002;

      if (positionDone && targetDone && quaternionDone) {
        // Animation beenden und Zielwerte exakt setzen
        cam.position.copy(toPos);
        tgt.copy(toTgt);
        cam.quaternion.copy(toQuat);
        isAnimatingRef.current = false;
      }

      controls.update();
    }
  });

  // Ref für den Timeout, der nach 200 ms ausgeführt wird
  const disableEditingTimeoutRef = useRef<number | null>(null);
  // Flag, ob wir editing wirklich schon deaktiviert haben
  const hasDisabledEditingRef = useRef(false);

  // === START / END Handler ===

  const handleControlStart = () => {
    isUserControllingRef.current = true;
    isAnimatingRef.current = false;

    // Reset Flags + bestehenden Timeout clearen
    hasDisabledEditingRef.current = false;
    if (disableEditingTimeoutRef.current !== null) {
      clearTimeout(disableEditingTimeoutRef.current);
    }

    // Timeout setzen: Wenn wir nach 200 ms noch im „Start“-Zustand sind,
    // dann war es kein Klick, sondern eine Pan/Zoom-Interaktion:
    disableEditingTimeoutRef.current = window.setTimeout(() => {
      setIsPanning(true);
      hasDisabledEditingRef.current = true;
    }, 200);
  };

  const handleControlEnd = () => {
    isUserControllingRef.current = false;

    // Timeout abbrechen, falls er noch nicht gefeuert hat (-> kurzer Klick)
    if (disableEditingTimeoutRef.current !== null) {
      clearTimeout(disableEditingTimeoutRef.current);
      disableEditingTimeoutRef.current = null;
    }

    setTimeout(() => {
      setIsPanning(false);
    }, 50);
  };

  return (
    <OrbitControls
      ref={controlsRef}
      makeDefault
      enableRotate={dreiD}
      onStart={handleControlStart}
      onEnd={handleControlEnd}
      minDistance={1}
      maxDistance={100}
      enableDamping={true}
      dampingFactor={0.1}
    />
  );
};
