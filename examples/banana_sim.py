#!/usr/bin/env python3
"""
banana_sim.py - Real-to-Sim pipeline para manos de bananas.

Carga el point cloud guardado por banana_capture.py y lo simula en PyBullet.
"""
import sys
import time
import json
import socket
import argparse
import subprocess
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data

ROOT = Path(__file__).resolve().parents[1]
SCENE_FILE = ROOT / "captures" / "banana_scene.npz"
TARGETS_FILE = ROOT / "captures" / "banana_targets.json"


# ─────────────────────────────────────────────
# 1. CAPTURA (subprocess con PYTHONNOUSERSITE)
# ─────────────────────────────────────────────

def capture_scene(ip, num_frames=5):
    """Lanza banana_capture.py en subproceso separado para evitar conflicto de libs."""
    print(f"\n📷 Capturando escena real ({num_frames} frames)...")
    capture_script = Path(__file__).parent / "banana_capture.py"

    result = subprocess.run(
        ["env", "PYTHONNOUSERSITE=1", "/usr/bin/python3",
         str(capture_script), ip, str(num_frames)],
        capture_output=False,
    )

    if result.returncode != 0:
        print("❌ Error en captura. Verifica la cámara.")
        sys.exit(1)

    if not SCENE_FILE.exists() or not TARGETS_FILE.exists():
        print("❌ Archivos de captura no generados.")
        sys.exit(1)


# ─────────────────────────────────────────────
# 2. CARGAR DATOS
# ─────────────────────────────────────────────

def load_scene():
    data = np.load(SCENE_FILE)
    xyz_mm = data["xyz_mm"]
    bgr = data["bgr"]

    targets_raw = json.loads(TARGETS_FILE.read_text())
    targets = [
        {
            "xyz_mm": np.array(t["xyz_mm"]),
            "approach_mm": np.array(t["approach_mm"]),
            "pixel_uv": tuple(t["pixel_uv"]),
            "confidence": t["confidence"],
        }
        for t in targets_raw
    ]

    print(f"\n📂 Escena cargada: xyz{xyz_mm.shape}")
    print(f"   Bananas: {len(targets)}")
    return xyz_mm, bgr, targets


# ─────────────────────────────────────────────
# 3. ESCENA PYBULLET
# ─────────────────────────────────────────────

def build_scene(xyz_mm, targets, gui):
    print(f"\n🎮 Construyendo escena PyBullet...")

    client = p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)
    p.loadURDF("plane.urdf")

    # Point cloud → metros
    valid = np.isfinite(xyz_mm[:, :, 2])
    pts_m = xyz_mm[valid] * 0.001
    scene_center = pts_m.mean(axis=0)

    # Visualizar muestra del cloud
    stride = max(1, len(pts_m) // 800)
    for pt in pts_m[::stride]:
        p.addUserDebugLine(
            pt.tolist(),
            (pt + [0, 0, 0.001]).tolist(),
            lineColorRGB=[0.5, 0.4, 0.2],
            lineWidth=1,
        )

    # Bananas como esferas con física
    banana_colors = [
        [0.95, 0.85, 0.05, 1.0],
        [0.85, 0.75, 0.05, 1.0],
        [0.75, 0.65, 0.05, 1.0],
    ]

    for i, t in enumerate(targets):
        pos_m = t["xyz_mm"] * 0.001
        approach_m = t["approach_mm"] * 0.001
        color = banana_colors[i % len(banana_colors)]

        vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.04, rgbaColor=color)
        col = p.createCollisionShape(p.GEOM_SPHERE, radius=0.04)
        p.createMultiBody(
            baseMass=0.15,
            baseCollisionShapeIndex=col,
            baseVisualShapeIndex=vis,
            basePosition=pos_m.tolist(),
        )

        # Línea approach
        p.addUserDebugLine(
            pos_m.tolist(), approach_m.tolist(),
            lineColorRGB=[0, 1, 0], lineWidth=2,
        )

        # Label
        p.addUserDebugText(
            f"🍌 {i+1}  {t['confidence']:.0%}",
            (pos_m + [0, 0, 0.08]).tolist(),
            textColorRGB=[1, 1, 0],
            textSize=1.2,
        )

    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance=1.2,
            cameraYaw=45,
            cameraPitch=-35,
            cameraTargetPosition=scene_center.tolist(),
        )
        print("  👁  Ventana 3D abierta (rota con el mouse)")
        time.sleep(1.5)

    print(f"  ✓ {len(targets)} bananas en escena")
    return client, scene_center


# ─────────────────────────────────────────────
# 4. SIMULAR PICK
# ─────────────────────────────────────────────

def simulate_pick(target, gui):
    pick_m = target["xyz_mm"] * 0.001
    approach_m = target["approach_mm"] * 0.001
    retract_m = approach_m.copy()
    retract_m[2] += 0.15

    vis = p.createVisualShape(p.GEOM_CYLINDER, radius=0.02, length=0.08,
                               rgbaColor=[0.6, 0.6, 0.8, 0.9])
    col = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.02, height=0.08)
    gripper = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=col,
                                 baseVisualShapeIndex=vis,
                                 basePosition=retract_m.tolist())

    waypoints = [retract_m, approach_m, pick_m, approach_m, retract_m]
    labels    = ["Inicio", "Approach", "Pick ✊", "Retract", "Final"]
    collision  = False

    for wp, label in zip(waypoints, labels):
        cur = np.array(p.getBasePositionAndOrientation(gripper)[0])
        for s in range(25):
            interp = cur + (wp - cur) * (s / 24)
            p.resetBasePositionAndOrientation(gripper, interp.tolist(), [0,0,0,1])
            p.stepSimulation()
            if p.getContactPoints(gripper):
                if any(c[2] != 0 for c in p.getContactPoints(gripper)):
                    collision = True
            if gui:
                time.sleep(0.015)

        print(f"    [{label}] → ({wp[0]:.3f}, {wp[1]:.3f}, {wp[2]:.3f})m")

    p.removeBody(gripper)
    return not collision


# ─────────────────────────────────────────────
# 5. ENVIAR AL ROBOT
# ─────────────────────────────────────────────

def send_to_robot(target, host, port):
    payload = {
        "pick": target["xyz_mm"].tolist(),
        "approach": target["approach_mm"].tolist(),
        "confidence": float(target["confidence"]),
        "pixel": list(target["pixel_uv"]),
        "validated": True,
    }
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        sock.connect((host, port))
        sock.sendall((json.dumps(payload) + "\n").encode())
        resp = sock.recv(1024).decode().strip()
        sock.close()
        ok = resp.lower() == "ok"
        print(f"  Robot respondió: {resp} → {'✅' if ok else '❌'}")
        return ok
    except ConnectionRefusedError:
        print(f"  ❌ Robot no disponible en {host}:{port}")
        return False
    except Exception as e:
        print(f"  ❌ Error TCP: {e}")
        return False


# ─────────────────────────────────────────────
# 6. MAIN
# ─────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Real-to-Sim banana picking")
    parser.add_argument("--ip", default="192.168.1.101")
    parser.add_argument("--robot-host", default="127.0.0.1")
    parser.add_argument("--robot-port", type=int, default=5005)
    parser.add_argument("--no-gui", action="store_true")
    parser.add_argument("--no-robot", action="store_true")
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--use-cached", action="store_true",
                        help="Usar captura anterior sin recapturar")
    args = parser.parse_args()

    gui = not args.no_gui

    print("=" * 60)
    print("🍌 BANANA PICKING: Real-to-Sim con PyBullet")
    print("=" * 60)

    # PASO 1: Capturar (o usar cache)
    if not args.use_cached or not SCENE_FILE.exists():
        capture_scene(args.ip, args.frames)
    else:
        print(f"\n📂 Usando captura en cache: {SCENE_FILE.name}")

    # PASO 2: Cargar datos
    xyz_mm, bgr, targets = load_scene()

    if not targets:
        print("❌ Sin bananas detectadas. Pon bananas frente a la cámara.")
        return

    # PASO 3: Escena PyBullet
    client, scene_center = build_scene(xyz_mm, targets, gui)

    # PASO 4: Simular picks
    print(f"\n🔄 Simulando {len(targets)} picks...")
    valid = []
    for i, t in enumerate(targets):
        print(f"\n  Banana {i+1}/{len(targets)} (conf={t['confidence']:.0%})")
        ok = simulate_pick(t, gui)
        status = "✅ válido" if ok else "❌ colisión"
        print(f"  Resultado: {status}")
        if ok:
            valid.append(t)

    print(f"\n📊 {len(valid)}/{len(targets)} picks válidos")

    if not valid:
        print("❌ Sin picks seguros.")
        p.disconnect()
        return

    # Mejor pick: más cercano con mayor confianza
    best = sorted(valid, key=lambda t: t["xyz_mm"][2])[0]

    print(f"\n⭐ Mejor pick:")
    print(f"   xyz   = {best['xyz_mm']}")
    print(f"   conf  = {best['confidence']:.0%}")

    if gui:
        pos_m = best["xyz_mm"] * 0.001
        p.addUserDebugText("★ MEJOR PICK", (pos_m + [0,0,0.12]).tolist(),
                           textColorRGB=[1, 0.4, 0], textSize=2.0)
        time.sleep(1.5)

    # PASO 5: Enviar al robot
    if not args.no_robot:
        confirm = input(f"\n¿Enviar al robot {args.robot_host}:{args.robot_port}? (s/n): ")
        if confirm.strip().lower() == "s":
            print("📡 Enviando...")
            send_to_robot(best, args.robot_host, args.robot_port)
    else:
        print("\n[--no-robot] Coordenadas listas (no enviadas):")
        print(f"  pick     = {best['xyz_mm']}")
        print(f"  approach = {best['approach_mm']}")

    # Mantener ventana
    if gui:
        print("\nCtrl+C para cerrar...")
        try:
            while p.isConnected():
                p.stepSimulation()
                time.sleep(1/60)
        except KeyboardInterrupt:
            pass

    p.disconnect()
    print("\n✅ Simulación finalizada.")


if __name__ == "__main__":
    main()
