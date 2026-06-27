#!/usr/bin/env python3
"""
ball_sim.py  —  Simulación PyBullet de pick de pelota naranja.

Lee captures/ball_latest.json y simula un gripper paralelo que:
  1. Baja hacia la pelota con los dedos abiertos
  2. Cierra los dedos alrededor de la pelota
  3. La levanta
  4. La suelta y retrocede

Controles PyBullet:
  Rotar  : Click izquierdo + arrastrar
  Zoom   : Rueda del mouse
  Pan    : Click medio + arrastrar  (o Ctrl + izquierdo)

Ejecutar:
  python3 examples/ball_sim.py [--no-gui]
"""
import sys, json, time, argparse
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_data

ROOT      = Path(__file__).resolve().parents[1]
DATA_FILE = ROOT / "captures" / "ball_latest.json"

# ── Conversión coords cámara → mundo PyBullet ────────────────────────────────
#   Cámara: X der+, Y abajo+, Z frente+
#   Mundo:  X der+, Y frente+, Z arriba+
def cam_to_world(xyz_mm):
    return np.array([
         xyz_mm[0] * 0.001,   # cam X → world X
         xyz_mm[2] * 0.001,   # cam Z → world Y  (profundidad = alejarse)
        -xyz_mm[1] * 0.001,   # cam Y → world Z  (abajo → negado)
    ])


# ── Crear el gripper paralelo ────────────────────────────────────────────────

def make_gripper(base_pos, open_width, finger_h=0.08):
    """
    Gripper de dos dedos (cajas grises) + palma (caja pequeña).
    Retorna (palm_id, left_id, right_id).
    """
    hw = open_width / 2      # semi-ancho de apertura

    # Palma
    palm_vis = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[0.03, 0.015, 0.02],
        rgbaColor=[0.6, 0.6, 0.7, 1.0],
    )
    palm_col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.015, 0.02])
    palm = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=palm_col,
                             baseVisualShapeIndex=palm_vis,
                             basePosition=(base_pos + [0, 0, finger_h/2 + 0.02]).tolist())

    # Dedo izquierdo
    lvis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.008, 0.012, finger_h/2],
                               rgbaColor=[0.4, 0.4, 0.5, 1.0])
    lcol = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.008, 0.012, finger_h/2])
    left = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=lcol,
                             baseVisualShapeIndex=lvis,
                             basePosition=(base_pos + [-hw, 0, 0]).tolist())

    # Dedo derecho
    rvis = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.008, 0.012, finger_h/2],
                               rgbaColor=[0.4, 0.4, 0.5, 1.0])
    rcol = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.008, 0.012, finger_h/2])
    right = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=rcol,
                              baseVisualShapeIndex=rvis,
                              basePosition=(base_pos + [+hw, 0, 0]).tolist())
    return palm, left, right


def move_gripper(palm, left, right, target_base, finger_open, finger_h=0.08):
    """Mueve todo el gripper a target_base con apertura finger_open."""
    hw = finger_open / 2
    p.resetBasePositionAndOrientation(
        palm, (target_base + [0, 0, finger_h/2 + 0.02]).tolist(), [0,0,0,1])
    p.resetBasePositionAndOrientation(
        left, (target_base + [-hw, 0, 0]).tolist(), [0,0,0,1])
    p.resetBasePositionAndOrientation(
        right,(target_base + [+hw, 0, 0]).tolist(), [0,0,0,1])


def lerp_gripper(palm, left, right,
                 pos_a, open_a,
                 pos_b, open_b,
                 steps=40, dt=1/60,
                 carry_body=None, carry_offset=None):
    """
    Interpola posición y apertura del gripper de A a B.
    Si carry_body está definido, mueve ese cuerpo junto al gripper
    (sin constraints, solo resetBasePositionAndOrientation).
    """
    for i in range(steps + 1):
        t   = i / steps
        pos = pos_a  * (1 - t) + pos_b  * t
        opn = open_a * (1 - t) + open_b * t
        move_gripper(palm, left, right, pos, opn)
        if carry_body is not None and carry_offset is not None:
            p.resetBasePositionAndOrientation(
                carry_body, (pos + carry_offset).tolist(), [0, 0, 0, 1]
            )
        p.stepSimulation()
        if dt:
            time.sleep(dt)


# ── Etiqueta de fase en pantalla ─────────────────────────────────────────────

def phase_label(text, ball_pos):
    uid = p.addUserDebugText(
        text,
        (ball_pos + [0, 0, 0.25]).tolist(),
        textColorRGB=[1, 1, 0],
        textSize=1.5,
    )
    return uid


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-gui", action="store_true")
    args = ap.parse_args()
    gui  = not args.no_gui

    # Leer datos de la pelota
    if not DATA_FILE.exists():
        print(f"ERROR: no existe {DATA_FILE}")
        print("       Presiona 'Simular Pick' en ball_viewer primero.")
        sys.exit(1)

    data      = json.loads(DATA_FILE.read_text())
    xyz_mm    = np.array(data["xyz_mm"])
    width_mm  = data.get("width_mm")  or 80.0
    height_mm = data.get("height_mm") or 80.0
    radius_m  = max(width_mm, height_mm) / 2 * 0.001

    print("═" * 52)
    print("  BALL SIM — PyBullet")
    print(f"  xyz_mm   = {xyz_mm}")
    print(f"  diámetro = {width_mm:.0f} × {height_mm:.0f} mm")
    print(f"  radio    = {radius_m*1000:.1f} mm")
    print("═" * 52)

    # ── PyBullet ──────────────────────────────────────────────────────────────
    client = p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setRealTimeSimulation(0)

    # Plano (mesa)
    plane = p.loadURDF("plane.urdf")
    p.changeVisualShape(plane, -1, rgbaColor=[0.6, 0.55, 0.5, 1.0])

    # Pelota naranja
    ball_pos = cam_to_world(xyz_mm)
    ball_pos[2] = max(ball_pos[2], radius_m + 0.001)   # sobre el suelo

    ball_vis = p.createVisualShape(p.GEOM_SPHERE, radius=radius_m,
                                   rgbaColor=[1.0, 0.45, 0.0, 1.0])
    ball_col = p.createCollisionShape(p.GEOM_SPHERE, radius=radius_m)
    ball_id  = p.createMultiBody(baseMass=0.08,
                                 baseCollisionShapeIndex=ball_col,
                                 baseVisualShapeIndex=ball_vis,
                                 basePosition=ball_pos.tolist())

    # Etiqueta de la pelota
    p.addUserDebugText(
        f"Pelota  ⌀{width_mm:.0f}mm",
        (ball_pos + [0, 0, radius_m + 0.06]).tolist(),
        textColorRGB=[1, 0.6, 0.1], textSize=1.1,
    )

    # Ejes del mundo (pequeños)
    o = ball_pos.tolist()
    p.addUserDebugLine(o, (ball_pos+[0.15,0,0]).tolist(), [1,0,0], 2)
    p.addUserDebugLine(o, (ball_pos+[0,0.15,0]).tolist(), [0,1,0], 2)
    p.addUserDebugLine(o, (ball_pos+[0,0,0.15]).tolist(), [0,0,1], 2)

    # ── Cámara inicial ────────────────────────────────────────────────────────
    if gui:
        p.resetDebugVisualizerCamera(
            cameraDistance   = max(ball_pos[1] * 0.9, 0.6),
            cameraYaw        = 35,
            cameraPitch      = -25,
            cameraTargetPosition = ball_pos.tolist(),
        )
        p.addUserDebugText(
            "Rotar: arrastrar  |  Zoom: rueda  |  Pan: Ctrl+arrastrar",
            (ball_pos + [-0.25, 0, 0.40]).tolist(),
            textColorRGB=[0.7, 0.7, 0.7], textSize=0.8,
        )

    # ── Gripper ───────────────────────────────────────────────────────────────
    OPEN   = radius_m * 2.6    # apertura inicial
    CLOSED = radius_m * 2.0    # apertura al agarrar (envuelve la pelota)

    # Posiciones clave del gripper (base = centro entre dedos, altura = centro dedo)
    pos_home    = ball_pos + np.array([0, -0.08, 0.28])   # reposo arriba
    pos_above   = ball_pos + np.array([0, -0.02, radius_m + 0.12])  # sobre la pelota
    pos_pick    = ball_pos + np.array([0, -0.02, radius_m * 0.3])   # al nivel de agarre
    pos_lift    = ball_pos + np.array([0, -0.02, 0.22])   # levantado con pelota
    pos_retract = pos_home.copy()

    palm, left, right = make_gripper(pos_home, OPEN)

    # Dejar que la física se asiente
    for _ in range(30):
        p.stepSimulation()

    # ── Secuencia de pick ─────────────────────────────────────────────────────
    phases = [
        ("Bajando al approach...", pos_home,  OPEN,   pos_above, OPEN,   50, 1/60),
        ("Bajando a la pelota...", pos_above, OPEN,   pos_pick,  OPEN,   40, 1/60),
        ("Cerrando dedos...",      pos_pick,  OPEN,   pos_pick,  CLOSED, 35, 1/60),
        ("Levantando...",          pos_pick,  CLOSED, pos_lift,  CLOSED, 60, 1/60),
        ("Abriendo dedos...",      pos_lift,  CLOSED, pos_lift,  OPEN,   25, 1/60),
        ("Retrocediendo...",       pos_lift,  OPEN,   pos_retract,OPEN,  50, 1/60),
    ]

    # Offset fijo pelota → base del gripper (pelota cuelga del centro)
    ball_offset = ball_pos - pos_pick   # vector desde pos_pick al centro de la pelota

    lbl_id   = None
    carrying = False   # si el gripper lleva la pelota

    for i, (name, pa, oa, pb, ob, steps, dt) in enumerate(phases):
        print(f"  [{i+1}/{len(phases)}] {name}")

        if lbl_id is not None:
            p.removeUserDebugItem(lbl_id)
        lbl_id = phase_label(name, ball_pos)

        if "Cerrando" in name:
            carrying = True   # a partir de aquí la pelota viaja con el gripper

        if "Abriendo" in name:
            carrying = False  # soltar: la pelota cae por gravedad

        lerp_gripper(
            palm, left, right, pa, oa, pb, ob,
            steps=steps, dt=dt if gui else 0,
            carry_body   = ball_id     if carrying else None,
            carry_offset = ball_offset if carrying else None,
        )

    if lbl_id is not None:
        p.removeUserDebugItem(lbl_id)
    phase_label("✓ Pick completado", ball_pos)
    print("\n  Pick completado.")

    if gui:
        print("\n  Ventana PyBullet abierta.")
        print("  Rotar: arrastrar con mouse")
        print("  Zoom:  rueda del mouse")
        print("  Ctrl+C para salir\n")
        try:
            while p.isConnected():
                p.stepSimulation()
                time.sleep(1/60)
        except KeyboardInterrupt:
            pass
    else:
        print("  Simulación (sin GUI) completa.")

    p.disconnect()


if __name__ == "__main__":
    main()
