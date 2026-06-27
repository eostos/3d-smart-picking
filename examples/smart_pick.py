#!/usr/bin/env python3
import argparse
import os
import site
import subprocess
import sys
import time
from pathlib import Path

if not os.environ.get("SCEPTER_ALLOW_USER_SITE"):
    user_site = site.getusersitepackages()
    sys.path = [path for path in sys.path if path != user_site]

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))

from picker.camera import ScepterCamera
from picker.detect import detect_pick_targets
from picker.robot import MockRobot, TcpRobot


def depth_preview(depth_mm):
    valid = depth_mm[(depth_mm > 0) & (depth_mm < 65535)]
    if valid.size == 0:
        return np.zeros((*depth_mm.shape, 3), dtype=np.uint8)

    near = np.percentile(valid, 2)
    far = np.percentile(valid, 98)
    if far <= near:
        far = near + 1

    scaled = np.clip((depth_mm.astype(np.float32) - near) * 255.0 / (far - near), 0, 255)
    scaled[(depth_mm == 0) | (depth_mm == 65535)] = 0
    return cv2.applyColorMap(scaled.astype(np.uint8), cv2.COLORMAP_TURBO)


def save_capture(output_dir, stem, bgr, depth_mm, xyz_mm):
    output_dir.mkdir(parents=True, exist_ok=True)

    color_path = output_dir / f"{stem}_rgb.png"
    depth_path = output_dir / f"{stem}_depth_mm.png"
    preview_path = output_dir / f"{stem}_depth_preview.png"
    npz_path = output_dir / f"{stem}_rgb_depth_xyz.npz"

    preview = depth_preview(depth_mm)
    cv2.imwrite(str(color_path), bgr)
    cv2.imwrite(str(depth_path), depth_mm)
    cv2.imwrite(str(preview_path), preview)
    np.savez_compressed(
        npz_path,
        rgb=bgr[:, :, ::-1],
        bgr=bgr,
        depth_mm=depth_mm,
        xyz_mm=xyz_mm,
        valid=np.isfinite(xyz_mm[:, :, 2]),
    )

    print(f"Guardado {color_path}")
    print(f"Guardado {depth_path}")
    print(f"Guardado {preview_path}")
    print(f"Guardado {npz_path}")


def main():
    parser = argparse.ArgumentParser(description="Smart picking con modos RGB/Depth/Cloud/Pick")
    parser.add_argument("--ip", default="192.168.1.101", help="IP de la camara")
    parser.add_argument("--mode", choices=["rgb", "depth", "cloud", "pick"], default="rgb", help="Modo inicial")
    parser.add_argument("--robot", choices=["mock", "tcp"], default="mock", help="Tipo de robot")
    parser.add_argument("--robot-host", default="192.168.1.200", help="Host del robot")
    parser.add_argument("--robot-port", type=int, default=5005, help="Puerto del robot")
    parser.add_argument("--max-points", type=int, default=80000, help="Max puntos en cloud mode")
    parser.add_argument(
        "--output-dir", default=str(Path(__file__).parent.parent / "captures"), help="Carpeta de salida"
    )
    parser.add_argument("--headless", action="store_true", help="Modo sin ventana gráfica (debug en consola)")
    parser.add_argument("--timeout", type=int, default=30, help="Timeout en segundos (0=infinito)")

    args = parser.parse_args()

    robot = MockRobot() if args.robot == "mock" else TcpRobot(args.robot_host, args.robot_port)

    current_mode = args.mode
    cloud_process = None
    frame_count = 0
    targets = []
    selected_target_idx = 0
    start_time = time.time()

    try:
        with ScepterCamera(args.ip) as camera:
            print(f"Modos disponibles: 1=RGB, 2=Depth, 3=Cloud, 4=Pick")
            print(f"Controles: SPACE=enviar pick, s=guardar, q/Esc=salir")
            if args.headless:
                print(f"[Modo HEADLESS] Sin ventana gráfica, output en consola")
            print()

            while True:
                # Verificar timeout
                if args.timeout > 0 and (time.time() - start_time) > args.timeout:
                    print(f"\n⏱️  Timeout después de {args.timeout}s")
                    break
                bgr, depth_mm, xyz_mm = camera.get_frames()
                if bgr is None or depth_mm is None or xyz_mm is None:
                    continue

                frame_count += 1

                if frame_count == 1 and args.headless:
                    print(f"[DEBUG] Modo: {current_mode}")
                    print(f"[DEBUG] Primer frame OK: bgr{bgr.shape} depth{depth_mm.shape} xyz{xyz_mm.shape}")
                if frame_count % 5 == 0 and args.headless:
                    print(f"[DEBUG] Frame {frame_count} (modo={current_mode})")

                # Ejecutar detección en modo pick (tanto GUI como headless)
                if current_mode == "pick":
                    if frame_count % 10 == 0:
                        try:
                            targets = detect_pick_targets(xyz_mm, camera.intrinsics, max_targets=5)
                            if args.headless:
                                if targets:
                                    print(f"\n[Frame {frame_count}] ✅ Detectados {len(targets)} objetos:")
                                    for idx, t in enumerate(targets):
                                        print(f"    {idx+1}. xyz={t.xyz_mm[:2].astype(int)} z={int(t.xyz_mm[2])}mm conf={t.confidence:.0%} pixel={t.pixel_uv}")
                                else:
                                    print(f"[Frame {frame_count}] ❌ Sin objetos detectados")
                            if targets:
                                selected_target_idx = min(selected_target_idx, len(targets) - 1)
                        except Exception as e:
                            print(f"\n[Frame {frame_count}] ⚠️  Error en detección: {e}")
                            targets = []

                if not args.headless:
                    if current_mode == "rgb":
                        display = bgr.copy()
                        cv2.putText(
                            display,
                            "RGB | 1=RGB 2=Depth 3=Cloud 4=Pick",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            1,
                        )
                        cv2.imshow("Smart Pick", display)

                    elif current_mode == "depth":
                        display = depth_preview(depth_mm)
                        cv2.putText(
                            display,
                            "Depth | 1=RGB 2=Depth 3=Cloud 4=Pick",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 255),
                            1,
                        )
                        cv2.imshow("Smart Pick", display)

                    elif current_mode == "cloud":
                        if cloud_process is None:
                            print("Abriendo viewer de nube...")
                            cloud_process = subprocess.Popen(
                                [
                                    sys.executable,
                                    str(Path(__file__).parent / "scepter_realtime_open3d.py"),
                                    "--ip",
                                    args.ip,
                                    "--max-points",
                                    str(args.max_points),
                                ],
                                stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL,
                            )
                        display = bgr.copy()
                        cv2.putText(
                            display,
                            "Cloud mode (Open3D en otra ventana) | 1=RGB 2=Depth 3=Cloud 4=Pick",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 255),
                            1,
                        )
                        cv2.imshow("Smart Pick", display)

                    elif current_mode == "pick":
                        display = bgr.copy()

                        for idx, target in enumerate(targets):
                            u, v = target.pixel_uv
                            if 0 <= u < display.shape[1] and 0 <= v < display.shape[0]:
                                color = (0, 255, 255) if idx == selected_target_idx else (0, 255, 0)
                                thickness = 3 if idx == selected_target_idx else 2
                                cv2.circle(display, (u, v), 15, color, thickness)
                                cv2.putText(
                                    display,
                                    f"{idx}: {target.confidence:.2f}",
                                    (u + 20, v),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    0.4,
                                    color,
                                    1,
                                )

                        cv2.putText(
                            display,
                            f"Pick mode ({len(targets)} objects) | 1=RGB 2=Depth 3=Cloud 4=Pick | "
                            f"SPACE=send UP/DOWN=select",
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            1,
                        )
                        cv2.imshow("Smart Pick", display)

                # En modo headless, no esperar por tecla de ventana
                if args.headless:
                    key = -1
                    time.sleep(0.1)  # Pequeña pausa para no saturar CPU
                else:
                    key = cv2.waitKey(1) & 0xFF

                if key == ord("1"):
                    if cloud_process:
                        cloud_process.terminate()
                        cloud_process = None
                    current_mode = "rgb"
                    selected_target_idx = 0
                    targets = []

                elif key == ord("2"):
                    if cloud_process:
                        cloud_process.terminate()
                        cloud_process = None
                    current_mode = "depth"
                    selected_target_idx = 0
                    targets = []

                elif key == ord("3"):
                    current_mode = "cloud"
                    selected_target_idx = 0
                    targets = []

                elif key == ord("4"):
                    if cloud_process:
                        cloud_process.terminate()
                        cloud_process = None
                    current_mode = "pick"

                elif key == ord(" "):
                    if targets and 0 <= selected_target_idx < len(targets):
                        robot.send_pick(targets[selected_target_idx])

                elif key == ord("s"):
                    stem = f"capture_{int(time.time())}"
                    save_capture(Path(args.output_dir), stem, bgr, depth_mm, xyz_mm)

                elif key in (ord("q"), 27):
                    break

    finally:
        cv2.destroyAllWindows()
        if cloud_process:
            cloud_process.terminate()
        robot.close()


if __name__ == "__main__":
    main()
