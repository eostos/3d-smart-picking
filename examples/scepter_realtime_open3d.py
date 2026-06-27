#!/usr/bin/env python3
import argparse
import os
import site
import sys
import time
from ctypes import c_uint16
from pathlib import Path

if not os.environ.get("SCEPTER_ALLOW_USER_SITE"):
    user_site = site.getusersitepackages()
    sys.path = [path for path in sys.path if path != user_site]

import numpy as np

try:
    import open3d as o3d
except ImportError as exc:
    raise SystemExit(
        "Falta Open3D. En esta maquina ARM instalalo con:\n"
        "  sudo apt update\n"
        "  sudo apt install python3-open3d\n"
        "Luego ejecuta este script con /usr/bin/python3."
    ) from exc


ROOT = Path(__file__).resolve().parents[1]
PYTHON_SDK = ROOT / "ScepterSDK" / "MultilanguageSDK" / "Python"
sys.path.insert(0, str(PYTHON_SDK))

from API.ScepterDS_api import ScepterTofCam  # noqa: E402
from API.ScepterDS_enums import ScConnectStatus, ScFrameType  # noqa: E402


def check_ok(ret, name):
    if ret != 0:
        raise RuntimeError(f"{name} fallo con ScStatus({ret})")


def find_device_by_ip(camera, ip, timeout_ms=3000):
    count = camera.scGetDeviceCount(timeout_ms)
    if count <= 0:
        raise RuntimeError("No se encontraron camaras en la red.")

    ret, devices = camera.scGetDeviceInfoList(count)
    check_ok(ret, "scGetDeviceInfoList")

    for device in devices:
        device_ip = device.ip.decode(errors="ignore").rstrip("\x00")
        serial = device.serialNumber.decode(errors="ignore").rstrip("\x00")
        print(f"Encontrada camara SN={serial} IP={device_ip} status={device.status}")
        if device_ip == ip:
            if device.status != ScConnectStatus.SC_CONNECTABLE.value:
                raise RuntimeError(f"La camara {ip} existe pero no esta conectable. Status={device.status}")
            return device

    raise RuntimeError(f"No se encontro una camara con IP {ip}.")


def point_cloud_to_numpy(points, scale, max_points):
    xyz = np.frombuffer(points, dtype=np.float32).reshape(-1, 3)
    valid = (xyz[:, 2] > 0) & (xyz[:, 2] < 65535)
    xyz = xyz[valid]

    if max_points > 0 and xyz.shape[0] > max_points:
        step = max(1, xyz.shape[0] // max_points)
        xyz = xyz[::step][:max_points]

    return xyz.astype(np.float64) * scale


def colors_by_depth(xyz):
    if xyz.size == 0:
        return xyz

    z = xyz[:, 2]
    near = np.percentile(z, 2)
    far = np.percentile(z, 98)
    if far <= near:
        far = near + 1.0

    t = np.clip((z - near) / (far - near), 0.0, 1.0)
    colors = np.empty_like(xyz)
    colors[:, 0] = t
    colors[:, 1] = 1.0 - np.abs(t - 0.5) * 2.0
    colors[:, 2] = 1.0 - t
    return colors


def main():
    parser = argparse.ArgumentParser(description="Point cloud en tiempo real con ScepterSDK + Open3D.")
    parser.add_argument("--ip", default="192.168.1.101", help="IP de la camara.")
    parser.add_argument("--max-points", type=int, default=80000, help="Maximo de puntos a dibujar por frame.")
    parser.add_argument("--scale", type=float, default=0.001, help="Escala de unidades. 0.001 convierte mm a metros.")
    parser.add_argument("--wait-ms", type=int, default=1200, help="Timeout para esperar cada frame.")
    args = parser.parse_args()

    vis = o3d.visualization.Visualizer()
    window_created = vis.create_window("Scepter point cloud realtime", width=1280, height=720)
    if not window_created:
        raise SystemExit(
            "Open3D no pudo abrir una ventana grafica. Ejecuta el comando desde una sesion de escritorio "
            "con DISPLAY activo, no desde SSH/headless."
        )

    camera = ScepterTofCam()
    opened = False
    streaming = False
    pcd = o3d.geometry.PointCloud()
    added = False

    try:
        device = find_device_by_ip(camera, args.ip)
        ret = camera.scOpenDeviceByIP(device.ip)
        check_ok(ret, "scOpenDeviceByIP")
        opened = True
        print(f"Camara abierta en {args.ip}", flush=True)

        ret = camera.scStartStream()
        check_ok(ret, "scStartStream")
        streaming = True
        time.sleep(1.0)

        while True:
            ret, ready = camera.scGetFrameReady(c_uint16(args.wait_ms))
            if ret != 0 or not ready.depth:
                if not vis.poll_events():
                    break
                continue

            ret, frame = camera.scGetFrame(ScFrameType.SC_DEPTH_FRAME)
            if ret != 0:
                print(f"scGetFrame fallo con ScStatus({ret})")
                continue

            ret, points = camera.scConvertDepthFrameToPointCloudVector(frame)
            if ret != 0:
                print(f"scConvertDepthFrameToPointCloudVector fallo con ScStatus({ret})")
                continue

            xyz = point_cloud_to_numpy(points, args.scale, args.max_points)
            pcd.points = o3d.utility.Vector3dVector(xyz)
            pcd.colors = o3d.utility.Vector3dVector(colors_by_depth(xyz))

            if not added:
                vis.add_geometry(pcd)
                added = True
                view = vis.get_view_control()
                view.set_front([0.0, 0.0, -1.0])
                view.set_up([0.0, -1.0, 0.0])
                view.set_zoom(0.6)
            else:
                vis.update_geometry(pcd)

            if not vis.poll_events():
                break
            vis.update_renderer()

    finally:
        if streaming:
            camera.scStopStream()
        if opened:
            camera.scCloseDevice()
        if window_created:
            vis.destroy_window()


if __name__ == "__main__":
    main()
