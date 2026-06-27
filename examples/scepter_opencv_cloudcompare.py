#!/usr/bin/env python3
import argparse
import os
import sys
import time
from ctypes import c_uint16
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
PYTHON_SDK = ROOT / "ScepterSDK" / "MultilanguageSDK" / "Python"
RUNTIME_HOME = ROOT / ".scepter_runtime_home"
RUNTIME_HOME.mkdir(parents=True, exist_ok=True)

os.environ["HOME"] = str(RUNTIME_HOME)
os.environ["XDG_CONFIG_HOME"] = str(RUNTIME_HOME / ".config")
os.environ["XDG_CACHE_HOME"] = str(RUNTIME_HOME / ".cache")
sys.path.insert(0, str(PYTHON_SDK))

from API.ScepterDS_api import ScepterTofCam  # noqa: E402
from API.ScepterDS_enums import ScConnectStatus, ScFrameType  # noqa: E402


def check_ok(ret, name):
    if ret != 0:
        raise RuntimeError(f"{name} fallo con ScStatus({ret})")


def depth_frame_to_numpy(frame):
    data = np.ctypeslib.as_array(frame.pFrameData, shape=(frame.dataLen,))
    depth = data.view(np.uint16)
    return depth.reshape((frame.height, frame.width)).copy()


def depth_preview(depth):
    valid = depth[(depth > 0) & (depth < 65535)]
    if valid.size == 0:
        return np.zeros((*depth.shape, 3), dtype=np.uint8)

    near = np.percentile(valid, 2)
    far = np.percentile(valid, 98)
    if far <= near:
        far = near + 1

    normalized = np.clip((depth.astype(np.float32) - near) * 255.0 / (far - near), 0, 255)
    normalized[(depth == 0) | (depth == 65535)] = 0
    return cv2.applyColorMap(normalized.astype(np.uint8), cv2.COLORMAP_TURBO)


def write_ply(path, points):
    valid = []
    for point in points:
        if point.z != 0 and point.z != 65535:
            valid.append((float(point.x), float(point.y), float(point.z)))

    with open(path, "w", encoding="ascii") as file:
        file.write("ply\n")
        file.write("format ascii 1.0\n")
        file.write(f"element vertex {len(valid)}\n")
        file.write("property float x\n")
        file.write("property float y\n")
        file.write("property float z\n")
        file.write("end_header\n")
        for x, y, z in valid:
            file.write(f"{x:.3f} {y:.3f} {z:.3f}\n")

    return len(valid)


def capture_point_cloud(camera, frame, output_dir, stem):
    ret, points = camera.scConvertDepthFrameToPointCloudVector(frame)
    check_ok(ret, "scConvertDepthFrameToPointCloudVector")

    ply_path = output_dir / f"{stem}.ply"
    count = write_ply(ply_path, points)
    return ply_path, count


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


def main():
    parser = argparse.ArgumentParser(description="Visualiza depth con OpenCV y guarda nube .ply para CloudCompare.")
    parser.add_argument("--ip", default="192.168.1.101", help="IP de la camara.")
    parser.add_argument("--output-dir", default=str(ROOT / "captures"), help="Carpeta de salida.")
    parser.add_argument("--save-first", action="store_true", help="Guardar automaticamente el primer frame valido.")
    parser.add_argument("--no-window", action="store_true", help="No abrir ventana OpenCV; captura, guarda y sale.")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    camera = ScepterTofCam()
    opened = False
    streaming = False

    try:
        device = find_device_by_ip(camera, args.ip)
        ret = camera.scOpenDeviceByIP(device.ip)
        check_ok(ret, "scOpenDeviceByIP")
        opened = True
        print(f"Camara abierta en {args.ip}")

        ret = camera.scStartStream()
        check_ok(ret, "scStartStream")
        streaming = True
        time.sleep(1.0)

        saved_once = False
        while True:
            ret, ready = camera.scGetFrameReady(c_uint16(1200))
            if ret != 0 or not ready.depth:
                continue

            ret, frame = camera.scGetFrame(ScFrameType.SC_DEPTH_FRAME)
            if ret != 0:
                print(f"scGetFrame fallo con ScStatus({ret})")
                continue

            depth = depth_frame_to_numpy(frame)
            preview = depth_preview(depth)
            key = -1
            if not args.no_window:
                cv2.imshow("Scepter depth - s guarda PLY, q sale", preview)
                key = cv2.waitKey(1) & 0xFF

            should_save = args.no_window or key == ord("s") or (args.save_first and not saved_once)
            if should_save:
                stem = "capture" if not saved_once else f"capture_{int(time.time())}"
                ply_path, count = capture_point_cloud(camera, frame, output_dir, stem)
                png_path = output_dir / f"{stem}_depth_preview.png"
                cv2.imwrite(str(png_path), preview)
                print(f"Guardado {ply_path} con {count} puntos")
                print(f"Guardado {png_path}")
                saved_once = True
                if args.no_window:
                    break

            if key in (ord("q"), 27):
                break

    finally:
        if streaming:
            camera.scStopStream()
        if opened:
            camera.scCloseDevice()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
