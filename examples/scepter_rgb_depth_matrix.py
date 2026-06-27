#!/usr/bin/env python3
import argparse
import sys
import time
from ctypes import c_uint16
from pathlib import Path

import cv2
import numpy as np


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


def frame_bytes(frame):
    return np.ctypeslib.as_array(frame.pFrameData, shape=(frame.dataLen,))


def color_frame_to_bgr(frame):
    raw = frame_bytes(frame)
    expected_bgr = frame.width * frame.height * 3

    if frame.dataLen == expected_bgr:
        return raw.reshape((frame.height, frame.width, 3)).copy()

    decoded = cv2.imdecode(raw.copy(), cv2.IMREAD_COLOR)
    if decoded is None:
        raise RuntimeError(
            f"No pude decodificar color frame: {frame.width}x{frame.height}, dataLen={frame.dataLen}"
        )
    return decoded


def depth_frame_to_mm(frame):
    raw = frame_bytes(frame)
    depth = raw.view(np.uint16)
    return depth.reshape((frame.height, frame.width)).copy()


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


def point_cloud_to_xyz_matrix(camera, depth_frame):
    ret, points = camera.scConvertDepthFrameToPointCloudVector(depth_frame)
    check_ok(ret, "scConvertDepthFrameToPointCloudVector")

    xyz = np.frombuffer(points, dtype=np.float32).reshape((depth_frame.height, depth_frame.width, 3)).copy()
    invalid = (xyz[:, :, 2] <= 0) | (xyz[:, :, 2] >= 65535)
    xyz[invalid] = np.nan
    return xyz


def write_colored_ply(path, xyz, bgr):
    valid = np.isfinite(xyz[:, :, 2])
    points = xyz[valid]
    colors_bgr = bgr[valid]
    colors_rgb = colors_bgr[:, ::-1]

    with open(path, "w", encoding="ascii") as file:
        file.write("ply\n")
        file.write("format ascii 1.0\n")
        file.write(f"element vertex {points.shape[0]}\n")
        file.write("property float x\n")
        file.write("property float y\n")
        file.write("property float z\n")
        file.write("property uchar red\n")
        file.write("property uchar green\n")
        file.write("property uchar blue\n")
        file.write("end_header\n")
        for point, color in zip(points, colors_rgb):
            file.write(
                f"{point[0]:.3f} {point[1]:.3f} {point[2]:.3f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )

    return points.shape[0]


def save_capture(output_dir, stem, bgr, depth_mm, xyz):
    output_dir.mkdir(parents=True, exist_ok=True)

    color_path = output_dir / f"{stem}_rgb.png"
    depth_png_path = output_dir / f"{stem}_depth_mm.png"
    preview_path = output_dir / f"{stem}_depth_preview.png"
    overlay_path = output_dir / f"{stem}_rgb_depth_overlay.png"
    npz_path = output_dir / f"{stem}_rgb_depth_xyz.npz"
    ply_path = output_dir / f"{stem}_colored_cloud.ply"

    preview = depth_preview(depth_mm)
    overlay = cv2.addWeighted(bgr, 0.55, preview, 0.45, 0.0)

    cv2.imwrite(str(color_path), bgr)
    cv2.imwrite(str(depth_png_path), depth_mm)
    cv2.imwrite(str(preview_path), preview)
    cv2.imwrite(str(overlay_path), overlay)
    np.savez_compressed(
        npz_path,
        rgb=bgr[:, :, ::-1],
        bgr=bgr,
        depth_mm=depth_mm,
        xyz_mm=xyz,
        valid=np.isfinite(xyz[:, :, 2]),
    )
    point_count = write_colored_ply(ply_path, xyz, bgr)

    print(f"Guardado {npz_path}")
    print(f"Guardado {ply_path} con {point_count} puntos coloreados")
    print(f"Guardado {color_path}")
    print(f"Guardado {depth_png_path}")
    print(f"Guardado {overlay_path}")


def main():
    parser = argparse.ArgumentParser(
        description="RGB + profundidad alineada: genera matriz depth_mm, matriz xyz y nube coloreada."
    )
    parser.add_argument("--ip", default="192.168.1.101", help="IP de la camara.")
    parser.add_argument("--color-width", type=int, default=800, help="Ancho RGB/alineado.")
    parser.add_argument("--color-height", type=int, default=600, help="Alto RGB/alineado.")
    parser.add_argument("--output-dir", default=str(ROOT / "captures"), help="Carpeta de salida.")
    parser.add_argument("--stem", default="rgb_depth", help="Prefijo de archivos.")
    parser.add_argument("--once", action="store_true", help="Guardar un frame y salir.")
    parser.add_argument("--no-window", action="store_true", help="No mostrar ventana OpenCV.")
    args = parser.parse_args()

    camera = ScepterTofCam()
    opened = False
    streaming = False
    latest_bgr = None
    latest_depth = None
    latest_depth_frame = None
    saved_once = False

    try:
        device = find_device_by_ip(camera, args.ip)
        ret = camera.scOpenDeviceByIP(device.ip)
        check_ok(ret, "scOpenDeviceByIP")
        opened = True
        print(f"Camara abierta en {args.ip}")

        ret = camera.scSetColorResolution(args.color_width, args.color_height)
        check_ok(ret, "scSetColorResolution")

        ret = camera.scStartStream()
        check_ok(ret, "scStartStream")
        streaming = True

        ret = camera.scSetTransformDepthImgToColorSensorEnabled(True)
        check_ok(ret, "scSetTransformDepthImgToColorSensorEnabled")
        time.sleep(1.0)

        while True:
            ret, ready = camera.scGetFrameReady(c_uint16(1200))
            if ret != 0:
                continue

            if ready.color:
                ret, color_frame = camera.scGetFrame(ScFrameType.SC_COLOR_FRAME)
                if ret == 0:
                    latest_bgr = color_frame_to_bgr(color_frame)

            if ready.transformedDepth:
                ret, depth_frame = camera.scGetFrame(ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
                if ret == 0:
                    latest_depth = depth_frame_to_mm(depth_frame)
                    latest_depth_frame = depth_frame

            if latest_bgr is None or latest_depth is None or latest_depth_frame is None:
                continue

            if latest_bgr.shape[:2] != latest_depth.shape:
                latest_bgr = cv2.resize(latest_bgr, (latest_depth.shape[1], latest_depth.shape[0]))

            preview = depth_preview(latest_depth)
            overlay = cv2.addWeighted(latest_bgr, 0.55, preview, 0.45, 0.0)

            key = -1
            if not args.no_window:
                side_by_side = np.hstack((latest_bgr, preview, overlay))
                cv2.imshow("RGB | Depth matrix | Overlay - s guarda, q sale", side_by_side)
                key = cv2.waitKey(1) & 0xFF

            should_save = args.once or args.no_window or key == ord("s")
            if should_save:
                xyz = point_cloud_to_xyz_matrix(camera, latest_depth_frame)
                stem = args.stem if not saved_once else f"{args.stem}_{int(time.time())}"
                save_capture(Path(args.output_dir), stem, latest_bgr, latest_depth, xyz)
                saved_once = True
                if args.once or args.no_window:
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
