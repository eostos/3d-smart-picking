#!/usr/bin/env python3
import sys
import time
from ctypes import c_uint16
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
PYTHON_SDK = ROOT / "ScepterSDK" / "MultilanguageSDK" / "Python"
sys.path.insert(0, str(PYTHON_SDK))

from API.ScepterDS_api import ScepterTofCam  # noqa: E402
from API.ScepterDS_enums import ScConnectStatus, ScFrameType, ScSensorType  # noqa: E402


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
            if device.status not in (ScConnectStatus.SC_CONNECTABLE.value, ScConnectStatus.SC_OPENED.value):
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


class ScepterCamera:
    def __init__(self, ip: str, color_width: int = 800, color_height: int = 600):
        self.ip = ip
        self.color_width = color_width
        self.color_height = color_height
        self.camera = ScepterTofCam()
        self.opened = False
        self.streaming = False
        self.intrinsics = None

    def open(self):
        if self.opened:
            return

        device = find_device_by_ip(self.camera, self.ip)
        ret = self.camera.scOpenDeviceByIP(device.ip)
        check_ok(ret, "scOpenDeviceByIP")
        self.opened = True

        ret = self.camera.scSetColorResolution(self.color_width, self.color_height)
        check_ok(ret, "scSetColorResolution")

        ret = self.camera.scStartStream()
        check_ok(ret, "scStartStream")
        self.streaming = True
        time.sleep(1.0)

        ret = self.camera.scSetTransformDepthImgToColorSensorEnabled(True)
        check_ok(ret, "scSetTransformDepthImgToColorSensorEnabled")

        ret, self.intrinsics = self.camera.scGetSensorIntrinsicParameters(ScSensorType.SC_TOF_SENSOR)
        check_ok(ret, "scGetSensorIntrinsicParameters")

        print(f"Camara abierta en {self.ip}")

    def close(self):
        if self.streaming:
            self.camera.scStopStream()
            self.streaming = False
        if self.opened:
            self.camera.scCloseDevice()
            self.opened = False

    def get_frames(self, wait_ms: int = 1200):
        if not self.opened:
            raise RuntimeError("Camera not opened")

        ret, ready = self.camera.scGetFrameReady(c_uint16(wait_ms))
        if ret != 0:
            return None, None, None

        bgr = None
        depth_mm = None
        xyz_mm = None

        if ready.color:
            ret, color_frame = self.camera.scGetFrame(ScFrameType.SC_COLOR_FRAME)
            if ret == 0:
                bgr = color_frame_to_bgr(color_frame)

        if ready.transformedDepth:
            ret, depth_frame = self.camera.scGetFrame(ScFrameType.SC_TRANSFORM_DEPTH_IMG_TO_COLOR_SENSOR_FRAME)
            if ret == 0:
                depth_mm = depth_frame_to_mm(depth_frame)
                ret, points = self.camera.scConvertDepthFrameToPointCloudVector(depth_frame)
                if ret == 0:
                    xyz_mm = np.frombuffer(points, dtype=np.float32).reshape(
                        (depth_frame.height, depth_frame.width, 3)
                    ).copy()
                    invalid = (xyz_mm[:, :, 2] <= 0) | (xyz_mm[:, :, 2] >= 65535)
                    xyz_mm[invalid] = np.nan

        if bgr is not None and depth_mm is not None and bgr.shape[:2] != depth_mm.shape:
            bgr = cv2.resize(bgr, (depth_mm.shape[1], depth_mm.shape[0]))

        return bgr, depth_mm, xyz_mm

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
