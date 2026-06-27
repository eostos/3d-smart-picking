#!/usr/bin/env python3
import json
import socket
from abc import ABC, abstractmethod

from .detect import PickTarget


class RobotInterface(ABC):
    @abstractmethod
    def send_pick(self, target: PickTarget) -> bool:
        pass

    @abstractmethod
    def close(self):
        pass


class MockRobot(RobotInterface):
    def send_pick(self, target: PickTarget) -> bool:
        print(
            f"[MockRobot] PICK → "
            f"xyz=[{target.xyz_mm[0]:.1f} {target.xyz_mm[1]:.1f} {target.xyz_mm[2]:.1f}] mm "
            f"approach=[{target.approach_mm[0]:.1f} {target.approach_mm[1]:.1f} {target.approach_mm[2]:.1f}] mm "
            f"confidence={target.confidence:.2f} "
            f"pixel=({target.pixel_uv[0]}, {target.pixel_uv[1]})"
        )
        return True

    def close(self):
        pass


class TcpRobot(RobotInterface):
    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout

    def send_pick(self, target: PickTarget) -> bool:
        payload = {
            "pick": target.xyz_mm.tolist(),
            "approach": target.approach_mm.tolist(),
            "confidence": float(target.confidence),
            "pixel": list(target.pixel_uv),
        }
        message = json.dumps(payload) + "\n"

        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self.timeout)
            sock.connect((self.host, self.port))
            sock.sendall(message.encode())

            response = sock.recv(1024).decode().strip()
            sock.close()

            success = response.lower() == "ok"
            status = "OK" if success else f"NACK: {response}"
            print(f"[TcpRobot {self.host}:{self.port}] sent pick → {status}")
            return success

        except socket.timeout:
            print(f"[TcpRobot] Connection timeout to {self.host}:{self.port}")
            return False
        except ConnectionRefusedError:
            print(f"[TcpRobot] Connection refused by {self.host}:{self.port}")
            return False
        except Exception as e:
            print(f"[TcpRobot] Error: {e}")
            return False

    def close(self):
        pass
