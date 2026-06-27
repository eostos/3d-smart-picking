#!/usr/bin/env python3
"""
Simulador de robot que escucha picks en TCP y visualiza la trayectoria.
"""
import argparse
import json
import socket
import time
from dataclasses import dataclass


@dataclass
class RobotState:
    x: float = 0
    y: float = 0
    z: float = 0
    gripper_open: bool = True


def interpolate(start, end, steps=5):
    """Interpolar posiciones linealmente"""
    for i in range(steps + 1):
        t = i / steps if steps > 0 else 1
        yield (
            start[0] + (end[0] - start[0]) * t,
            start[1] + (end[1] - start[1]) * t,
            start[2] + (end[2] - start[2]) * t,
        )


def run_robot_server(host="0.0.0.0", port=5005, verbose=True):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((host, port))
    sock.listen(1)

    robot = RobotState()
    pick_count = 0

    print(f"🤖 Simulador de robot escuchando en {host}:{port}")
    print(f"   Estado inicial: x={robot.x}, y={robot.y}, z={robot.z}")
    print()

    try:
        while True:
            try:
                client, addr = sock.accept()
                if verbose:
                    print(f"📨 Conexión de {addr[0]}:{addr[1]}")

                try:
                    data = client.recv(4096).decode().strip()
                    if not data:
                        continue

                    pick = json.loads(data)
                    approach = pick["approach"]
                    target = pick["pick"]
                    confidence = pick["confidence"]
                    pixel = pick.get("pixel", [0, 0])
                    pick_count += 1

                    print(f"\n[Pick #{pick_count}] Detectado en pixel {pixel} (confianza: {confidence:.0%})")
                    print(f"  📍 Target XYZ: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}) mm")

                    # Fase 1: mover a posición de aproximación
                    print(f"  🤖 Moviendo a aproximación...", end="", flush=True)
                    start_pos = (robot.x, robot.y, robot.z)
                    for pos in interpolate(start_pos, tuple(approach), steps=4):
                        robot.x, robot.y, robot.z = pos
                        if verbose:
                            print(f"\r  🤖 Moviendo a aproximación... xyz=({pos[0]:.0f}, {pos[1]:.0f}, {pos[2]:.0f})", end="", flush=True)
                        time.sleep(0.15)
                    print()

                    # Fase 2: abrir gripper
                    robot.gripper_open = True
                    print(f"  ✋ Gripper abierto")
                    time.sleep(0.2)

                    # Fase 3: descender al pick
                    print(f"  🤖 Descendiendo al objeto...", end="", flush=True)
                    z_approach = approach[2]
                    z_pick = target[2]
                    approach_pos = (robot.x, robot.y, robot.z)
                    target_pos = (target[0], target[1], target[2])

                    for pos in interpolate(approach_pos, target_pos, steps=3):
                        robot.x, robot.y, robot.z = pos
                        if verbose:
                            print(f"\r  🤖 Descendiendo... z={pos[2]:.0f}", end="", flush=True)
                        time.sleep(0.15)
                    print()

                    # Fase 4: cerrar gripper
                    robot.gripper_open = False
                    print(f"  ✊ Gripper cerrado")
                    time.sleep(0.3)

                    # Fase 5: retract a aproximación
                    print(f"  🤖 Retrayendo...", end="", flush=True)
                    current_pos = (robot.x, robot.y, robot.z)
                    for pos in interpolate(current_pos, tuple(approach), steps=3):
                        robot.x, robot.y, robot.z = pos
                        if verbose:
                            print(f"\r  🤖 Retrayendo... z={pos[2]:.0f}", end="", flush=True)
                        time.sleep(0.15)
                    print()

                    print(f"  ✅ Pick completado!")
                    print(f"     Estado final: x={robot.x:.1f}, y={robot.y:.1f}, z={robot.z:.1f}\n")

                    client.send(b"OK\n")

                except json.JSONDecodeError:
                    print(f"  ❌ JSON inválido recibido: {data[:50]}")
                    client.send(b"ERROR\n")
                except KeyError as e:
                    print(f"  ❌ Campo faltante en JSON: {e}")
                    client.send(b"ERROR\n")
                except Exception as e:
                    print(f"  ❌ Error procesando pick: {e}")
                    client.send(b"ERROR\n")
                finally:
                    client.close()

            except socket.timeout:
                continue
            except Exception as e:
                print(f"❌ Error en servidor: {e}")
                break

    except KeyboardInterrupt:
        print("\n\n👋 Servidor detenido")
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(description="Simulador de robot para smart picking")
    parser.add_argument("--host", default="0.0.0.0", help="Host de escucha (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=5005, help="Puerto de escucha (default: 5005)")
    parser.add_argument("--quiet", action="store_true", help="Modo silencioso")

    args = parser.parse_args()
    run_robot_server(args.host, args.port, verbose=not args.quiet)


if __name__ == "__main__":
    main()
