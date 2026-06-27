#!/usr/bin/env python3
"""
banana_capture.py - Captura point cloud de bananas y guarda a disco.
Se ejecuta con PYTHONNOUSERSITE=1 para evitar conflictos con PyBullet.
"""
import os
import site
import sys
import json

if not os.environ.get("SCEPTER_ALLOW_USER_SITE"):
    user_site = site.getusersitepackages()
    sys.path = [p for p in sys.path if p != user_site]

import numpy as np
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).parent))

from picker.camera import ScepterCamera
from picker.detect import detect_pick_targets

OUTPUT = ROOT / "captures" / "banana_scene.npz"
OUTPUT_TARGETS = ROOT / "captures" / "banana_targets.json"

def main():
    ip = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.101"
    num_frames = int(sys.argv[2]) if len(sys.argv) > 2 else 5

    print(f"📷 Capturando escena de bananas ({num_frames} frames)...")

    with ScepterCamera(ip) as cam:
        xyz_accum = None
        bgr_last = None
        count = 0

        while count < num_frames + 5:
            bgr, depth_mm, xyz_mm = cam.get_frames()
            if bgr is None or xyz_mm is None:
                continue

            count += 1
            if count <= 5:
                continue

            if xyz_accum is None:
                xyz_accum = xyz_mm.astype(np.float64)
            else:
                valid = np.isfinite(xyz_mm[:, :, 2])
                xyz_accum[valid] = (xyz_accum[valid] + xyz_mm[valid]) / 2.0

            bgr_last = bgr

        xyz_final = xyz_accum.astype(np.float32)
        invalid = ~np.isfinite(xyz_final[:, :, 2])
        xyz_final[invalid] = np.nan

        targets = detect_pick_targets(xyz_final, cam.intrinsics, max_targets=10)

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        OUTPUT,
        xyz_mm=xyz_final,
        bgr=bgr_last,
    )

    targets_data = [
        {
            "xyz_mm": t.xyz_mm.tolist(),
            "approach_mm": t.approach_mm.tolist(),
            "pixel_uv": list(t.pixel_uv),
            "confidence": float(t.confidence),
        }
        for t in targets
    ]
    OUTPUT_TARGETS.write_text(json.dumps(targets_data, indent=2))

    print(f"✓ Guardado: {OUTPUT}")
    print(f"✓ Guardado: {OUTPUT_TARGETS}")
    print(f"✓ Bananas detectadas: {len(targets)}")
    for i, t in enumerate(targets):
        print(f"  {i+1}. z={t.xyz_mm[2]:.0f}mm conf={t.confidence:.0%} pixel={t.pixel_uv}")


if __name__ == "__main__":
    main()
