#!/usr/bin/env python3
from dataclasses import dataclass

import numpy as np

try:
    import open3d as o3d
except ImportError as exc:
    raise SystemExit(
        "Falta Open3D. En esta maquina ARM instalalo con:\n"
        "  sudo apt update\n"
        "  sudo apt install python3-open3d"
    ) from exc


@dataclass
class PickTarget:
    xyz_mm: np.ndarray
    approach_mm: np.ndarray
    pixel_uv: tuple
    confidence: float


def detect_pick_targets(xyz_mm: np.ndarray, intrinsics, max_targets: int = 5) -> list[PickTarget]:
    valid_mask = np.isfinite(xyz_mm[:, :, 2])
    if not valid_mask.any():
        return []

    valid_points = xyz_mm[valid_mask]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(valid_points)

    plane_model, inliers = pcd.segment_plane(distance_threshold=10, ransac_n=3, num_iterations=1000)
    plane_points = pcd.select_by_index(inliers)

    plane_center = np.asarray(plane_points.points).mean(axis=0)

    object_inliers = np.setdiff1d(np.arange(len(pcd.points)), inliers)
    if len(object_inliers) == 0:
        return []

    object_pcd = pcd.select_by_index(object_inliers.tolist())
    labels = np.array(object_pcd.cluster_dbscan(eps=15, min_points=50))

    unique_labels = set(labels)
    if -1 in unique_labels:
        unique_labels.remove(-1)

    if not unique_labels:
        return []

    targets = []
    for label in sorted(unique_labels):
        cluster_mask = labels == label
        cluster_points = np.asarray(object_pcd.points)[cluster_mask]

        centroid = cluster_points.mean(axis=0)
        approach = centroid.copy()
        approach[2] -= 50

        confidence = len(cluster_points) / len(valid_points)

        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.cx
        cy = intrinsics.cy

        if centroid[2] > 0:
            u = int(fx * centroid[0] / centroid[2] + cx)
            v = int(fy * centroid[1] / centroid[2] + cy)
            u = max(0, min(u, xyz_mm.shape[1] - 1))
            v = max(0, min(v, xyz_mm.shape[0] - 1))

            targets.append(
                PickTarget(
                    xyz_mm=centroid,
                    approach_mm=approach,
                    pixel_uv=(u, v),
                    confidence=confidence,
                )
            )

    targets.sort(key=lambda t: t.xyz_mm[2])
    return targets[:max_targets]
