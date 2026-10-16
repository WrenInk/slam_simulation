"""Exact visible-surface lidar rays for the competition scene.

MuJoCo 3.3.7 mj_multiRay's angular broad phase can miss a long ramp from
some side views, even when mj_ray correctly hits it. Use the single-ray
entry point so scans cannot see the floor through the solid ramp. This
changes sensor sampling only; no simulator geometry enters SLAM/Nav2.
"""
import mujoco
import numpy as np


def exact_rays(model, data, origin, directions, geom_group, body_exclude):
    directions = np.ascontiguousarray(directions, dtype=np.float64).reshape(-1, 3)
    ids = np.full(len(directions), -1, dtype=np.int32)
    distances = np.empty(len(directions), dtype=np.float64)
    for i, direction in enumerate(directions):
        distances[i] = mujoco.mj_ray(model, data, origin, direction, geom_group,
                                     1, body_exclude, ids[i:i+1])
    return distances, ids
