"""LIO-SAM mapOptmization.cpp backend algorithms adapted to FAST-LIO2.

Original: Tixiao Shan et al., LIO-SAM, BSD-3-Clause. See THIRD_PARTY.md.
Ports saveFrame, addOdomFactor, detectLoopClosureDistance, loopFindNearKeyframes,
performLoopClosure, addLoopFactor, correctPoses. Uses full deskewed clouds instead
of LOAM corner/surface features; SciPy ICP replaces PCL ICP; GTSAM is unchanged.
"""
from dataclasses import dataclass
import gtsam
import numpy as np
from scipy.spatial.transform import Rotation
from .geometry import inverse, transform, voxelize, icp, supported_points


@dataclass
class GraphConfig:
    key_distance: float = 0.4
    key_angle: float = 0.2
    key_period: float = 1.0
    voxel_size: float = 0.15
    map_outlier_radius: float = 0.05
    map_min_neighbors: int = 1
    loop_radius: float = 1.5
    loop_min_time: float = 15.0
    loop_min_travel: float = 1.0
    loop_neighbors: int = 6
    loop_fitness: float = 0.06
    loop_overlap: float = 0.6
    loop_max_correction: float = 1.0
    loop_max_rotation: float = 0.5
    loop_interval: float = 2.0


class PoseGraph:
    def __init__(self, config=None):
        self.config = config or GraphConfig()
        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(0.1)
        params.relinearizeSkip = 1
        self.isam = gtsam.ISAM2(params)
        self.times, self.raw, self.poses, self.clouds = [], [], [], []
        self.loops, self.attempts = [], []
        self.map_to_odom = np.eye(4)
        self.last_loop_time = -np.inf

    def add(self, stamp, odom_pose, body_cloud):
        cfg = self.config
        if self.times:
            if stamp <= self.times[-1]:
                return False
            delta = inverse(self.raw[-1]) @ odom_pose
            if (np.linalg.norm(delta[:3, 3]) < cfg.key_distance and
                Rotation.from_matrix(delta[:3, :3]).magnitude() < cfg.key_angle and
                (cfg.key_period <= 0 or stamp-self.times[-1] < cfg.key_period)):
                return False
        cloud = voxelize(body_cloud, cfg.voxel_size)
        if len(cloud) < 30 or not np.isfinite(odom_pose).all():
            return False
        k = len(self.raw)
        graph, initial = gtsam.NonlinearFactorGraph(), gtsam.Values()
        pose = gtsam.Pose3(self.map_to_odom @ odom_pose)
        if k == 0:
            # Anchor the gauge because there is no GPS absolute-position factor.
            noise = gtsam.noiseModel.Diagonal.Variances(np.full(6, 1.e-8))
            graph.add(gtsam.PriorFactorPose3(0, pose, noise))
        else:
            noise = gtsam.noiseModel.Diagonal.Variances(np.array([1.e-6]*3+[1.e-4]*3))
            relative = gtsam.Pose3(inverse(self.raw[-1]) @ odom_pose)
            graph.add(gtsam.BetweenFactorPose3(k-1, k, relative, noise))
        initial.insert(k, pose)
        self.times.append(stamp)
        self.raw.append(odom_pose.copy())
        self.clouds.append(cloud)
        self.isam.update(graph, initial)
        self.isam.update()
        self._correct_poses()
        if stamp-self.last_loop_time >= cfg.loop_interval:
            self._try_loop()
        return True

    def _correct_poses(self):
        estimate = self.isam.calculateEstimate()
        self.poses = [estimate.atPose3(k).matrix() for k in range(len(self.raw))]
        self.map_to_odom = self.poses[-1] @ inverse(self.raw[-1])

    def _try_loop(self):
        cfg = self.config
        k = len(self.poses)-1
        self.last_loop_time = self.times[k]
        travel = np.r_[0., np.cumsum([np.linalg.norm(b[:3, 3]-a[:3, 3])
                     for a,b in zip(self.raw[:-1],self.raw[1:])])]
        candidates = [(np.linalg.norm(p[:3, 3]-self.poses[k][:3, 3]), j)
                      for j, p in enumerate(self.poses)
                      if self.times[k]-self.times[j] > cfg.loop_min_time
                      and travel[k]-travel[j] > cfg.loop_min_travel]
        candidates = sorted((d, j) for d, j in candidates if d < cfg.loop_radius)
        if not candidates:
            return
        _, j = candidates[0]
        source = transform(self.clouds[k], self.poses[k])
        # Keep temporally recent frames out of the historical target to avoid self-matches.
        neighbors = [n for n in range(max(0, j-cfg.loop_neighbors), min(k, j+cfg.loop_neighbors+1))
                     if self.times[k]-self.times[n] > cfg.loop_min_time]
        if not neighbors:
            return
        target = voxelize(np.concatenate([transform(self.clouds[n], self.poses[n])
                                          for n in neighbors]), cfg.voxel_size)
        correction, score, overlap, converged = icp(source, target,
            max_distance=min(2*cfg.loop_radius, 1.0), min_overlap=cfg.loop_overlap)
        accepted = bool(converged and score < cfg.loop_fitness and
                        np.linalg.norm(correction[:3, 3]) < cfg.loop_max_correction and
                        Rotation.from_matrix(correction[:3, :3]).magnitude() < cfg.loop_max_rotation)
        event = dict(current=k, previous=j, time=self.times[k], mse=score,
                     overlap=overlap, accepted=accepted)
        self.attempts.append(event)
        if not accepted:
            return
        # LIO-SAM: correction pre-multiplies the current world pose, then current.between(previous).
        corrected = gtsam.Pose3(correction @ self.poses[k])
        measured = corrected.between(gtsam.Pose3(self.poses[j]))
        noise = gtsam.noiseModel.Diagonal.Variances(np.full(6, max(score, 1.e-5)))
        graph = gtsam.NonlinearFactorGraph()
        graph.add(gtsam.BetweenFactorPose3(k, j, measured, noise))
        self.isam.update(graph, gtsam.Values())
        for _ in range(5):
            self.isam.update()
        self.loops.append(event)
        self._correct_poses()

    def map_cloud(self):
        if not self.clouds:
            return np.empty((0, 3))
        cloud = voxelize(np.concatenate([transform(c, p) for c, p in zip(self.clouds, self.poses)]),
                         self.config.voxel_size)
        return supported_points(cloud, self.config.map_outlier_radius, self.config.map_min_neighbors)
