"""RC2027 field with a free, wheel-driven differential chassis.

MuJoCo wheel/caster contacts and spring travel determine height and attitude on
ramps. IMU readings come from site sensors; rays use the same full SE(3) sensor
pose. Tire friction and spring parameters are illustrative, not hardware-calibrated.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import mujoco
from scipy.spatial.transform import Rotation
from .field_assets import load_field


class World:
    def __init__(self, field_path, x=-4.7, y=-4.7, yaw=0., rays=2400, seed=7):
        path = Path(field_path).resolve()
        root = load_field(path)
        root.find('option').set('timestep', '0.001')
        root.find('option').set('integrator', 'implicitfast')
        root.find('option').set('cone', 'elliptic')
        # Raycast the visual triangle meshes. Invisible collision primitives are
        # deliberately omitted; MuJoCo's mj_ray ignores fully transparent geoms.
        for geom in root.iter('geom'):
            geom.set('group', '0')
        wb = root.find('worldbody')
        robot = ET.SubElement(wb, 'body', name='nav_robot', pos=f'{x} {y} 0.028',
                              quat=f'{np.cos(yaw/2)} 0 0 {np.sin(yaw/2)}')
        ET.SubElement(robot, 'freejoint', name='nav_free')
        ET.SubElement(robot, 'geom', name='nav_chassis', type='cylinder', size='0.145 0.055',
                      pos='0 0 0.105', mass='8', rgba='0.12 0.18 0.22 1', group='1')
        ET.SubElement(robot, 'geom', name='nav_lidar', type='cylinder', size='0.032 0.015',
                      pos='0 0 0.18', mass='0.1', rgba='0.1 0.1 0.1 1',
                      contype='0', conaffinity='0', group='1')
        ET.SubElement(robot, 'site', name='nav_imu', pos='0 0 0.18', size='.005', group='1')
        for side, sign in [('left', 1), ('right', -1)]:
            carrier = ET.SubElement(robot, 'body', name='nav_'+side+'_carrier', pos=f'0 {sign*.165} .060')
            ET.SubElement(carrier, 'joint', name='nav_'+side+'_suspension', type='slide',
                          axis='0 0 1', stiffness='3000', damping='70', range='-.025 .025')
            ET.SubElement(carrier, 'inertial', pos='0 0 0', mass='.05', diaginertia='.0001 .0001 .0001')
            wheel = ET.SubElement(carrier, 'body', name='nav_'+side)
            ET.SubElement(wheel, 'joint', name='nav_'+side, type='hinge', axis='0 1 0', damping='.01')
            ET.SubElement(wheel, 'geom', name='nav_'+side, type='cylinder', size='.075 .015',
                          euler='1.5707963267948966 0 0', mass='.4', friction='1 .005 .0001',
                          rgba='.04 .04 .04 1', group='1')
        for name, cx in [('front', .115), ('rear', -.115)]:
            ET.SubElement(robot, 'geom', name='nav_caster_'+name, type='sphere', size='.025',
                          pos=f'{cx} 0 .025', mass='.05', friction='.005 .0001 .0001',
                          priority='1', rgba='.2 .2 .2 1', group='1')
        actuator = ET.SubElement(root, 'actuator')
        for side in ['left', 'right']:
            ET.SubElement(actuator, 'velocity', name='nav_'+side+'_vel', joint='nav_'+side,
                          kv='2', forcelimited='true', forcerange='-5 5')
        sensors = ET.SubElement(root, 'sensor')
        ET.SubElement(sensors, 'accelerometer', name='nav_acc', site='nav_imu')
        ET.SubElement(sensors, 'gyro', name='nav_gyro', site='nav_imu')
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(root).decode())
        self.data = mujoco.MjData(self.model)
        self.free_qpos = self.model.joint('nav_free').qposadr[0]
        self.robot_id = self.model.body('nav_robot').id
        self.imu_id = self.model.site('nav_imu').id
        self.robot_geoms = set(g for g in range(self.model.ngeom)
            if self.model.body_rootid[self.model.geom_bodyid[g]] == self.robot_id)
        # Settle rigid contacts before publishing time or initializing FAST-LIO.
        for _ in range(1000):
            mujoco.mj_step(self.model, self.data)
        self.data.time = 0.
        self.rays = rays
        self.rng = np.random.default_rng(seed)
        self.collision_steps = 0
        self.steps = 0
        self.ray_group = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.lidar_to_imu = np.diag([1., -1., -1.])
        mujoco.mj_forward(self.model, self.data)

    @property
    def transform(self):
        pose = np.eye(4)
        pose[:3, :3] = self.data.xmat[self.robot_id].reshape(3, 3)
        pose[:3, 3] = self.data.xpos[self.robot_id]
        return pose

    @property
    def pose(self):
        p = self.transform
        return np.array([p[0, 3], p[1, 3], np.arctan2(p[1, 0], p[0, 0])])

    def set_replay_pose(self, row):
        """Log replay only; never called by the live simulation or navigation."""
        q = self.free_qpos
        self.data.qpos[q:q+3] = [row[1], row[2], row[4] if len(row) > 4 else .025]
        quat = Rotation.from_euler('xyz', [row[5], row[6], row[3]] if len(row) > 6 else [0, 0, row[3]]).as_quat()
        self.data.qpos[q+3:q+7] = quat[[3, 0, 1, 2]]
        mujoco.mj_forward(self.model, self.data)

    def step(self, linear, angular):
        self.data.ctrl[-2:] = [(linear-angular*.165)/.075, (linear+angular*.165)/.075]
        acc_samples, gyro_samples = [], []
        for _ in range(5):
            mujoco.mj_step(self.model, self.data)
            acc_samples.append(self.data.sensor('nav_acc').data.copy())
            gyro_samples.append(self.data.sensor('nav_gyro').data.copy())
        self.steps += 1
        for contact in self.data.contact:
            if (contact.geom1 in self.robot_geoms or contact.geom2 in self.robot_geoms):
                # Normal wheel-ground and wheel-ramp support is not a collision.
                # Vertical obstacle contact or chassis scraping is counted.
                robot_geom = contact.geom1 if contact.geom1 in self.robot_geoms else contact.geom2
                chassis = self.model.geom(robot_geom).name == 'nav_chassis'
                if contact.dist < -.001 and (abs(contact.frame[2]) < .7 or chassis):
                    self.collision_steps += 1
                    break
        acc = np.mean(acc_samples, axis=0)+self.rng.normal(0., .002, 3)
        gyro = np.mean(gyro_samples, axis=0)+self.rng.normal(0., .0001, 3)
        return acc, gyro

    def scan_chunk(self, indices):
        origin = self.data.site_xpos[self.imu_id].copy()
        rot = self.data.site_xmat[self.imu_id].reshape(3, 3)
        global_ids = indices + (self.steps//20)*self.rays
        if hasattr(self, 'scan_directions'):
            dirs = self.scan_directions[(global_ids*self.scan_stride) % len(self.scan_directions)]
        else:
            azimuth = global_ids * (np.pi*(3-np.sqrt(5)))
            elevation = np.deg2rad(-7+59*((global_ids*np.sqrt(2)) % 1))
            dirs = np.column_stack([np.cos(elevation)*np.cos(azimuth),
                np.cos(elevation)*np.sin(azimuth), np.sin(elevation)])
        world_dirs = np.ascontiguousarray(dirs @ self.lidar_to_imu.T @ rot.T)
        ids = np.full(len(indices), -1, dtype=np.int32)
        distances = np.zeros(len(indices), dtype=np.float64)
        if getattr(self, 'competition', False):
            from .raycast import exact_rays
            distances, ids = exact_rays(self.model, self.data, origin, world_dirs,
                                        self.ray_group, self.robot_id)
        else:
            mujoco.mj_multiRay(self.model, self.data, origin, world_dirs.ravel(), self.ray_group,
                              1, self.robot_id, ids, distances, len(indices), 20.)
        valid = (distances>0.2)&(distances<20.)
        distance = distances[valid]+self.rng.normal(0., 0.003, valid.sum())
        return dirs[valid]*distance[:, None], indices[valid]
