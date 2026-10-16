"""Free-body mecanum chassis with an equivalent roller traction contact law.

Motor servos rotate four real wheel joints. At each supporting wheel the force
along its 45-degree traction direction depends on measured wheel speed and
contact slip, bounded by the measured normal load. Passive-roller friction is
approximated, not a calibrated detailed roller model. No base pose is prescribed.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from .robot_description import add_urdf
from .sim_world import World

WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')
RADIUS, HALF_LENGTH, HALF_WIDTH = .075, .165, .19
SLOPES = np.array([-1., 1., 1., -1.])
OFFSETS = np.array([[HALF_LENGTH, HALF_WIDTH], [HALF_LENGTH, -HALF_WIDTH],
                    [-HALF_LENGTH, HALF_WIDTH], [-HALF_LENGTH, -HALF_WIDTH]])
KINEMATICS = np.column_stack([np.ones(4), SLOPES, SLOPES*OFFSETS[:,0]-OFFSETS[:,1]])/RADIUS


def wheel_rates(vx, vy, wz):
    return KINEMATICS @ np.array([vx, vy, wz])


class OmniWorld(World):
    imu_height = .15
    model_description = 'MuJoCo free chassis; four wheel velocity servos and load-limited equivalent mecanum roller traction'

    def __init__(self, field_path, x=-3.8, y=-3.8, yaw=0., rays=12000, seed=7, robot_path=None, competition=False):
        path = Path(field_path).resolve()
        if competition:
            from .field_assets import load_field
            root = load_field(path, competition=True)
            root.find('option').set('timestep', '.001')
            root.find('option').set('integrator', 'implicitfast')
            root.find('option').set('cone', 'elliptic')
            for geom in root.iter('geom'): geom.set('group', '0')
        else:
            root = ET.parse(path).getroot()
        robot_path = robot_path or path.parent.parent/'urdf/omni_mid360.urdf'
        add_urdf(root, robot_path, 'omni', [x,y,.028 if competition else .003], contact_clamp=True, driven_wheels=True)
        body = root.find("worldbody/body[@name='omni/base_link']")
        body.attrib.pop('mocap')
        body.set('quat', f'{np.cos(yaw/2)} 0 0 {np.sin(yaw/2)}')
        ET.SubElement(body,'freejoint',name='omni/free_base')
        ET.SubElement(body,'site',name='nav_imu',pos=f'0 0 {self.imu_height}',size='.005',rgba='0 0 0 0')
        for joint in body.iter('joint'):
            if joint.get('name','').endswith('_suspension_joint'):
                continue
            joint.set('damping','.002'); joint.set('armature','.0001')
        for geom in body.iter('geom'):
            if 'wheel_visual_0' in geom.get('name',''):
                # Small residual friction models freely rotating passive rollers;
                # longitudinal/diagonal drive force is supplied by traction().
                geom.set('priority','1'); geom.set('friction','.002 .0001 .0001')
                geom.set('solref','.004 1')
        for actuator in root.find('actuator'):
            actuator.tag='velocity'; actuator.attrib.pop('kp',None)
            actuator.set('kv','.5'); actuator.set('forcerange','-2 2')
        sensors=ET.SubElement(root,'sensor')
        ET.SubElement(sensors,'accelerometer',name='nav_acc',site='nav_imu')
        ET.SubElement(sensors,'gyro',name='nav_gyro',site='nav_imu')
        self.xml=ET.tostring(root).decode()
        self.model=mujoco.MjModel.from_xml_string(self.xml)
        self.data=mujoco.MjData(self.model)
        self.robot_id=self.model.body('omni/base_link').id
        self.imu_id=self.model.site('nav_imu').id
        self.free_qpos=self.model.joint('omni/free_base').qposadr[0]
        self.robot_geoms=set(g for g in range(self.model.ngeom)
            if self.model.body_rootid[self.model.geom_bodyid[g]] == self.robot_id)
        self.wheel_geoms=[self.model.geom('omni/'+n+'_wheel_visual_0').id for n in WHEELS]
        self.wheel_dofs=[self.model.joint('omni/'+n+'_wheel_joint').dofadr[0] for n in WHEELS]
        self.wheel_qpos=[self.model.joint('omni/'+n+'_wheel_joint').qposadr[0] for n in WHEELS]
        self.suspension_names=[n+'_suspension_joint' for n in WHEELS
            if mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,'omni/'+n+'_suspension_joint')>=0]
        self.suspension_qpos=[self.model.joint('omni/'+n).qposadr[0] for n in self.suspension_names]
        self.floor=self.model.geom('col_ground' if competition else 'floor').id
        self.support_geoms = {self.floor}
        if competition:
            self.support_geoms.update(i for i in range(self.model.ngeom)
                if self.model.geom(i).name in ['col_l1', 'col_l2'] or
                   (self.model.geom(i).name.startswith('col_band_') and
                    self.model.geom(i).name.endswith(('_ramp', '_plat'))))
        self.competition = competition
        self.support_mask = np.zeros(self.model.ngeom, dtype=bool)
        self.support_mask[list(self.support_geoms)] = True
        self.robot_mask = np.zeros(self.model.ngeom, dtype=bool)
        self.robot_mask[list(self.robot_geoms)] = True
        self.wheel_index = np.full(self.model.ngeom, -1, dtype=int)
        self.wheel_index[self.wheel_geoms] = np.arange(4)
        self.slip_displacement = np.zeros(4)
        self.rays, self.rng = rays, np.random.default_rng(seed)
        # Directions from Livox-SDK/livox_laser_simulation mid360.csv.
        # Retain the official 24000 samples/frame sequence with a reduced ray budget.
        self.scan_directions = np.load(Path(robot_path).resolve().parent.parent /
                                       'scan_patterns/mid360_directions.npy')
        self.scan_stride = max(1, 24000 // self.rays)
        self.steps, self.collision_steps, self.max_obstacle_penetration = 0, 0, 0.
        self.first_collision=None
        self.command=np.zeros(3)
        self.ray_group=np.array([1,0,0,0,0,0],dtype=np.uint8)
        self.lidar_to_imu=np.eye(3)
        self.snapshots=[]
        self.command_trace=[]
        # Initialize contacts before the first IMU sample and SLAM initialization.
        for _ in range(1500):
            self.traction(); mujoco.mj_step(self.model,self.data)
        self.data.time=0.
        mujoco.mj_forward(self.model,self.data)

    def traction(self):
        m,d=self.model,self.data
        d.qfrc_applied[:]=0.
        load=np.zeros(4)
        pairs=d.contact.geom
        wheels=np.max(self.wheel_index[pairs],axis=1)
        supported=np.any(self.support_mask[pairs],axis=1)&(np.abs(d.contact.frame[:,2])>=.8)&(wheels>=0)
        for index in np.flatnonzero(supported):
            f=np.zeros(6); mujoco.mj_contactForce(m,d,int(index),f)
            load[wheels[index]] += max(0., f[0])
        velocity=np.zeros(6)
        mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_BODY,self.robot_id,velocity,1)
        planar=np.array([velocity[3],velocity[4],velocity[2]])
        slip=RADIUS*(d.qvel[self.wheel_dofs]-KINEMATICS @ planar)
        # A bounded elastic contact component supplies static traction on a
        # slope; a purely viscous slip law would roll downhill at zero command.
        if self.competition:
            self.slip_displacement += slip/np.sqrt(2)*m.opt.timestep
            self.slip_displacement = np.clip(self.slip_displacement, -.8*load/350, .8*load/350)
        strength=np.clip(55.*slip/np.sqrt(2)+350*self.slip_displacement,-.8*load,.8*load)
        rot=d.xmat[self.robot_id].reshape(3,3)
        for i,(xy,s,f) in enumerate(zip(OFFSETS,SLOPES,strength)):
            force=rot @ (np.array([1.,s,0.])*f/np.sqrt(2))
            point=d.xpos[self.robot_id]+rot @ [*xy,0.]
            mujoco.mj_applyFT(m,d,force,np.zeros(3),point,self.robot_id,d.qfrc_applied)

    def step(self, vx, vy, wz):
        target=np.clip([vx,vy,wz],[-.5,-.5,-.8],[.5,.5,.8])
        self.command += np.clip(target-self.command,-np.array([.7,.7,1.2])*.005,np.array([.7,.7,1.2])*.005)
        self.data.ctrl[:]=wheel_rates(*self.command)
        acc,gyro=[],[]
        collided=False
        for _ in range(5):
            self.traction(); mujoco.mj_step(self.model,self.data)
            acc.append(self.data.sensor('nav_acc').data.copy())
            gyro.append(self.data.sensor('nav_gyro').data.copy())
            contacts=self.data.contact;pairs=contacts.geom
            wheel_support=(np.any(self.wheel_index[pairs]>=0,axis=1)&
                np.any(self.support_mask[pairs],axis=1)&(np.abs(contacts.frame[:,2])>.8))
            bad=(self.robot_mask[pairs[:,0]]^self.robot_mask[pairs[:,1]])&\
                np.all(pairs!=self.floor,axis=1)&~wheel_support&(contacts.dist<-.0001)
            for index in np.flatnonzero(bad):
                collided=True
                self.max_obstacle_penetration=max(self.max_obstacle_penetration,-float(contacts.dist[index]))
                if self.first_collision is None:
                    self.first_collision=dict(time=self.data.time,geoms=[self.model.geom(int(g)).name for g in pairs[index]])
        self.collision_steps += int(collided)
        self.steps += 1
        if self.steps % 20 == 0:
            self.snapshots.append([self.data.time,*self.data.qpos])
            self.command_trace.append([self.data.time,*self.command,*self.data.qvel[self.wheel_dofs]])
        return np.mean(acc,axis=0)+self.rng.normal(0,.002,3), np.mean(gyro,axis=0)+self.rng.normal(0,.0001,3)
