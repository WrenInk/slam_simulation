import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock as RclClock, ClockType
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from .sim_world import World
from .ros_utils import stamp, to_pose
from .geometry import matrix


class Simulator(Node):
    def __init__(self):
        super().__init__('rc2027_simulator')
        self.world_type = self.declare_parameter('world_type', 'rc2027').value
        path = self.declare_parameter('field_path', '').value
        self.output = Path(self.declare_parameter('output_dir', 'artifacts/sim').value)
        self.realtime_factor = self.declare_parameter('realtime_factor', 1.0).value
        self.spawn = [self.declare_parameter('spawn_x', -4.7).value,
                      self.declare_parameter('spawn_y', -4.7).value,
                      self.declare_parameter('spawn_yaw', 0.).value]
        rays = self.declare_parameter('rays', 8000).value
        self.omni = self.world_type in ['omni_arena', 'omni_competition']
        if self.omni:
            from .omni_world import OmniWorld
            self.world = OmniWorld(path, *self.spawn, rays=rays,
                robot_path=self.declare_parameter('robot_path', '').value or None,
                competition=self.world_type == 'omni_competition')
        elif self.world_type == 'rc2027':
            self.world = World(path, *self.spawn, rays=rays)
        else:
            raise ValueError('Unknown world_type: '+self.world_type)
        self.initial_imu_world_pose = self.world.transform.copy()
        self.initial_imu_world_pose[:3, 3] += self.initial_imu_world_pose[:3, :3] @ [0., 0., getattr(self.world, 'imu_height', .18)]
        self.camera_center = [self.spawn[0]+2*np.cos(self.spawn[2]),
                              self.spawn[1]+2*np.sin(self.spawn[2]), .2]
        self.camera_distance, self.camera_elevation = 7., -85.
        self.camera_azimuth = 90.
        if self.omni:
            self.camera_center = [0., 0., 0.]
            self.camera_distance, self.camera_elevation = 13.5, -65.
        if self.world_type == 'omni_competition':
            self.camera_center = [0.,0.,.3]
            self.camera_distance, self.camera_elevation, self.camera_azimuth = 19.5, -50., 110.
        self.viewer = None
        if self.declare_parameter('gui', False).value:
            import mujoco.viewer
            self.viewer = mujoco.viewer.launch_passive(self.world.model, self.world.data,
                                                       show_left_ui=False, show_right_ui=False)
            with self.viewer.lock():
                self.viewer.cam.lookat[:] = self.camera_center
                self.viewer.cam.distance = self.camera_distance
                self.viewer.cam.azimuth = self.camera_azimuth
                self.viewer.cam.elevation = self.camera_elevation
        self.clock_pub = self.create_publisher(Clock, '/clock', 10)
        self.imu_pub = self.create_publisher(Imu, '/livox/imu', 200)
        self.lidar_pub = self.create_publisher(CustomMsg, '/livox/lidar', 10)
        self.truth_pub = self.create_publisher(Odometry, '/ground_truth/odom', 10)
        self.status_pub = self.create_publisher(String, '/sim/status', 10)
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.linear, self.lateral, self.angular, self.command_time = 0., 0., 0., -10.
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10) if self.omni else None
        self.points, self.rows = [], []
        self.step_index = 0
        # Wall timer advances /clock; a ROS-time timer would deadlock before the first clock.
        self.wall_clock = RclClock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(0.005/self.realtime_factor, self.tick, clock=self.wall_clock)
        self.get_logger().info(self.world_type+' MuJoCo sensor simulation ready; ground truth is evaluation-only')

    def command(self, msg):
        if not np.isfinite([msg.linear.x, msg.linear.y, msg.angular.z]).all():
            self.linear = self.lateral = self.angular = 0.
            return
        self.linear = float(np.clip(msg.linear.x, -0.5, 0.5))
        self.lateral = float(np.clip(msg.linear.y, -0.5, 0.5))
        self.angular = float(np.clip(msg.angular.z, -0.8, 0.8))
        self.command_time = self.world.data.time

    def tick(self):
        if self.world.data.time-self.command_time > 0.5:
            self.linear = self.lateral = self.angular = 0.
        if self.omni:
            acc, gyro = self.world.step(self.linear, self.lateral, self.angular)
        else:
            acc, gyro = self.world.step(self.linear, self.angular)
        t = self.world.data.time
        # Offset from zero makes TF and rosbag timestamp initialization unambiguous.
        ts = stamp(t+1.)
        self.clock_pub.publish(Clock(clock=ts))
        imu = Imu()
        imu.header.stamp, imu.header.frame_id = ts, 'imu_link'
        imu.orientation_covariance[0] = -1.
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = map(float, acc)
        imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = map(float, gyro)
        self.imu_pub.publish(imu)
        n = self.step_index % 20
        start = n*self.world.rays//20
        stop = (n+1)*self.world.rays//20
        points, ids = self.world.scan_chunk(np.arange(start, stop))
        for p, i in zip(points, ids):
            # Each chunk is sampled at its actual acquisition pose and time.
            self.points.append(CustomPoint(x=float(p[0]), y=float(p[1]), z=float(p[2]),
                reflectivity=100, tag=0x10, line=int(i % 4), offset_time=n*5000000))
        if n == 19:
            if self.joint_pub:
                from .omni_world import WHEELS
                joints = JointState()
                joints.header.stamp = ts
                joints.name = [n+'_wheel_joint' for n in WHEELS]
                joints.name += self.world.suspension_names
                joints.position = self.world.data.qpos[self.world.wheel_qpos+self.world.suspension_qpos].tolist()
                self.joint_pub.publish(joints)
            cloud = CustomMsg()
            cloud.header.frame_id, cloud.header.stamp = 'lidar_link', stamp(t+1.-0.095)
            cloud.timebase = int(round((t+1.-0.095)*1.e9))
            cloud.points, cloud.point_num = self.points, len(self.points)
            self.lidar_pub.publish(cloud)
            self.points = []
            x, y, yaw = self.world.pose
            truth = Odometry()
            truth.header.stamp, truth.header.frame_id = ts, 'world'
            truth.child_frame_id = 'truth_base_link'
            truth.pose.pose = to_pose(self.world.transform)
            self.truth_pub.publish(truth)
            roll, pitch, _ = Rotation.from_matrix(self.world.transform[:3, :3]).as_euler('xyz')
            z = self.world.transform[2, 3]
            self.rows.append([t+1., x, y, yaw, z, roll, pitch])
            self.status_pub.publish(String(data=json.dumps(dict(time=t,
                collision_steps=self.world.collision_steps, x=x, y=y, yaw=yaw, z=z, roll=roll, pitch=pitch))))
            if self.viewer is not None and self.viewer.is_running():
                if self.step_index == 19:
                    # The viewer thread loads the scene asynchronously; apply the
                    # initial camera after loading, then leave user camera control alone.
                    with self.viewer.lock():
                        self.viewer.cam.lookat[:] = self.camera_center
                        self.viewer.cam.distance = self.camera_distance
                        self.viewer.cam.azimuth = self.camera_azimuth
                        self.viewer.cam.elevation = self.camera_elevation
                self.viewer.sync()
            elif self.viewer is not None and self.omni:
                raise KeyboardInterrupt
        self.step_index += 1

    def save(self):
        self.output.mkdir(parents=True, exist_ok=True)
        np.savetxt(self.output/'ground_truth.csv', self.rows, delimiter=',',
                   header='time,x,y,yaw,z,roll,pitch', comments='')
        (self.output/'simulation.json').write_text(json.dumps(dict(
            duration=self.world.data.time, collision_steps=self.world.collision_steps,
            steps=self.world.steps, model=getattr(self.world, 'model_description', 'MuJoCo free chassis with driven wheels and passive caster contacts'),
            spawn=self.spawn,
            initial_imu_world_pose=self.initial_imu_world_pose.tolist(),
            sensor='inverted MID360 FOV, body-aligned external IMU, quasi-uniform pattern'), indent=2))
        if self.omni:
            (self.output/'scene.xml').write_text(self.world.xml)
            np.savez_compressed(self.output/'replay.npz', states=np.array(self.world.snapshots))
            np.savetxt(self.output/'wheel_commands.csv', self.world.command_trace, delimiter=',',
                header='time,vx,vy,wz,front_left,front_right,rear_left,rear_right', comments='')
            (self.output/'collision.json').write_text(json.dumps(dict(
                steps=self.world.collision_steps, maximum_penetration_m=self.world.max_obstacle_penetration,
                first_collision=self.world.first_collision), indent=2))
        if self.viewer is not None:
            self.viewer.close()


def main(args=None):
    rclpy.init(args=args)
    node = Simulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
