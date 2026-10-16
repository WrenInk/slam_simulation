import json
import signal
from pathlib import Path
from dataclasses import fields
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path as RosPath, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from message_filters import Subscriber, TimeSynchronizer
from .pose_graph import PoseGraph, GraphConfig
from .ros_utils import seconds, stamp, from_pose, to_pose, tf_msg, xyz, cloud_msg
from .geometry import write_pcd, matrix, inverse
from .terrain import TerrainConfig, classify


class Backend(Node):
    def __init__(self):
        super().__init__('lio_sam_backend')
        cfg = GraphConfig()
        for f in fields(cfg):
            setattr(cfg, f.name, self.declare_parameter(f.name, getattr(cfg, f.name)).value)
        self.graph = PoseGraph(cfg)
        self.output = Path(self.declare_parameter('output_dir', 'artifacts/slam').value)
        self.map_save_dir = self.declare_parameter('map_save_dir', 'maps').value
        self.save_on_shutdown = self.declare_parameter('save_on_shutdown', False).value
        self.resolution = self.declare_parameter('map_resolution', 0.05).value
        self.map_display_height = self.declare_parameter('map_display_height', 0.).value
        self.terrain = TerrainConfig.from_node(self)
        self.footprint_length = self.declare_parameter('observed_footprint_length', .2).value
        self.footprint_width = self.declare_parameter('observed_footprint_width', .2).value
        self.base_to_imu = matrix(x=self.declare_parameter('imu_x', 0.).value,
                                  z=self.declare_parameter('imu_height', .18).value)
        self.latest = None
        self.received = 0
        self.tf = TransformBroadcaster(self)
        self.path_pub = self.create_publisher(RosPath, 'slam/path', 1)
        self.cloud_pub = self.create_publisher(PointCloud2, 'slam/map_cloud', 1)
        self.status_pub = self.create_publisher(String, 'slam/status', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos)
        self.odom_sub = Subscriber(self, Odometry, 'Odometry', qos_profile=50)
        self.cloud_sub = Subscriber(self, PointCloud2, 'cloud_registered_body', qos_profile=50)
        self.sync = TimeSynchronizer([self.odom_sub, self.cloud_sub], 100)
        self.sync.registerCallback(self.on_frame)
        self.create_timer(1.0, self.publish_map)
        self.create_timer(0.05, self.publish_tf)
        self.create_service(Trigger, 'slam/save_map', self.save)
        self.get_logger().info('LIO-SAM pose graph ready: exact stamped FAST-LIO odometry + body cloud')

    def on_frame(self, odom, cloud):
        t = seconds(odom.header.stamp)
        if self.latest is not None and t <= self.latest:
            return
        self.latest = t
        self.received += 1
        self.graph.add(t, from_pose(odom.pose.pose), xyz(cloud))
        self.publish_tf()

    def publish_tf(self):
        if self.latest is not None:
            # Corrections stay valid until the next graph update; use current ROS time.
            self.tf.sendTransform(tf_msg(self.graph.map_to_odom, 'map', 'odom',
                                         self.get_clock().now().to_msg()))

    def occupancy(self, cloud):
        """Rebuild terrain after every graph correction; slopes use local elevation."""
        out = OccupancyGrid()
        out.header.frame_id = 'map'
        out.header.stamp = stamp(self.latest)
        out.info.resolution = self.resolution
        out.info.origin.orientation.w = 1.
        out.info.origin.position.z = self.map_display_height
        if not len(cloud):
            return out
        terrain = classify(cloud, self.terrain)
        scale = int(round(terrain.resolution/self.resolution))
        if scale < 1 or not np.isclose(scale*self.resolution, terrain.resolution):
            raise ValueError('terrain_resolution must be an integer multiple of map_resolution')
        grid = np.repeat(np.repeat(terrain.occupancy, scale, axis=0), scale, axis=1)
        lo = terrain.origin
        wh = [grid.shape[1], grid.shape[0]]
        # Fill only unknown cells under the observed robot footprint. Measured
        # obstacle evidence always wins, including curbs alongside the trajectory.
        from .terrain import clear_unknown_footprints
        bases = [pose @ inverse(self.base_to_imu) for pose in self.graph.poses]
        clear_unknown_footprints(grid, lo, self.resolution, bases,
                                 self.footprint_length, self.footprint_width)
        out.info.width, out.info.height = map(int, wh)
        out.info.origin.position.x, out.info.origin.position.y = map(float, lo)
        out.data = grid.ravel().tolist()
        return out

    def publish_map(self):
        if not self.graph.poses:
            return
        path = RosPath()
        path.header.frame_id = 'map'
        path.header.stamp = stamp(self.latest)
        for t, p in zip(self.graph.times, self.graph.poses):
            ps = PoseStamped()
            ps.header.frame_id, ps.header.stamp = 'map', stamp(t)
            ps.pose = to_pose(p)
            path.poses.append(ps)
        self.path_pub.publish(path)
        cloud = self.graph.map_cloud()
        self.cloud_pub.publish(cloud_msg(cloud, 'map', stamp(self.latest)))
        grid = self.occupancy(cloud)
        if grid.info.width:
            self.map_pub.publish(grid)
        self.status_pub.publish(String(data=json.dumps(dict(frames=self.received,
            keyframes=len(self.graph.poses), loops=len(self.graph.loops), points=len(cloud)))))

    def save(self, request, response):
        if not self.graph.poses:
            response.success, response.message = False, 'No synchronized keyframes received'
            return response
        try:
            self.output.mkdir(parents=True, exist_ok=True)
            cloud = self.graph.map_cloud()
            np.savez_compressed(self.output/'map.npz', points=cloud, times=self.graph.times,
                                raw_poses=self.graph.raw, optimized_poses=self.graph.poses)
            write_pcd(self.output/'map.pcd', cloud)
            for name, poses in [('trajectory', self.graph.poses), ('odometry', self.graph.raw)]:
                from scipy.spatial.transform import Rotation
                rows = [[t, *p[:3, 3], *Rotation.from_matrix(p[:3, :3]).as_quat()]
                        for t, p in zip(self.graph.times, poses)]
                np.savetxt(self.output/(name+'.tum'), rows, fmt='%.9f')
            summary = dict(frames=self.received, keyframes=len(self.graph.poses),
                points=len(cloud), loops=self.graph.loops, attempts=self.graph.attempts,
                duration=self.graph.times[-1]-self.graph.times[0])
            (self.output/'metrics.json').write_text(json.dumps(summary, indent=2))
            grid = self.occupancy(cloud)
            if grid.info.width:
                from PIL import Image
                import yaml
                a = np.array(grid.data).reshape(grid.info.height, grid.info.width)
                image = np.where(a<0, 205, np.where(a>50, 0, 254)).astype('uint8')
                Image.fromarray(np.flipud(image)).save(self.output/'map.pgm')
                (self.output/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',
                    resolution=grid.info.resolution,
                    origin=[grid.info.origin.position.x, grid.info.origin.position.y, 0.],
                    negate=0, occupied_thresh=0.65, free_thresh=0.25)))
            if self.map_save_dir:
                try:
                    target_dir = Path(self.map_save_dir).resolve()
                    target_dir.mkdir(parents=True, exist_ok=True)
                    import shutil
                    for fname in ['map.pcd', 'map.pgm', 'map.yaml', 'map.npz']:
                        f_src = self.output / fname
                        if f_src.is_file():
                            shutil.copy2(f_src, target_dir / fname)
                    print(f'已同步保存至标准地图目录：{target_dir}', flush=True)
                except Exception as copy_exc:
                    print(f'同步到标准地图目录警告: {copy_exc}', flush=True)
            response.success, response.message = True, str(self.output.resolve())
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Backend()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Terminal SIGINT and launch forwarding can arrive twice during saving.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node.save_on_shutdown and node.graph.poses:
            result = node.save(Trigger.Request(), Trigger.Response())
            # ROS logging may already be shut down after Ctrl+C.
            if result.success:
                print(f'自动保存完成：{node.output.resolve() / "map.pcd"}（含定位与栅格地图）', flush=True)
            else:
                print(f'自动保存失败：{result.message}', flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
