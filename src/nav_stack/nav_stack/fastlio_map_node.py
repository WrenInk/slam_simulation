"""Pure FAST-LIO2 Mapping Node.

Subscribes directly to FAST-LIO2 /cloud_registered and /Odometry, maintains
a global voxelized point cloud and 2D occupancy grid, broadcasts map->odom TF,
and on shutdown automatically writes map.pcd, map.pgm, and map.yaml.
Zero dependency on GTSAM, LIO-SAM, or factor graphs.
"""
from collections import deque
import json
from pathlib import Path
import shutil
import signal
import sys
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .geometry import voxelize, write_pcd, matrix, inverse
from .ros_utils import xyz, cloud_msg, seconds, tf_msg, stamp, to_pose, from_pose
from .terrain import TerrainConfig, classify, clear_unknown_footprints, pcd_to_occupancy


class FastLioMapNode(Node):
    def __init__(self):
        super().__init__('fastlio_map_node')
        self.output_dir = Path(self.declare_parameter('output_dir', 'artifacts/mapping/slam').value)
        self.map_save_dir = Path(self.declare_parameter('map_save_dir', 'maps').value)
        self.save_on_shutdown = bool(self.declare_parameter('save_on_shutdown', True).value)
        self.voxel_size = float(self.declare_parameter('voxel_size', 0.04).value)
        self.map_resolution = float(self.declare_parameter('map_resolution', 0.05).value)
        self.map_display_height = float(self.declare_parameter('map_display_height', 0.0).value)
        self.footprint_length = float(self.declare_parameter('observed_footprint_length', 0.54).value)
        self.footprint_width = float(self.declare_parameter('observed_footprint_width', 0.48).value)
        self.imu_height = float(self.declare_parameter('imu_height', 0.15).value)
        self.base_to_imu = matrix(x=0.0, y=0.0, z=self.imu_height)

        self.terrain = TerrainConfig.from_node(self)

        # Buffers for point accumulation
        self.recent_points = []
        self.accumulated_cloud = np.empty((0, 3), dtype=np.float32)
        self.times = []
        self.poses = []  # list of 4x4 base poses
        self.raw_imu_poses = []
        self.received_clouds = 0
        self.latest_time = None

        # Publishers
        self.static_tf = StaticTransformBroadcaster(self)
        self.tf = TransformBroadcaster(self)
        self.path_pub = self.create_publisher(RosPath, 'slam/path', 1)
        self.cloud_pub = self.create_publisher(PointCloud2, 'slam/map_cloud', 1)
        self.status_pub = self.create_publisher(String, 'slam/status', 1)

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', qos)

        # Subscriptions to FAST-LIO2
        self.cloud_sub = self.create_subscription(
            PointCloud2, '/cloud_registered', self.on_cloud, qos_profile=10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/Odometry', self.on_odom, qos_profile=10
        )

        # Services
        self.save_srv = self.create_service(Trigger, 'slam/save_map', self.save_service)

        # Timers
        self.create_timer(1.0, self.publish_map)
        self.create_timer(0.05, self.publish_tf)

        # Broadcast map -> odom identity transform
        now = self.get_clock().now().to_msg()
        self.static_tf.sendTransform(tf_msg(np.eye(4), 'map', 'odom', now))

        self.get_logger().info('Pure FAST-LIO2 map node initialized (high-frequency, zero factor graph overhead).')

    def publish_tf(self):
        now = self.get_clock().now().to_msg()
        self.tf.sendTransform(tf_msg(np.eye(4), 'map', 'odom', now))

    def on_odom(self, msg: Odometry):
        t = seconds(msg.header.stamp)
        self.latest_time = t
        imu_pose = from_pose(msg.pose.pose)
        base_pose = imu_pose @ inverse(self.base_to_imu)
        self.times.append(t)
        self.raw_imu_poses.append(imu_pose)
        self.poses.append(base_pose)

    def on_cloud(self, msg: PointCloud2):
        t = seconds(msg.header.stamp)
        self.latest_time = t
        self.received_clouds += 1
        pts = xyz(msg)
        if len(pts):
            self.recent_points.append(pts)
            # Merge and voxelize every 10 frames (~1 second) to save memory
            if len(self.recent_points) >= 10:
                merged = np.vstack([self.accumulated_cloud, *self.recent_points])
                self.accumulated_cloud = voxelize(merged, self.voxel_size).astype(np.float32)
                self.recent_points = []

    def occupancy(self, cloud):
        out = OccupancyGrid()
        out.header.frame_id = 'map'
        out.header.stamp = stamp(self.latest_time or 0.0)
        out.info.resolution = self.map_resolution
        out.info.origin.orientation.w = 1.0
        out.info.origin.position.z = self.map_display_height
        if not len(cloud):
            return out
        terrain = classify(cloud, self.terrain)
        scale = max(1, int(round(terrain.resolution / self.map_resolution)))
        grid = np.repeat(np.repeat(terrain.occupancy, scale, axis=0), scale, axis=1)
        lo = terrain.origin
        wh = [grid.shape[1], grid.shape[0]]

        if self.poses:
            # Subsample poses to avoid excessive clearing computations
            stride = max(1, len(self.poses) // 200)
            clear_unknown_footprints(
                grid, lo, self.map_resolution, self.poses[::stride],
                self.footprint_length, self.footprint_width
            )

        out.info.width, out.info.height = map(int, wh)
        out.info.origin.position.x, out.info.origin.position.y = map(float, lo)
        out.data = grid.ravel().tolist()
        return out

    def get_full_cloud(self):
        if self.recent_points:
            merged = np.vstack([self.accumulated_cloud, *self.recent_points]) if len(self.accumulated_cloud) else np.vstack(self.recent_points)
            self.accumulated_cloud = voxelize(merged, self.voxel_size).astype(np.float32)
            self.recent_points = []
        return self.accumulated_cloud

    def publish_map(self):
        cloud = self.get_full_cloud()
        if not len(cloud):
            return

        ts = stamp(self.latest_time or 0.0)

        # Publish trajectory path
        if self.poses:
            path = RosPath()
            path.header.frame_id = 'map'
            path.header.stamp = ts
            stride = max(1, len(self.poses) // 500)
            for t, p in zip(self.times[::stride], self.poses[::stride]):
                ps = PoseStamped()
                ps.header.frame_id, ps.header.stamp = 'map', stamp(t)
                ps.pose = to_pose(p)
                path.poses.append(ps)
            self.path_pub.publish(path)

        # Publish accumulated map cloud
        self.cloud_pub.publish(cloud_msg(cloud, 'map', ts))

        # Publish 2D occupancy grid
        grid = self.occupancy(cloud)
        if grid.info.width:
            self.map_pub.publish(grid)

        self.status_pub.publish(String(data=json.dumps(dict(
            frames=self.received_clouds,
            points=len(cloud),
            poses=len(self.poses)
        ))))

    def save_service(self, request, response):
        success, msg = self.save()
        response.success = success
        response.message = msg
        return response

    def save(self):
        cloud = self.get_full_cloud()
        if not len(cloud):
            return False, 'No point cloud accumulated'

        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self.map_save_dir.mkdir(parents=True, exist_ok=True)

            # 1. Save PCD map
            pcd_file = self.output_dir / 'map.pcd'
            write_pcd(pcd_file, cloud)
            target_pcd = self.map_save_dir / 'map.pcd'
            shutil.copyfile(pcd_file, target_pcd)

            # 2. Convert PCD to 2D Occupancy Grid map (.yaml + .pgm)
            pcd_to_occupancy(target_pcd, self.map_save_dir, resolution=self.map_resolution)
            if self.map_save_dir != self.output_dir:
                try:
                    for ext in ['pgm', 'yaml']:
                        if (self.map_save_dir / f'map.{ext}').exists():
                            shutil.copyfile(self.map_save_dir / f'map.{ext}', self.output_dir / f'map.{ext}')
                except Exception as e:
                    pass

            # 3. Save trajectory TUM format
            if self.times and self.poses:
                rows = []
                for t, p in zip(self.times, self.poses):
                    quat = Rotation.from_matrix(p[:3, :3]).as_quat()
                    rows.append([t, p[0, 3], p[1, 3], p[2, 3], quat[0], quat[1], quat[2], quat[3]])
                np.savetxt(self.output_dir / 'trajectory.tum', rows, fmt='%.9f')

            # 4. Save summary metrics
            duration = (self.times[-1] - self.times[0]) if len(self.times) > 1 else 0.0
            metrics = dict(
                frames=self.received_clouds,
                points=len(cloud),
                poses=len(self.poses),
                duration=duration
            )
            (self.output_dir / 'metrics.json').write_text(json.dumps(metrics, indent=2))

            msg = f"Map successfully saved to {target_pcd} and converted to grid ({self.map_save_dir / 'map.pgm'}, {self.map_save_dir / 'map.yaml'})."
            return True, msg
        except Exception as e:
            return False, f"Save failed: {e}"


def main(args=None):
    rclpy.init(args=args)
    node = FastLioMapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node.save_on_shutdown:
            success, msg = node.save()
            if success:
                print(f"[FAST-LIO Map] 自动保存完成：{node.map_save_dir / 'map.pcd'}（含 2D 栅格地图 map.pgm/yaml）", flush=True)
            else:
                print(f"[FAST-LIO Map] 自动保存提示：{msg}", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
