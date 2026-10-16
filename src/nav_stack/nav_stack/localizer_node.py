"""Prior PCD Map Relocalization & Occupancy Grid Publisher Node.

Loads an existing 3D PCD point cloud map and 2D occupancy grid, performs
robust coarse-to-fine ICP matching to localize FAST-LIO2 odometry in the
prior map, broadcasts map -> odom TF, and provides /map to Nav2.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import numpy as np
from PIL import Image
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from message_filters import Subscriber, TimeSynchronizer
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from scipy.spatial.transform import Rotation

from .geometry import icp, inverse, transform, voxelize, read_pcd
from .ros_utils import xyz, from_pose, to_pose, tf_msg, cloud_msg, seconds


def load_grid_from_yaml(yaml_path):
    yaml_path = Path(yaml_path).resolve()
    cfg = yaml.safe_load(yaml_path.read_text())
    image_path = yaml_path.parent / cfg['image']
    pgm = np.flipud(np.asarray(Image.open(image_path).convert('L')))
    occupancy = (pgm / 255.) if cfg.get('negate', 0) else (1. - pgm / 255.)
    values = np.full(pgm.shape, -1, np.int8)
    values[occupancy < cfg.get('free_thresh', 0.25)] = 0
    values[occupancy > cfg.get('occupied_thresh', 0.65)] = 100

    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.info.resolution = float(cfg['resolution'])
    grid.info.width = int(pgm.shape[1])
    grid.info.height = int(pgm.shape[0])
    origin = cfg['origin']
    grid.info.origin.position.x = float(origin[0])
    grid.info.origin.position.y = float(origin[1])
    grid.info.origin.position.z = float(origin[2]) if len(origin) > 2 else 0.0
    grid.info.origin.orientation.w = 1.0
    grid.data = values.ravel().tolist()
    return grid


class PriorMapLocalizer(Node):
    def __init__(self):
        super().__init__('prior_map_localizer')
        self.pcd_path = Path(self.declare_parameter('map_pcd', 'maps/map.pcd').value).resolve()
        self.yaml_path = Path(self.declare_parameter('map_yaml', 'maps/map.yaml').value).resolve()
        self.imu_height = float(self.declare_parameter('imu_height', 0.15).value)
        self.init_x = float(self.declare_parameter('initial_pose_x', 0.0).value)
        self.init_y = float(self.declare_parameter('initial_pose_y', 0.0).value)
        self.init_yaw = float(self.declare_parameter('initial_pose_yaw', 0.0).value)
        self.manual_mode = bool(self.declare_parameter('manual_mode', False).value)
        self.manual_triggered = False

        self.get_logger().info(f"加载先验 PCD 点云地图: {self.pcd_path}")
        if not self.pcd_path.is_file():
            raise FileNotFoundError(f"先验点云不存在: {self.pcd_path}")

        raw_points = read_pcd(self.pcd_path)
        self.get_logger().info(f"先验地图总点数: {len(raw_points)}")
        self.prior_coarse = voxelize(raw_points, 0.15)
        self.prior_fine = voxelize(raw_points, 0.06)
        self.prior_cloud_vis = voxelize(raw_points, 0.08)

        # Load or generate 2D Occupancy Grid
        if self.yaml_path.is_file():
            self.get_logger().info(f"加载先验栅格地图配置: {self.yaml_path}")
            self.grid = load_grid_from_yaml(self.yaml_path)
        else:
            self.get_logger().info("未找到预置 map.yaml，从 PCD 自动计算生成占据栅格...")
            from nav_stack.terrain import classify, TerrainConfig
            terrain = classify(raw_points, TerrainConfig(terrain_resolution=0.10))
            scale = 2
            grid_data = np.repeat(np.repeat(terrain.occupancy, scale, axis=0), scale, axis=1)
            h, w = grid_data.shape
            image = np.where(grid_data < 0, 205, np.where(grid_data > 50, 0, 254)).astype(np.uint8)
            Image.fromarray(np.flipud(image)).save(self.yaml_path.parent / 'map.pgm')
            cfg_data = {
                'image': 'map.pgm', 'resolution': 0.05,
                'origin': [float(terrain.origin[0]), float(terrain.origin[1]), 0.0],
                'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.25
            }
            self.yaml_path.write_text(yaml.safe_dump(cfg_data, sort_keys=False))
            self.grid = load_grid_from_yaml(self.yaml_path)

        # State
        self.latest_odom = None
        self.latest_cloud = None
        self.latest_time = None
        self.map_to_odom = np.eye(4)
        self.map_to_odom[:3, 3] = [self.init_x, self.init_y, 0.0]
        self.map_to_odom[:3, :3] = Rotation.from_euler('z', self.init_yaw).as_matrix()

        self.localized = False
        self.consecutive_good = 0
        self.last_match_time = 0.0

        self.pool = ThreadPoolExecutor(max_workers=1)
        self.future = None

        # Publishers
        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_latched)
        self.cloud_pub = self.create_publisher(PointCloud2, '/slam/map_cloud', qos_latched)
        self.status_pub = self.create_publisher(String, '/localization/status', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/localization/pose', 10)
        self.path_pub = self.create_publisher(RosPath, '/slam/path', 1)
        self.path = RosPath()
        self.path.header.frame_id = 'map'

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriptions
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_initial_pose, 10)
        self.odom_sub = Subscriber(self, Odometry, '/Odometry', qos_profile=20)
        self.cloud_sub = Subscriber(self, PointCloud2, '/cloud_registered_body', qos_profile=20)
        self.sync = TimeSynchronizer([self.odom_sub, self.cloud_sub], 30)
        self.sync.registerCallback(self.on_frame)

        self.create_service(Trigger, '/localization/relocalize', self.on_relocalize)

        # Timers
        self.create_timer(0.05, self.publish_tf_and_pose)  # 20 Hz TF
        self.create_timer(0.15, self.match_cycle)           # ~7 Hz ICP matching
        self.create_timer(2.0, self.publish_map_periodic)

        self.get_logger().info("先验 PCD 地图重定位节点已就绪。")

    def on_frame(self, odom_msg, cloud_msg_in):
        self.latest_time = seconds(odom_msg.header.stamp)
        self.latest_odom = from_pose(odom_msg.pose.pose)
        self.latest_cloud = xyz(cloud_msg_in)

    def on_initial_pose(self, msg):
        p = msg.pose.pose
        x, y = p.position.x, p.position.y
        q = p.orientation
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.get_logger().info(f"收到 RViz 初始位姿设定: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad")
        if self.latest_odom is not None:
            init_map_pose = np.eye(4)
            init_map_pose[:3, 3] = [x, y, 0.0]
            init_map_pose[:3, :3] = Rotation.from_euler('z', yaw).as_matrix()
            self.map_to_odom = init_map_pose @ inverse(self.latest_odom)
            self.localized = False
            self.consecutive_good = 0
            self.manual_triggered = True

    def on_relocalize(self, req, res):
        self.localized = False
        self.consecutive_good = 0
        self.manual_triggered = True
        res.success = True
        res.message = "重置定位状态，开始重新匹配"
        return res

    def compute_match(self, body_cloud, odom_pose, current_map_to_odom, is_global=False):
        cloud_ds = voxelize(body_cloud, 0.10)
        if len(cloud_ds) < 80:
            return None, {'ok': False, 'reason': 'insufficient_points'}

        base_guess = current_map_to_odom @ odom_pose
        yaw_offsets = [0.0] if not is_global else [0.0, -0.4, 0.4, -0.8, 0.8, -1.5, 1.5, 3.14]

        best_score = float('inf')
        best_correction = None
        best_stats = None

        for d_yaw in yaw_offsets:
            trial_guess = base_guess.copy()
            if abs(d_yaw) > 1e-3:
                trial_guess[:3, :3] = Rotation.from_euler('z', d_yaw).as_matrix() @ trial_guess[:3, :3]

            source_world = transform(cloud_ds, trial_guess)
            # Step 1: Coarse ICP with adaptive search basin
            max_d_c = 0.85 if self.manual_mode else 0.7
            min_o_c = 0.45 if self.manual_mode else 0.55
            step_c, score_c, overlap_c, ok_c = icp(
                source_world, self.prior_coarse, max_distance=max_d_c, max_iterations=40, min_overlap=min_o_c
            )
            if not ok_c:
                continue

            refined_pose = step_c @ trial_guess
            source_refined = transform(cloud_ds, refined_pose)
            # Step 2: Fine ICP
            max_d_f = 0.35 if self.manual_mode else 0.30
            min_o_f = 0.60 if self.manual_mode else 0.65
            step_f, score_f, overlap_f, ok_f = icp(
                source_refined, self.prior_fine, max_distance=max_d_f, max_iterations=30, min_overlap=min_o_f
            )
            if ok_f and score_f < best_score:
                best_score = score_f
                final_pose = step_f @ refined_pose
                best_correction = final_pose @ inverse(odom_pose)
                best_stats = {
                    'ok': True, 'score': score_f, 'overlap': overlap_f,
                    'delta_trans': float(np.linalg.norm(final_pose[:3, 3] - base_guess[:3, 3])),
                    'delta_rot': float(Rotation.from_matrix(final_pose[:3, :3] @ base_guess[:3, :3].T).magnitude())
                }

        return best_correction, best_stats

    def match_cycle(self):
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            try:
                corr, stats = future.result()
                if corr is not None and stats['ok']:
                    # Smooth update
                    self.map_to_odom = corr
                    self.consecutive_good += 1
                    self.last_match_time = self.get_clock().now().nanoseconds * 1e-9
                    if self.consecutive_good >= 2:
                        if not self.localized and self.manual_mode:
                            self.get_logger().info(f"手动重定位匹配成功: 残差={stats['score']:.4f}m, 重合度={stats['overlap']*100:.1f}%")
                        self.localized = True
                    self.status_pub.publish(String(data=json.dumps(dict(
                        status='tracking', localized=self.localized,
                        score=stats['score'], overlap=stats['overlap']
                    ))))
                else:
                    self.consecutive_good = max(0, self.consecutive_good - 1)
            except Exception as exc:
                self.get_logger().warn(f"ICP 匹配异常: {exc}")

        if self.future is not None or self.latest_odom is None or self.latest_cloud is None:
            return

        if self.manual_mode and not self.manual_triggered:
            return

        is_global = not self.localized
        self.future = self.pool.submit(
            self.compute_match,
            self.latest_cloud.copy(),
            self.latest_odom.copy(),
            self.map_to_odom.copy(),
            is_global
        )

    def publish_tf_and_pose(self):
        if self.latest_odom is None:
            return
        now = self.get_clock().now()

        # Broadcast map -> odom
        self.tf_broadcaster.sendTransform(tf_msg(self.map_to_odom, 'map', 'odom', now.to_msg()))

        # Publish estimated base pose in map frame
        base_map = self.map_to_odom @ self.latest_odom
        p_msg = PoseWithCovarianceStamped()
        p_msg.header.frame_id = 'map'
        p_msg.header.stamp = now.to_msg()
        p_msg.pose.pose = to_pose(base_map)
        # Low covariance when localized
        cov_val = 0.01 if self.localized else 0.5
        p_msg.pose.covariance[0] = cov_val
        p_msg.pose.covariance[7] = cov_val
        p_msg.pose.covariance[35] = cov_val * 2
        self.pose_pub.publish(p_msg)

        # Path trace
        ps = PoseStamped()
        ps.header = p_msg.header
        ps.pose = p_msg.pose.pose
        self.path.header.stamp = now.to_msg()
        self.path.poses.append(ps)
        if len(self.path.poses) > 800:
            self.path.poses.pop(0)
        self.path_pub.publish(self.path)

    def publish_map_periodic(self):
        now = self.get_clock().now().to_msg()
        self.grid.header.stamp = now
        self.map_pub.publish(self.grid)
        self.cloud_pub.publish(cloud_msg(self.prior_cloud_vis, 'map', now))


def main(args=None):
    rclpy.init(args=args)
    node = PriorMapLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.pool.shutdown(wait=False, cancel_futures=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
