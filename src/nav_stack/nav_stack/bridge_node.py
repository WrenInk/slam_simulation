from collections import deque
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from message_filters import Subscriber, TimeSynchronizer
from .geometry import matrix, inverse, transform, voxelize
from .terrain import TerrainConfig, classify
from .ros_utils import from_pose, to_pose, tf_msg, seconds, xyz


class Bridge(Node):
    def __init__(self):
        super().__init__('lio_nav_bridge')
        self.height = self.declare_parameter('imu_height', 0.15).value
        self.base_to_imu = matrix(x=self.declare_parameter('imu_x', 0.0).value,
                                  y=self.declare_parameter('imu_y', 0.0).value, z=self.height)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        lidar_t = self.declare_parameter('mapping.extrinsic_T', [-0.011, -0.02329, 0.04412]).value
        lidar_r = self.declare_parameter('mapping.extrinsic_R', [1.,0.,0.,0.,1.,0.,0.,0.,1.]).value
        imu_to_lidar = np.eye(4)
        imu_to_lidar[:3, 3] = lidar_t
        imu_to_lidar[:3, :3] = np.array(lidar_r).reshape(3, 3)
        now = self.get_clock().now().to_msg()
        static_transforms = []
        if self.declare_parameter('publish_sensor_tf', True).value:
            static_transforms.extend([tf_msg(self.base_to_imu, 'base_link', 'imu_link', now),
                                      tf_msg(imu_to_lidar, 'imu_link', 'lidar_link', now)])
        if self.declare_parameter('publish_map_tf', False).value:
            static_transforms.append(tf_msg(np.eye(4), 'map', 'odom', now))
        if static_transforms:
            self.static.sendTransform(static_transforms)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 20)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.terrain = TerrainConfig.from_node(self)
        self.clouds = deque(maxlen=8)
        self.odom_sub = Subscriber(self, Odometry, 'Odometry', qos_profile=50)
        self.cloud_sub = Subscriber(self, PointCloud2, 'cloud_registered_body', qos_profile=50)
        self.sync = TimeSynchronizer([self.odom_sub, self.cloud_sub], 100)
        self.sync.registerCallback(self.on_frame)
        self.scan_counter = 0
        self.previous = None

    def on_frame(self, odom, cloud):
        self.on_odom(odom)
        self.on_cloud(cloud, from_pose(odom.pose.pose))

    def on_odom(self, msg):
        pose = from_pose(msg.pose.pose) @ inverse(self.base_to_imu)
        out = Odometry()
        out.header, out.child_frame_id = msg.header, 'base_footprint'
        out.header.frame_id = 'odom'
        yaw = np.arctan2(pose[1, 0], pose[0, 0])
        level_pose = matrix(*pose[:3, 3], yaw)
        out.pose.pose = to_pose(level_pose)
        out.pose.covariance = msg.pose.covariance
        t = seconds(msg.header.stamp)
        if self.previous:
            previous_t, previous_pose = self.previous
            dt = t-previous_t
            if 0.001 < dt < 1.0:
                delta = inverse(previous_pose) @ level_pose
                velocity = delta[:3, 3]/dt
                out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, velocity)
                omega = Rotation.from_matrix(delta[:3, :3]).as_rotvec()/dt
                out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, omega)
        self.previous = t, level_pose
        self.odom_pub.publish(out)
        self.tf.sendTransform([tf_msg(pose, 'odom', 'base_link', msg.header.stamp),
                               tf_msg(level_pose, 'odom', 'base_footprint', msg.header.stamp)])

    def on_cloud(self, msg, imu_pose):
        # Deskewed body clouds + exactly matching FAST-LIO pose, in a gravity
        # aligned frame. Sensor pitch on ramps must not look like terrain slope.
        points = transform(xyz(msg), imu_pose)
        self.clouds.append(points)
        self.scan_counter += 1
        if self.scan_counter % 2:
            return
        base = imu_pose @ inverse(self.base_to_imu)
        yaw = np.arctan2(base[1, 0], base[0, 0])
        level_pose = matrix(*base[:3, 3], yaw)
        accumulated = np.concatenate(self.clouds)
        nearby = np.linalg.norm(accumulated[:, :2]-base[:2, 3], axis=1) < 6.
        terrain = classify(voxelize(accumulated[nearby], .04), self.terrain)
        obstacles = terrain.obstacle_points()
        obstacles = obstacles[(obstacles[:, 2] < base[2, 3]+.65) &
                              (obstacles[:, 2] > base[2, 3]-.7)]
        points = transform(obstacles, inverse(level_pose))
        ranges = np.hypot(points[:, 0], points[:, 1])
        angle = np.arctan2(points[:, 1], points[:, 0])
        n = 720
        ids = np.floor((angle+np.pi)*n/(2*np.pi)).astype(int) % n
        values = np.full(n, np.inf)
        valid = (ranges>=0.2)&(ranges<=20.)
        np.minimum.at(values, ids[valid], ranges[valid])
        out = LaserScan()
        out.header = msg.header
        out.header.frame_id = 'base_footprint'
        out.angle_min, out.angle_increment = -np.pi, 2*np.pi/n
        out.angle_max = out.angle_min+(n-1)*out.angle_increment
        out.range_min, out.range_max, out.scan_time = 0.2, 20., 0.2
        out.ranges = values.astype('float32').tolist()
        self.scan_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Bridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except RuntimeError:
        # Humble can invalidate a subscription while its context is being
        # shut down. Preserve all runtime errors during normal operation.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
