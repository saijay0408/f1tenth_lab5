#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


def scan_to_points(msg, downsample=4, max_range_cutoff=10.0):
    ranges = np.array(msg.ranges, dtype=np.float64)
    angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment

    valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < min(msg.range_max, max_range_cutoff))
    ranges = ranges[valid]
    angles = angles[valid]

    xs = ranges * np.cos(angles)
    ys = ranges * np.sin(angles)
    return np.stack([xs, ys], axis=1)[::downsample]


def nearest_neighbors(src, dst):
    diff = src[:, None, :] - dst[None, :, :]
    dist2 = np.sum(diff ** 2, axis=-1)
    indices = np.argmin(dist2, axis=1)
    dists = np.sqrt(dist2[np.arange(len(src)), indices])
    return dists, indices


def compute_rigid_transform(src, dst):
    src_mean = src.mean(axis=0)
    dst_mean = dst.mean(axis=0)
    src_c = src - src_mean
    dst_c = dst - dst_mean

    H = src_c.T @ dst_c
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = dst_mean - R @ src_mean
    return R, t


def icp(source_pts, target_pts, max_iterations=20, tolerance=1e-5, max_corr_dist=0.5,
        R_init=None, t_init=None):
    R_total = np.eye(2) if R_init is None else R_init.copy()
    t_total = np.zeros(2) if t_init is None else t_init.copy()
    src = (R_total @ source_pts.T).T + t_total
    prev_error = None

    for _ in range(max_iterations):
        if len(src) == 0 or len(target_pts) == 0:
            break

        dists, indices = nearest_neighbors(src, target_pts)
        mask = dists < max_corr_dist
        if mask.sum() < 10:
            break

        R, t = compute_rigid_transform(src[mask], target_pts[indices[mask]])

        src = (R @ src.T).T + t
        R_total = R @ R_total
        t_total = R @ t_total + t

        mean_error = float(np.mean(dists[mask]))
        if prev_error is not None and abs(prev_error - mean_error) < tolerance:
            break
        prev_error = mean_error

    dtheta = np.arctan2(R_total[1, 0], R_total[0, 0])
    return t_total[0], t_total[1], dtheta, prev_error


class ScanMatcher(Node):
    def __init__(self):
        super().__init__('scan_matcher')

        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'scan_match_odom', 10)

        self.prev_points = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_R = np.eye(2)
        self.last_t = np.zeros(2)

        self.min_dt = 0.1
        self.last_stamp = None

    def scan_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_stamp is not None and (stamp - self.last_stamp) < self.min_dt:
            return
        self.last_stamp = stamp

        curr_points = scan_to_points(msg, downsample=2)

        if self.prev_points is None:
            self.prev_points = curr_points
            self.publish_odom(msg)
            return

        dx, dy, dtheta, error = icp(
            curr_points, self.prev_points,
            max_corr_dist=1.0,
            R_init=self.last_R, t_init=self.last_t)

        self.last_R = np.array([[np.cos(dtheta), -np.sin(dtheta)],
                                 [np.sin(dtheta), np.cos(dtheta)]])
        self.last_t = np.array([dx, dy])

        self.x += dx * np.cos(self.theta) - dy * np.sin(self.theta)
        self.y += dx * np.sin(self.theta) + dy * np.cos(self.theta)
        self.theta += dtheta

        self.prev_points = curr_points
        self.publish_odom(msg, error)

    def publish_odom(self, scan_msg, error=None):
        odom = Odometry()
        odom.header.stamp = scan_msg.header.stamp
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'scan_match_base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = np.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = np.cos(self.theta / 2.0)

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMatcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
