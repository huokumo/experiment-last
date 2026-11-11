#!/usr/bin/env python3
"""逐关卡统计点数：回答"这一块障碍，是哪一路贡献的"。

    ros2 run experiment9_fusion cloud_check.py
    ros2 run experiment9_fusion cloud_check.py --wall

表格里某一路为 0 是**设计如此**，不是故障：

* K2 立柱在 5 m 外，深度图 160×120 下不足 2 像素 → 深度列本来就该接近 0；
* K4 横梁在 1.6 m 高，高于相机光轴 → 深度列必然是 0；
* K1 托盘只有 0.35 m 高，雷达在 7.52 m 内下视够不着 → 雷达列接近 0。

该为 0 的有数、该有数的为 0，才是要查的问题。
"""

import argparse
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

from experiment9_core import cloud_io
from experiment9_core.scene9 import CHECK_REGIONS, REFERENCE_WALL

TOPICS = [
    ('/r1/fusion/points_lidar', '雷达'),
    ('/r1/fusion/points_depth', '深度'),
    ('/r1/fusion/points', '合并'),
]


class Check(Node):
    def __init__(self):
        super().__init__('experiment9_cloud_check')
        self.clouds = {}
        for topic, _label in TOPICS:
            self.create_subscription(PointCloud2, topic, self._make_cb(topic), qos_profile_sensor_data)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def _make_cb(self, topic):
        def callback(msg):
            self.clouds[topic] = msg
        return callback

    def to_map(self, msg):
        points = cloud_io.read_xyz(msg)
        if points.shape[0] == 0:
            return points
        try:
            transform = self.buffer.lookup_transform('map', msg.header.frame_id or 'base_link',
                                                     Time())
        except Exception:  # noqa: BLE001 - TF not ready yet
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = _quaternion_matrix((rotation.x, rotation.y, rotation.z, rotation.w))
        moved = points.astype(np.float64) @ matrix[:3, :3].T + np.array([translation.x, translation.y, translation.z])
        return moved.astype(np.float32)


def _quaternion_matrix(quaternion):
    """Minimal quaternion -> 4x4 rotation, avoids a hard dependency on tf_transformations."""
    x, y, z, w = quaternion
    norm = np.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(4)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    matrix = np.eye(4)
    matrix[0, 0] = 1 - 2 * (y * y + z * z)
    matrix[0, 1] = 2 * (x * y - z * w)
    matrix[0, 2] = 2 * (x * z + y * w)
    matrix[1, 0] = 2 * (x * y + z * w)
    matrix[1, 1] = 1 - 2 * (x * x + z * z)
    matrix[1, 2] = 2 * (y * z - x * w)
    matrix[2, 0] = 2 * (x * z - y * w)
    matrix[2, 1] = 2 * (y * z + x * w)
    matrix[2, 2] = 1 - 2 * (x * x + y * y)
    return matrix


def count_in_region(points, center, half):
    if points is None or points.shape[0] == 0:
        return 0
    mask = np.all(np.abs(points - np.asarray(center, dtype=np.float32)) <= np.asarray(half, dtype=np.float32), axis=1)
    return int(np.count_nonzero(mask))


def wall_distance(points):
    """Median distance of the near-wall points to the blue wall plane x = 2.9."""
    if points is None or points.shape[0] == 0:
        return None
    value = REFERENCE_WALL['value']
    mask = ((np.abs(points[:, 1]) < 1.3) & (points[:, 0] > value - 0.8) & (points[:, 0] < value + 1.2)
            & (points[:, 2] > 0.3) & (points[:, 2] < 1.8))
    if not np.any(mask):
        return None
    return float(np.median(np.abs(points[mask, 0] - value)))


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--timeout', type=float, default=12.0)
    parser.add_argument('--wall', action='store_true', help='also report the distance to the blue wall')
    args = parser.parse_args()

    rclpy.init()
    node = Check()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if all(topic in node.clouds for topic, _ in TOPICS):
                break
        missing = [topic for topic, _ in TOPICS if topic not in node.clouds]
        if missing:
            print('没有收到: ' + ', '.join(missing), file=sys.stderr)
            print('融合节点在跑吗？mode 是 student 时，三个 TODO 没写完是不会发布点云的。', file=sys.stderr)
            return 1

        mapped = {topic: node.to_map(node.clouds[topic]) for topic, _ in TOPICS}
        robot = None
        try:
            transform = node.buffer.lookup_transform('map', 'base_link', Time())
            robot = (transform.transform.translation.x, transform.transform.translation.y)
        except Exception:  # noqa: BLE001
            pass

        print(f'机器人位置: ({robot[0]:+.2f}, {robot[1]:+.2f})' if robot else '机器人位置: 未知')
        header = f'{"关卡":<14}{"雷达":>8}{"深度":>8}{"合并":>8}   判定'
        print(header)
        print('-' * 52)
        for name, (center, half) in CHECK_REGIONS.items():
            counts = [count_in_region(mapped[topic], center, half) for topic, _ in TOPICS]
            merged = counts[2]
            verdict = 'OK' if merged > 0 else '空 —— 该关卡的障碍没进融合'
            print(f'{name:<14}{counts[0]:>8}{counts[1]:>8}{counts[2]:>8}   {verdict}')

        if args.wall:
            print()
            print(f'参考面: {REFERENCE_WALL["name"]} 平面 x = {REFERENCE_WALL["value"]}')
            for topic, label in TOPICS:
                distance = wall_distance(mapped[topic])
                text = 'n/a' if distance is None else f'{distance * 100:+.1f} cm'
                print(f'  {label:<4} 墙面距离偏差 {text}')
            print('  两路偏差应小于 5 cm；一路整体偏大说明外参或轴向错了。')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
