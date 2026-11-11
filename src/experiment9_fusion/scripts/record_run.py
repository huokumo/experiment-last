#!/usr/bin/env python3
"""录制一次运行：轨迹 CSV + 结束时的指标摘要。

    ros2 run experiment9_fusion record_run.py --out runs/k1_bad_zmin.csv

摘要里的四项就是报告要填的数：

* 里程：轨迹长度，绕远了多少一眼可见；
* 碰撞帧：``/r1/collision_blocked`` 为 true 的采样数，正常应为 0；
* 最小安全距离：轨迹上离障碍物表面最近的距离，小于 0 就是蹭上了；
* 终点距离：收尾时离目标多远。

按 Ctrl-C 结束并落盘。不依赖任何图像，数字可以复算。
"""

import argparse
import math
import pathlib
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool

from experiment9_core.scene9 import FUSION_OBSTACLES, GOAL_XY


class Recorder(Node):
    def __init__(self):
        super().__init__('experiment9_record_run')
        self.samples = []
        self.blocked = False
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(Bool, '/r1/collision_blocked', self._on_blocked, 10)

    def _on_blocked(self, msg):
        self.blocked = bool(msg.data)

    def _on_odom(self, msg):
        pose = msg.pose.pose
        yaw = _yaw(pose.orientation)
        self.samples.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                             pose.position.x, pose.position.y, yaw, self.blocked))


def _yaw(quaternion):
    x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def clearance(x, y):
    """Distance from (x, y) to the nearest blocking obstacle surface, metres."""
    best = float('inf')
    for _name, _label, center, size, blocks, _note in FUSION_OBSTACLES:
        if not blocks:
            continue
        dx = max(abs(x - center[0]) - size[0] / 2, 0.0)
        dy = max(abs(y - center[1]) - size[1] / 2, 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


def summary(samples):
    if len(samples) < 2:
        return None
    duration = samples[-1][0] - samples[0][0]
    length = sum(math.hypot(b[1] - a[1], b[2] - a[2]) for a, b in zip(samples, samples[1:]))
    blocked_frames = sum(1 for sample in samples if sample[4])
    worst = min(clearance(sample[1], sample[2]) for sample in samples)
    final = samples[-1]
    goal_distance = math.hypot(final[1] - GOAL_XY[0], final[2] - GOAL_XY[1])
    return {
        'duration': duration,
        'length': length,
        'blocked_frames': blocked_frames,
        'min_clearance': worst,
        'goal_distance': goal_distance,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', default=None, help='CSV path (default runs/run_<timestamp>.csv)')
    args = parser.parse_args()

    out = pathlib.Path(args.out) if args.out else pathlib.Path(
        'runs') / time.strftime('run_%Y%m%d_%H%M%S.csv')
    out.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Recorder()
    print(f'录制中 -> {out}   (Ctrl-C 结束)')
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        with out.open('w', encoding='utf-8') as handle:
            handle.write('t_sec,x,y,yaw_rad,blocked\n')
            for sample in node.samples:
                handle.write(f'{sample[0]:.3f},{sample[1]:.3f},{sample[2]:.3f},{sample[3]:.3f},{int(sample[4])}\n')
        stats = summary(node.samples)
        print(f'\n已写入 {out}（{len(node.samples)} 个采样）')
        if stats:
            print(f'  里程          {stats["length"]:.2f} m')
            print(f'  用时          {stats["duration"]:.1f} s')
            print(f'  碰撞帧        {stats["blocked_frames"]}')
            print(f'  最小安全距离  {stats["min_clearance"] * 100:+.1f} cm')
            print(f'  终点距离      {stats["goal_distance"]:.2f} m')
        else:
            print('  采样太少，检查 /odom 是否在发布。')
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
