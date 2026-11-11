#!/usr/bin/env python3
"""One command that answers: is the chain up?

Five checks, in the order they fail in practice. Run it after every restart,
before blaming your code:

    ros2 run experiment9_fusion check_ready.py

Exit code 0 means every check passed. ``--wait`` keeps retrying for the given
number of seconds, which is what you want right after ``launch``.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import Image, PointCloud2
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

# topic, type, minimum acceptable rate, what it is for
TOPICS = [
    ('/livox/points', PointCloud2, 3.0, '仿真 Mid-360 点云'),
    ('/r1/depth/image', Image, 1.0, '深度图'),
    ('/r1/fusion/points', PointCloud2, 1.5, '融合后的障碍点云'),
    ('/odom', Odometry, 20.0, '仿真里程计'),
]

FRAME_CHAIN = [('map', 'odom'), ('odom', 'base_link'), ('base_link', 'lidar_frame')]

# Per-sensor clouds, not part of pass/fail: they exist as soon as TODO 1 / TODO 2
# work, so their presence tells the student which stage the chain stops at.
DEBUG_TOPICS = ['/r1/fusion/points_lidar', '/r1/fusion/points_depth']


class Ready(Node):
    def __init__(self):
        super().__init__('experiment9_check_ready')
        self.counts = {name: 0 for name, *_ in TOPICS}
        for topic, kind, *_ in TOPICS:
            self.create_subscription(kind, topic, self._make_counter(topic), qos_profile_sensor_data)
        self.debug_counts = {topic: 0 for topic in DEBUG_TOPICS}
        for topic in DEBUG_TOPICS:
            self.create_subscription(PointCloud2, topic, self._make_debug_counter(topic),
                                     qos_profile_sensor_data)
        self.costmap = None
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._on_costmap,
                                 qos_profile_sensor_data)
        self.static_map = None
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def _make_counter(self, topic):
        def callback(_msg):
            self.counts[topic] += 1
        return callback

    def _make_debug_counter(self, topic):
        def callback(_msg):
            self.debug_counts[topic] += 1
        return callback

    def _on_costmap(self, msg):
        self.costmap = msg

    def _on_map(self, msg):
        self.static_map = msg


def check_rates(node, seconds):
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = max(1e-6, time.monotonic() - start)
    rows, ok = [], True
    for topic, _kind, minimum, label in TOPICS:
        rate = node.counts[topic] / elapsed
        good = rate >= minimum
        ok = ok and good
        rows.append((topic, f'{rate:5.1f} Hz', f'>= {minimum:.1f}', label, good))
    return rows, ok


def check_tf(node, timeout=3.0):
    rows, ok = [], True
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.buffer.can_transform('base_link', 'lidar_frame', Time()):
            break
    for parent, child in FRAME_CHAIN:
        good, detail = False, 'missing'
        try:
            transform = node.buffer.lookup_transform(parent, child, Time())
            translation = transform.transform.translation
            good = True
            detail = f'({translation.x:+.2f}, {translation.y:+.2f}, {translation.z:+.2f})'
        except (LookupException, ConnectivityException, ExtrapolationException) as error:
            detail = type(error).__name__
        ok = ok and good
        rows.append((f'{parent} -> {child}', detail, '', 'TF 链', good))
    return rows, ok


def check_costmap(node, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.costmap is not None and node.static_map is not None:
            break
    rows, ok = [], True
    if node.static_map is None:
        rows.append(('/map', 'no data', '', '静态地图', False))
        ok = False
    else:
        occupied = sum(1 for value in node.static_map.data if value >= 50)
        good = occupied > 100
        ok = ok and good
        rows.append(('/map', f'{occupied} cells', '> 100', '静态地图', good))
    if node.costmap is None:
        rows.append(('/global_costmap/costmap', 'no data', '', '代价地图', False))
        ok = False
    else:
        lethal = sum(1 for value in node.costmap.data if value >= 99)
        inflated = sum(1 for value in node.costmap.data if 0 < value < 99)
        good = lethal > 100
        ok = ok and good
        rows.append(('/global_costmap/costmap', f'{lethal} lethal / {inflated} inflated', '> 100 lethal',
                     '代价地图', good))
    return rows, ok


def check_action(node):
    services = [name for name, _types in node.get_service_names_and_types()]
    good = any(name.startswith('/navigate_to_pose/_action/send_goal') for name in services)
    return [('/navigate_to_pose', 'server ' + ('found' if good else 'absent'), 'required',
             'Nav2 动作服务', good)], good


def render(rows):
    width = max(len(row[0]) for row in rows)
    for topic, value, limit, label, good in rows:
        mark = 'OK  ' if good else 'FAIL'
        limit_text = f' ({limit})' if limit else ''
        print(f'  [{mark}] {topic:<{width}}  {value}{limit_text:<12}  {label}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seconds', type=float, default=6.0, help='rate measurement window')
    parser.add_argument('--wait', type=float, default=0.0, help='retry the whole check for this long')
    args = parser.parse_args()

    rclpy.init()
    node = Ready()
    deadline = time.monotonic() + max(1.0, args.wait)
    results = None
    try:
        while True:
            rate_rows, rate_ok = check_rates(node, args.seconds)
            tf_rows, tf_ok = check_tf(node)
            costmap_rows, costmap_ok = check_costmap(node, timeout=3.0)
            action_rows, action_ok = check_action(node)
            results = [
                ('1 传感器与融合输出频率', rate_rows, rate_ok),
                ('2 TF 链', tf_rows, tf_ok),
                ('3 地图与代价地图', costmap_rows, costmap_ok),
                ('4 导航动作服务', action_rows, action_ok),
            ]
            if all(ok for _title, _rows, ok in results) or time.monotonic() >= deadline:
                break
            node.counts = {name: 0 for name, *_ in TOPICS}
            print('... 未全部就绪，重试')
        passed = sum(1 for _t, _r, ok in results if ok)
        print()
        for title, rows, ok in results:
            print(f'{title}  [{"PASS" if ok else "FAIL"}]')
            render(rows)
            print()
        print(f'结果：{passed}/{len(results)} 项通过')
        if passed != len(results):
            print('提示：没有 /map 与 /navigate_to_pose 时说明还停在 experiment9.launch.py，'
                  '导航相关检查需要 nav_demo.launch.py。')
        if node.counts.get('/r1/fusion/points', 0) == 0:
            live = [topic for topic in DEBUG_TOPICS if node.debug_counts.get(topic, 0) > 0]
            if live:
                print('融合链路只走到一半：' + '、'.join(live) + ' 有数据，但 /r1/fusion/points 没有。')
                print('=> 三个 TODO 里 merge_clouds（TODO 3）还没完成。')
            else:
                print('融合节点没有任何点云输出：mode=student 时先确认 TODO 1（transform_cloud）能跑通。')
        return 0 if passed == len(results) else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
