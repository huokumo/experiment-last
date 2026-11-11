#!/usr/bin/env python3
"""打印代价地图的字符画 —— 不开 RViz 也能看出"障碍进没进去"。

    ros2 run experiment9_fusion costmap_dump.py
    ros2 run experiment9_fusion costmap_dump.py --width 70 --save /tmp/run_k1.txt

图例::

    █  静态地图里的障碍（预置，融合不负责）
    @  只有融合点云才标出来的障碍  <- 这就是你要的结果
    ·  膨胀区
    A  机器人   G  目标点

``@`` 是关键：它们不在 /map 里，只可能来自 /r1/fusion/points。
K1 托盘有没有出现，看左下角那两个 ``@``。
"""

import argparse
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

from experiment9_core.scene9 import GOAL_XY


class Dump(Node):
    def __init__(self):
        super().__init__('experiment9_costmap_dump')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static = None
        self.costmap = None
        self.create_subscription(OccupancyGrid, '/map', self._on_static, latched)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', self._on_costmap, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def _on_static(self, msg):
        self.static = msg

    def _on_costmap(self, msg):
        self.costmap = msg
        self.costmap_stamp = time.monotonic()

    def robot_pose(self):
        try:
            transform = self.buffer.lookup_transform('map', 'base_link', Time())
        except Exception:  # noqa: BLE001 - any lookup failure means "unknown yet"
            return None
        return (transform.transform.translation.x, transform.transform.translation.y)


def cell_of(grid, x, y):
    column = int((x - grid.info.origin.position.x) / grid.info.resolution)
    row = int((y - grid.info.origin.position.y) / grid.info.resolution)
    if not (0 <= column < grid.info.width and 0 <= row < grid.info.height):
        return None
    return column, row


def value_at(grid, column, row):
    index = row * grid.info.width + column
    if not (0 <= index < len(grid.data)):
        return None
    return grid.data[index]


def render(node, width, goal):
    costmap = node.costmap
    static = node.static
    height_cells, width_cells = costmap.info.height, costmap.info.width
    step_x = max(1, width_cells // width)
    # terminal cells are about twice as tall as wide
    step_y = max(1, int(round(step_x * 2)))
    robot = node.robot_pose()

    lines = []
    resolution = costmap.info.resolution
    origin = costmap.info.origin.position
    for row in range(height_cells - 1, -1, -step_y):
        cells = []
        for column in range(0, width_cells, step_x):
            block = []
            for dy in range(0, step_y, max(1, step_y // 2)):
                for dx in range(0, step_x, max(1, step_x // 2)):
                    block.append((column + dx, row - dy))
            cost_values = [value_at(costmap, c, r) for c, r in block if value_at(costmap, c, r) is not None]
            static_values = [value_at(static, c, r) for c, r in block
                             if static is not None and value_at(static, c, r) is not None]
            cost = max(cost_values) if cost_values else 0
            static_cost = max(static_values) if static_values else -1
            if static_cost >= 50:
                cells.append('█')
            elif cost >= 99:
                cells.append('@')
            elif cost > 0:
                cells.append('·')
            else:
                cells.append(' ')
        lines.append(list(cells))

    def put(x, y, glyph):
        column = int((x - origin.x) / (resolution * step_x))
        row = (height_cells - 1 - int((y - origin.y) / resolution)) // step_y
        row = len(lines) - 1 - row
        if 0 <= row < len(lines) and 0 <= column < len(lines[row]):
            lines[row][column] = glyph

    if goal is not None:
        put(goal[0], goal[1], 'G')
    if robot is not None:
        put(robot[0], robot[1], 'A')

    header = (f'map {costmap.info.width * resolution:.1f}x{costmap.info.height * resolution:.1f} m '
              f'| 机器人 ({robot[0]:+.2f}, {robot[1]:+.2f})' if robot else 'map | 机器人未知')
    header += f' | 目标 ({goal[0]:+.2f}, {goal[1]:+.2f})' if goal else ''
    header += f' | 代价地图 {(time.monotonic() - node.costmap_stamp):.1f} s 前'
    print(header)
    for line in lines:
        print('  |' + ''.join(line) + '|')
    print('  图例: █ 静态障碍   @ 融合点云新标出的障碍   · 膨胀区   A 机器人   G 目标')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--width', type=int, default=60, help='character columns')
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument('--save', help='also write the character grid to this file')
    parser.add_argument('--goal', nargs=2, type=float, default=list(GOAL_XY), help='goal marker position')
    args = parser.parse_args()

    rclpy.init()
    node = Dump()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.costmap is not None and node.static is not None:
                break
        if node.costmap is None:
            print('没有收到 /global_costmap/costmap。先确认 nav_demo.launch.py 已启动，'
                  '或 lifecycle_manager 还没把 controller/planner 激活。', file=sys.stderr)
            return 1
        text = render(node, args.width, tuple(args.goal))
        if args.save:
            with open(args.save, 'w', encoding='utf-8') as handle:
                handle.write(text + '\n')
            print(f'已写入 {args.save}')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
