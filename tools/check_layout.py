#!/usr/bin/env python3
"""Check the checkpoint layout before you trust the lab.

This answers one question: **if the robot could only see the static map, would
it actually drive into the checkpoint?** If not, the checkpoint is decoration
and the experiment proves nothing.

It builds two occupancy grids - map only, and map plus checkpoints - runs A* on
both, and reports:

* naive path: what the planner does with no sensor input at all;
* aware path: what it does once the fusion cloud is in the cost map;
* per checkpoint: closest approach of the naive path, and whether that means a
  collision.

Usage:
    python3 tools/check_layout.py
    python3 tools/check_layout.py --verbose
"""

import argparse
import heapq
import math
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src' / 'experiment9_fusion'))

from experiment9_core import scene9  # noqa: E402

ROBOT_RADIUS = 0.45
RESOLUTION = 0.05


class Grid:
    def __init__(self, boxes):
        self.origin_x, self.origin_y = scene9.MAP_ORIGIN
        self.width = int(round(scene9.MAP_SIZE[0] / RESOLUTION))
        self.height = int(round(scene9.MAP_SIZE[1] / RESOLUTION))
        self.blocked = [[False] * self.width for _ in range(self.height)]
        for _name, center, size in boxes:
            self._stamp(center, size)
        self._confine()

    def _cells(self, x, y):
        return (int((x - self.origin_x) / RESOLUTION), int((y - self.origin_y) / RESOLUTION))

    def _stamp(self, center, size):
        x0, y0 = self._cells(center[0] - size[0] / 2 - ROBOT_RADIUS, center[1] - size[1] / 2 - ROBOT_RADIUS)
        x1, y1 = self._cells(center[0] + size[0] / 2 + ROBOT_RADIUS, center[1] + size[1] / 2 + ROBOT_RADIUS)
        for row in range(max(0, y0), min(self.height, y1 + 1)):
            for column in range(max(0, x0), min(self.width, x1 + 1)):
                self.blocked[row][column] = True

    def _confine(self):
        """Keep the robot inside the room walls, footprint included."""
        x0, y0 = self._cells(-5.425 + ROBOT_RADIUS, -4.425 + ROBOT_RADIUS)
        x1, y1 = self._cells(5.425 - ROBOT_RADIUS, 4.425 - ROBOT_RADIUS)
        for row in range(self.height):
            for column in range(self.width):
                if not (x0 <= column <= x1 and y0 <= row <= y1):
                    self.blocked[row][column] = True

    def free(self, column, row):
        return 0 <= column < self.width and 0 <= row < self.height and not self.blocked[row][column]

    def path(self, start, goal):
        def cell(point):
            return self._cells(*point)

        start_cell, goal_cell = cell(start), cell(goal)
        if not self.free(*start_cell) or not self.free(*goal_cell):
            return None
        queue = [(0.0, start_cell)]
        cost = {start_cell: 0.0}
        previous = {}
        while queue:
            _estimate, current = heapq.heappop(queue)
            if current == goal_cell:
                break
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    neighbour = (current[0] + dx, current[1] + dy)
                    if not self.free(*neighbour):
                        continue
                    candidate = cost[current] + math.hypot(dx, dy)
                    if candidate < cost.get(neighbour, 1e18):
                        cost[neighbour] = candidate
                        previous[neighbour] = current
                        heapq.heappush(queue, (candidate + math.hypot(neighbour[0] - goal_cell[0],
                                                                     neighbour[1] - goal_cell[1]), neighbour))
        if goal_cell not in previous:
            return None
        route = [goal_cell]
        while route[-1] != start_cell:
            route.append(previous[route[-1]])
        return [(self.origin_x + c * RESOLUTION, self.origin_y + r * RESOLUTION) for c, r in reversed(route)]


def distance_to_box(point, center, size):
    dx = max(abs(point[0] - center[0]) - size[0] / 2, 0.0)
    dy = max(abs(point[1] - center[1]) - size[1] / 2, 0.0)
    return math.hypot(dx, dy)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--verbose', action='store_true')
    args = parser.parse_args()

    static = [(name, center, size) for name, center, size in scene9.STATIC_BOXES if name != 'floor']
    blocking = [(name, center, size) for name, _label, center, size, blocks, _note in scene9.FUSION_OBSTACLES
                if blocks]

    naive = Grid(static).path(scene9.START_XY, scene9.GOAL_XY)
    aware_grid = Grid(static + blocking)
    aware = aware_grid.path(scene9.START_XY, scene9.GOAL_XY)

    print(f'起点 {scene9.START_XY}  终点 {scene9.GOAL_XY}  机器人半径 {ROBOT_RADIUS} m')
    print(f'纯静态地图路径: {"无解" if naive is None else f"{len(naive) * RESOLUTION:.2f} m"}'
          f'   含关卡路径: {"无解" if aware is None else f"{len(aware) * RESOLUTION:.2f} m"}')
    if naive is None:
        print('错误: 只用静态地图都走不到终点，起点或终点落在障碍里。')
        return 1
    if aware is None:
        print('错误: 加上关卡后无路可走，检查关卡是否把通道堵死了。')
        return 1

    problems = 0
    print()
    print(f'{"关卡":<18}{"类型":<8}{"最近距离":>10}  结论')
    for name, label, center, size, blocks, _note in scene9.FUSION_OBSTACLES:
        closest = min(distance_to_box(point, center, size) for point in naive)
        kind = '阻断型' if blocks else '上方/诊断'
        if blocks:
            functional = closest < ROBOT_RADIUS
            verdict = '在必经路径上，未感知会撞' if functional else '不在最短路径上：仅诊断，不影响通行'
            if not functional:
                problems += 1
        else:
            under = min(math.hypot(point[0] - center[0], point[1] - center[1]) for point in aware)
            verdict = f'车从下方通过(水平距离 {under:.2f} m)，检验高度过滤' if under < 0.6 else '不在路径上'
        print(f'{label:<18}{kind:<8}{closest:>9.3f}m  {verdict}')

    if args.verbose:
        print()
        print('纯静态地图路径采样:')
        step = max(1, len(naive) // 10)
        for point in naive[::step]:
            print(f'  ({point[0]:+.2f}, {point[1]:+.2f})')

    print()
    if problems:
        print(f'提示: {problems} 个阻断型关卡不在最短路径上，它们不会影响这次导航。'
              '要么接受它们只作诊断，要么调整坐标。')
    else:
        print('全部阻断型关卡都在最短路径上，未感知时必然被 2D 保护判据拦住。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
