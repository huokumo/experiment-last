#!/usr/bin/env python3
"""Check that the path Nav2 plans is a path the simulator guard will accept.

Two different models of "is this position free" meet in this lab:

* **Nav2** treats the robot as a **circle** of ``robot_radius`` and plans on a
  cost map where the lethal disc is exactly that radius, plus an inflation ring
  that only adds *cost* (it does not block).
* **the Gazebo 2D motion guard** (``planar_motion_system``) expands every
  obstacle box by ``footprint_radius`` **on each axis** - a Chebyshev test, not
  a Euclidean one.

The two agree on a flat wall and disagree at a corner: a circle sitting at
Euclidean distance ``R`` from a corner is only ``R/sqrt(2)`` away in the
worst axis. With ``robot_radius == footprint_radius == 0.45`` that is a 0.19 m
blind spot, and NavFn happily routes straight through it. The robot then stops
dead in open floor: the guard refuses every translation while Nav2 keeps
re-planning the same path, so ``number_of_recoveries`` climbs and
``distance_remaining`` never moves.

This tool plans with the Nav2 model and then asks the guard model about every
step of the result. It fails if any step is inside the guard's forbidden band.

Usage:
    python3 tools/check_route.py
    python3 tools/check_route.py --clearance 0.10      # require extra margin
"""

import argparse
import heapq
import math
import pathlib
import sys

import yaml

REPO = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = REPO / 'src' / 'experiment9_fusion'
sys.path.insert(0, str(PACKAGE))

from experiment9_core import scene9  # noqa: E402

RESOLUTION = 0.05
NEUTRAL_COST = 50.0     # nav2_navfn_planner defaults
COST_FACTOR = 3.0
INSCRIBED = 253.0


def load_params():
    """Read the local cost map block - the one the controller actually checks."""
    raw = yaml.safe_load((PACKAGE / 'config/nav2_params.yaml').read_text(encoding='utf-8'))
    block = raw['local_costmap']['local_costmap']['ros__parameters']
    inflation = block['inflation_layer']
    return float(block['robot_radius']), float(inflation['inflation_radius']), float(inflation['cost_scaling_factor'])


def guard_radius():
    """Footprint radius the world generator hands to the motion guard."""
    import re
    source = (PACKAGE / 'experiment9_core/scene9.py').read_text(encoding='utf-8')
    match = re.search(r"'footprint_radius'\s*:\s*([0-9.]+)", source)
    if not match:
        raise SystemExit('scene9.py 里找不到 footprint_radius')
    return float(match.group(1))


class Field:
    """Planar distance to every box, both metrics, sampled once on a grid."""

    def __init__(self, boxes):
        self.origin_x, self.origin_y = scene9.MAP_ORIGIN
        self.width = int(round(scene9.MAP_SIZE[0] / RESOLUTION))
        self.height = int(round(scene9.MAP_SIZE[1] / RESOLUTION))
        self.boxes = boxes
        self.euclid = [[math.inf] * self.width for _ in range(self.height)]
        self.chebyshev = [[math.inf] * self.width for _ in range(self.height)]
        for row in range(self.height):
            y = self.origin_y + row * RESOLUTION
            for column in range(self.width):
                x = self.origin_x + column * RESOLUTION
                best_e = best_c = math.inf
                for _name, center, size in boxes:
                    dx = abs(x - center[0]) - size[0] / 2.0
                    dy = abs(y - center[1]) - size[1] / 2.0
                    best_e = min(best_e, math.hypot(max(dx, 0.0), max(dy, 0.0)))
                    best_c = min(best_c, max(dx, dy))
                self.euclid[row][column] = best_e
                self.chebyshev[row][column] = best_c

    def cell(self, point):
        return (int(round((point[0] - self.origin_x) / RESOLUTION)),
                int(round((point[1] - self.origin_y) / RESOLUTION)))

    def point(self, column, row):
        return (self.origin_x + column * RESOLUTION, self.origin_y + row * RESOLUTION)


def nav2_cost(distance, radius, inflation_radius, scaling):
    """What the cost map would put in that cell (0..253)."""
    if distance <= radius:
        return INSCRIBED
    if distance > inflation_radius:
        return 0.0
    return INSCRIBED * math.exp(-scaling * (distance - radius))


def plan(field, radius, inflation_radius, scaling):
    start = field.cell(scene9.START_XY)
    goal = field.cell(scene9.GOAL_XY)
    cost = {}
    for row in range(field.height):
        for column in range(field.width):
            if 0 <= column < field.width:
                c = nav2_cost(field.euclid[row][column], radius, inflation_radius, scaling)
                if c < INSCRIBED - 0.5:      # lethal cells are not traversable
                    cost[(column, row)] = c
    if start not in cost or goal not in cost:
        return None

    best = {start: 0.0}
    previous = {}
    queue = [(0.0, start)]
    while queue:
        spent, current = heapq.heappop(queue)
        if spent > best[current]:
            continue
        if current == goal:
            break
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                neighbour = (current[0] + dx, current[1] + dy)
                if neighbour not in cost:
                    continue
                step = math.hypot(dx, dy) * RESOLUTION
                candidate = spent + step * (NEUTRAL_COST + COST_FACTOR * cost[neighbour])
                if candidate < best.get(neighbour, math.inf):
                    best[neighbour] = candidate
                    previous[neighbour] = current
                    heapq.heappush(queue, (candidate, neighbour))
    if goal not in previous:
        return None
    route = [goal]
    while route[-1] != start:
        route.append(previous[route[-1]])
    return [field.point(*cell) for cell in reversed(route)]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--clearance', type=float, default=0.05,
                        help='extra margin required on top of the guard radius (default 0.05 m)')
    args = parser.parse_args()

    robot_radius, inflation_radius, scaling = load_params()
    footprint = guard_radius()

    static = [(name, center, size) for name, center, size in scene9.STATIC_BOXES if name != 'floor']
    blocking = [(name, center, size) for name, _label, center, size, blocks, _note in scene9.FUSION_OBSTACLES
                if blocks]
    field = Field(static + blocking)

    print(f'规划半径 robot_radius={robot_radius:.2f} m  膨胀圈={inflation_radius:.2f} m '
          f'(cost_scaling_factor={scaling:.1f})')
    print(f'仿真保护判据 footprint_radius={footprint:.2f} m（方形膨胀，逐轴判定）')

    route = plan(field, robot_radius, inflation_radius, scaling)
    if route is None:
        print('\n[FAIL] 这个参数组合下起点到终点无解。')
        return 1

    length = sum(math.hypot(route[index + 1][0] - route[index][0], route[index + 1][1] - route[index][1])
                 for index in range(len(route) - 1))
    closest_e = math.inf
    closest_c = math.inf
    tight = route[0]
    for x, y in route:
        column, row = field.cell((x, y))
        e = field.euclid[row][column]
        c = field.chebyshev[row][column]
        if c < closest_c:
            closest_c = c
            tight = (x, y)
        closest_e = min(closest_e, e)

    print(f'\n路径长度 {length:.2f} m（{len(route)} 个路点）')
    print(f'离障碍最近（欧氏，Nav2 口径）  {closest_e:+.3f} m  需 > {robot_radius:.2f}')
    print(f'离障碍最近（切比雪夫，保护判据口径）{closest_c:+.3f} m  需 > {footprint:.2f} + {args.clearance:.2f}')
    print(f'最紧的位置 ({tight[0]:+.2f}, {tight[1]:+.2f})')

    problems = []
    if closest_e < robot_radius:
        problems.append('路径有一段落进了 Nav2 自己的致命区')
    if closest_c < footprint + args.clearance:
        problems.append('路径在拐角处被 2D 保护判据判撞，车会原地打转')
    if inflation_radius - robot_radius < 0.30:
        problems.append('膨胀圈只比车身大 %.2f m，规划器会贴着障碍走' % (inflation_radius - robot_radius))

    print()
    if problems:
        for text in problems:
            print(f'[FAIL] {text}')
        return 1
    print('[PASS] 规划出来的路径，保护判据每一步都接受。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
