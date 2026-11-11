#!/usr/bin/env python3
"""Offline ray casting: predict what each sensor sees along the actual route.

The teaching platform is the only place that can prove the render path works,
but the geometry can be checked anywhere. This script:

1. loads the real Mid-360 scan pattern and the real boxes,
2. walks the planned path (the one Nav2 will follow once it can see the
   checkpoints) in 0.4 m steps,
3. at every step casts 12k lidar rays and renders a full 160x120 depth image,
4. runs the three reference functions exactly as the node would,
5. reports, per checkpoint, the **best single-pose observation** of each sensor.

Best-per-pose rather than a sum, because the question is "at some moment during
the run, could this sensor have seen the checkpoint at all".

Usage:
    python3 tools/simulate_pipeline.py --pattern src/experiment7/vendor/livox/mid360.csv
    python3 tools/simulate_pipeline.py --pattern ... --pose -4.4 -3.4 0
"""

import argparse
import csv
import math
import pathlib
import sys

import numpy as np

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src' / 'experiment9_fusion'))
sys.path.insert(0, str(REPO / 'tools'))

from experiment9_core import reference, scene9  # noqa: E402

# From model_depth.sdf: 160x120, HFOV 60 deg, mounted 1.154 m up, pitched 19.48 deg down.
WIDTH, HEIGHT = 160, 120
HFOV = math.radians(60.0)
FX = WIDTH / (2 * math.tan(HFOV / 2))
FY = FX
CX, CY = WIDTH / 2.0, HEIGHT / 2.0
NEAR, FAR = 0.1, 10.0

CAMERA_TRANSLATION = np.array([0.185795, 0.0, 1.154059])
CAMERA_ROTATION = np.array([[0.0, -0.333487, 0.942755],
                            [-1.0, 0.0, 0.0],
                            [0.0, -0.942755, -0.333487]])
LIDAR_TRANSLATION = np.array([0.0, 0.0, 1.3])

EXPECTATIONS = {
    # name: (label, predicate(lidar_best, depth_best))
    'K1_pallet': ('深度为主', lambda l, d: d > 100 and l < 40),
    'K2_column': ('雷达为主', lambda l, d: l > 20),
    'K3_panel': ('两路都有', lambda l, d: l > 10 and d > 10),
    'K4_beam': ('只有雷达', lambda l, d: l > 0 and d == 0),
    'K5_rack': ('两路可见', lambda l, d: l > 0 and d > 0),
}


def all_boxes():
    items = [(name, center, size) for name, center, size in scene9.STATIC_BOXES]
    items += [(name, center, size) for name, _label, center, size, _b, _n in scene9.FUSION_OBSTACLES]
    return items


BOXES = all_boxes()


def raycast(origin, directions):
    """Vectorised slab test. Returns t per ray (inf when nothing is hit)."""
    best = np.full(directions.shape[0], np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        for _name, center, size in BOXES:
            half = np.asarray(size, dtype=np.float64) / 2.0
            low = np.asarray(center, dtype=np.float64) - half - origin
            high = np.asarray(center, dtype=np.float64) + half - origin
            inverse = 1.0 / directions
            t_low = low * inverse
            t_high = high * inverse
            t_near = np.minimum(t_low, t_high).max(axis=1)
            t_far = np.maximum(t_low, t_high).min(axis=1)
            hit = (t_far >= np.maximum(t_near, 0.0)) & (t_near > 1e-6)
            best = np.where(hit & (t_near < best), t_near, best)
    return best


def yaw_rotation(yaw):
    return np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                     [math.sin(yaw), math.cos(yaw), 0.0],
                     [0.0, 0.0, 1.0]])


class Sensors:
    def __init__(self, pattern_path, samples=12000, frequency=5.0):
        azimuth, zenith = [], []
        with open(pattern_path, newline='', encoding='utf-8') as handle:
            reader = csv.reader(handle)
            next(reader)
            for row in reader:
                if len(row) >= 3:
                    azimuth.append(float(row[1]))
                    zenith.append(float(row[2]))
        azimuth = np.radians(np.array(azimuth))
        zenith = np.radians(np.array(zenith))
        pattern = np.stack([np.sin(zenith) * np.cos(azimuth),
                            np.sin(zenith) * np.sin(azimuth),
                            np.cos(zenith)], axis=1)
        stride = max(1, round(200000 / (samples * frequency)))
        self.directions = pattern[(np.arange(samples) * stride) % pattern.shape[0]]

        columns, rows = np.meshgrid(np.arange(WIDTH), np.arange(HEIGHT))
        # z = 1 so the ray parameter equals optical-axis depth
        self.pixels = np.stack([(columns - CX) / FX,
                                (rows - CY) / FY,
                                np.ones_like(columns, dtype=np.float64)], axis=-1).reshape(-1, 3)

    def observe(self, pose):
        """Return (lidar points, depth image) in the sensor frames."""
        rotation = yaw_rotation(pose[2])
        lidar_origin = np.array([pose[0], pose[1], 0.0]) + LIDAR_TRANSLATION
        distance = raycast(lidar_origin, self.directions @ rotation.T)
        valid = (distance >= 0.2) & (distance <= 20.0)
        lidar = (self.directions[valid] * distance[valid][:, None]).astype(np.float32)

        camera_rotation = rotation @ CAMERA_ROTATION
        camera_origin = np.array([pose[0], pose[1], 0.0]) + rotation @ CAMERA_TRANSLATION
        depth = raycast(camera_origin, self.pixels @ camera_rotation.T)
        depth = np.where((depth >= NEAR) & (depth <= FAR), depth, np.nan)
        return lidar, depth.reshape(HEIGHT, WIDTH).astype(np.float32)


def world_points(points_base, pose):
    rotation = yaw_rotation(pose[2])
    return points_base.astype(np.float64) @ rotation.T + np.array([pose[0], pose[1], 0.0])


def route():
    from check_layout import Grid  # noqa: E402  (tools/check_layout.py)

    static = [(name, center, size) for name, center, size in scene9.STATIC_BOXES if name != 'floor']
    blocking = [(name, center, size) for name, _l, center, size, blocks, _n in scene9.FUSION_OBSTACLES if blocks]
    path = Grid(static + blocking).path(scene9.START_XY, scene9.GOAL_XY)
    if path is None:
        raise SystemExit('路径无解，先修 tools/check_layout.py 报的几何问题')

    spaced, last = [], None
    for x, y in path:
        if last is None or math.hypot(x - last[0], y - last[1]) >= 0.4:
            spaced.append((x, y))
            last = (x, y)
    spaced.append(path[-1])
    # heading of each sample = direction to the next one
    poses = []
    for index, (x, y) in enumerate(spaced):
        nxt = spaced[min(index + 1, len(spaced) - 1)]
        previous = spaced[max(index - 1, 0)]
        yaw = math.atan2(nxt[1] - previous[1], nxt[0] - previous[0]) if (nxt != previous) else 0.0
        poses.append((x, y, yaw))
    # The robot spawns facing START_YAW and only then turns onto the path, so the
    # first observation must use the spawn heading, not the path heading.
    poses[0] = (poses[0][0], poses[0][1], scene9.START_YAW)
    return poses


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--pattern', required=True, help='vendor/livox/mid360.csv from Experiment 7')
    parser.add_argument('--pose', nargs=3, type=float, default=None,
                        help='single pose x y yaw(rad); default walks the whole planned route')
    parser.add_argument('--samples', type=int, default=12000)
    parser.add_argument('--frequency', type=float, default=5.0)
    args = parser.parse_args()

    sensors = Sensors(args.pattern, args.samples, args.frequency)
    poses = [tuple(args.pose)] if args.pose else route()
    print(f'{len(poses)} 个位姿' + ('' if args.pose else f'（沿规划路径，起点 {scene9.START_XY} → 终点 {scene9.GOAL_XY}）'))

    best_lidar = {name: 0 for name in scene9.CHECK_REGIONS}
    best_depth = {name: 0 for name in scene9.CHECK_REGIONS}
    for index, pose in enumerate(poses):
        lidar, depth = sensors.observe(pose)
        lidar_base = reference.transform_cloud(lidar, (np.eye(3), LIDAR_TRANSLATION))
        depth_base = reference.depth_to_points(depth, {'fx': FX, 'fy': FY, 'cx': CX, 'cy': CY},
                                               (CAMERA_ROTATION, CAMERA_TRANSLATION))
        lidar_world = world_points(lidar_base, pose)
        depth_world = world_points(depth_base, pose)
        for name, (center, half) in scene9.CHECK_REGIONS.items():
            center = np.asarray(center, dtype=np.float64)
            half = np.asarray(half, dtype=np.float64)
            best_lidar[name] = max(best_lidar[name], int(np.count_nonzero(
                np.all(np.abs(lidar_world - center) <= half, axis=1))))
            best_depth[name] = max(best_depth[name], int(np.count_nonzero(
                np.all(np.abs(depth_world - center) <= half, axis=1))))
        if (index + 1) % 10 == 0:
            print(f'  ... {index + 1}/{len(poses)}')

    print()
    print(f'{"关卡":<16}{"雷达峰值":>10}{"深度峰值":>10}   设计意图')
    failures = 0
    for name, (label, check) in EXPECTATIONS.items():
        lidar_best, depth_best = best_lidar[name], best_depth[name]
        ok = check(lidar_best, depth_best)
        failures += 0 if ok else 1
        print(f'{name:<16}{lidar_best:>10}{depth_best:>10}   {label:<8}{"OK" if ok else "不符合预期"}')

    print()
    print('说明: 峰值 = 整个行程中某一位姿下落在该关卡区域内的最多点数。')
    print('     "雷达看不见近处低矮物"的物理原因是 1.3 m 安装高度 + 7.2 度下视：')
    print('     7.5 m 内，0.35 m 高的物体完全落在最低那条射线之下。')
    if failures:
        print(f'{failures} 个关卡的观测结构与设计不符，需调整几何再上平台。')
        return 1
    print('观测结构与设计一致，可以拿去对平台实测结果。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
