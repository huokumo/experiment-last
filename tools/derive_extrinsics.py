#!/usr/bin/env python3
"""Derive the sensor extrinsics used by the fusion node from the R1 model.

The teaching image ships ``models/unitree_r1/model_depth.sdf`` (Experiment 5).
The fusion node needs ``base_link -> depth_camera_optical_frame`` and
``base_link -> lidar_frame``. Both are written by hand in most handouts, which
is exactly where sign errors come from. This script reads the SDF and prints
the numbers that go into ``config/fusion.yaml``.

Usage:
    python3 tools/derive_extrinsics.py --model /root/exp9/src/experiment5/models/unitree_r1/model_depth.sdf
"""

import argparse
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rpy_to_matrix(roll, pitch, yaw):
    """SDF uses fixed-axis roll/pitch/yaw, applied as R = Rz(y) * Ry(p) * Rx(r)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def parse_pose(element):
    """Return (R, t) for a <pose> element, defaulting to identity."""
    if element is None or not (element.text or '').strip():
        return np.eye(3), np.zeros(3)
    values = [float(v) for v in element.text.split()]
    if len(values) != 6:
        raise ValueError(f'pose needs 6 numbers, got {element.text!r}')
    return rpy_to_matrix(*values[3:]), np.array(values[:3])


def build_graph(model):
    """Map frame name -> (parent name, R, t) for every link and frame."""
    graph = {}
    for link in model.findall('link'):
        name = link.get('name')
        pose = link.find('pose')
        parent = link.get('relative_to') or (pose.get('relative_to') if pose is not None else None) or '__model__'
        graph[name] = (parent, *parse_pose(pose))
    for frame in model.findall('frame'):
        name = frame.get('name')
        pose = frame.find('pose')
        parent = pose.get('relative_to') if pose is not None and pose.get('relative_to') else frame.get('attached_to')
        graph[name] = (parent or '__model__', *parse_pose(pose))
    return graph


def chain(graph, target, root='base_link'):
    """Compose root -> target as (R, t), walking parents until root."""
    if target not in graph:
        raise KeyError(f'{target} is not defined in the model')
    rotation = np.eye(3)
    translation = np.zeros(3)
    current = target
    seen = []
    while current != root:
        if current in seen:
            raise RuntimeError('cycle detected in the frame tree: ' + ' -> '.join(seen + [current]))
        seen.append(current)
        if current not in graph:
            raise KeyError(f'frame {current} (reached from {target}) is missing')
        parent, local_r, local_t = graph[current]
        # root <- ... <- parent <- current, prepend the local step
        translation = local_r @ translation + local_t
        rotation = local_r @ rotation
        current = parent
    return rotation, translation


def rotation_to_rpy(rotation):
    """Return SDF roll/pitch/yaw for a rotation matrix."""
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2, 0])))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:  # gimbal lock
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        yaw = 0.0
    return roll, pitch, yaw


def as_dict(rotation, translation):
    return {
        'translation': [round(float(v), 6) for v in translation],
        'rotation_matrix': [[round(float(v), 6) for v in row] for row in rotation],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', required=True, help='path to model_depth.sdf')
    parser.add_argument('--root', default='base_link')
    parser.add_argument('--lidar-z', type=float, default=1.3,
                        help='lidar mount height written by the scene generator')
    args = parser.parse_args()

    model = ET.parse(args.model).getroot().find('model')
    if model is None:
        print('error: no <model> element in the SDF', file=sys.stderr)
        return 2
    graph = build_graph(model)

    print(f'frame tree from {args.root}:')
    for name in ('torso_link', 'head_link', 'depth_camera_link', 'depth_camera_optical_frame'):
        if name in graph:
            print(f'  {name:32s} parent={graph[name][0]}')

    optical_r, optical_t = chain(graph, 'depth_camera_optical_frame', args.root)
    link_r, link_t = chain(graph, 'depth_camera_link', args.root)

    print('\n# --- config/fusion.yaml ---')
    print('extrinsics:')
    print('  # base_link (ground level, +x forward) -> lidar_frame')
    print('  lidar:')
    print(f'    translation: [0.0, 0.0, {args.lidar_z}]')
    print('    rotation_matrix: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]')
    print('  # base_link -> depth_camera_optical_frame (z forward, x right, y down)')
    print('  depth:')
    print(f'    translation: {as_dict(optical_r, optical_t)["translation"]}')
    print(f'    rotation_matrix: {as_dict(optical_r, optical_t)["rotation_matrix"]}')
    print(f'  depth_housing_translation: {as_dict(link_r, link_t)["translation"]}  # depth_camera_link, for checks only')

    roll, pitch, yaw = rotation_to_rpy(optical_r)
    print('\n# quaternion of the same rotation, for TF debugging:')
    qw = math.sqrt(max(0.0, 1 + optical_r[0, 0] + optical_r[1, 1] + optical_r[2, 2])) / 2
    qx = (optical_r[2, 1] - optical_r[1, 2]) / (4 * qw)
    qy = (optical_r[0, 2] - optical_r[2, 0]) / (4 * qw)
    qz = (optical_r[1, 0] - optical_r[0, 1]) / (4 * qw)
    print(f'# rpy(rad) = [{roll:.6f}, {pitch:.6f}, {yaw:.6f}]')
    print(f'# quat     = [x={qx:.6f}, y={qy:.6f}, z={qz:.6f}, w={qw:.6f}]')

    # Optical axis check: z axis of the optical frame must point forward and down.
    axis = optical_r[:, 2]
    print(f'\n# optical +z axis in base_link = [{axis[0]:.4f}, {axis[1]:.4f}, {axis[2]:.4f}]')
    print(f'#   -> pitch down {math.degrees(math.asin(-axis[2])):.2f} deg, height {optical_t[2]:.4f} m')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
