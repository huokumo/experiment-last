"""Reference implementation of the three functions students have to write.

The node loads this module when it is started with ``mode:=solution`` so that
the lab can be run end to end (and regression tested) before anyone touches
``experiment9_student``. It is deliberately short: every line here exists in the
student handout as a formula, not as code.
"""

import numpy as np


def transform_cloud(points, extrinsic):
    """Move lidar points from ``lidar_frame`` into ``base_link``.

    ``points``   : (N, 3) float32, metres
    ``extrinsic``: (R, t) with R (3, 3) and t (3,) describing *lidar in
                   base_link*, i.e. ``p_base = R @ p_lidar + t``
    returns      : (N, 3) float32
    """
    rotation, translation = extrinsic
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if points.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return (points @ rotation.T + translation).astype(np.float32)


def depth_to_points(depth, intrinsics, extrinsic, min_range=0.1, max_range=10.0):
    """Back-project a depth image and move it into ``base_link``.

    ``depth``     : (H, W) float32 metres, +z forward in the optical frame
    ``intrinsics``: dict with fx, fy, cx, cy
    ``extrinsic`` : (R, t) describing *optical frame in base_link*
    returns       : (M, 3) float32, M <= H*W (invalid pixels dropped)
    """
    depth = np.asarray(depth, dtype=np.float32)
    height, width = depth.shape
    fx, fy = float(intrinsics['fx']), float(intrinsics['fy'])
    cx, cy = float(intrinsics['cx']), float(intrinsics['cy'])
    if fx <= 0 or fy <= 0:
        raise ValueError('fx and fy must be positive')

    valid = np.isfinite(depth) & (depth >= min_range) & (depth <= max_range)
    if not np.any(valid):
        return np.zeros((0, 3), dtype=np.float32)

    rows, cols = np.nonzero(valid)
    z = depth[rows, cols].astype(np.float64)
    # Pinhole model, optical convention: x right, y down, z forward.
    x = (cols - cx) * z / fx
    y = (rows - cy) * z / fy
    points_camera = np.stack([x, y, z], axis=1)

    rotation, translation = extrinsic
    return (points_camera @ rotation.T + translation).astype(np.float32)


def merge_clouds(clouds, z_range, voxels):
    """Height filter, per-cloud voxel downsample and concatenate.

    ``clouds``  : list of (N_i, 3) arrays, already in ``base_link``
    ``z_range`` : (z_min, z_max) metres, kept inclusive
    ``voxels``  : one voxel size per cloud, metres
    returns     : (M, 3) float32

    Downsampling per cloud rather than after the merge matters: the lidar has
    ~5 cm point spacing at 10 m while the depth camera has ~6 mm spacing at
    1 m. A single 10 cm voxel after concatenation would keep whichever cloud
    happened to be first and throw the near-field detail away - which is
    exactly the detail the low pallet at K1 depends on.
    """
    z_min, z_max = float(z_range[0]), float(z_range[1])
    if z_min >= z_max:
        raise ValueError('z_range must be (min, max) with min < max')
    clouds = list(clouds)
    voxels = list(voxels)
    if len(clouds) != len(voxels):
        raise ValueError('one voxel size per cloud is required')

    kept = []
    for cloud, voxel in zip(clouds, voxels):
        points = np.asarray(cloud, dtype=np.float64).reshape(-1, 3)
        if points.shape[0] == 0:
            continue
        points = points[(points[:, 2] >= z_min) & (points[:, 2] <= z_max)]
        if points.shape[0] == 0:
            continue
        if voxel and voxel > 0:
            points = voxel_downsample(points, float(voxel))
        kept.append(points)
    if not kept:
        return np.zeros((0, 3), dtype=np.float32)
    return np.vstack(kept).astype(np.float32)


def voxel_downsample(points, voxel):
    """Keep one point per voxel, first occurrence in buffer order."""
    keys = np.floor(points / voxel).astype(np.int64)
    _, index = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(index)]
