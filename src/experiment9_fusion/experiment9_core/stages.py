"""Run the three fusion stages so that a half-finished file still produces output.

The three functions students write are independent, but the obvious way to call
them (one ``try`` around all three) makes them behave as one: an unimplemented
``merge_clouds`` would hide a perfectly good ``transform_cloud`` and the node
would publish nothing at all. For most of the lab the file *is* half finished,
so that behaviour would leave the student staring at an empty RViz with no way
to tell which step is wrong.

This module therefore runs each stage on its own and reports what worked. It is
kept out of the node on purpose: no ROS import, so the "only TODO 1 is written"
case can be unit tested.

Returned dict::

    {
        'lidar':  (N, 3) float32 or None,   # TODO 1 succeeded
        'depth':  (M, 3) float32 or None,   # TODO 2 succeeded
        'merged': (K, 3) float32 or None,   # TODO 3 succeeded (needs both above)
        'errors': [str, ...],               # one human-readable line per failure
    }
"""

import numpy as np

LABELS = {
    'lidar': 'TODO 1 transform_cloud',
    'depth': 'TODO 2 depth_to_points',
    'merged': 'TODO 3 merge_clouds',
}


def _call(stage, function, *args):
    """Run one stage. Returns (cloud_or_None, error_or_None)."""
    try:
        cloud = np.asarray(function(*args), dtype=np.float32)
    except NotImplementedError:
        return None, f'{LABELS[stage]} not implemented yet'
    except Exception as error:  # noqa: BLE001 - one bad stage must not kill the frame
        return None, f'{LABELS[stage]} raised {type(error).__name__}: {error}'
    if cloud.ndim != 2 or cloud.shape[1] != 3:
        return None, f'{LABELS[stage]} returned shape {cloud.shape}, expected (N, 3)'
    return cloud, None


def run(impl, lidar_raw, depth, intrinsics, extrinsics, depth_range, z_range, voxels):
    """Run the three stages, isolating failures.

    ``impl``        : module or object exposing the three functions
    ``lidar_raw``   : (N, 3) in ``lidar_frame``
    ``depth``       : (H, W) metres along the optical axis
    ``intrinsics``  : dict with fx/fy/cx/cy
    ``extrinsics``  : (lidar_extrinsic, depth_extrinsic)
    ``depth_range`` : (min, max) valid depth, metres
    ``z_range``     : (z_min, z_max) height band, metres
    ``voxels``      : one voxel edge per cloud
    """
    ext_lidar, ext_depth = extrinsics
    errors = []

    lidar, error = _call('lidar', impl.transform_cloud, lidar_raw, ext_lidar)
    if error:
        errors.append(error)

    depth_base, error = _call('depth', impl.depth_to_points, depth, intrinsics, ext_depth,
                              depth_range[0], depth_range[1])
    if error:
        errors.append(error)

    merged = None
    if lidar is not None and depth_base is not None:
        merged, error = _call('merged', impl.merge_clouds, [lidar, depth_base], z_range, voxels)
        if error:
            errors.append(error)

    return {'lidar': lidar, 'depth': depth_base, 'merged': merged, 'errors': errors}
