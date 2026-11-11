"""TODO file: the only file you edit in this experiment.

Three functions stand between the raw sensor topics and a usable obstacle
cloud. Each one is a formula from the lecture; none of them needs a library
beyond NumPy.

Read ``docs/STUDENT_GUIDE.md`` section 4 before starting. When a run fails, the
symptom tells you which function is wrong:

===========================  ====================================
symptom                      function to check
===========================  ====================================
cloud sits 1.3 m too low     ``transform_cloud``
cloud tilted / mirrored      ``transform_cloud``, extrinsic sign
depth cloud rotated 90 deg   ``depth_to_points``, optical frame axes
robot blocked by the floor   ``merge_clouds``, ``z_min`` too low
robot misses the pallet      ``merge_clouds``, ``z_min`` too high
robot stops under the beam   ``merge_clouds``, ``z_max`` too high
nothing published at all     ``depth_to_points`` returning an empty array
===========================  ====================================
"""

import numpy as np


def transform_cloud(points, extrinsic):
    """TODO 1 - move lidar points from ``lidar_frame`` into ``base_link``.

    Parameters
    ----------
    points : (N, 3) float32
        Raw ``/livox/points``. The publisher already states
        ``frame_id=lidar_frame``; the values are metres in that frame.
    extrinsic : (R, t)
        ``R`` is (3, 3), ``t`` is (3,), and both describe *lidar_frame
        expressed in base_link*:
        ``p_base = R @ p_lidar + t``.

    Returns
    -------
    (N, 3) float32

    Notes
    -----
    * In this simulator ``R`` is the identity and ``t = (0, 0, 1.3)``: the
      simulated Mid-360 is bolted 1.3 m above the ground with no rotation.
      Getting the sign wrong here lowers the whole cloud by 2.6 m and makes the
      robot drive over the pallet it should avoid.
    * Don't mutate the input array; the caller reuses it.
    """
    raise NotImplementedError('TODO 1: transform lidar points into base_link')


def depth_to_points(depth, intrinsics, extrinsic, min_range=0.1, max_range=10.0):
    """TODO 2 - turn a depth image into a point cloud in ``base_link``.

    Parameters
    ----------
    depth : (H, W) float32
        Metres along the optical axis (+z), already decoded from the message.
        Zero, NaN and values outside ``[min_range, max_range]`` are invalid.
    intrinsics : dict
        ``fx``, ``fy``, ``cx``, ``cy`` taken from ``/r1/depth/camera_info``
        (160x120, fx = fy = 138.56, cx = 80, cy = 60).
    extrinsic : (R, t)
        ``depth_camera_optical_frame`` expressed in ``base_link``. The optical
        frame is x-right, y-down, z-forward; ``base_link`` is x-forward,
        y-left, z-up. The rotation is not a multiple of 90 degrees: the camera
        is pitched down 19.5 degrees, so the matrix is given, not derived.

    Returns
    -------
    (M, 3) float32, one row per valid pixel

    Notes
    -----
    * Pinhole back-projection, in the optical frame first::

          z = depth[v, u]
          x = (u - cx) * z / fx
          y = (v - cy) * z / fy

    * Then ``p_base = R @ p_cam + t``. Skipping the rotation gives a cloud
      rotated 90 degrees about the optical axis - the classic symptom is a wall
      appearing along the wrong axis.
    * ``u`` is the column index, ``v`` is the row index. Swapping them mirrors
      the cloud left-right and is invisible until the two clouds are overlaid.
    """
    raise NotImplementedError('TODO 2: back-project the depth image into base_link')


def merge_clouds(clouds, z_range, voxels):
    """TODO 3 - filter by height, downsample per cloud, concatenate.

    Parameters
    ----------
    clouds : list of (N_i, 3) float32
        Two clouds, both already in ``base_link``.
    z_range : (z_min, z_max)
        Height band kept, metres, inclusive. The robot is 0.5 m tall; the
        overhead beam at K4 sits at 1.6 m and must fall outside the band.
    voxels : list of float
        One voxel edge per cloud, metres. Use the larger value for the sparse
        lidar and the smaller one for the dense depth cloud.

    Returns
    -------
    (M, 3) float32

    Notes
    -----
    * Order matters: filter first (cheap), then downsample, then concatenate.
      Downsampling after the concatenation lets the dense cloud win every
      occupied voxel and erases the lidar returns that justify the fusion.
    * ``z_min`` below 0 puts the floor into the cost map and the robot refuses
      to move at all.
    * ``z_max`` at or above 1.6 m puts the beam into the cost map and the robot
      stops in the middle of the passage it can physically drive through.
    """
    raise NotImplementedError('TODO 3: filter, downsample and merge the two clouds')
