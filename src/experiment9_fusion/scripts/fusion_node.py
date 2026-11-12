#!/usr/bin/env python3
"""Fusion node: two sensors in, one obstacle cloud out.

The node owns everything that is not the algorithm: subscriptions, time
pairing, parameter loading, publishing and status reporting. The three
functions students write are called from :meth:`FusionNode._on_pair`.

Chain::

    /livox/points (lidar_frame) --TODO 1--> base_link  \\
                                                         >--TODO 3--> /r1/fusion/points
    /r1/depth/image + camera_info --TODO 2--> base_link /

The merged cloud is published in ``base_link`` because Nav2's cost map already
knows how to get from ``base_link`` to ``map``; publishing in ``map`` would
silently double-apply the robot pose.

Each stage publishes on its own (``/r1/fusion/points_lidar`` after TODO 1,
``/r1/fusion/points_depth`` after TODO 2, ``/r1/fusion/points`` after TODO 3),
so the three TODOs can be written and checked one at a time instead of all at
once. ``/r1/fusion/status`` reports which stages are live under ``stage``.
"""

import json
import math

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String

import experiment9_student.fusion as student_fusion
from experiment9_core import cloud_io, reference, stages

class SingleSource:
    """Reference implementation restricted to one sensor.

    This exists for the negative check in ``docs/PLATFORM_VALIDATION.md``: if
    the robot still reaches the goal with one cloud thrown away, the scenario is
    not actually testing fusion and the checkpoints need re-placing. Teachers
    must run both modes once; students may run them as a bonus exercise.
    """

    def __init__(self, keep):
        # 0 = lidar, 1 = depth, matching the [lidar, depth] order in _on_pair
        self.keep = keep
        self.name = ('lidar', 'depth')[keep]

    def transform_cloud(self, points, extrinsic):
        return reference.transform_cloud(points, extrinsic)

    def depth_to_points(self, depth, intrinsics, extrinsic, min_range=0.1, max_range=10.0):
        return reference.depth_to_points(depth, intrinsics, extrinsic, min_range, max_range)

    def merge_clouds(self, clouds, z_range, voxels):
        clouds = list(clouds)
        voxels = list(voxels)
        return reference.merge_clouds([clouds[self.keep]], z_range, [voxels[self.keep]])


IMPLS = {
    'student': student_fusion,
    'solution': reference,
    # diagnostics, reference-quality but deliberately blind in one modality
    'lidar': SingleSource(0),
    'depth': SingleSource(1),
}


class FusionNode(Node):
    def __init__(self):
        super().__init__('experiment9_fusion')
        self.declare_parameter('mode', 'student')
        self.declare_parameter('lidar_translation', [0.0, 0.0, 1.3])
        self.declare_parameter('lidar_rotation_rowmajor', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('depth_translation', [0.185795, 0.0, 1.154059])
        self.declare_parameter('depth_rotation_rowmajor',
                               [0.0, -0.333487, 0.942755, -1.0, 0.0, 0.0, 0.0, -0.942755, -0.333487])
        self.declare_parameter('z_min', 0.10)
        self.declare_parameter('z_max', 1.20)
        self.declare_parameter('voxel_lidar', 0.10)
        self.declare_parameter('voxel_depth', 0.04)
        self.declare_parameter('depth_min_range', 0.10)
        self.declare_parameter('depth_max_range', 10.0)
        self.declare_parameter('sync_slop', 0.6)
        self.declare_parameter('depth_hfov', 1.047197551196598)

        self.mode = str(self.get_parameter('mode').value)
        if self.mode not in IMPLS:
            raise ValueError(f'mode must be one of {sorted(IMPLS)}, got {self.mode!r}')
        self.impl = IMPLS[self.mode]
        self.impl_name = self.mode

        self.ext_lidar = cloud_io.extrinsic(
            self.get_parameter('lidar_translation').value,
            self.get_parameter('lidar_rotation_rowmajor').value)
        self.ext_depth = cloud_io.extrinsic(
            self.get_parameter('depth_translation').value,
            self.get_parameter('depth_rotation_rowmajor').value)
        self.z_range = (float(self.get_parameter('z_min').value), float(self.get_parameter('z_max').value))
        self.voxels = [float(self.get_parameter('voxel_lidar').value),
                       float(self.get_parameter('voxel_depth').value)]
        self.depth_min = float(self.get_parameter('depth_min_range').value)
        self.depth_max = float(self.get_parameter('depth_max_range').value)
        self.depth_hfov = float(self.get_parameter('depth_hfov').value)
        slop = float(self.get_parameter('sync_slop').value)

        self.camera_info = None
        self._warned_intrinsics = False
        self.failures = 0
        self.last_complaint = 0.0
        self._stages_ok = set()

        self.create_subscription(CameraInfo, '/r1/depth/camera_info', self._on_camera_info, qos_profile_sensor_data)
        lidar_sub = Subscriber(self, PointCloud2, '/livox/points', qos_profile=qos_profile_sensor_data)
        depth_sub = Subscriber(self, Image, '/r1/depth/image', qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([lidar_sub, depth_sub], queue_size=20, slop=slop)
        self.sync.registerCallback(self._on_pair)

        self.points_pub = self.create_publisher(PointCloud2, '/r1/fusion/points', qos_profile_sensor_data)
        self.lidar_pub = self.create_publisher(PointCloud2, '/r1/fusion/points_lidar', qos_profile_sensor_data)
        self.depth_pub = self.create_publisher(PointCloud2, '/r1/fusion/points_depth', qos_profile_sensor_data)
        self.status_pub = self.create_publisher(String, '/r1/fusion/status', 10)

        self.get_logger().info(
            f'fusion ready | mode={self.mode} | z=[{self.z_range[0]:.2f}, {self.z_range[1]:.2f}] '
            f'| voxels={self.voxels} | slop={slop:.2f}s')
        if self.mode == 'student':
            self.get_logger().info('fill the three TODO functions in experiment9_student/fusion.py')
        elif self.mode in ('lidar', 'depth'):
            self.get_logger().warn(
                f'diagnostic mode {self.mode}: the other cloud is dropped on purpose '
                '(negative check, not a lab step)')

    # -- inputs ------------------------------------------------------------
    def _on_camera_info(self, msg):
        self.camera_info = cloud_io.intrinsics_from_camera_info(msg)

    @staticmethod
    def _intrinsics_from_geometry(image_msg, hfov):
        """Intrinsics for the case where CameraInfo never shows up.

        Gazebo's depth camera only publishes CameraInfo while its render loop
        is running for a subscriber, and on a software-rendered VM that topic
        can stay silent for minutes (see gz-sensors#309). Waiting for it used
        to drop every frame and leave ``/r1/fusion/points`` at 0 Hz with only a
        "waiting for /r1/depth/camera_info" line as a clue. The numbers below
        are the same ones the sensor reports: square pixels, non-distorted,
        principal point at the image centre.
        """
        width = int(image_msg.width)
        height = int(image_msg.height)
        fx = (width / 2.0) / math.tan(hfov / 2.0)
        return {'fx': fx, 'fy': fx,
                'cx': width / 2.0, 'cy': height / 2.0,
                'width': width, 'height': height}

    def _on_pair(self, cloud_msg, image_msg):
        intrinsics = self.camera_info
        if intrinsics is None:
            intrinsics = self._intrinsics_from_geometry(image_msg, self.depth_hfov)
            if not self._warned_intrinsics:
                self._warned_intrinsics = True
                self.get_logger().warn(
                    'no /r1/depth/camera_info yet, using the sensor geometry instead '
                    f'(fx={intrinsics["fx"]:.2f}, cx={intrinsics["cx"]:.1f}, '
                    f'cy={intrinsics["cy"]:.1f}); the result is unchanged')
        stamp = cloud_msg.header.stamp
        dt = abs(float(cloud_msg.header.stamp.sec - image_msg.header.stamp.sec)
                 + float(cloud_msg.header.stamp.nanosec - image_msg.header.stamp.nanosec) * 1e-9)

        try:
            lidar_raw = cloud_io.read_xyz(cloud_msg)
            depth = cloud_io.decode_depth(image_msg)
        except Exception as error:  # noqa: BLE001 - a bad frame must not kill the node
            self._complain(f'decode failed: {type(error).__name__}: {error}')
            return

        # Stages run in isolation (see experiment9_core/stages.py), so the three
        # TODOs can be written one at a time: /points_lidar lights up after
        # TODO 1, /points_depth after TODO 2, /points (the one Nav2 reads) after
        # TODO 3. One try around all three would hide finished work behind the
        # unfinished part and leave the student with no feedback at all.
        result = stages.run(self.impl, lidar_raw, depth, intrinsics,
                            (self.ext_lidar, self.ext_depth),
                            (self.depth_min, self.depth_max),
                            self.z_range, self.voxels)
        for message in result['errors']:
            self._complain(message)
        lidar_base, depth_base, merged = result['lidar'], result['depth'], result['merged']

        done = []
        for key, cloud, publisher in (('lidar', lidar_base, self.lidar_pub),
                                      ('depth', depth_base, self.depth_pub),
                                      ('merged', merged, self.points_pub)):
            if cloud is None:
                continue
            publisher.publish(cloud_io.write_xyz(cloud, 'base_link', stamp))
            done.append(key)
            if key not in self._stages_ok:
                self._stages_ok.add(key)
                self.get_logger().info(f'{stages.LABELS[key]} OK -> publishing {self._topic_of(key)}')
        if merged is not None:
            self.failures = 0

        status = {
            'mode': self.mode,
            'stage': '+'.join(done) if done else 'none',
            'lidar_in': int(lidar_raw.shape[0]),
            'lidar_base': None if lidar_base is None else int(lidar_base.shape[0]),
            'depth_in': int(np.count_nonzero(np.isfinite(depth) & (depth > 0))),
            'depth_base': None if depth_base is None else int(depth_base.shape[0]),
            'merged': None if merged is None else int(merged.shape[0]),
            'pair_dt': round(dt, 4),
            'intrinsics': 'camera_info' if self.camera_info is not None else 'geometry',
            'z_min': round(self.z_range[0], 3),
            'z_max': round(self.z_range[1], 3),
        }
        self.status_pub.publish(String(data=json.dumps(status, ensure_ascii=False)))

    @staticmethod
    def _topic_of(key):
        return {'lidar': '/r1/fusion/points_lidar',
                'depth': '/r1/fusion/points_depth',
                'merged': '/r1/fusion/points'}[key]

    def _complain(self, text):
        """Rate-limit failure messages so a 5 Hz loop cannot flood the screen."""
        self.failures += 1
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.failures == 1 or now - self.last_complaint > 5.0:
            self.last_complaint = now
            self.get_logger().error(text if self.failures == 1 else f'{text} (x{self.failures})')


def main():
    rclpy.init()
    node = None
    try:
        node = FusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
