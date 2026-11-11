"""Algorithm-level tests. No simulator, no ROS graph - just NumPy.

Run them before touching the simulator:

    cd src/experiment9_fusion && python3 -m unittest discover -s tests -v

``test_student_todos_are_open`` is expected to FAIL while you are working: it
turns green once you have replaced the three ``NotImplementedError`` lines.
Every other test must pass from the start - if one of them fails, the check
geometry or the shipped configuration is wrong, not your code.
"""

import math
import pathlib
import sys
import unittest

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from experiment9_core import cloud_io, reference, scene9, stages  # noqa: E402
from experiment9_student import fusion as student  # noqa: E402


def read_pgm(path):
    data = path.read_bytes()
    assert data.startswith(b'P5')
    fields, offset = [], 2
    while len(fields) < 3:
        while offset < len(data) and data[offset:offset + 1].isspace():
            offset += 1
        start = offset
        while offset < len(data) and not data[offset:offset + 1].isspace():
            offset += 1
        fields.append(int(data[start:offset]))
    offset += 1
    width, height, _maxval = fields
    return np.frombuffer(data[offset:offset + width * height], dtype=np.uint8).reshape(height, width)


class ExtrinsicTests(unittest.TestCase):
    def test_identity_is_accepted(self):
        rotation, translation = cloud_io.extrinsic([0, 0, 1.3], np.eye(3).ravel().tolist())
        self.assertTrue(np.allclose(rotation, np.eye(3)))
        self.assertTrue(np.allclose(translation, [0, 0, 1.3]))

    def test_mirrored_matrix_is_rejected(self):
        with self.assertRaises(ValueError):
            cloud_io.extrinsic([0, 0, 0], [-1, 0, 0, 0, 1, 0, 0, 0, 1])

    def test_scaled_matrix_is_rejected(self):
        # A matrix that is not orthonormal (a scale factor left in, a degree
        # converted twice) must not be accepted silently.
        with self.assertRaises(ValueError):
            cloud_io.extrinsic([0, 0, 0], (np.eye(3) * 1.02).ravel().tolist())


class TransformCloudTests(unittest.TestCase):
    def test_lidar_points_are_lifted_by_the_mount_height(self):
        points = np.array([[1.0, 0.0, 0.0], [2.0, 1.0, -0.5]], dtype=np.float32)
        moved = reference.transform_cloud(points, (np.eye(3), np.array([0.0, 0.0, 1.3])))
        self.assertTrue(np.allclose(moved[:, 2], [1.3, 0.8], atol=1e-5))
        self.assertTrue(np.allclose(moved[:, 0], points[:, 0]))

    def test_empty_input_returns_empty(self):
        empty = np.zeros((0, 3), dtype=np.float32)
        moved = reference.transform_cloud(empty, (np.eye(3), np.zeros(3)))
        self.assertEqual(moved.shape, (0, 3))


class DepthTests(unittest.TestCase):
    def setUp(self):
        self.width, self.height = 160, 120
        self.intrinsics = {'fx': 138.564, 'fy': 138.564, 'cx': 80.0, 'cy': 60.0,
                           'width': self.width, 'height': self.height}

    def test_center_pixel_lands_on_the_optical_axis(self):
        depth = np.full((self.height, self.width), 2.0, dtype=np.float32)
        points = reference.depth_to_points(depth, self.intrinsics, (np.eye(3), np.zeros(3)))
        self.assertEqual(points.shape[0], self.height * self.width)
        central = points.reshape(self.height, self.width, 3)[60, 80]
        self.assertTrue(np.allclose(central, [0.0, 0.0, 2.0], atol=1e-3))

    def test_column_zero_maps_to_negative_x(self):
        depth = np.full((self.height, self.width), 2.0, dtype=np.float32)
        points = reference.depth_to_points(depth, self.intrinsics, (np.eye(3), np.zeros(3)))
        grid = points.reshape(self.height, self.width, 3)
        expected = (0 - self.intrinsics['cx']) * 2.0 / self.intrinsics['fx']
        self.assertAlmostEqual(float(grid[60, 0, 0]), expected, places=3)
        self.assertLess(float(grid[60, 0, 0]), 0.0)

    def test_invalid_pixels_are_dropped(self):
        depth = np.zeros((self.height, self.width), dtype=np.float32)
        depth[10, 10] = 3.0
        depth[20, 20] = float('nan')
        points = reference.depth_to_points(depth, self.intrinsics, (np.eye(3), np.zeros(3)))
        self.assertEqual(points.shape[0], 1)

    def test_optical_to_base_rotation_is_applied(self):
        # A point 3 m straight ahead in the optical frame must end up 3 m ahead
        # in base_link, 1.154 m up: the camera looks along base_link +x with a
        # 19.5 degree downward tilt.
        rotation = np.array([[0.0, -0.333487, 0.942755],
                             [-1.0, 0.0, 0.0],
                             [0.0, -0.942755, -0.333487]])
        translation = np.array([0.185795, 0.0, 1.154059])
        direction = rotation @ np.array([0.0, 0.0, 3.0]) + translation
        self.assertAlmostEqual(direction[0], 0.185795 + 2.828, places=3)
        self.assertLess(direction[2], translation[2])
        self.assertAlmostEqual(direction[2], 1.154059 - 1.000, places=2)
        downward = math.degrees(math.asin(-(rotation @ np.array([0, 0, 1.0]))[2]))
        self.assertAlmostEqual(downward, 19.48, places=1)


class MergeTests(unittest.TestCase):
    def test_height_filter_removes_floor_and_beam(self):
        floor = np.array([[0.0, 0.0, -0.05], [1.0, 0.0, 0.02]])
        wall = np.array([[2.0, 0.0, 0.8], [2.0, 0.0, 1.6]])
        merged = reference.merge_clouds([floor, wall], (0.10, 1.20), [0.10, 0.04])
        self.assertAlmostEqual(float(merged[:, 2].min()), 0.8)
        self.assertTrue(np.all(merged[:, 2] <= 1.20))

    def test_voxel_downsample_reduces_dense_cloud(self):
        rng = np.random.default_rng(7)
        dense = rng.uniform(0.2, 0.8, size=(5000, 3)).astype(np.float32)
        merged = reference.merge_clouds([dense], (0.10, 1.20), [0.05])
        self.assertLess(merged.shape[0], dense.shape[0])
        self.assertGreater(merged.shape[0], 0)

    def test_per_cloud_voxel_keeps_near_detail(self):
        # Coarse lidar far away and a dense depth patch up close. Downsampling
        # both with the lidar voxel (0.10 m) would collapse the whole depth
        # patch into a single point and take the K1 pallet with it.
        lidar = np.zeros((1, 3), dtype=np.float32)
        lidar[0] = [5.0, 0.0, 0.5]
        dense = np.stack([np.linspace(0.90, 1.00, 3000),
                          np.linspace(-0.05, 0.05, 3000),
                          np.full(3000, 0.30)], axis=1).astype(np.float32)
        tuned = reference.merge_clouds([lidar, dense], (0.10, 1.20), [0.10, 0.02])
        blunt = reference.merge_clouds([lidar, dense], (0.10, 1.20), [0.10, 0.10])
        self.assertGreater(tuned.shape[0], blunt.shape[0])
        self.assertGreaterEqual(tuned.shape[0], 5, 'the near patch must keep several distinct points')

    def test_z_range_is_validated(self):
        with self.assertRaises(ValueError):
            reference.merge_clouds([np.zeros((0, 3))], (1.2, 0.1), [0.1])


class SceneTests(unittest.TestCase):
    def test_checkpoints_are_not_static_geometry(self):
        static = {name for name, *_ in scene9.STATIC_BOXES}
        for name, *_rest in scene9.FUSION_OBSTACLES:
            self.assertNotIn(name, static, f'{name} must only exist in the live scene')

    def test_overhead_beam_does_not_block_motion(self):
        beam = [item for item in scene9.FUSION_OBSTACLES if item[0].startswith('K4')]
        self.assertEqual(len(beam), 1)
        self.assertFalse(beam[0][4], 'a 1.6 m beam cannot stop a 0.5 m robot')

    def test_map_is_free_at_every_checkpoint(self):
        grid = read_pgm(ROOT / 'config/map/warehouse.pgm')
        resolution = scene9.MAP_RESOLUTION
        origin_x, origin_y = scene9.MAP_ORIGIN
        height = grid.shape[0]
        for name, _label, center, _size, _blocks, _note in scene9.FUSION_OBSTACLES:
            column = int((center[0] - origin_x) / resolution)
            row = int((center[1] - origin_y) / resolution)
            value = grid[height - 1 - row, column]
            self.assertEqual(int(value), 254,
                             f'{name} is painted into the map: students could pass without any fusion code')

    def test_map_has_the_room_walls(self):
        grid = read_pgm(ROOT / 'config/map/warehouse.pgm')
        resolution = scene9.MAP_RESOLUTION
        origin_x, origin_y = scene9.MAP_ORIGIN
        height = grid.shape[0]

        def value(x, y):
            return grid[height - 1 - int((y - origin_y) / resolution), int((x - origin_x) / resolution)]

        self.assertEqual(int(value(0.0, 4.5)), 0, 'north wall must be occupied')
        self.assertEqual(int(value(3.0, 0.0)), 0, 'blue wall must be occupied')
        self.assertGreater(int(value(2.25, 3.4)), 0, 'the doorway must stay open')

    def test_goal_is_inside_the_room_and_clear_of_walls(self):
        goal_x, goal_y = scene9.GOAL_XY
        self.assertLess(abs(goal_x), 5.0)
        self.assertLess(abs(goal_y), 4.0)
        self.assertGreater(abs(goal_x), 0.45)

    def test_config_matches_the_derived_extrinsics(self):
        import yaml
        with (ROOT / 'config/fusion.yaml').open(encoding='utf-8') as handle:
            params = yaml.safe_load(handle)['experiment9_fusion']['ros__parameters']
        rotation, translation = cloud_io.extrinsic(params['depth_translation'],
                                                    params['depth_rotation_rowmajor'])
        axis = rotation @ np.array([0.0, 0.0, 1.0])
        self.assertAlmostEqual(math.degrees(math.asin(-axis[2])), 19.48, places=1)
        self.assertAlmostEqual(translation[2], 1.154, places=3)
        self.assertLess(params['z_max'], 1.6, 'z_max must exclude the overhead beam')
        self.assertGreater(params['sync_slop'], 0.5, 'slop must exceed one depth frame (0.5 s)')


class _OnlyLidar:
    """TODO 1 written, the other two still open - the state after the first lab step."""

    def transform_cloud(self, points, extrinsic):
        rotation, translation = extrinsic
        return (np.asarray(points, dtype=np.float32) @ rotation.T + translation).astype(np.float32)

    def depth_to_points(self, *args, **kwargs):
        raise NotImplementedError('TODO 2')

    def merge_clouds(self, *args, **kwargs):
        raise NotImplementedError('TODO 3')


class _OnlyDepth:
    """TODO 2 written, TODO 1 and 3 still open."""

    def transform_cloud(self, *args, **kwargs):
        raise NotImplementedError('TODO 1')

    def depth_to_points(self, depth, intrinsics, extrinsic, min_range, max_range):
        return np.zeros((7, 3), dtype=np.float32)

    def merge_clouds(self, *args, **kwargs):
        raise NotImplementedError('TODO 3')


class _BrokenLidar:
    """Raises instead of returning - one bad stage must not kill the frame."""

    def transform_cloud(self, points, extrinsic):
        raise ValueError('deliberate failure')

    def depth_to_points(self, depth, intrinsics, extrinsic, min_range, max_range):
        return np.zeros((7, 3), dtype=np.float32)

    def merge_clouds(self, clouds, z_range, voxels):
        return np.zeros((3, 3), dtype=np.float32)


class _WrongShape:
    """Returns (N, 2) instead of (N, 3)."""

    def transform_cloud(self, points, extrinsic):
        return np.zeros((5, 2), dtype=np.float32)

    def depth_to_points(self, depth, intrinsics, extrinsic, min_range, max_range):
        return np.zeros((5, 3), dtype=np.float32)

    def merge_clouds(self, clouds, z_range, voxels):
        return np.zeros((5, 3), dtype=np.float32)


def stage_inputs():
    """Fixed inputs; the geometry is deliberately simple - these tests check
    failure isolation, not correctness of the maths."""
    lidar = np.array([[0.0, 0.0, 0.5], [0.5, 0.2, 0.8], [1.0, -0.3, 1.0]], dtype=np.float32)
    depth = np.full((12, 16), 0.3, dtype=np.float32)
    intrinsics = {'fx': 20.0, 'fy': 20.0, 'cx': 8.0, 'cy': 6.0}
    extrinsic = (np.eye(3, dtype=np.float32), np.array([0.0, 0.0, 0.5], dtype=np.float32))
    return {
        'lidar_raw': lidar,
        'depth': depth,
        'intrinsics': intrinsics,
        'extrinsics': (extrinsic, extrinsic),
        'depth_range': (0.05, 10.0),
        'z_range': (0.10, 1.20),
        'voxels': [0.10, 0.04],
    }


class StageIsolationTests(unittest.TestCase):
    """The three TODOs must light up one at a time.

    If the node ran them inside a single ``try``, a student who has only written
    TODO 1 would see an empty topic and no way to tell whether the transform is
    right. ``stages.run`` is the function that guarantees otherwise, so it is
    tested without ROS.
    """

    def test_nothing_written_publishes_nothing(self):
        result = stages.run(student, **stage_inputs())
        self.assertIsNone(result['lidar'])
        self.assertIsNone(result['depth'])
        self.assertIsNone(result['merged'])
        # merge is not attempted while its two inputs are missing, so two errors.
        self.assertEqual(len(result['errors']), 2)
        self.assertTrue(any('TODO 1' in e for e in result['errors']))
        self.assertTrue(any('TODO 2' in e for e in result['errors']))

    def test_only_todo1_is_written_still_produces_lidar(self):
        result = stages.run(_OnlyLidar(), **stage_inputs())
        self.assertEqual(result['lidar'].shape, (3, 3))
        self.assertIsNone(result['depth'])
        self.assertIsNone(result['merged'])
        self.assertEqual(len(result['errors']), 1)
        self.assertIn('TODO 2', result['errors'][0])

    def test_only_todo2_is_written_still_produces_depth(self):
        result = stages.run(_OnlyDepth(), **stage_inputs())
        self.assertIsNone(result['lidar'])
        self.assertEqual(result['depth'].shape, (7, 3))
        self.assertIsNone(result['merged'])
        self.assertEqual(len(result['errors']), 1)
        self.assertIn('TODO 1', result['errors'][0])

    def test_merge_is_only_attempted_once_both_inputs_exist(self):
        result = stages.run(reference, **stage_inputs())
        self.assertIsNotNone(result['lidar'])
        self.assertIsNotNone(result['depth'])
        self.assertIsNotNone(result['merged'])
        self.assertGreater(result['merged'].shape[0], 0)
        self.assertEqual(result['errors'], [])

    def test_a_raising_stage_does_not_block_the_others(self):
        result = stages.run(_BrokenLidar(), **stage_inputs())
        self.assertIsNone(result['lidar'])
        # TODO 2 still runs even though TODO 1 raised: that is the whole point.
        self.assertEqual(result['depth'].shape, (7, 3))
        # merge needs both inputs; with lidar missing it is not attempted.
        self.assertIsNone(result['merged'])
        self.assertEqual(len(result['errors']), 1)
        self.assertIn('ValueError', result['errors'][0])
        self.assertIn('deliberate failure', result['errors'][0])

    def test_wrong_shape_is_reported_not_raised(self):
        result = stages.run(_WrongShape(), **stage_inputs())
        self.assertIsNone(result['lidar'])
        self.assertIn('shape', result['errors'][0])
        self.assertIn('(5, 2)', result['errors'][0])


class StudentTests(unittest.TestCase):
    def test_student_todos_are_open(self):
        points = np.zeros((1, 3), dtype=np.float32)
        with self.assertRaises(NotImplementedError):
            student.transform_cloud(points, (np.eye(3), np.zeros(3)))
        with self.assertRaises(NotImplementedError):
            student.depth_to_points(np.ones((120, 160), dtype=np.float32),
                                    {'fx': 138.564, 'fy': 138.564, 'cx': 80.0, 'cy': 60.0},
                                    (np.eye(3), np.zeros(3)))
        with self.assertRaises(NotImplementedError):
            student.merge_clouds([], (0.1, 1.2), [])


if __name__ == '__main__':
    unittest.main(verbosity=2)
