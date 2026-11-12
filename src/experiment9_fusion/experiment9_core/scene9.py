"""Scene geometry for Experiment 9 (multi-sensor fusion).

Two lists matter and they are deliberately kept apart:

``STATIC_BOXES``
    Geometry that exists before the robot starts and is therefore baked into the
    pre-built occupancy map (``config/map/warehouse.pgm``). The global planner
    already knows about it, so it can never demonstrate anything about fusion.

``FUSION_OBSTACLES``
    Geometry that only exists in the live scene. It is *not* in the map, so the
    robot can only learn about it from the fused point cloud. If the fusion
    chain is wrong, the robot drives into one of these and the motion guard
    stops it dead - that is the whole pass/fail signal of the experiment.

The same list drives three consumers, which is why it lives in one place:

* the Gazebo world (visual + raycast geometry),
* the CPU ray caster that plays the role of the Mid-360 (``obstacle`` tags),
* the planar motion guard (2D ``obstacle`` tags, only for boxes that block).
"""

# --- static scene: same room as Experiment 7, baked into the map -------------
# (name, center xyz, size xyz)
STATIC_BOXES = [
    ('floor', (0.0, 0.0, -0.05), (14.0, 12.0, 0.1)),
    ('north', (0.0, 4.5, 1.4), (11.0, 0.15, 2.8)),
    ('south', (0.0, -4.5, 1.4), (11.0, 0.15, 2.8)),
    ('east', (5.5, 0.0, 1.4), (0.15, 9.0, 2.8)),
    ('west', (-5.5, 0.0, 1.4), (0.15, 9.0, 2.8)),
    ('blue_wall', (3.0, 0.0, 1.2), (0.2, 3.0, 2.4)),
    ('pillar', (-1.4, 1.4, 1.0), (0.25, 0.25, 2.0)),
    ('thin_pillar', (1.5, -1.6, 1.0), (0.12, 0.12, 2.0)),
    ('crate', (-2.3, -1.7, 0.45), (0.9, 0.7, 0.9)),
    ('door_left', (0.2, 3.4, 1.2), (2.5, 0.15, 2.4)),
    ('door_right', (3.8, 3.4, 1.2), (1.5, 0.15, 2.4)),
]

# --- checkpoints: live scene only, absent from the map -----------------------
# name, label, center xyz, size xyz, blocks the robot?, why it is here
FUSION_OBSTACLES = [
    ('K1_pallet', 'K1 低矮托盘',
     (-1.7, -3.25, 0.175), (1.2, 0.8, 0.35), True,
     '0.35 m tall: below the lidar depression line inside 7.52 m, inside the depth FOV from 1.3 m out'),
    ('K2_column', 'K2 细立柱',
     (0.30, -0.85, 1.0), (0.10, 0.10, 2.0), True,
     '0.10 m wide: about 2 px at 5 m in 160x120, stable lidar return'),
    ('K3_panel', 'K3 薄板（玻璃替代物）',
     (1.55, 0.35, 1.0), (0.04, 1.2, 2.0), True,
     '4 cm thick: lidar only catches a grazing line, depth sees a full quad; alignment check'),
    ('K4_beam', 'K4 悬空横梁',
     (3.875, 0.30, 1.6), (1.45, 0.15, 0.15), False,
     '1.6 m high: above the robot and above the depth FOV, passable underneath'),
    ('K5_rack', 'K5 侧向货架',
     (2.60, 2.15, 0.75), (0.6, 0.6, 1.5), True,
     'stands 1.2 m beside the route: both sensors report it, but only the 360 deg lidar does so '
     'while it is still off-axis, so a filter that keeps only x>0 points loses it'),
]

# --- route ------------------------------------------------------------------
# Goal sits inside the main room: the corridor behind the y=3.4 wall is 1.1 m
# deep and the robot needs a 0.9 m circle, so it is scenery, not a destination.
START_XY = (-4.4, -3.4)
START_YAW = 0.0
GOAL_XY = (4.6, 2.6)
GOAL_YAW = 0.0

# Region used by cloud_check.py: name -> (center xyz, half extents xyz)
CHECK_REGIONS = {
    'K1_pallet': ((-1.7, -3.25, 0.175), (0.6, 0.4, 0.25)),
    'K2_column': ((0.30, -0.85, 1.0), (0.12, 0.12, 0.6)),
    'K3_panel': ((1.55, 0.35, 1.0), (0.10, 0.6, 0.6)),
    'K4_beam': ((3.875, 0.30, 1.6), (0.72, 0.15, 0.15)),
    'K5_rack': ((2.60, 2.15, 0.75), (0.3, 0.3, 0.6)),
}

# Reference plane for the extrinsic check (cloud_check --wall).
REFERENCE_WALL = {'name': 'blue_wall', 'axis': 'x', 'value': 2.9}

# Map raster: covers the room plus a 1.5 m margin of unknown space.
MAP_ORIGIN = (-7.0, -6.0)
MAP_SIZE = (14.0, 12.0)
MAP_RESOLUTION = 0.05


def color_for(name):
    if name == 'blue_wall':
        return '0.08 0.25 0.85 1'
    if name == 'floor':
        return '0.55 0.55 0.55 1'
    if name.startswith('K1'):
        return '0.55 0.35 0.18 1'
    if name.startswith('K2'):
        return '0.30 0.30 0.30 1'
    if name.startswith('K3'):
        return '0.55 0.80 0.90 1'
    if name.startswith('K4'):
        return '0.85 0.55 0.15 1'
    if name.startswith('K5'):
        return '0.45 0.45 0.50 1'
    return '0.75 0.70 0.58 1'


def _text(parent, tag, value):
    import xml.etree.ElementTree as ET
    ET.SubElement(parent, tag).text = str(value)
    return None


def _add_box(parent, name, center, size, static=True):
    import xml.etree.ElementTree as ET
    model = ET.SubElement(parent, 'model', name=name)
    _text(model, 'static', 'true' if static else 'false')
    _text(model, 'pose', ' '.join(map(str, (*center, 0, 0, 0))))
    link = ET.SubElement(model, 'link', name='link')
    for kind in ('collision', 'visual'):
        node = ET.SubElement(link, kind, name=kind)
        box = ET.SubElement(ET.SubElement(node, 'geometry'), 'box')
        _text(box, 'size', ' '.join(map(str, size)))
        if kind == 'visual':
            material = ET.SubElement(node, 'material')
            color = color_for(name)
            _text(material, 'ambient', color)
            _text(material, 'diffuse', color)
    return model


def generate(exp5, destination, pattern_csv, samples=12000, frequency=5.0):
    """Write the world SDF and return its path.

    ``exp5`` is the share directory of ``experiment5_depth``; the R1 model with
    the depth camera lives there and is reused unchanged.
    """
    import xml.etree.ElementTree as ET
    from pathlib import Path

    if not 100 <= samples <= 40000:
        raise ValueError('samples must be within [100, 40000]')
    if not 0 < frequency <= 20:
        raise ValueError('frequency must be within (0, 20]')

    root = ET.Element('sdf', version='1.8')
    world = ET.SubElement(root, 'world', name='experiment9')
    physics = ET.SubElement(world, 'physics', name='physics', type='ignored')
    _text(physics, 'max_step_size', 0.001)
    _text(physics, 'real_time_factor', 1.0)
    _text(world, 'gravity', '0 0 -9.80665')
    for lib, name in (('physics', 'Physics'), ('user-commands', 'UserCommands'),
                      ('scene-broadcaster', 'SceneBroadcaster')):
        ET.SubElement(world, 'plugin',
                      filename=f'libignition-gazebo-{lib}-system.so',
                      name=f'ignition::gazebo::systems::{name}')

    # 深度相机是渲染类传感器，必须由 Sensors 系统驱动才会出帧。实验五的世界里
    # 显式写了这个插件，本实验的 SDF 是在代码里现生成的，漏了它会出现一个很
    # 迷惑的现象：/livox/points 正常（雷达是实验七自写的 CPU 光线投射插件，不走
    # 渲染），而 /r1/depth/image 一直是 0 Hz。render_engine 与实验五保持一致。
    sensors = ET.SubElement(world, 'plugin',
                            filename='libignition-gazebo-sensors-system.so',
                            name='ignition::gazebo::systems::Sensors')
    _text(sensors, 'render_engine', 'ogre2')
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    _text(light, 'pose', '0 0 8 0 0 0')
    _text(light, 'direction', '-0.5 0.2 -1')
    _text(light, 'diffuse', '0.9 0.9 0.9 1')

    for name, center, size in STATIC_BOXES:
        _add_box(world, name, center, size)
    for name, _label, center, size, _blocks, _note in FUSION_OBSTACLES:
        _add_box(world, name, center, size)

    # --- robot: Experiment 5 model, depth camera sensor kept ----------------
    robot = ET.parse(Path(exp5) / 'models/unitree_r1/model_depth.sdf').getroot().find('model')
    robot.set('name', 'r1')
    _text(robot, 'pose', ' '.join(map(str, (*START_XY, 0.0, 0.0, 0.0, START_YAW))))

    # housing for the simulated Mid-360, purely visual
    mount = ET.SubElement(robot, 'link', name='mid360_visual')
    _text(mount, 'pose', '0 0 1.3 0 0 0')
    visual = ET.SubElement(mount, 'visual', name='mid360_housing')
    cylinder = ET.SubElement(ET.SubElement(visual, 'geometry'), 'cylinder')
    _text(cylinder, 'radius', 0.035)
    _text(cylinder, 'length', 0.06)
    material = ET.SubElement(visual, 'material')
    _text(material, 'ambient', '0.05 0.05 0.05 1')
    _text(material, 'diffuse', '0.05 0.05 0.05 1')

    motion = ET.SubElement(robot, 'plugin', filename='libr1_planar_motion_system.so',
                           name='experiment6::PlanarMotion')
    motion_settings = {
        'min_x': -5.3, 'max_x': 5.3, 'min_y': -4.3, 'max_y': 4.3,
        'footprint_radius': 0.45, 'max_linear': 0.25, 'max_angular': 0.5,
        'command_timeout': 0.35, 'publish_frequency': 50,
    }
    for key, value in motion_settings.items():
        _text(motion, key, value)

    sensor = ET.SubElement(robot, 'plugin', filename='libmid360_teaching_system.so',
                           name='experiment7::Mid360Teaching')
    sensor_settings = {
        'frequency': frequency, 'samples': samples, 'min_range': 0.2,
        'max_range': 20.0, 'range_noise': 0.01, 'mounting': '0 0 1.3 0 0 0',
        'pattern_csv': pattern_csv,
    }
    for key, value in sensor_settings.items():
        _text(sensor, key, value)

    # ray cast geometry: everything, including the overhead beam and the floor
    for name, center, size in STATIC_BOXES + [(n, c, s) for n, _l, c, s, _b, _n in FUSION_OBSTACLES]:
        obstacle = ET.SubElement(sensor, 'obstacle')
        _text(obstacle, 'pose', ' '.join(map(str, (*center, 0, 0, 0))))
        _text(obstacle, 'size', ' '.join(map(str, size)))

    # 2D motion guard: same list minus the overhead beam (a 1.6 m bar cannot
    # stop a 0.5 m tall robot, and a 2D guard has no way to know that).
    for name, center, size in STATIC_BOXES:
        if name == 'floor':
            continue
        _guard_box(motion, center, size)
    for name, _label, center, size, blocks, _note in FUSION_OBSTACLES:
        if blocks:
            _guard_box(motion, center, size)

    world.append(robot)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(destination, encoding='utf-8', xml_declaration=True)
    return str(destination)


def _guard_box(parent, center, size):
    obstacle = _sub(parent, 'obstacle')
    for key, value in zip(('x', 'y', 'yaw', 'half_x', 'half_y'),
                          (center[0], center[1], 0.0, size[0] / 2.0, size[1] / 2.0)):
        _text(obstacle, key, value)
    return obstacle


def _sub(parent, tag):
    import xml.etree.ElementTree as ET
    return ET.SubElement(parent, tag)
