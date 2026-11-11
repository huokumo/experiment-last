#!/usr/bin/env python3
"""Pre-flight: everything that can be checked without Gazebo.

This is the "closed loop" that runs on a laptop before the lab: if it passes,
the geometry, the map, the parameters, the launch inputs and the three
algorithms are self-consistent. It does not prove the simulator renders or that
Nav2 converges - only a run on the teaching image proves that.

    python3 tools/preflight.py
    python3 tools/preflight.py --model /root/exp9/src/experiment5/models/unitree_r1/model_depth.sdf
"""

import argparse
import importlib.util
import py_compile
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PACKAGE = REPO / 'src' / 'experiment9_fusion'
sys.path.insert(0, str(PACKAGE))

import yaml  # noqa: E402

from experiment9_core import cloud_io, scene9  # noqa: E402

results = []


def record(name, ok, detail=''):
    results.append((name, ok, detail))
    print(f'[{"PASS" if ok else "FAIL"}] {name}' + (f'  {detail}' if detail else ''))


def check_syntax():
    bad = []
    for path in list(REPO.rglob('*.py')):
        if 'build' in path.parts or 'install' in path.parts:
            continue
        try:
            py_compile.compile(str(path), doraise=True, cfile=str(Path(str(path) + '.pyc-tmp')))
        except py_compile.PyCompileError as error:
            bad.append(f'{path.relative_to(REPO)}: {error.msg.splitlines()[-1]}')
        finally:
            Path(str(path) + '.pyc-tmp').unlink(missing_ok=True)
    record('Python 语法', not bad, '; '.join(bad))


def check_xml():
    bad = []
    for path in REPO.rglob('*.xml'):
        try:
            ET.parse(path)
        except ET.ParseError as error:
            bad.append(f'{path.name}: {error}')
    package = ET.parse(PACKAGE / 'package.xml').getroot()
    name = package.findtext('name')
    record('XML 与 package.xml', not bad and name == 'experiment9_fusion', bad and '; '.join(bad) or name)


def check_yaml():
    bad = []
    for path in list(REPO.rglob('*.yaml')) + list(REPO.rglob('*.yml')):
        try:
            list(yaml.safe_load_all(path.read_text(encoding='utf-8')))
        except yaml.YAMLError as error:
            bad.append(f'{path.name}: {error}')
    record('YAML 解析', not bad, '; '.join(bad))


def check_topics():
    bridge = yaml.safe_load((PACKAGE / 'config/bridge9.yaml').read_text(encoding='utf-8'))
    bridged = {entry['ros_topic_name'] for entry in bridge}
    # /r1/fusion/points is produced by the fusion node, not bridged.
    needed = {'/livox/points', '/r1/depth/image', '/r1/depth/camera_info', '/odom', '/r1/cmd_vel'}
    missing = needed - bridged
    record('桥接话题', not missing, f'缺 {sorted(missing)}' if missing else f'{len(bridged)} 条')

    nav = yaml.safe_load((PACKAGE / 'config/nav2_params.yaml').read_text(encoding='utf-8'))
    cloud = nav['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['cloud']['topic']
    nav_bad = cloud != '/r1/fusion/points'
    record('代价地图订阅融合点云', not nav_bad, cloud)
    z_max = nav['local_costmap']['local_costmap']['ros__parameters']['obstacle_layer']['cloud']['max_obstacle_height']
    record('代价地图高度上限 >= 融合上限', z_max >= 1.2, f'costmap {z_max}')


def check_map():
    grid_path = PACKAGE / 'config/map/warehouse.pgm'
    data = grid_path.read_bytes()
    header = data.split(b'\n', 3)
    width, height = (int(v) for v in header[1].split())
    expected = (int(round(scene9.MAP_SIZE[0] / scene9.MAP_RESOLUTION)),
                int(round(scene9.MAP_SIZE[1] / scene9.MAP_RESOLUTION)))
    record('地图尺寸与场景一致', (width, height) == expected, f'{width}x{height} vs {expected}')

    meta = yaml.safe_load((PACKAGE / 'config/map/warehouse.yaml').read_text(encoding='utf-8'))
    origin_ok = (abs(meta['origin'][0] - scene9.MAP_ORIGIN[0]) < 1e-6
                 and abs(meta['origin'][1] - scene9.MAP_ORIGIN[1]) < 1e-6)
    record('地图 yaml 原点一致', origin_ok, str(meta['origin']))


def check_extrinsics(model=None):
    params = yaml.safe_load((PACKAGE / 'config/fusion.yaml').read_text(encoding='utf-8'))
    params = params['experiment9_fusion']['ros__parameters']
    rotation, translation = cloud_io.extrinsic(params['depth_translation'], params['depth_rotation_rowmajor'])
    axis = rotation @ [0.0, 0.0, 1.0]
    record('相机外参正交且朝前下', axis[0] > 0.9 and axis[2] < -0.3,
           f'光轴 {axis.round(3).tolist()}，高 {translation[2]:.3f} m')

    if model is None:
        record('外参与 SDF 一致', True, '未提供 --model，跳过')
        return
    out = subprocess.run([sys.executable, str(REPO / 'tools/derive_extrinsics.py'), '--model', str(model)],
                         capture_output=True, text=True)
    if out.returncode != 0:
        record('外参与 SDF 一致', False, out.stderr.strip()[:120])
        return
    text = out.stdout
    same = all(f'{v:.6f}'.rstrip('0') in text or f'{v:.6f}' in text
               for v in [translation[0], translation[1], translation[2]])
    record('外参与 SDF 一致', same, 'compared with derive_extrinsics.py')


def check_layout():
    out = subprocess.run([sys.executable, str(REPO / 'tools/check_layout.py')], capture_output=True, text=True)
    ok = out.returncode == 0 and '全部阻断型关卡都在最短路径上' in out.stdout
    detail = '有阻断型关卡不在必经路径上' if not ok else 'K1-K3、K5 阻断，K4 从下方通过'
    record('关卡布局', ok, detail)


def check_pipeline(pattern):
    if pattern is None:
        record('观测结构（离线光线投射）', True, '未提供 --pattern，跳过')
        return
    out = subprocess.run([sys.executable, str(REPO / 'tools/simulate_pipeline.py'),
                          '--pattern', str(pattern)], capture_output=True, text=True)
    ok = out.returncode == 0
    detail = 'K1 雷达 0/深度有数，K4 深度 0/雷达有数' if ok else out.stdout.strip().splitlines()[-1][:80]
    record('观测结构（离线光线投射）', ok, detail)


def check_tests():
    out = subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests'],
                         capture_output=True, text=True, cwd=str(PACKAGE))
    tail = [line for line in out.stderr.strip().splitlines() if line.startswith(('OK', 'FAILED'))]
    record('算法单元测试', out.returncode == 0, tail[-1] if tail else out.stderr.strip()[-100:])


def check_student_file():
    spec = importlib.util.spec_from_file_location('student_fusion', PACKAGE / 'experiment9_student/fusion.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    open_todos = sum(1 for name in ('transform_cloud', 'depth_to_points', 'merge_clouds')
                     if 'NotImplementedError' in (PACKAGE / 'experiment9_student/fusion.py').read_text(encoding='utf-8'))
    record('学生文件处于待完成状态', open_todos == 3, f'{open_todos}/3 个 TODO 未实现（交付时是正常状态）')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--model', help='path to model_depth.sdf for the extrinsic cross-check')
    parser.add_argument('--pattern', help='path to vendor/livox/mid360.csv for the ray-cast check')
    args = parser.parse_args()

    print(f'仓库: {REPO}\n')
    check_syntax()
    check_xml()
    check_yaml()
    check_topics()
    check_map()
    check_extrinsics(args.model)
    check_layout()
    check_pipeline(args.pattern)
    check_tests()
    check_student_file()

    failed = [name for name, ok, _detail in results if not ok]
    print()
    print(f'{len(results) - len(failed)}/{len(results)} 项通过')
    if failed:
        print('未通过: ' + ', '.join(failed))
        return 1
    print('静态闭环通过。下一步只能在教学平台上运行验证（见 docs/PLATFORM_VALIDATION.md）。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
