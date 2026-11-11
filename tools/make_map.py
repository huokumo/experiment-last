#!/usr/bin/env python3
"""Render the pre-built occupancy map from the static scene geometry.

The map must contain the room and nothing else. In particular it must NOT
contain K1-K5: those only exist in the live scene, and if they were painted
into the map a student could pass the navigation task without writing a single
line of fusion code.

Usage:
    python3 tools/make_map.py                 # rewrite config/map/*
    python3 tools/make_map.py --preview       # ASCII preview on stdout
"""

import argparse
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'src' / 'experiment9_fusion'))

from experiment9_core import scene9  # noqa: E402

FREE = 254
OCCUPIED = 0
UNKNOWN = 205


def rasterize(preview=False):
    resolution = scene9.MAP_RESOLUTION
    width = int(round(scene9.MAP_SIZE[0] / resolution))
    height = int(round(scene9.MAP_SIZE[1] / resolution))
    origin_x, origin_y = scene9.MAP_ORIGIN
    grid = bytearray([UNKNOWN]) * (width * height)

    def cell_of(x, y):
        return int((x - origin_x) / resolution), int((y - origin_y) / resolution)

    # Free space: the room interior, one cell inside the outer walls.
    (x0, y0), (x1, y1) = cell_of(-5.35, -4.35), cell_of(5.35, 4.35)
    for row in range(y0, y1 + 1):
        for col in range(x0, x1 + 1):
            grid[(height - 1 - row) * width + col] = FREE

    # Occupied: every static box footprint except the floor slab.
    for name, center, size in scene9.STATIC_BOXES:
        if name == 'floor':
            continue
        cx0, cy0 = cell_of(center[0] - size[0] / 2, center[1] - size[1] / 2)
        dx1, dy1 = cell_of(center[0] + size[0] / 2, center[1] + size[1] / 2)
        for row in range(cy0, max(dy1, cy0 + 1) + 1):
            for col in range(cx0, max(dx1, cx0 + 1) + 1):
                grid[(height - 1 - row) * width + col] = OCCUPIED

    if preview:
        step = 4
        for row in range(height - 1, -1, -step):
            line = ''
            for col in range(0, width, step):
                value = grid[(height - 1 - row) * width + col]
                line += ' ' if value == FREE else ('#' if value == OCCUPIED else '.')
            print(f'{origin_y + row * resolution:6.2f} |{line}|')
        print('       +' + '-' * (width // step) + '+')
        print('        ' + ''.join(str(int(origin_x + i * step * resolution / 10) % 10)
                                   for i in range(0, width // step // 10)))
    return grid, width, height


def write_files():
    grid, width, height = rasterize()
    target = REPO / 'src' / 'experiment9_fusion' / 'config' / 'map'
    target.mkdir(parents=True, exist_ok=True)
    pgm = target / 'warehouse.pgm'
    with pgm.open('wb') as handle:
        handle.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        handle.write(bytes(grid))
    yaml_path = target / 'warehouse.yaml'
    yaml_path.write_text(
        'image: warehouse.pgm\n'
        f'resolution: {scene9.MAP_RESOLUTION}\n'
        f'origin: [{scene9.MAP_ORIGIN[0]:.3f}, {scene9.MAP_ORIGIN[1]:.3f}, 0.0]\n'
        'negate: 0\n'
        'occupied_thresh: 0.65\n'
        'free_thresh: 0.25\n',
        encoding='utf-8')
    occupied = sum(1 for v in grid if v == OCCUPIED)
    free = sum(1 for v in grid if v == FREE)
    print(f'wrote {pgm.relative_to(REPO)} ({width}x{height}, occupied={occupied}, free={free})')
    print(f'wrote {yaml_path.relative_to(REPO)}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--preview', action='store_true', help='print an ASCII preview instead of writing files')
    args = parser.parse_args()
    if args.preview:
        rasterize(preview=True)
    else:
        write_files()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
