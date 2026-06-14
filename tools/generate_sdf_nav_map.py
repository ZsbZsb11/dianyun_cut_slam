#!/usr/bin/env python3
import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
import yaml


def parse_pose(text):
    values = [float(v) for v in (text or '').split()]
    values += [0.0] * (6 - len(values))
    return values[:6]


def world_to_pixel(x, y, origin_x, origin_y, resolution, height):
    px = int(round((x - origin_x) / resolution))
    py_from_bottom = int(round((y - origin_y) / resolution))
    return px, height - 1 - py_from_bottom


def draw_box(img, cx, cy, sx, sy, yaw, origin_x, origin_y, resolution):
    height = img.shape[0]
    c = math.cos(yaw)
    s = math.sin(yaw)
    corners = []
    for dx, dy in [(-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2)]:
        wx = cx + c * dx - s * dy
        wy = cy + s * dx + c * dy
        corners.append(world_to_pixel(wx, wy, origin_x, origin_y, resolution, height))
    cv2.fillPoly(img, [np.array(corners, dtype=np.int32)], 0)


def draw_circle(img, cx, cy, radius, origin_x, origin_y, resolution):
    px, py = world_to_pixel(cx, cy, origin_x, origin_y, resolution, img.shape[0])
    cv2.circle(img, (px, py), max(1, int(round(radius / resolution))), 0, -1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--world', required=True)
    parser.add_argument('--output-base', required=True)
    parser.add_argument('--resolution', type=float, default=0.05)
    parser.add_argument('--margin', type=float, default=0.15)
    args = parser.parse_args()

    world_path = Path(args.world)
    output_base = Path(args.output_base)
    output_base.parent.mkdir(parents=True, exist_ok=True)

    root = ET.parse(world_path).getroot()
    floor = root.find('.//world/model[@name="floor"]')
    floor_size = floor.find('.//box/size').text.split()
    floor_x = float(floor_size[0])
    floor_y = float(floor_size[1])

    origin_x = -floor_x / 2 - args.margin
    origin_y = -floor_y / 2 - args.margin
    width = int(math.ceil((floor_x + 2 * args.margin) / args.resolution))
    height = int(math.ceil((floor_y + 2 * args.margin) / args.resolution))

    img = np.full((height, width), 254, dtype=np.uint8)

    for model in root.findall('.//world/model'):
        name = model.get('name')
        if name == 'floor':
            continue

        mx, my, mz, mroll, mpitch, myaw = parse_pose(model.findtext('pose'))
        for collision in model.findall('.//collision'):
            cx, cy, cz, croll, cpitch, cyaw = parse_pose(collision.findtext('pose'))
            wx = mx + math.cos(myaw) * cx - math.sin(myaw) * cy
            wy = my + math.sin(myaw) * cx + math.cos(myaw) * cy
            wz = mz + cz
            yaw = myaw + cyaw

            box = collision.find('.//box/size')
            if box is not None:
                sx, sy, sz = [float(v) for v in box.text.split()]
                if wz + sz / 2 < 0.10:
                    continue
                draw_box(img, wx, wy, sx, sy, yaw, origin_x, origin_y, args.resolution)
                continue

            cylinder = collision.find('.//cylinder')
            if cylinder is not None:
                radius = float(cylinder.findtext('radius'))
                length = float(cylinder.findtext('length') or '0.0')
                if wz + length / 2 < 0.10:
                    continue
                draw_circle(img, wx, wy, radius, origin_x, origin_y, args.resolution)

    pgm_path = output_base.with_suffix('.pgm')
    yaml_path = output_base.with_suffix('.yaml')
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n# CREATOR: generate_sdf_nav_map.py\n{width} {height}\n255\n'.encode('ascii'))
        f.write(img.tobytes())

    metadata = {
        'image': pgm_path.name,
        'mode': 'trinary',
        'resolution': args.resolution,
        'origin': [origin_x, origin_y, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.25,
    }
    yaml_path.write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    print(f'saved {yaml_path} {pgm_path}')
    print(f'size {width}x{height} origin=({origin_x:.3f}, {origin_y:.3f}) resolution={args.resolution}')


if __name__ == '__main__':
    main()
