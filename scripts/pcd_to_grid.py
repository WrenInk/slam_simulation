#!/usr/bin/env python3
"""Convert a 3D PCD point cloud into 2D Nav2 Occupancy Grid Map (.yaml + .pgm)."""
import argparse
from pathlib import Path

from nav_stack.map_io import DEFAULT_FREE_THRESH, DEFAULT_OCCUPIED_THRESH
from nav_stack.terrain import pcd_to_occupancy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', type=Path, default=Path('maps/map.pcd'), help='输入 PCD 文件路径')
    parser.add_argument('--output-dir', type=Path, default=None, help='输出目录（默认与 PCD 同目录）')
    parser.add_argument('--resolution', type=float, default=0.05, help='栅格地图分辨率 (米)')
    parser.add_argument('--name', default='map', help='输出地图文件名（不含扩展名，默认 map）')
    parser.add_argument('--occupied-thresh', type=float, default=DEFAULT_OCCUPIED_THRESH,
                        help='图像加载时的占据概率阈值（默认 0.65）')
    parser.add_argument('--free-thresh', type=float, default=DEFAULT_FREE_THRESH,
                        help='图像加载时的空闲概率阈值（默认 0.196）')
    args = parser.parse_args()
    try:
        yaml_path, pgm_path = pcd_to_occupancy(
            args.pcd, args.output_dir, args.resolution, args.name,
            occupied_thresh=args.occupied_thresh, free_thresh=args.free_thresh)
    except (OSError, ValueError) as exc:
        parser.error(str(exc))
    print(f'[PCD -> Grid] 栅格地图转换成功：\n  YAML: {yaml_path}\n  PGM: {pgm_path}')


if __name__ == '__main__':
    main()
