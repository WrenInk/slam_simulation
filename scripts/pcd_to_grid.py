#!/usr/bin/env python3
"""Convert a 3D PCD point cloud into 2D Nav2 Occupancy Grid Map (.yaml + .pgm)."""
import argparse
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
from nav_stack.geometry import read_pcd
from nav_stack.terrain import classify, TerrainConfig


def pcd_to_occupancy(pcd_path, output_dir=None, resolution=0.05, name='map',
                      occupied_thresh=0.65, free_thresh=0.25):
    pcd_path = Path(pcd_path).resolve()
    if not pcd_path.is_file():
        raise FileNotFoundError(f"PCD file not found: {pcd_path}")
    output_dir = Path(output_dir or pcd_path.parent).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"[PCD -> Grid] 读取点云文件: {pcd_path}")
    points = read_pcd(pcd_path)
    print(f"[PCD -> Grid] 加载点数: {len(points)}，正在进行地面与障碍物分类...")

    terrain_res = max(resolution, 0.10)
    terrain = classify(points, TerrainConfig(terrain_resolution=terrain_res))
    scale = max(1, int(round(terrain.resolution / resolution)))
    grid = np.repeat(np.repeat(terrain.occupancy, scale, axis=0), scale, axis=1)
    origin = terrain.origin  # [min_x, min_y]
    height, width = grid.shape

    # Map format: -1 -> unknown (205), >50 -> occupied (0), 0 -> free (254)
    image = np.where(grid < 0, 205, np.where(grid > 50, 0, 254)).astype(np.uint8)
    pgm_path = output_dir / f"{name}.pgm"
    yaml_path = output_dir / f"{name}.yaml"

    Image.fromarray(np.flipud(image)).save(pgm_path)
    yaml_content = {
        'image': f"{name}.pgm",
        'resolution': float(resolution),
        'origin': [float(origin[0]), float(origin[1]), 0.0],
        'negate': 0,
        'occupied_thresh': float(occupied_thresh),
        'free_thresh': float(free_thresh)
    }
    yaml_path.write_text(yaml.safe_dump(yaml_content, sort_keys=False))

    print(f"[PCD -> Grid] 栅格地图转换成功！")
    print(f"  - 图像尺寸: {width} x {height} (像素)")
    print(f"  - 分辨率: {resolution} m/cell")
    print(f"  - 原点坐标: ({origin[0]:.2f}, {origin[1]:.2f})")
    print(f"  - PGM 图像: {pgm_path}")
    print(f"  - YAML 配置: {yaml_path}")
    return yaml_path, pgm_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pcd', type=Path, default=Path('maps/map.pcd'), help='输入 PCD 文件路径')
    parser.add_argument('--output-dir', type=Path, default=None, help='输出目录（默认与 PCD 同目录）')
    parser.add_argument('--resolution', type=float, default=0.05, help='栅格地图分辨率 (米)')
    parser.add_argument('--occupied-thresh', type=float, default=0.65, help='障碍物占据判定阈值 (0.0~1.0，默认 0.65)')
    parser.add_argument('--free-thresh', type=float, default=0.25, help='可行走空闲判定阈值 (0.0~1.0，默认 0.25)')
    args = parser.parse_args()

    pcd_to_occupancy(args.pcd, args.output_dir, args.resolution, args.name,
                     occupied_thresh=args.occupied_thresh, free_thresh=args.free_thresh)


if __name__ == '__main__':
    main()
