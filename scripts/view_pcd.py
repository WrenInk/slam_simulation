#!/usr/bin/env python3
"""Interactive 3D PCD Point Cloud Viewer for SLAM Maps.

Supports elevation-based color rendering, coordinate frames, point size adjustment,
voxel filtering, and detailed point cloud statistics.
"""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import numpy as np

try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

try:
    import matplotlib.cm as cm
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False


def find_default_pcd(workspace_root: Path) -> Path:
    candidates = [
        workspace_root / "maps/map.pcd",
        workspace_root / "artifacts/mapping/slam/map.pcd",
    ]
    for c in candidates:
        if c.is_file():
            return c
    # Fallback to any recent .pcd in artifacts
    found = sorted(workspace_root.glob("artifacts/**/*.pcd"))
    if found:
        return found[-1]
    return workspace_root / "maps/map.pcd"


def print_pcd_stats(pcd_path: Path, points: np.ndarray):
    file_size_kb = pcd_path.stat().st_size / 1024.0
    file_size_str = f"{file_size_kb / 1024.0:.2f} MB" if file_size_kb > 1024 else f"{file_size_kb:.1f} KB"

    min_b = points.min(axis=0)
    max_b = points.max(axis=0)
    dim = max_b - min_b
    center = points.mean(axis=0)

    print("\n" + "=" * 62)
    print(f"               PCD 点云地图信息概览")
    print("=" * 62)
    print(f" 文件路径: {pcd_path.resolve()}")
    print(f" 文件大小: {file_size_str}")
    print(f" 点云总数: {len(points):,} 点")
    print(f" 空间范围:")
    print(f"   X 范围: [{min_b[0]:.3f}, {max_b[0]:.3f}] m  (跨度: {dim[0]:.3f} m)")
    print(f"   Y 范围: [{min_b[1]:.3f}, {max_b[1]:.3f}] m  (跨度: {dim[1]:.3f} m)")
    print(f"   Z 范围: [{min_b[2]:.3f}, {max_b[2]:.3f}] m  (跨度: {dim[2]:.3f} m)")
    print(f" 点云质心: ({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}) m")
    print("=" * 62)


def view_open3d(pcd_path: Path, color_mode="elevation", point_size=3.0,
                voxel_size=None, show_axes=True):
    if not HAS_OPEN3D:
        print("[错误] 未安装 open3d，请使用 --backend pcl 或在虚拟环境中安装 open3d。")
        return False

    print(f"[PCD Viewer] 正在加载: {pcd_path} ...")
    pcd = o3d.io.read_point_cloud(str(pcd_path))
    if pcd.is_empty():
        print(f"[错误] 点云文件为空或格式不受支持: {pcd_path}")
        return False

    points = np.asarray(pcd.points)
    print_pcd_stats(pcd_path, points)

    if voxel_size and voxel_size > 0:
        print(f"[PCD Viewer] 执行体素降采样 (voxel_size = {voxel_size} m)...")
        pcd = pcd.voxel_down_sample(voxel_size)
        points = np.asarray(pcd.points)
        print(f"[PCD Viewer] 降采样后点数: {len(points):,} 点")

    # Color mapping
    if HAS_MATPLOTLIB and color_mode in ["elevation", "z", "turbo", "viridis", "jet"]:
        z = points[:, 2]
        z_min, z_max = np.percentile(z, 1), np.percentile(z, 99)
        norm_z = np.clip((z - z_min) / (z_max - z_min + 1e-6), 0.0, 1.0)
        cmap_name = "turbo" if color_mode in ["elevation", "z", "turbo"] else color_mode
        cmap = getattr(cm, cmap_name, cm.turbo)
        colors = cmap(norm_z)[:, :3]
        pcd.colors = o3d.utility.Vector3dVector(colors)
    elif color_mode == "uniform":
        pcd.paint_uniform_color([0.2, 0.8, 0.9])

    print("\n[交互快捷键操作指南]:")
    print("  • 鼠标左键按住拖动: 旋转 3D 视角 (Rotate)")
    print("  • 鼠标右键/Shift+左键拖动: 平移视图 (Pan)")
    print("  • 鼠标滚轮: 缩放距离 (Zoom In / Out)")
    print("  • [+] / [-] 键: 增大 / 减小点云点尺寸")
    print("  • [R] 键: 复位视角 (Reset View)")
    print("  • [Q] 或 [Esc]: 退出查看器\n")

    vis = o3d.visualization.Visualizer()
    vis.create_window(
        window_name=f"PCD Viewer - {pcd_path.name} ({len(points):,} pts)",
        width=1280, height=800
    )
    vis.add_geometry(pcd)

    if show_axes:
        # Create XYZ coordinate axes at origin: Red=X, Green=Y, Blue=Z
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        vis.add_geometry(axes)

    render_opt = vis.get_render_option()
    render_opt.point_size = float(point_size)
    render_opt.background_color = np.array([0.12, 0.14, 0.18])  # Dark theme
    render_opt.show_coordinate_frame = False

    vis.run()
    vis.destroy_window()
    return True


def view_pcl(pcd_path: Path):
    pcl_bin = "/usr/bin/pcl_viewer"
    if not os.path.exists(pcl_bin):
        print("[错误] 系统中未找到 pcl_viewer，请安装 pcl-tools 或使用 open3d 后端。")
        return False
    print(f"[PCD Viewer] 启动 pcl_viewer 查看 {pcd_path} ...")
    subprocess.run([pcl_bin, str(pcd_path)])
    return True


def main():
    workspace_root = Path(__file__).resolve().parent.parent
    default_pcd = find_default_pcd(workspace_root)

    parser = argparse.ArgumentParser(description="3D PCD Point Cloud Viewer")
    parser.add_argument("pcd", nargs="?", type=Path, default=default_pcd,
                        help=f"PCD 文件路径 (默认: {default_pcd.relative_to(workspace_root) if default_pcd.is_file() else 'maps/map.pcd'})")
    parser.add_argument("--backend", choices=["open3d", "pcl"], default="open3d",
                        help="渲染后端 (默认: open3d, 可选: pcl)")
    parser.add_argument("--color", choices=["elevation", "turbo", "viridis", "jet", "uniform"],
                        default="elevation", help="点云色彩模式 (默认: elevation 高度渲染)")
    parser.add_argument("--point-size", type=float, default=3.0, help="点粒子显示大小 (默认: 3.0)")
    parser.add_argument("--voxel-size", type=float, default=None, help="体素滤波降采样大小 (米，默认不降采样)")
    parser.add_argument("--no-axes", action="store_true", help="隐藏坐标轴标记")

    args = parser.parse_args()
    pcd_path = Path(args.pcd).resolve()

    if not pcd_path.is_file():
        print(f"[错误] PCD 点云文件不存在: {pcd_path}")
        print("提示: 可先运行建图模式生成地图：./scripts/mapping.sh")
        sys.exit(1)

    if args.backend == "pcl":
        view_pcl(pcd_path)
    else:
        try:
            view_open3d(
                pcd_path,
                color_mode=args.color,
                point_size=args.point_size,
                voxel_size=args.voxel_size,
                show_axes=not args.no_axes
            )
        except Exception as e:
            print(f"[警告] Open3D GUI 初始化遇到问题: {e}")
            print("[提示] 尝试切换至 pcl_viewer 后端启动...")
            view_pcl(pcd_path)


if __name__ == "__main__":
    main()
