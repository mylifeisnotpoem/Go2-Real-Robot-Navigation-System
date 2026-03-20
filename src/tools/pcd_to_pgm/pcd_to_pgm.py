#!/usr/bin/env python3
"""
Offline PCD -> PGM converter for ROS1 map_server compatibility.

参考 ROS2 版本 /Volumes/ssd/back/zs/zs-slam_v1/src/pcd2pgm，提供：
- PCD 读取
- 可选 Z 轴截取与半径离群滤波
- 占用栅格化、膨胀
- 输出 map.pgm + map.yaml（occupied_thresh/free_thresh 兼容 map_server）
- 支持多文件目录批处理（多楼层预留）

默认参数：resolution=0.05m, inflate=1, occupied_thresh=0.65, free_thresh=0.2
"""
import argparse
import math
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import numpy as np
import yaml

try:
    import open3d as o3d  # type: ignore
except ImportError:
    o3d = None


def _require_open3d():
    if o3d is None:
        raise RuntimeError("依赖 open3d，请先安装：pip install open3d")


@dataclass
class Params:
    pcd: Path
    out_dir: Path
    resolution: float = 0.05
    height_min: float = -np.inf
    height_max: float = np.inf
    inflate: int = 1
    radius: float = 0.0
    radius_min_neighbors: int = 0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.2
    origin_z: float = 0.0
    transform_xyzrpy: Tuple[float, float, float, float, float, float] = (
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def load_pcd_points(pcd_path: Path) -> np.ndarray:
    _require_open3d()
    pcd = o3d.io.read_point_cloud(str(pcd_path))
    if pcd.is_empty():
        raise RuntimeError(f"PCD 空文件: {pcd_path}")
    return np.asarray(pcd.points)


def apply_transform(points: np.ndarray, xyzrpy: Tuple[float, float, float, float, float, float]) -> np.ndarray:
    tx, ty, tz, rr, rp, ry = xyzrpy
    # Rotation matrices
    cx, sx = math.cos(rr), math.sin(rr)
    cy, sy = math.cos(rp), math.sin(rp)
    cz, sz = math.cos(ry), math.sin(ry)

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    R = Rz @ Ry @ Rx
    t = np.array([tx, ty, tz])
    return (R @ points.T).T + t


def passthrough_filter(points: np.ndarray, z_min: float, z_max: float) -> np.ndarray:
    mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
    return points[mask]


def radius_outlier_filter(points: np.ndarray, radius: float, min_neighbors: int) -> np.ndarray:
    if radius <= 0 or min_neighbors <= 0:
        return points
    _require_open3d()
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    _, ind = pcd.remove_radius_outlier(nb_points=min_neighbors, radius=radius)
    return points[ind]


def compute_bounds_xy(points: np.ndarray) -> Tuple[float, float, float, float]:
    x_min, y_min = np.min(points[:, 0]), np.min(points[:, 1])
    x_max, y_max = np.max(points[:, 0]), np.max(points[:, 1])
    return x_min, x_max, y_min, y_max


def points_to_grid(points: np.ndarray, resolution: float, inflate: int) -> Tuple[np.ndarray, Tuple[float, float]]:
    x_min, x_max, y_min, y_max = compute_bounds_xy(points)
    width = int(math.ceil((x_max - x_min) / resolution)) + 1
    height = int(math.ceil((y_max - y_min) / resolution)) + 1

    grid = np.zeros((height, width), dtype=np.uint8)  # 0=free, 100=occupied, 255=unknown
    # unknown 初始化为 255 符合 map_server 读取，但最终 PGM 会重新映射
    grid.fill(255)

    xi = np.floor((points[:, 0] - x_min) / resolution).astype(np.int32)
    yi = np.floor((points[:, 1] - y_min) / resolution).astype(np.int32)

    grid[yi, xi] = 100

    if inflate > 0:
        grid = inflate_grid(grid, inflate)

    origin = (x_min, y_min)
    return grid, origin


def inflate_grid(grid: np.ndarray, radius: int) -> np.ndarray:
    padded = np.pad(grid, radius, constant_values=255)
    h, w = grid.shape
    inflated = grid.copy()
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            if dx == 0 and dy == 0:
                continue
            sub = padded[radius + dy : radius + dy + h, radius + dx : radius + dx + w]
            inflated = np.minimum(inflated, sub)
    return inflated


def grid_to_pgm(grid: np.ndarray, out_path: Path) -> None:
    # map_server: occupied=0, free=254, unknown=205
    pgm_grid = np.full_like(grid, 205, dtype=np.uint8)
    pgm_grid[grid == 100] = 0
    pgm_grid[grid == 255] = 205
    pgm_grid[(grid != 100) & (grid != 255)] = 254

    h, w = pgm_grid.shape
    header = f"P5\n{w} {h}\n255\n"
    with open(out_path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(np.flipud(pgm_grid).tobytes())  # flip Y so map origin at bottom-left


def write_yaml(yaml_path: Path, pgm_path: Path, resolution: float, origin: Tuple[float, float], origin_z: float, occupied_thresh: float, free_thresh: float) -> None:
    data = {
        "image": pgm_path.name,
        "resolution": float(resolution),
        "origin": [float(origin[0]), float(origin[1]), float(origin_z)],
        "negate": 0,
        "occupied_thresh": float(occupied_thresh),
        "free_thresh": float(free_thresh),
    }
    with open(yaml_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, default_flow_style=False)


def process_single(params: Params) -> Path:
    points = load_pcd_points(params.pcd)
    points = apply_transform(points, params.transform_xyzrpy)
    points = passthrough_filter(points, params.height_min, params.height_max)
    points = radius_outlier_filter(points, params.radius, params.radius_min_neighbors)

    if points.shape[0] == 0:
        raise RuntimeError(f"过滤后点云为空: {params.pcd}")

    grid, origin = points_to_grid(points, params.resolution, params.inflate)
    params.out_dir.mkdir(parents=True, exist_ok=True)
    pgm_path = params.out_dir / "map.pgm"
    yaml_path = params.out_dir / "map.yaml"
    grid_to_pgm(grid, pgm_path)
    write_yaml(yaml_path, pgm_path, params.resolution, origin, params.origin_z, params.occupied_thresh, params.free_thresh)
    return yaml_path


def process_directory(pcd_dir: Path, out_dir: Path, base_params: Params) -> List[Path]:
    outputs: List[Path] = []
    pcd_files = sorted(p for p in pcd_dir.iterdir() if p.suffix.lower() == ".pcd")
    if not pcd_files:
        raise RuntimeError(f"目录下没有 PCD 文件: {pcd_dir}")
    index = []
    for idx, pcd_path in enumerate(pcd_files):
        floor_id = idx
        floor_dir = out_dir / f"floor_{floor_id}"
        yaml_path = process_single(
            Params(
                pcd=pcd_path,
                out_dir=floor_dir,
                resolution=base_params.resolution,
                height_min=base_params.height_min,
                height_max=base_params.height_max,
                inflate=base_params.inflate,
                radius=base_params.radius,
                radius_min_neighbors=base_params.radius_min_neighbors,
                occupied_thresh=base_params.occupied_thresh,
                free_thresh=base_params.free_thresh,
                origin_z=base_params.origin_z,
                transform_xyzrpy=base_params.transform_xyzrpy,
            )
        )
        outputs.append(yaml_path)
        index.append(
            {
                "floor": floor_id,
                "pcd": str(pcd_path.name),
                "map": str(yaml_path.relative_to(out_dir)),
            }
        )
    index_yaml = out_dir / "index.yaml"
    with open(index_yaml, "w", encoding="utf-8") as f:
        yaml.safe_dump(index, f, default_flow_style=False)
    return outputs


def parse_args(argv: Optional[Iterable[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Offline PCD -> PGM converter (ROS1 map_server compatible)")
    parser.add_argument("--pcd", type=Path, required=True, help="PCD 文件路径或目录（目录则批处理，多楼层输出）")
    parser.add_argument("--out", type=Path, required=True, help="输出目录")
    parser.add_argument("--resolution", type=float, default=0.05, help="栅格分辨率，米")
    parser.add_argument("--height_min", type=float, default=-np.inf, help="Z 下界")
    parser.add_argument("--height_max", type=float, default=np.inf, help="Z 上界")
    parser.add_argument("--inflate", type=int, default=1, help="膨胀半径，单位：cell")
    parser.add_argument("--radius", type=float, default=0.0, help="半径离群滤波的半径，<=0 则跳过")
    parser.add_argument("--radius_min_neighbors", type=int, default=0, help="半径滤波最少邻居数，<=0 跳过")
    parser.add_argument("--occupied_thresh", type=float, default=0.65, help="YAML occupied_thresh")
    parser.add_argument("--free_thresh", type=float, default=0.2, help="YAML free_thresh")
    parser.add_argument("--origin_z", type=float, default=0.0, help="YAML 原点 z")
    parser.add_argument(
        "--transform",
        type=float,
        nargs=6,
        default=(0, 0, 0, 0, 0, 0),
        metavar=("x", "y", "z", "roll", "pitch", "yaw"),
        help="先旋转后平移（米/弧度），与 ROS2 组件 odom_to_lidar_odom 一致",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = parse_args(argv)
    pcd_path: Path = args.pcd
    out_dir: Path = args.out

    base_params = Params(
        pcd=pcd_path,
        out_dir=out_dir,
        resolution=args.resolution,
        height_min=args.height_min,
        height_max=args.height_max,
        inflate=args.inflate,
        radius=args.radius,
        radius_min_neighbors=args.radius_min_neighbors,
        occupied_thresh=args.occupied_thresh,
        free_thresh=args.free_thresh,
        origin_z=args.origin_z,
        transform_xyzrpy=tuple(args.transform),
    )
    try:
        if pcd_path.is_dir():
            process_directory(pcd_path, out_dir, base_params)
        else:
            process_single(base_params)
    except Exception as e:  # pylint: disable=broad-except
        print(f"[pcd_to_pgm] 失败: {e}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())

