"""RGB-D 프레임 + 카메라 pose 읽기. 입력 형식마다 reader 를 두고, 모두 같은 Frames 를 돌려준다.

pose 는 **카메라 광학 좌표계(x 오른쪽 · y 아래 · z 앞) → world** 4x4 행렬.
RTAB-Map 에서 꺼낼 때는 `rtabmap-export --poses_camera` (로봇 base_link 기준인 --poses 가 아님).
"""
import os
from dataclasses import dataclass

import numpy as np
import open3d as o3d


@dataclass
class Frames:
    color: list          # 컬러 이미지 경로
    depth: list          # 깊이 이미지 경로 (16bit PNG)
    poses: list          # 4x4 np.ndarray, 카메라 → world
    intrinsic: o3d.camera.PinholeCameraIntrinsic
    depth_scale: float   # 깊이 PNG 값 / depth_scale = m  (mm 단위 PNG 면 1000)


def read_log(path):
    """Redwood .log 궤적: 5줄 단위(헤더 1줄 + 4x4 행렬)."""
    lines = open(path, encoding="utf-8").read().strip().split("\n")
    return [np.array([list(map(float, l.split())) for l in lines[i + 1:i + 5]]) for i in range(0, len(lines), 5)]


def read_tum_list(path):
    """`timestamp 값 ...` 형식 텍스트 (# 주석·빈 줄 무시) → (시각 배열, 나머지 칸 목록)."""
    stamps, rest = [], []
    for line in open(path, encoding="utf-8"):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        stamps.append(float(parts[0]))
        rest.append(parts[1:])
    return np.array(stamps), rest


def tum_to_matrix(t, q):
    """TUM pose (tx ty tz, qx qy qz qw) → 4x4."""
    x, y, z, w = q
    T = np.eye(4)
    T[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion([w, x, y, z])
    T[:3, 3] = t
    return T


def nearest(src, dst, max_dt):
    """src 시각마다 dst(오름차순)에서 가장 가까운 인덱스. max_dt 초를 넘으면 -1."""
    idx = np.clip(np.searchsorted(dst, src), 1, len(dst) - 1)
    idx = np.where(np.abs(src - dst[idx - 1]) <= np.abs(dst[idx] - src), idx - 1, idx)
    return np.where(np.abs(dst[idx] - src) <= max_dt, idx, -1)


def load_open3d_lounge(cfg):
    """Open3D 샘플 Lounge (PrimeSense, 640x480). 파이프라인 시험용."""
    data = o3d.data.LoungeRGBDImages()
    color, depth = sorted(data.color_paths), sorted(data.depth_paths)
    poses = read_log(data.trajectory_log_path)
    n = min(len(color), len(depth), len(poses))
    intr = o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    return Frames(color[:n], depth[:n], poses[:n], intr, cfg.get("depth_scale", 1000.0))


def load_tum(cfg):
    """TUM RGB-D 형식: rgb.txt · depth.txt (`시각 파일경로`) + pose 파일 (`시각 tx ty tz qx qy qz qw`).

    컬러 시각마다 가장 가까운 깊이·pose 를 max_dt 초 안에서 짝짓는다.
    """
    root = os.path.expanduser(cfg["root"])
    rs, rgb = read_tum_list(os.path.join(root, cfg.get("rgb_list", "rgb.txt")))
    ds, dep = read_tum_list(os.path.join(root, cfg.get("depth_list", "depth.txt")))
    ps, pv = read_tum_list(os.path.join(root, cfg["poses"]))
    d_ord, p_ord = np.argsort(ds), np.argsort(ps)
    max_dt = cfg.get("max_dt", 0.02)
    di = nearest(rs, ds[d_ord], max_dt)
    pi = nearest(rs, ps[p_ord], max_dt)
    keep = np.where((di >= 0) & (pi >= 0))[0]
    print(f"TUM: 컬러 {len(rs)} | 깊이 {len(ds)} | pose {len(ps)} → 짝 맞은 프레임 {len(keep)} (허용 {max_dt}s)")

    k = cfg["intrinsics"]
    intr = o3d.camera.PinholeCameraIntrinsic(k["width"], k["height"], k["fx"], k["fy"], k["cx"], k["cy"])
    color = [os.path.join(root, rgb[i][0]) for i in keep]
    depth = [os.path.join(root, dep[d_ord[di[i]]][0]) for i in keep]
    poses = []
    for i in keep:
        v = list(map(float, pv[p_ord[pi[i]]]))
        poses.append(tum_to_matrix(v[0:3], v[3:7]))
    return Frames(color, depth, poses, intr, cfg.get("depth_scale", 1000.0))


LOADERS = {"open3d_lounge": load_open3d_lounge, "tum": load_tum}


def load(cfg):
    fmt = cfg["format"]
    if fmt not in LOADERS:
        raise SystemExit(f"input.format '{fmt}' 은 지원하지 않음 — {list(LOADERS)}")
    return LOADERS[fmt](cfg)
