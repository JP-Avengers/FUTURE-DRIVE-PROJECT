#!/usr/bin/env python3
"""벽 거리 보정 녹화 분석: D435i 원본 depth가 줄자 거리보다 얼마나 길게/짧게 나오는지 거리별로 재고 보정식을 맞춘다.
각 bag(벽을 정면으로 보고 10초 정지)에서 원본 depth 화면 가운데 영역의 벽 평면까지 거리(평면 맞춤, 법선 방향)를 구해
줄자 거리(렌즈 → 벽)와 비교한다. 정렬 depth가 있으면 같은 방식으로 함께 본다.

사용: wall_depth_fit.py <bag_dir>:<줄자거리m> [<bag_dir>:<m> ...]
  예) wall_depth_fit.py ~/data/wall/wall_3m/bag:3.00 ~/data/wall/wall_5m/bag:5.02 ...
출력: 거리별 측정값·오차(%), 그리고 두 모델의 계수
  (1) 현재 쓰는 모델  Z_meas = Z_true · (1 + c·Z_true)   → 보정 Z' = Z / (1 + c·Z)   (c>0: 멀수록 길게 나옴)
  (2) 선형 모델       Z_meas = a·Z_true + b
(~/slam_env 또는 ~/tsdf_env 로 실행)"""
import sys, os
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader

NS = '/camera/camera'
TOPICS = {f'{NS}/depth/image_rect_raw': '원본', f'{NS}/aligned_depth_to_color/image_raw': '정렬'}
INFO = {f'{NS}/depth/camera_info': '원본', f'{NS}/color/camera_info': '정렬'}


def plane_dist(d, K, roi=0.25):
    """화면 가운데 roi 비율 영역의 depth로 평면을 맞춰, 카메라 원점에서 평면까지 수직 거리(m)."""
    h, w = d.shape
    v0, v1, u0, u1 = int(h * (0.5 - roi / 2)), int(h * (0.5 + roi / 2)), int(w * (0.5 - roi / 2)), int(w * (0.5 + roi / 2))
    z = d[v0:v1, u0:u1].astype(float) / 1000
    vv, uu = np.mgrid[v0:v1, u0:u1]
    m = z > 0.2
    if m.sum() < 500:
        return np.nan, np.nan
    P = np.c_[(uu[m] - K[2]) * z[m] / K[0], (vv[m] - K[5]) * z[m] / K[4], z[m]]
    c = np.median(P, 0)
    for _ in range(3):  # 평면 맞춤 + 이상치 제거
        n = np.linalg.svd(P - c, full_matrices=False)[2][-1]
        r = (P - c) @ n
        keep = np.abs(r) < max(3 * np.median(np.abs(r)), 0.01)
        P = P[keep]
        c = P.mean(0)
    n = np.linalg.svd(P - c, full_matrices=False)[2][-1]
    return abs(c @ n), np.degrees(np.arccos(abs(n[2])))  # 거리, 벽 법선과 광축 사이 각도


rows = []
for arg in sys.argv[1:]:
    bag, dist = arg.rsplit(':', 1)
    true = float(dist)
    res = {k: [] for k in TOPICS.values()}
    K = {}
    with AnyReader([Path(bag)]) as r:
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic in INFO]):
            K.setdefault(INFO[c.topic], r.deserialize(raw, c.msgtype).k)
            if len(K) == 2:
                break
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic in TOPICS]):
            lab = TOPICS[c.topic]
            if lab not in K:
                continue
            m = r.deserialize(raw, c.msgtype)
            d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
            res[lab].append(plane_dist(d, K[lab]))
    for f in Path(bag).glob('*.mcap'):
        fd = os.open(f, os.O_RDONLY); os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED); os.close(fd)
    for lab, v in res.items():
        v = np.array([x for x in v if not np.isnan(x[0])])
        if len(v) == 0:
            continue
        meas, ang = np.median(v[:, 0]), np.median(v[:, 1])
        rows.append((lab, true, meas, np.std(v[:, 0]), ang, len(v)))

print('종류  줄자(m)  depth(m)  오차(%)   흔들림(cm)  벽 기울기(°)  프레임')
for lab, true, meas, sd, ang, n in rows:
    print(f'{lab}  {true:7.3f}  {meas:8.3f}  {100*(meas/true-1):+6.2f}   {sd*100:8.1f}   {ang:10.1f}  {n:6d}')
for lab in ('원본', '정렬'):
    R = np.array([(t, m) for l, t, m, *_ in rows if l == lab])
    if len(R) < 2:
        continue
    T, M = R[:, 0], R[:, 1]
    c = np.sum((M / T - 1) * T) / np.sum(T * T)          # M/T - 1 = c·T 의 최소제곱
    a, b = np.polyfit(T, M, 1)
    rc = M - T * (1 + c * T); rl = M - (a * T + b)
    print(f'\n[{lab}] 모델 (1) Z_meas = Z(1 + c·Z): c = {c:+.4f}/m  (잔차 RMS {np.sqrt(np.mean(rc**2))*100:.1f}cm)'
          f'  → 지금 보정 계수 0.007과 {"같은 방향" if c > 0 else "반대 방향"}')
    print(f'[{lab}] 모델 (2) Z_meas = a·Z + b: a = {a:.4f}, b = {b*100:+.1f}cm  (잔차 RMS {np.sqrt(np.mean(rl**2))*100:.1f}cm)')
