#!/usr/bin/env python3
"""근접 촬영 확인: 프레임마다 화면 아래 절반(raw depth)의 가까운 거리(5% 분위)를 계산하고,
가장 가까웠던 장면들의 컬러+depth를 나란히 이어 붙인 PNG를 만든다.
사용: near_check.py <bag> <out_prefix> [every_s=0.5] [n_show=8]"""
import sys
from pathlib import Path
import numpy as np, open3d as o3d
from rosbags.highlevel import AnyReader
bag, out = Path(sys.argv[1]), sys.argv[2]
every = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5; nshow = int(sys.argv[4]) if len(sys.argv) > 4 else 8
NS = '/camera/camera'
frames = {}; last = {'d': -1e9, 'c': -1e9}; t0 = None
with AnyReader([bag]) as r:
    conns = [c for c in r.connections if c.topic in (f'{NS}/depth/image_rect_raw', f'{NS}/color/image_raw')]
    for c, t, raw in r.messages(connections=conns):
        k = 'd' if 'depth' in c.topic else 'c'
        ts = t * 1e-9; t0 = t0 or ts
        if ts - last[k] < every: continue
        last[k] = ts; m = r.deserialize(raw, c.msgtype)
        key = round((ts - t0) / every)
        if k == 'd':
            frames.setdefault(key, {})['d'] = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()
        else:
            frames.setdefault(key, {})['c'] = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).copy()
rows = []
for key, f in sorted(frames.items()):
    if 'd' not in f or 'c' not in f: continue
    d = f['d'][240:, :].astype(float) / 1000; v = d[d > 0]
    if v.size < 1000: continue
    rows.append((key * every, np.percentile(v, 5), np.percentile(v, 50), key))
A = np.array(rows)
print(f'프레임 {len(A)}개 (raw depth 아래 절반)')
print(f'  5% 분위 거리: 최소 {A[:,1].min():.2f}m, 10% {np.percentile(A[:,1],10):.2f}m, 중앙 {np.median(A[:,1]):.2f}m')
print(f'  2m 미만 프레임 {(A[:,1]<2).sum()}개, 3m 미만 {(A[:,1]<3).sum()}개')
pick = A[np.argsort(A[:, 1])[:nshow]]
pick = pick[np.argsort(pick[:, 0])]
tiles = []
for t, p5, p50, key in pick:
    f = frames[int(key)]; c = f['c'][::2, ::2]
    d = f['d'].astype(float) / 1000; dn = np.clip(d / 8.0, 0, 1)  # 0~8m 흑→백, 0(없음)=빨강
    dimg = (np.stack([dn] * 3, -1) * 255).astype(np.uint8); dimg[d == 0] = (180, 0, 0); dimg = dimg[::2, ::2]
    tiles.append(np.hstack([c, dimg]))
    print(f'  t={t:6.1f}s  가까운 거리(5%) {p5:.2f}m  중앙 {p50:.2f}m')
o3d.io.write_image(out + '_near.png', o3d.geometry.Image(np.ascontiguousarray(np.vstack(tiles))))
print(f'저장: {out}_near.png (왼쪽 컬러, 오른쪽 raw depth 0~8m 흑→백, 빨강=값 없음)')
