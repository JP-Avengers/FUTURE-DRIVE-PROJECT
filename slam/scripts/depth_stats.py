#!/usr/bin/env python3
"""Depth/RGB 품질 샘플링: 유효 depth 비율, 거리 분포, RGB 밝기. 사용: depth_stats.py <bag> [every_n=20]"""
import sys
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
bag = Path(sys.argv[1]); N = int(sys.argv[2]) if len(sys.argv) > 2 else 20
NS = '/camera/camera'
with AnyReader([bag]) as r:
    t0 = None; i = {'d': 0, 'c': 0}
    conns = [c for c in r.connections if c.topic in (f'{NS}/depth/image_rect_raw', f'{NS}/color/image_raw')]
    print(' t[s] | depth valid% | p10/p50/p90 m | >10m% || rgb mean/std | sat%')
    row = {}
    for c, t, raw in r.messages(connections=conns):
        k = 'd' if 'depth' in c.topic else 'c'
        i[k] += 1
        if i[k] % N: continue
        m = r.deserialize(raw, c.msgtype)
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t0 = t0 or ts
        if k == 'd':
            d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(float) / 1000
            v = d[d > 0]
            row['d'] = (ts - t0, 100 * v.size / d.size, *(np.percentile(v, [10, 50, 90]) if v.size else (0, 0, 0)), 100 * (v > 10).mean() if v.size else 0)
        else:
            g = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3).mean(2)
            row['c'] = (g.mean(), g.std(), 100 * ((g > 250) | (g < 5)).mean())
        if 'd' in row and 'c' in row:
            d, cc = row.pop('d'), row.pop('c')
            print(f'{d[0]:5.0f} | {d[1]:6.1f} | {d[2]:4.1f}/{d[3]:4.1f}/{d[4]:4.1f} | {d[5]:5.1f} || {cc[0]:5.1f}/{cc[1]:5.1f} | {cc[2]:4.1f}')
