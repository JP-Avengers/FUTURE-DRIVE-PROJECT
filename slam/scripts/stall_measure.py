#!/usr/bin/env python3
"""주차칸 측정. 영역의 흰 선을 두 방향으로 본다:
  주방향(가장 강한 평행선 무리)과 그 수직방향(±15° 탐색). 각 방향의 투영 히스토그램 피크 간격을 출력.
  긴 앞뒤선 간격 = 칸 길이, 짧은 칸막이선 간격 = 칸 폭.
사용: stall_measure.py <run> <xmin> <xmax> <ymin> <ymax> [name]"""
import sys
from pathlib import Path
import numpy as np
run = sys.argv[1]; x0, x1, y0, y1 = map(float, sys.argv[2:6]); name = sys.argv[6] if len(sys.argv) > 6 else 'region'
XY = np.load(Path.home() / f'b_slam/exports/{run}/paint_xy.npy')
R = XY[(XY[:, 0] > x0) & (XY[:, 0] < x1) & (XY[:, 1] > y0) & (XY[:, 1] < y1)]
bw = 0.04
def proj(th):
    n = np.array([np.cos(th), np.sin(th)]); return R @ n, R @ np.array([-n[1], n[0]])
def peaky(th):
    p, _ = proj(th); h, _ = np.histogram(p, bins=np.arange(p.min(), p.max() + bw, bw)); return (h.astype(float) ** 2).sum()
def lines(th, label):
    p, q = proj(th)
    h, e = np.histogram(p, bins=np.arange(p.min(), p.max() + bw, bw)); c = (e[:-1] + e[1:]) / 2
    hs = np.convolve(h, [1, 2, 1], 'same') / 4
    cand = [i for i in range(1, len(hs) - 1) if hs[i] >= hs[i - 1] and hs[i] >= hs[i + 1] and hs[i] > 0.3 * hs.max()]
    peaks = []
    for i in sorted(cand, key=lambda i: -hs[i]):
        if all(abs(c[i] - c[j]) > 1.5 for j in peaks): peaks.append(i)
    pos = []
    for i in sorted(peaks):
        m = abs(p - c[i]) < 0.10; pos.append((p[m].mean(), np.percentile(q[m], 95) - np.percentile(q[m], 5), m.sum()))
    print(f'  [{label}] 법선 {np.degrees(th) % 180:.1f}°, 선 {len(pos)}개')
    if not pos: return
    pos = np.array(pos); gaps = np.diff(pos[:, 0])
    print('    선 위치(m):', ' '.join(f'{v:.2f}' for v in pos[:, 0]))
    print('    선 길이(m):', ' '.join(f'{v:.1f}' for v in pos[:, 1]), ' 점수:', ' '.join(str(int(v)) for v in pos[:, 2]))
    if len(gaps): print('    인접 간격(m):', ' '.join(f'{g:.2f}' for g in gaps))
ths = np.radians(np.arange(0, 180, 0.5)); main = ths[np.argmax([peaky(t) for t in ths])]
perp = main + np.pi / 2; ths2 = perp + np.radians(np.arange(-15, 15.5, 0.5)); perp = ths2[np.argmax([peaky(t) for t in ths2])]
print(f'[{name}] 점 {len(R)}개, 두 방향 사이각 {np.degrees(perp - main):.1f}°')
lines(main, '주방향'); lines(perp, '수직방향')
