#!/usr/bin/env python3
"""현장 QC: python3 field_qc.py <bag_dir>   (필요: pip install rosbags numpy)"""
import sys
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader

NS = '/camera/camera'
want = {f'{NS}/color/image_raw': 'color', f'{NS}/aligned_depth_to_color/image_raw': 'aligned',
        f'{NS}/gyro/sample': 'gyro', f'{NS}/accel/sample': 'accel'}
T = {k: [] for k in want.values()}; G = []; A = []
with AnyReader([Path(sys.argv[1])]) as r:
    conns = [c for c in r.connections if c.topic in want]
    for c, t, raw in r.messages(connections=conns):
        k = want[c.topic]; T[k].append(t * 1e-9)
        if k in ('gyro', 'accel'):
            m = r.deserialize(raw, c.msgtype)
            v = m.angular_velocity if k == 'gyro' else m.linear_acceleration
            (G if k == 'gyro' else A).append((t * 1e-9, v.x, v.y, v.z))
ok = True
def check(name, val, good, msg):
    global ok; ok &= good
    print(f"[{'OK ' if good else 'NG '}] {name}: {val}  {'' if good else '<- ' + msg}")
dur = T['color'][-1] - T['color'][0] if T['color'] else 0
check('길이', f'{dur:.0f}s', 180 <= dur <= 480, '루프 1바퀴는 4~6분')
for k in ('color', 'aligned'):
    n = len(T[k]); hz = max(n - 1, 0) / dur if dur else 0
    gap = np.diff(T[k]).max() if n > 1 else 0
    check(f'{k} Hz', f'{hz:.1f} (최대 간격 {gap*1e3:.0f}ms)', hz >= 27 and gap < 0.2, '30Hz 미달 또는 끊김')
G, A = np.array(G), np.array(A)
check('IMU', f'gyro {len(G)}, accel {len(A)}', len(G) > 150 * dur and len(A) > 80 * dur, 'IMU 누락')
t0 = A[0, 0]; sec = np.arange(t0, A[-1, 0], 1.0)
up = np.array([A[(A[:, 0] >= s) & (A[:, 0] < s + 1), 1:].mean(0) for s in sec])
up /= np.linalg.norm(up, axis=1, keepdims=True)
pitch = np.degrees(np.arcsin(up[:, 2]))
check('피치(+위)', f'평균 {pitch.mean():.0f}°, 10~90% {np.percentile(pitch,10):.0f}~{np.percentile(pitch,90):.0f}°',
      -5 <= np.median(pitch) <= 12, '카메라가 아래를 봄' if np.median(pitch) < -5 else '너무 위를 봄')
idx = np.clip(np.searchsorted(sec, G[:, 0]) - 1, 0, len(sec) - 1)
yr = np.degrees((G[:, 1:] * up[idx]).sum(1))
yr = np.convolve(yr, np.ones(100) / 100, 'same')
fast = (abs(yr) > 30).sum() / 200
check('최대 회전', f'{abs(yr).max():.0f}°/s', abs(yr).max() < 45, '급회전 있음')
check('30°/s 초과 시간', f'{fast:.1f}s', fast < 5, '코너를 더 천천히')
yaw = np.cumsum(yr * np.diff(G[:, 0], prepend=G[0, 0]))
check('총 회전', f'{yaw[-1]:+.0f}° (+반시계)', 250 <= abs(yaw[-1]) <= 500, '한 바퀴(약 360°)가 아님')
print('\n판정:', '합격' if ok else '불합격 항목 확인 후 재촬영 판단')
