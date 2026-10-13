#!/usr/bin/env python3
"""촬영 조건 점검: IMU로 카메라 피치·회전 방향·회전 속도, 일정 간격 RGB 프레임 contact sheet.
사용: capture_audit.py <bag> <out_prefix> [frame_every_s=10]"""
import sys
from pathlib import Path
import numpy as np
import open3d as o3d
from rosbags.highlevel import AnyReader

bag, out = Path(sys.argv[1]), sys.argv[2]
every = float(sys.argv[3]) if len(sys.argv) > 3 else 10
NS = '/camera/camera'
G, A, frames = [], [], []
with AnyReader([bag]) as r:
    conns = [c for c in r.connections if c.topic in (f'{NS}/gyro/sample', f'{NS}/accel/sample', f'{NS}/color/image_raw')]
    t0 = None; next_f = 0
    for c, t, raw in r.messages(connections=conns):
        m = r.deserialize(raw, c.msgtype)
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t0 = t0 or ts; ts -= t0
        if 'gyro' in c.topic:
            v = m.angular_velocity; G.append((ts, v.x, v.y, v.z))
        elif 'accel' in c.topic:
            v = m.linear_acceleration; A.append((ts, v.x, v.y, v.z))
        elif ts >= next_f:
            img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)[::2, ::2]
            frames.append((ts, img.copy())); next_f += every
G, A = np.array(G), np.array(A)

# 1초 창 평균 accel → 위쪽 단위벡터 (specific force는 정지 시 위를 향함)
sec = np.arange(0, A[-1, 0], 1.0)
up = np.array([A[(A[:, 0] >= s) & (A[:, 0] < s + 1), 1:].mean(0) for s in sec])
up /= np.linalg.norm(up, axis=1, keepdims=True)
pitch = np.degrees(np.arcsin(up[:, 2]))  # optical z(전방)의 수평 대비 각도, +면 위를 향함
roll = np.degrees(np.arcsin(up[:, 0]))
# 위축 기준 yaw rate (+ = 위에서 볼 때 반시계)
idx = np.clip(np.searchsorted(sec, G[:, 0]) - 1, 0, len(sec) - 1)
yr = np.degrees((G[:, 1:] * up[idx]).sum(1))
dt = np.diff(G[:, 0], prepend=G[0, 0])
yaw = np.cumsum(yr * dt)
# 0.5초 이동평균 yaw rate
k = 100; yr_s = np.convolve(yr, np.ones(k) / k, 'same')

print(f'duration {G[-1,0]:.1f}s')
print(f'pitch(+위) mean {pitch.mean():.1f}°, p10/p50/p90 {np.percentile(pitch,10):.1f}/{np.median(pitch):.1f}/{np.percentile(pitch,90):.1f}°')
print(f'roll mean {roll.mean():.1f}°, |roll| p90 {np.percentile(abs(roll),90):.1f}°')
print(f'total yaw {yaw[-1]:.0f}° (+반시계/-시계)')
print(f'|yaw rate| 0.5s평균 p50/p90/p99/max {np.percentile(abs(yr_s),50):.1f}/{np.percentile(abs(yr_s),90):.1f}/{np.percentile(abs(yr_s),99):.1f}/{abs(yr_s).max():.1f} °/s')
for th in (20, 30, 45):
    print(f'  >{th}°/s 구간 시간: {(abs(yr_s) > th).sum() / 200:.1f}s')
# 회전 이벤트: |yr_s|>10°/s 연속 구간에서 누적 >45°
ev, s = [], None
for i, v in enumerate(abs(yr_s) > 10):
    if v and s is None: s = i
    if not v and s is not None:
        ang = yaw[i] - yaw[s]
        if abs(ang) > 45: ev.append((G[s, 0], G[i, 0], ang, abs(yr_s[s:i]).max()))
        s = None
print('주요 회전(>45°): 시작s 끝s 각도 최대°/s')
for e in ev: print(f'  {e[0]:6.1f} {e[1]:6.1f} {e[2]:+6.0f} {e[3]:5.1f}')
print('yaw 누적(20s 간격):', ' '.join(f'{yaw[np.searchsorted(G[:,0], s)]:.0f}' for s in np.arange(0, G[-1,0], 20)))

# contact sheet
cols = 4; h, w = frames[0][1].shape[:2]
rows = -(-len(frames) // cols)
sheet = np.zeros((rows * h, cols * w, 3), np.uint8)
for j, (ts, im) in enumerate(frames):
    sheet[(j // cols) * h:(j // cols + 1) * h, (j % cols) * w:(j % cols + 1) * w] = im
o3d.io.write_image(out + '_sheet.png', o3d.geometry.Image(sheet))
print('frames at:', ' '.join(f'{f[0]:.0f}' for f in frames))
