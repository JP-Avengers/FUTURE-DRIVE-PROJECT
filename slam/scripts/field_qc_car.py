#!/usr/bin/env python3
"""차량 촬영 현장 QC: python3 field_qc_car.py <bag_dir>   (필요: pip install rosbags numpy)"""
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
bad = 0
def check(name, val, level, msg=''):
    global bad; bad += level == 'NG'
    print(f'[{level:<4}] {name}: {val}' + (f'  <- {msg}' if level != 'OK' else ''))
dur = T['color'][-1] - T['color'][0] if T['color'] else 0
check('길이', f'{dur:.0f}s', 'OK' if 70 <= dur <= 240 else 'NG', '차량 루프 1바퀴는 70~240초')
for k in ('color', 'aligned'):
    n = len(T[k]); hz = max(n - 1, 0) / dur if dur else 0
    gap = np.diff(T[k]).max() if n > 1 else 0
    check(f'{k} Hz', f'{hz:.1f} (최대 간격 {gap*1e3:.0f}ms)', 'OK' if hz >= 27 and gap < 0.2 else 'NG', '30Hz 미달 또는 끊김')
G, A = np.array(G), np.array(A)
check('IMU', f'gyro {len(G)}, accel {len(A)}', 'OK' if len(G) > 150 * dur and len(A) > 80 * dur else 'NG', 'IMU 누락')
sec = np.arange(A[0, 0], A[-1, 0], 1.0)
up = np.array([A[(A[:, 0] >= s) & (A[:, 0] < s + 1), 1:].mean(0) for s in sec])
up /= np.linalg.norm(up, axis=1, keepdims=True)
pitch = np.degrees(np.arcsin(up[:, 2])); pm = np.median(pitch)
check('피치(+위)', f'중앙값 {pm:.0f}°, 10~90% {np.percentile(pitch,10):.0f}~{np.percentile(pitch,90):.0f}°',
      'OK' if -5 <= pm <= 12 else 'NG', '카메라가 아래를 봄' if pm < -5 else '너무 위를 봄')
check('피치 변화폭', f'{np.percentile(pitch,90) - np.percentile(pitch,10):.1f}°',
      'OK' if np.percentile(pitch, 90) - np.percentile(pitch, 10) < 6 else 'WARN', '받침판이 밀렸거나 흔들림')
idx = np.clip(np.searchsorted(sec, G[:, 0]) - 1, 0, len(sec) - 1)
raw_yr = np.degrees((G[:, 1:] * up[idx]).sum(1))
yr = np.convolve(raw_yr, np.ones(100) / 100, 'same')
mx = abs(yr).max()
check('최대 회전', f'{mx:.0f}°/s', 'OK' if mx < 30 else ('WARN' if mx < 40 else 'NG'), '코너를 더 크게·천천히')
vib = np.degrees(np.linalg.norm(G[:, 1:], axis=1)); vib = np.std(vib - np.convolve(vib, np.ones(100) / 100, 'same'))
print(f'[INFO] 진동(고주파 각속도 std): {vib:.1f}°/s  <- 루프 A와 B 비교용, 크게 늘면 마운트 확인')
yaw = np.cumsum(yr * np.diff(G[:, 0], prepend=G[0, 0]))
check('총 회전', f'{yaw[-1]:+.0f}° (+반시계)', 'OK' if 250 <= abs(yaw[-1]) <= 500 else 'WARN', '한 바퀴(약 360°)가 아님. 기록만')
print('\n판정:', '합격' if bad == 0 else f'NG {bad}개 - 재촬영')
