#!/usr/bin/env python3
"""10/9 추가 촬영(본네트 카메라) QC. 사용: field_qc_v2.py <bag_dir> [기대 방향 cw|ccw]
기준(추가 촬영 가이드): 컬러·원본 depth·정렬 depth 각 29.5fps 이상, 누락 1% 이하, IMU 기록,
피치 아래 8~12°(정지 구간 가속도계), 출발·도착 정지 구간, 회전 속도·진동은 10/5 수준(최대 12°/s, 진동 0.3°/s) 대비 참고."""
import sys, os
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
bag = Path(sys.argv[1]); want_dir = sys.argv[2] if len(sys.argv) > 2 else None
NS = '/camera/camera'
IMG = {f'{NS}/color/image_raw': '컬러', f'{NS}/depth/image_rect_raw': '원본 depth', f'{NS}/aligned_depth_to_color/image_raw': '정렬 depth'}
T = {k: [] for k in IMG}; G = []; A = []
def drop():
    for f in bag.glob('*.mcap'):
        fd = os.open(f, os.O_RDONLY); os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED); os.close(fd)
with AnyReader([bag]) as r:
    conns = [c for c in r.connections if c.topic in IMG or c.topic in (f'{NS}/gyro/sample', f'{NS}/accel/sample')]
    n = 0
    for c, t, raw in r.messages(connections=conns):
        n += 1
        if n % 20000 == 0: drop()
        if c.topic in IMG: T[c.topic].append(t * 1e-9); continue
        m = r.deserialize(raw, c.msgtype); s = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if 'gyro' in c.topic: v = m.angular_velocity; G.append((s, v.x, v.y, v.z))
        else: v = m.linear_acceleration; A.append((s, v.x, v.y, v.z))
drop()
bad = 0
def chk(name, val, ok, msg='', level='NG'):
    global bad
    lv = 'OK' if ok else level; bad += lv == 'NG'
    print(f'[{lv:<4}] {name}: {val}' + ('' if ok else f'  <- {msg}'))
dur = None
for k, lab in IMG.items():
    x = np.array(T[k])
    if len(x) < 2: chk(lab, '없음', False, '토픽 누락'); continue
    d = x[-1] - x[0]; dur = dur or d; hz = (len(x) - 1) / d; dt = np.diff(x); per = np.median(dt)
    miss = int(np.clip(np.round(dt / per) - 1, 0, None).sum()); missp = 100 * miss / (len(x) + miss)
    chk(f'{lab} fps·누락', f'{hz:.2f}fps, 누락 추정 {miss}장({missp:.2f}%), 최대 간격 {dt.max()*1e3:.0f}ms', hz >= 29.5 and missp <= 1.0, '29.5fps 미만 또는 누락 1% 초과')
G, A = np.array(G), np.array(A)
chk('IMU', f'gyro {len(G)} ({len(G)/dur:.0f}Hz), accel {len(A)} ({len(A)/dur:.0f}Hz)', len(G) > 150 * dur and len(A) > 80 * dur, 'IMU 누락')
gm = np.degrees(np.linalg.norm(G[:, 1:], axis=1)); gs = np.convolve(gm, np.ones(200) / 200, 'same')  # 1초 평균 각속도
t0, t1 = G[0, 0], G[-1, 0]
still = gs < 1.5
def run_len(mask, ts, from_end=False):
    idx = np.arange(len(mask))[::-1] if from_end else np.arange(len(mask))
    k = 0
    for i in idx:
        if not mask[i]: break
        k += 1
    return k / 200.0
s_start, s_end = run_len(still, G[:, 0]), run_len(still, G[:, 0], True)
chk('출발·도착 정지', f'출발 {s_start:.0f}s, 도착 {s_end:.0f}s', s_start >= 5 and s_end >= 5, '정지 구간 5초 미만(가이드 10초)', 'WARN')
# 피치: 출발 정지 구간(최대 10초) 가속도 평균 → 위쪽 벡터
w = A[A[:, 0] < A[0, 0] + max(min(s_start, 10), 2)]
up = w[:, 1:].mean(0); up /= np.linalg.norm(up); pitch = np.degrees(np.arcsin(up[2]))
roll = np.degrees(np.arcsin(up[0]))
chk('피치(정지 구간)', f'{pitch:+.1f}° (−가 아래), 롤 {roll:+.1f}°', -12.5 <= pitch <= -7.5, '아래 8~12° 범위 밖')
# 회전·진동·방향 (1초 단위 위쪽 벡터 기준 yaw rate)
sec = np.arange(A[0, 0], A[-1, 0], 1.0)
U = np.array([A[(A[:, 0] >= s) & (A[:, 0] < s + 1), 1:].mean(0) for s in sec]); U /= np.linalg.norm(U, axis=1, keepdims=True)
ii = np.clip(np.searchsorted(sec, G[:, 0]) - 1, 0, len(sec) - 1)
yr = np.degrees((G[:, 1:] * U[ii]).sum(1)); yrs = np.convolve(yr, np.ones(100) / 100, 'same')
yaw = np.cumsum(yr * np.diff(G[:, 0], prepend=G[0, 0]))
vib = np.std(gm - np.convolve(gm, np.ones(100) / 100, 'same'))
chk('최대 회전(0.5s 평균)', f'{abs(yrs).max():.0f}°/s', abs(yrs).max() < 30, '급회전', 'WARN')
chk('진동(고주파 각속도 std)', f'{vib:.2f}°/s (10/5: 0.2~0.3)', vib < 0.6, '진동 큼(거치대 확인)', 'WARN')
d = 'ccw' if yaw[-1] > 0 else 'cw'
chk('총 회전·방향', f'{yaw[-1]:+.0f}° → {d}' + (f' (계획 {want_dir})' if want_dir else ''), (want_dir is None or want_dir == d) and 250 <= abs(yaw[-1]) <= 700, '계획 방향과 다르거나 한 바퀴가 아님', 'WARN')
print(f'길이 {dur:.0f}s → 판정:', '합격' if bad == 0 else f'NG {bad}개')
