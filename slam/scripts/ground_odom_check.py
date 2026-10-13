#!/usr/bin/env python3
"""오도메트리 이동량 스케일 검증 (depth 미사용).
  STEP 프레임 간격의 짧은 이동을 바닥 특징점(SIFT, 화면 아래쪽 ≈5~9 m)으로 구한다:
  픽셀 → 바닥 평면(렌즈 높이 h 실측, 정지 구간 가속도계 중력 방향) → 2D 강체 정합.
  짧은 이동을 키프레임 구간마다 이어 붙여, RTAB-Map 궤적(TUM)의 같은 구간 수평 이동 거리와 비교 (비율 = 궤적/바닥).
사용: ground_odom_check.py <bag_dir> <traj_tum.txt> [--h 1.55] [--step 3]   (~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, cv2
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

args = sys.argv[1:]
def opt(name, default):
    if name in args:
        i = args.index(name); v = float(args[i + 1]); del args[i:i + 2]; return v
    return default
H_CAM = opt('--h', 1.55); STEP = int(opt('--step', 3))
bag, traj = Path(args[0]), Path(args[1])
NS = '/camera/camera'; ts = get_typestore(Stores.ROS2_JAZZY)


def qR(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def T_of(tr):
    T = np.eye(4); q = tr.rotation; T[:3, :3] = qR(q.x, q.y, q.z, q.w)
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]; return T
def stamp(m): return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


tr = np.loadtxt(traj, comments='#', ndmin=2)
with AnyReader([bag], default_typestore=ts) as r:
    C = {c.topic: c for c in r.connections}
    for c, _, raw in r.messages(connections=[C[f'{NS}/color/camera_info']]):
        K = np.array(r.deserialize(raw, c.msgtype).k).reshape(3, 3); break
    tf = {}
    for c, _, raw in r.messages(connections=[C['/tf_static']]):
        for t in r.deserialize(raw, c.msgtype).transforms: tf[t.child_frame_id] = t.transform
    acc = []; fr = None; t0 = None
    for c, _, raw in r.messages(connections=[C[f'{NS}/accel/sample']]):
        m = r.deserialize(raw, c.msgtype); s = stamp(m); t0 = t0 or s; fr = m.header.frame_id
        if s - t0 > 1.0: break
        a = m.linear_acceleration; acc.append([a.x, a.y, a.z])
    frames = []
    for i, (c, _, raw) in enumerate(r.messages(connections=[C[f'{NS}/color/image_raw']])):
        if i % STEP: continue
        m = r.deserialize(raw, c.msgtype); s = stamp(m)
        if s < tr[0, 0] - 0.2 or s > tr[-1, 0] + 0.2: continue
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
        frames.append((s, cv2.cvtColor(a, cv2.COLOR_RGB2GRAY if m.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY)))

chain = {'camera_color_optical_frame': 'camera_color_frame', 'camera_accel_optical_frame': 'camera_accel_frame'}
def link_R(f):
    R = np.eye(3)
    while f != 'camera_link':
        R = T_of(tf[f])[:3, :3] @ R; f = chain.get(f, 'camera_link')
    return R
acc = np.array(acc)
up = link_R('camera_color_optical_frame').T @ link_R(fr) @ acc.mean(0); up /= np.linalg.norm(up)
fwd = np.cross(up, [1, 0, 0]); fwd /= np.linalg.norm(fwd); fwd *= np.sign(fwd[2]); left = np.cross(up, fwd)
print(f'광축 피치 {np.degrees(np.arcsin(up[2])):+.2f}°, 렌즈 높이 {H_CAM} m, 프레임 {len(frames)}개 (STEP {STEP}), 키프레임 {len(tr)}개')


def to_ground(pts):  # 픽셀 → 바닥 (전방, 왼쪽) m
    ray = np.c_[(pts[:, 0] - K[0, 2]) / K[0, 0], (pts[:, 1] - K[1, 2]) / K[1, 1], np.ones(len(pts))]
    down = -(ray @ up); lam = np.where(down > 1e-3, H_CAM / np.maximum(down, 1e-9), np.nan)
    P = ray * lam[:, None]; return np.c_[P @ fwd, P @ left]


sift = cv2.SIFT_create(3000); bf = cv2.BFMatcher(cv2.NORM_L2)
mask = np.zeros((480, 640), np.uint8); mask[300:] = 255
clahe = cv2.createCLAHE(3.0, (8, 8))
prev = None; steps = []; scales = []
for s, g in frames:
    k, d = sift.detectAndCompute(clahe.apply(g), mask); cur = (s, k, d)
    if prev is not None and d is not None and prev[2] is not None:
        mm = [a for a, b in bf.knnMatch(prev[2], d, k=2) if a.distance < 0.8 * b.distance]
        M = None
        if len(mm) >= 20:
            g1 = to_ground(np.float64([prev[1][m.queryIdx].pt for m in mm])); g2 = to_ground(np.float64([k[m.trainIdx].pt for m in mm]))
            ok = np.isfinite(g1).all(1) & np.isfinite(g2).all(1) & (g1[:, 0] < 12) & (g2[:, 0] < 12)
            if ok.sum() >= 15:
                Ms, inl = cv2.estimateAffinePartial2D(g1[ok], g2[ok], method=cv2.RANSAC, ransacReprojThreshold=0.05, maxIters=5000)
                if Ms is not None and inl.sum() >= 12:
                    A, B = g1[ok][inl.ravel() == 1], g2[ok][inl.ravel() == 1]
                    ca, cb = A.mean(0), B.mean(0); U, _, Vt = np.linalg.svd((A - ca).T @ (B - cb))
                    R = Vt.T @ U.T
                    if np.linalg.det(R) < 0: Vt[1] *= -1; R = Vt.T @ U.T
                    M = np.eye(3); M[:2, :2] = R; M[:2, 2] = cb - R @ ca  # 점: 이전 프레임 좌표 → 현재 프레임 좌표
                    scales.append(np.hypot(Ms[0, 0], Ms[1, 0]))
        steps.append((prev[0], s, M))
    prev = cur
nok = sum(m is not None for _, _, m in steps)
print(f'짧은 이동 {len(steps)}개 중 성공 {nok}개, 유사변환 배율 중앙값 {np.median(scales):.3f} (1이면 평면 가정 일관)')

rows = []
for k in range(len(tr) - 1):
    ta, tb = tr[k, 0], tr[k + 1, 0]
    seg = [st for st in steps if st[0] >= ta - 0.07 and st[1] <= tb + 0.07]
    if not seg or any(m is None for _, _, m in seg): continue
    if abs(seg[0][0] - ta) > 0.07 or abs(seg[-1][1] - tb) > 0.07: continue
    Tc = np.eye(3)  # 카메라 a → b 의 점 변환 누적
    for _, _, M in seg: Tc = M @ Tc
    d_ground = np.linalg.norm(np.linalg.inv(Tc)[:2, 2])  # 카메라 b 위치(a 좌표계)
    d_traj = np.linalg.norm(tr[k + 1, 1:3] - tr[k, 1:3])
    rows.append((ta - tr[0, 0], d_traj, d_ground, len(seg)))
R_ = np.array(rows)
print(f'\n비교 가능한 키프레임 구간 {len(R_)}개   (시각s, 궤적m, 바닥m, 비율, 단계수)')
for t_, a, b, n in R_: print(f'  {t_:6.1f}  {a:6.3f}  {b:6.3f}  {a / max(b, 1e-6):6.3f}  {int(n)}')
use = R_[:, 2] > 0.5
rat = R_[use, 1] / R_[use, 2]
print(f'\n궤적/바닥 비율 (구간 0.5 m 이상): 중앙값 {np.median(rat):.3f}, 합계 기준 {R_[use, 1].sum() / R_[use, 2].sum():.3f}, '
      f'n={use.sum()}, IQR {np.percentile(rat, 25):.3f}~{np.percentile(rat, 75):.3f}')
