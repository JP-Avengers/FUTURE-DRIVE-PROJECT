#!/usr/bin/env python3
"""컬러 카메라 fx 검증 (depth·거리와 무관): 회전 구간에서 영상으로 구한 회전각 ÷ 자이로 적분 회전각.
 영상 회전각은 픽셀 이동 / fx 로 정해지므로, camera_info fx가 실제보다 k배 작으면 비율 ≈ k.
 회전은 SIFT + 기본행렬 아닌 Essential(USAC_MAGSAC) + recoverPose. 컬러 K 배율 s를 바꿔 자이로와 맞는 s도 탐색.
사용: gyro_fx_check.py <bag_dir> [간격초=0.4] [최소각도=4]   (~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, cv2
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

bag = Path(sys.argv[1]); DT = float(sys.argv[2]) if len(sys.argv) > 2 else 0.4
MINDEG = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0
NS = '/camera/camera'; ts = get_typestore(Stores.ROS2_JAZZY)


def qR(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def T_of(tr):
    T = np.eye(4); q = tr.rotation; T[:3, :3] = qR(q.x, q.y, q.z, q.w)
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]; return T
def stamp(m): return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
def rodrigues_small(w, dt):  # 회전 벡터 → 행렬
    return cv2.Rodrigues((w * dt).reshape(3, 1))[0]


with AnyReader([bag], default_typestore=ts) as r:
    C = {c.topic: c for c in r.connections}
    for c, _, raw in r.messages(connections=[C[f'{NS}/color/camera_info']]):
        K0 = np.array(r.deserialize(raw, c.msgtype).k).reshape(3, 3); break
    tf = {}
    for c, _, raw in r.messages(connections=[C['/tf_static']]):
        for t in r.deserialize(raw, c.msgtype).transforms: tf[t.child_frame_id] = t.transform
    G = []; gframe = None
    for c, _, raw in r.messages(connections=[C[f'{NS}/gyro/sample']]):
        m = r.deserialize(raw, c.msgtype); gframe = m.header.frame_id
        G.append((stamp(m), m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
    G = np.array(G); G = G[np.argsort(G[:, 0])]
    cstamps = []
    for c, _, raw in r.messages(connections=[C[f'{NS}/color/image_raw']]):
        cstamps.append(stamp(r.deserialize(raw, c.msgtype)))
    cstamps = np.array(cstamps)

# 자이로 좌표 → 컬러 광학 좌표 회전
chain = {'camera_color_optical_frame': 'camera_color_frame', 'camera_gyro_optical_frame': 'camera_gyro_frame',
         'camera_imu_optical_frame': 'camera_imu_frame', 'camera_imu_frame': 'camera_gyro_frame'}
def link_R(f):
    R = np.eye(3)
    while f != 'camera_link':
        R = T_of(tf[f])[:3, :3] @ R; f = chain.get(f, 'camera_link')
    return R
R_cg = link_R('camera_color_optical_frame').T @ link_R(gframe)
bias = np.median(G[:200, 1:], 0) if (np.linalg.norm(G[:200, 1:], axis=1) < 0.02).all() else np.zeros(3)
print(f'자이로 frame={gframe}, 시작 정지구간 bias={bias.round(4)} rad/s, 샘플 {len(G)}개')


def gyro_R(t0, t1):  # t0→t1 동안 카메라 회전 (컬러 광학 좌표, R: cam1 ← cam0 아님, 적분 그대로)
    s = (G[:, 0] >= t0) & (G[:, 0] < t1); g = G[s]; R = np.eye(3)
    for i in range(len(g)):
        dt = (g[i + 1, 0] if i + 1 < len(g) else t1) - g[i, 0]
        R = R @ rodrigues_small(R_cg @ (g[i, 1:] - bias), dt)
    return R


# 회전 큰 구간에서 이미지 쌍 고르기
pairs = []; last = -1e9
for i, t in enumerate(cstamps):
    j = np.searchsorted(cstamps, t + DT)
    if j >= len(cstamps) or t < last + 0.5: continue
    Rg = gyro_R(t, cstamps[j]); ang = np.degrees(np.linalg.norm(cv2.Rodrigues(Rg)[0]))
    if ang >= MINDEG: pairs.append((i, j, Rg, ang)); last = t
print(f'회전 {MINDEG}° 이상 쌍 {len(pairs)}개 (간격 {DT}s)')
need = {i for p in pairs for i in p[:2]}; imgs = {}
with AnyReader([bag], default_typestore=ts) as r:
    C = {c.topic: c for c in r.connections}
    for k, (c, _, raw) in enumerate(r.messages(connections=[C[f'{NS}/color/image_raw']])):
        if k in need:
            m = r.deserialize(raw, c.msgtype); a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
            imgs[k] = cv2.cvtColor(a, cv2.COLOR_RGB2GRAY if m.encoding == 'rgb8' else cv2.COLOR_BGR2GRAY)

sift = cv2.SIFT_create(3000); bf = cv2.BFMatcher(cv2.NORM_L2)
matches = []
for i, j, Rg, ang in pairs:
    k1, d1 = sift.detectAndCompute(imgs[i], None); k2, d2 = sift.detectAndCompute(imgs[j], None)
    if d1 is None or d2 is None: continue
    mm = [a for a, b in bf.knnMatch(d1, d2, k=2) if a.distance < 0.75 * b.distance]
    if len(mm) < 60: continue
    p1 = np.float64([k1[m.queryIdx].pt for m in mm]); p2 = np.float64([k2[m.trainIdx].pt for m in mm])
    matches.append((p1, p2, Rg, ang))
print(f'매칭 충분한 쌍 {len(matches)}개')


def img_rot(p1, p2, K):
    E, msk = cv2.findEssentialMat(p1, p2, K, method=cv2.USAC_MAGSAC, prob=0.999, threshold=1.0)
    if E is None or E.shape != (3, 3): return None, 0
    n, R, t, msk2 = cv2.recoverPose(E, p1, p2, K, mask=msk)
    return R, int(n)


print('\n[개별 쌍, s=1.00] 자이로각  영상각  비율  축 차이')
ratios = []
for p1, p2, Rg, ang in matches:
    R, n = img_rot(p1, p2, K0)
    if R is None or n < 40: continue
    a_img = np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0]))
    # recoverPose의 R은 cam1→cam2 점 변환 = 자이로 적분 회전의 전치
    ax_i = cv2.Rodrigues(R.T)[0].ravel(); ax_g = cv2.Rodrigues(Rg)[0].ravel()
    axd = np.degrees(np.arccos(np.clip(ax_i @ ax_g / np.linalg.norm(ax_i) / np.linalg.norm(ax_g), -1, 1)))
    ratios.append(a_img / ang); print(f'  {ang:6.2f}° {a_img:6.2f}°  {a_img/ang:.3f}  {axd:5.1f}°  (인라이어 {n})')
if ratios:
    r_ = np.array(ratios); print(f'\n영상각/자이로각: 중앙값 {np.median(r_):.3f}, 평균 {r_.mean():.3f} ± {r_.std()/np.sqrt(len(r_)):.3f} (n={len(r_)})')

print('\n[배율 s 탐색] 컬러 fx,fy ×s 일 때 영상각/자이로각 중앙값 (1.00에 가까운 s가 실제 배율)')
for s in (0.92, 0.96, 1.00, 1.04, 1.08, 1.12):
    K = K0.copy(); K[0, 0] *= s; K[1, 1] *= s; rr = []
    for p1, p2, Rg, ang in matches:
        R, n = img_rot(p1, p2, K)
        if R is not None and n >= 40: rr.append(np.degrees(np.linalg.norm(cv2.Rodrigues(R)[0])) / ang)
    print(f'  s={s:.2f}: {np.median(rr):.3f} (n={len(rr)})')
