#!/usr/bin/env python3
"""스케일 오차 원인 확인 (SLAM 없이 원본 bag만 사용)
 A) 컬러↔depth 정렬: 원본 depth를 컬러 내부 파라미터 fx·sx, fy·sy 로 정렬했을 때
    depth 윤곽(가림 경계)이 컬러 윤곽(Canny)과 가장 잘 겹치는 (sx, sy). 1.00이면 camera_info가 맞음.
 B) depth 스케일: 원본 depth(depth 내부 파라미터)로 바닥 평면 RANSAC → 카메라 높이·피치.
사용: align_scale_test.py <bag_dir> [프레임 수=24]   (~/tsdf_env 로 실행)"""
import sys
from pathlib import Path
import numpy as np, cv2
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

bag = Path(sys.argv[1]); N = int(sys.argv[2]) if len(sys.argv) > 2 else 24
NS = '/camera/camera'
ts = get_typestore(Stores.ROS2_JAZZY)


def qR(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


with AnyReader([bag], default_typestore=ts) as r:
    conns = {c.topic: c for c in r.connections}
    info = {}
    for t in ('color/camera_info', 'depth/camera_info'):
        for c, _, raw in r.messages(connections=[conns[f'{NS}/{t}']]):
            info[t] = r.deserialize(raw, c.msgtype); break
    tf = {}
    for c, _, raw in r.messages(connections=[conns['/tf_static']]):
        for t in r.deserialize(raw, c.msgtype).transforms:
            tf[t.child_frame_id] = t.transform
    # 프레임 수집: 컬러·depth 타임스탬프 짝 (5ms 이내)
    nmsg = conns[f'{NS}/color/image_raw'].msgcount
    pick = set(np.linspace(30, nmsg - 30, N * 4).astype(int))  # 후보(정지/저속 선택용으로 여유)
    col, dep = {}, []
    for i, (c, _, raw) in enumerate(r.messages(connections=[conns[f'{NS}/color/image_raw']])):
        if i in pick or i - 1 in pick:
            m = r.deserialize(raw, c.msgtype)
            col[i] = (stamp(m), np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1).copy(), m.encoding)
    want = sorted(v[0] for v in col.values())
    for c, _, raw in r.messages(connections=[conns[f'{NS}/depth/image_rect_raw']]):
        m = r.deserialize(raw, c.msgtype); s = stamp(m)
        k = np.searchsorted(want, s)
        if any(0 <= j < len(want) and abs(want[j] - s) < 0.005 for j in (k - 1, k)):
            dep.append((s, np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy()))

Kc = np.array(info['color/camera_info'].k).reshape(3, 3); Kd = np.array(info['depth/camera_info'].k).reshape(3, 3)
print(f'color K fx={Kc[0,0]:.2f} fy={Kc[1,1]:.2f} cx={Kc[0,2]:.2f} cy={Kc[1,2]:.2f}  D={list(info["color/camera_info"].d)}')
print(f'depth K fx={Kd[0,0]:.2f} fy={Kd[1,1]:.2f} cx={Kd[0,2]:.2f} cy={Kd[1,2]:.2f}  size={info["depth/camera_info"].width}x{info["depth/camera_info"].height}')
# depth optical → color optical (camera_link 기준 depth_frame=I 가정 확인)
def T_of(tr):
    T = np.eye(4); q = tr.rotation; T[:3, :3] = qR(q.x, q.y, q.z, q.w)
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]; return T
L_d = T_of(tf['camera_depth_frame']) @ T_of(tf['camera_depth_optical_frame'])
L_c = T_of(tf['camera_color_frame']) @ T_of(tf['camera_color_optical_frame'])
C_D = np.linalg.inv(L_c) @ L_d
print('depth→color 평행이동(m):', C_D[:3, 3].round(4), ' 회전각(°):', np.degrees(np.arccos((np.trace(C_D[:3, :3]) - 1) / 2)).round(3))

# 짝 맞추기 + 움직임(직전 프레임과 밝기 차) 계산
cs = sorted(col.items()); pairs = []
for idx, (s, img, enc) in cs:
    if idx - 1 not in col: continue
    d = min(dep, key=lambda x: abs(x[0] - s)) if dep else None
    if d is None or abs(d[0] - s) > 0.005: continue
    g = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY if enc == 'rgb8' else cv2.COLOR_BGR2GRAY)
    g0 = cv2.cvtColor(col[idx - 1][1], cv2.COLOR_RGB2GRAY if enc == 'rgb8' else cv2.COLOR_BGR2GRAY)
    pairs.append((np.abs(g.astype(int) - g0).mean(), idx, g, d[1].astype(float) / 1000, abs(d[0] - s)))
pairs.sort(key=lambda p: p[0]); pairs = pairs[:N]  # 움직임 적은 프레임 우선
print(f'사용 프레임 {len(pairs)}개 (움직임 점수 {pairs[0][0]:.1f}~{pairs[-1][0]:.1f}, 시각차 최대 {max(p[4] for p in pairs)*1e3:.1f}ms)')

H, W = 480, 640
vv, uu = np.mgrid[0:H, 0:W]


def backproject(D):
    z = D.ravel(); ok = (z > 0.2) & (z < 12)
    x = (uu.ravel() - Kd[0, 2]) * z / Kd[0, 0]; y = (vv.ravel() - Kd[1, 2]) * z / Kd[1, 1]
    P = np.c_[x, y, z][ok]
    return P @ C_D[:3, :3].T + C_D[:3, 3]


def register(P, sx, sy):
    u = Kc[0, 0] * sx * P[:, 0] / P[:, 2] + Kc[0, 2]; v = Kc[1, 1] * sy * P[:, 1] / P[:, 2] + Kc[1, 2]
    Z = np.full((H, W), np.inf)
    for du in (0, 1):
        for dv in (0, 1):  # 1.57배 업샘플 구멍 메우기
            ui = np.floor(u).astype(int) + du; vi = np.floor(v).astype(int) + dv
            ok = (ui >= 0) & (ui < W) & (vi >= 0) & (vi < H)
            np.minimum.at(Z, (vi[ok], ui[ok]), P[ok, 2])
    Z[np.isinf(Z)] = 0; return Z


def depth_edges(Z):
    E = np.zeros_like(Z, bool)
    for ax in (0, 1):
        a = Z; b = np.roll(Z, -1, ax); ok = (a > 0) & (b > 0)
        jump = ok & (np.abs(a - b) > 0.08 * np.minimum(a, b)) & (np.abs(a - b) > 0.3)
        fg_a = jump & (a < b); fg_b = jump & (b < a)
        E |= fg_a; E |= np.roll(fg_b, 1, ax)  # 앞쪽(가까운) 물체 쪽 픽셀
    E[:3] = E[-3:] = False; E[:, :3] = E[:, -3:] = False
    return E


prep = []
for _, idx, g, D, _ in pairs:
    ce = cv2.Canny(cv2.GaussianBlur(g, (5, 5), 1.2), 40, 120)
    dist = cv2.distanceTransform((ce == 0).astype(np.uint8), cv2.DIST_L2, 3)
    prep.append((dist, backproject(D), D))


def score(sx, sy):
    vals = []
    for dist, P, _ in prep:
        E = depth_edges(register(P, sx, sy))
        if E.sum() < 200: continue
        vals.append(np.minimum(dist[E], 8).mean())
    return np.median(vals) if vals else np.nan, len(vals)


print('\n[A] 등방 배율 s (fx,fy 모두) → 경계 거리 중앙값(px, 작을수록 정렬 좋음)')
ss = np.round(np.arange(0.88, 1.141, 0.01), 2); res = []
for s in ss:
    v, n = score(s, s); res.append(v); print(f'  s={s:.2f}  {v:.3f}  (n={n})')
best = ss[int(np.nanargmin(res))]; print(f'  → 최적 s = {best:.2f}')
print('[A] fx·fy 따로 (최적 주변 ±0.06)')
grid = np.round(np.arange(-0.06, 0.061, 0.02), 2); M = np.array([[score(best + a, best + b)[0] for b in grid] for a in grid])
i, j = np.unravel_index(np.nanargmin(M), M.shape)
print(f'  → 최적 sx = {best + grid[i]:.2f}, sy = {best + grid[j]:.2f}  (점수 {M[i, j]:.3f}, s=1.00 점수 {score(1, 1)[0]:.3f})')

print('\n[B] 원본 depth 바닥 평면 (depth 광학 좌표, 3~8m, 화면 아래쪽)')
hs, ps = [], []
rng = np.random.default_rng(0)
for _, _, D in prep:
    z = D; ok = (z > 3) & (z < 8); ok[:260] = False
    v, u = np.nonzero(ok); zz = z[v, u]
    P = np.c_[(u - Kd[0, 2]) * zz / Kd[0, 0], (v - Kd[1, 2]) * zz / Kd[1, 1], zz]
    if len(P) < 3000: continue
    bestn, bestm = None, 0
    for _ in range(300):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]; n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-6: continue
        n /= np.linalg.norm(n); m = (np.abs((P - a) @ n) < 0.04).sum()
        if m > bestm: bestm, bestn, d0 = m, n, -a @ n
    inl = np.abs(P @ bestn + d0) < 0.04
    if inl.mean() < 0.5: continue
    c0 = P[inl].mean(0); _, _, Vt = np.linalg.svd(P[inl] - c0, full_matrices=False); n = Vt[2]; n *= np.sign(n[1])  # 아래(+y) 방향
    hs.append(abs(c0 @ n)); ps.append(np.degrees(np.arcsin(-n[2])))  # 광축이 바닥과 이루는 각(+면 위로)
if hs:
    print(f'  프레임 {len(hs)}개: 카메라 높이 중앙값 {np.median(hs):.3f} m (p10 {np.percentile(hs,10):.3f}, p90 {np.percentile(hs,90):.3f}), 실측 1.55 → {100*(np.median(hs)/1.55-1):+.1f}%')
    print(f'  광축 피치 중앙값 {np.median(ps):+.1f}° (IMU 추정 위로 약 5°)')
else:
    print('  평면 프레임 없음')
