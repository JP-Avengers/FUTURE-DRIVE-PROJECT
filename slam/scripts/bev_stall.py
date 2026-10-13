#!/usr/bin/env python3
"""depth 없이 주차칸 치수 재기: 바닥 평면 = (실측 렌즈 높이 h, IMU 중력 방향) → 컬러 영상을 위에서 본 영상(BEV)으로 펼침.
  sheet 모드:  bev_stall.py <bag> sheet <out.jpg> [간격초=2]      프레임 고르기용 썸네일 시트 (번호=초)
  bev 모드:    bev_stall.py <bag> bev <outdir> <초> [<초>...] [--h 1.55] [--fx_scale 1.0]
               → BEV jpg (1px=1cm, 50cm 격자) + 원본 depth 바닥 평면 높이/피치 비교 출력
(~/tsdf_env 로 실행)"""
import sys
from pathlib import Path
import numpy as np, cv2
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

NS = '/camera/camera'; ts = get_typestore(Stores.ROS2_JAZZY)
args = sys.argv[1:]
def opt(name, default):
    if name in args:
        i = args.index(name); v = float(args[i + 1]); del args[i:i + 2]; return v
    return default
H_CAM = opt('--h', 1.55); FXS = opt('--fx_scale', 1.0)
bag, mode, out = Path(args[0]), args[1], Path(args[2]); rest = args[3:]


def qR(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
def T_of(tr):
    T = np.eye(4); q = tr.rotation; T[:3, :3] = qR(q.x, q.y, q.z, q.w)
    T[:3, 3] = [tr.translation.x, tr.translation.y, tr.translation.z]; return T
def stamp(m): return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
def to_gray_bgr(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
    return cv2.cvtColor(a, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else a.copy()


with AnyReader([bag], default_typestore=ts) as r:
    C = {c.topic: c for c in r.connections}
    t0 = r.start_time * 1e-9
    if mode == 'sheet':
        step = float(rest[0]) if rest else 2.0; thumbs, nxt = [], 0.0
        for c, t, raw in r.messages(connections=[C[f'{NS}/color/image_raw']]):
            s = t * 1e-9 - t0
            if s >= nxt:
                im = cv2.resize(to_gray_bgr(r.deserialize(raw, c.msgtype)), (213, 160))
                cv2.putText(im, f'{s:.0f}', (4, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                thumbs.append(im); nxt += step
        while len(thumbs) % 6: thumbs.append(np.zeros_like(thumbs[0]))
        rows = [np.hstack(thumbs[i:i + 6]) for i in range(0, len(thumbs), 6)]
        cv2.imwrite(str(out), np.vstack(rows)); print(f'{len(rows)}줄 → {out}'); sys.exit()

    times = sorted(float(x) for x in rest); out.mkdir(parents=True, exist_ok=True)
    info = {}
    for t in ('color/camera_info', 'depth/camera_info'):
        for c, _, raw in r.messages(connections=[C[f'{NS}/{t}']]):
            info[t] = r.deserialize(raw, c.msgtype); break
    tf = {}
    for c, _, raw in r.messages(connections=[C['/tf_static']]):
        for t in r.deserialize(raw, c.msgtype).transforms: tf[t.child_frame_id] = t.transform
    def link_T(frame):  # camera_link → frame (부모 따라 올라감)
        T = np.eye(4); f = frame
        chain = {'camera_color_optical_frame': 'camera_color_frame', 'camera_depth_optical_frame': 'camera_depth_frame',
                 'camera_accel_optical_frame': 'camera_accel_frame', 'camera_gyro_optical_frame': 'camera_gyro_frame',
                 'camera_imu_optical_frame': 'camera_imu_frame', 'camera_imu_frame': 'camera_gyro_frame'}
        while f != 'camera_link':
            T = T_of(tf[f]) @ T; f = chain.get(f, 'camera_link')
        return T
    Kc = np.array(info['color/camera_info'].k).reshape(3, 3); Kd = np.array(info['depth/camera_info'].k).reshape(3, 3)
    Kc[0, 0] *= FXS; Kc[1, 1] *= FXS
    R_lc = link_T('camera_color_optical_frame')[:3, :3]; R_ld = link_T('camera_depth_optical_frame')
    want = [t0 + s for s in times]
    cols, deps, acc = {}, {}, []
    for c, t, raw in r.messages(connections=[C[f'{NS}/color/image_raw'], C[f'{NS}/depth/image_rect_raw'], C[f'{NS}/accel/sample']]):
        s = t * 1e-9
        if c.topic.endswith('accel/sample'):
            if any(abs(s - w) < 0.5 for w in want):
                m = r.deserialize(raw, c.msgtype); a = m.linear_acceleration; acc.append((s, m.header.frame_id, [a.x, a.y, a.z]))
            continue
        for k, w in enumerate(want):
            tgt = cols if 'color' in c.topic else deps
            if abs(s - w) < 0.04 and (k not in tgt or abs(s - w) < abs(tgt[k][0] - w)):
                m = r.deserialize(raw, c.msgtype)
                tgt[k] = (s, to_gray_bgr(m) if 'color' in c.topic else np.frombuffer(m.data, np.uint16).reshape(m.height, m.width) / 1000.0)

print(f'color fx={Kc[0,0]:.1f} (배율 {FXS}), 렌즈 높이 {H_CAM} m')
res = 0.01; XF, YL = (2.0, 14.0), (-7.0, 7.0)  # 전방 2~14 m, 좌우 ±7 m
gx, gy = np.meshgrid(np.arange(YL[1], YL[0], -res), np.arange(XF[1], XF[0], -res))  # 위=먼 곳, 왼=왼쪽
EXP = Path.home() / 'b_slam/exports'


def make_bev(img, up_c, h, tag):
    fwd = np.cross(up_c, [1, 0, 0]); fwd /= np.linalg.norm(fwd)
    if fwd[2] < 0: fwd = -fwd
    left = np.cross(up_c, fwd)
    P = gy[..., None] * fwd + gx[..., None] * left - h * up_c  # 바닥 점(카메라 기준) → 컬러 영상 좌표
    u = Kc[0, 0] * P[..., 0] / P[..., 2] + Kc[0, 2]; v = Kc[1, 1] * P[..., 1] / P[..., 2] + Kc[1, 2]
    ok = (P[..., 2] > 0) & (u >= 0) & (u < 639) & (v >= 0) & (v < 479)
    bev = cv2.remap(img, u.astype(np.float32), v.astype(np.float32), cv2.INTER_LINEAR, borderValue=(40, 40, 40))
    hsv = cv2.cvtColor(bev, cv2.COLOR_BGR2HSV); V = hsv[..., 2].astype(float)
    white = ok & (V > np.median(V[ok]) + 60) & (hsv[..., 1] < 50)
    xy = np.c_[gy[white], gx[white]]  # (전방, 왼쪽) m
    d = EXP / f'bev_{bag.parent.name[:20]}_{tag}'; d.mkdir(parents=True, exist_ok=True); np.save(d / 'paint_xy.npy', xy)
    for i in range(0, bev.shape[0], 50): cv2.line(bev, (0, i), (bev.shape[1], i), (0, 160, 255) if i % 100 == 0 else (0, 90, 140), 1)
    for j in range(0, bev.shape[1], 50): cv2.line(bev, (j, 0), (j, bev.shape[0]), (0, 160, 255) if j % 100 == 0 else (0, 90, 140), 1)
    cv2.imwrite(str(out / f'bev_{tag}.jpg'), bev)
    return d.name, len(xy)


for k, s in enumerate(times):
    if k not in cols: print(f'{s}s: 프레임 없음'); continue
    A = np.array([a for (ta, fr, a) in acc if abs(ta - want[k]) < 0.5]); fr = acc[0][1]
    g_up_link = link_T(fr)[:3, :3] @ A.mean(0); g_up_link /= np.linalg.norm(g_up_link)  # 가속도계 = 위쪽 방향
    up_c = R_lc.T @ g_up_link                       # 컬러 광학 좌표에서 위쪽
    pitch = np.degrees(np.arcsin(np.clip(up_c @ [0, 0, 1], -1, 1)))
    name, n1 = make_bev(cols[k][1], up_c, H_CAM, f'{s:.1f}s_imu_h{H_CAM}_fx{FXS}')
    line = f'{s:.1f}s: IMU 피치(광축, +=위) {pitch:+.2f}° → {name} ({n1}점)'
    if k in deps:  # 원본 depth 평면
        D = deps[k][1]; vv, uu = np.nonzero((D > 2.5) & (D < 8)); vv, uu = vv[vv > 260], uu[vv > 260]; zz = D[vv, uu]
        Pd = np.c_[(uu - Kd[0, 2]) * zz / Kd[0, 0], (vv - Kd[1, 2]) * zz / Kd[1, 1], zz]
        Pd = Pd @ (R_lc.T @ R_ld[:3, :3]).T  # 회전만 (평행이동 1.5cm 무시)
        up_d = -up_c; hgt = Pd @ up_d
        inl = np.abs(hgt - np.median(hgt)) < 0.08
        c0 = Pd[inl].mean(0); _, _, Vt = np.linalg.svd(Pd[inl] - c0, full_matrices=False); n = Vt[2] * np.sign(Vt[2] @ up_d)
        tilt = np.degrees(np.arccos(np.clip(n @ up_d, -1, 1))); hd = c0 @ n
        name2, n2 = make_bev(cols[k][1], -n, hd, f'{s:.1f}s_depth_fx{FXS}')
        line += f'\n        depth 평면: 높이 {hd:.3f} m, IMU 수직과 {tilt:.2f}° 차이, 피치 {np.degrees(np.arcsin(-n[2])):+.2f}° → {name2} ({n2}점)'
    print(line)
