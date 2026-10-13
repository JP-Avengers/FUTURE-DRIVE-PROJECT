#!/usr/bin/env python3
"""맵 간 변환 추정 v2 (원본은 읽기만). 기본행렬 검증 대신 depth 기반 PnP + 중력 검증.
각 src 키프레임 × ref 키프레임: RootSIFT, 비율 0.75 + 상호 최근접 매칭 → 30개 이상이면
ref depth(0.5~12m)로 3D(ref 월드) ↔ src 2D → PnP RANSAC(3px) → 인라이어 25개 이상 + 기울기 5° 미만만 채택.
채택된 변환들이 서로 일치(이동 1.5m, yaw 5°)하는 가장 큰 그룹을 결과로.
사용: xmap_match2.py <ref_run> <src_run> <out_dir>"""
import sys, re, json, time
from pathlib import Path
import numpy as np, cv2
ref_run, src_run, out = sys.argv[1], sys.argv[2], Path(sys.argv[3]); out.mkdir(parents=True, exist_ok=True)
H = Path.home() / 'b_slam/exports'
def qR(qx, qy, qz, qw):
    return np.array([[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)], [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)], [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
def load(run):
    P = {}
    for r in np.loadtxt(H / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2):
        T = np.eye(4); T[:3, :3] = qR(*r[4:8]); T[:3, 3] = r[1:4]; P[int(r[8])] = T
    return P
def intr(run, i):
    t = (H / run / f'keyframes/kf_calib/{i}.yaml').read_text()
    a = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', t, re.S).group(1).replace('\n', ' ').split(',')]
    return np.array([[a[0], 0, a[2]], [0, a[4], a[5]], [0, 0, 1]])
sift = cv2.SIFT_create(nfeatures=1500)
def feats(run, ids):
    F = {}
    for i in ids:
        kp, d = sift.detectAndCompute(cv2.imread(str(H / run / f'keyframes/kf_rgb/{i}.jpg'), cv2.IMREAD_GRAYSCALE), None)
        if d is None or len(kp) < 50: continue
        d = np.sqrt(d / (np.abs(d).sum(1, keepdims=True) + 1e-7)).astype(np.float32)
        F[i] = (np.array([k.pt for k in kp], np.float64), d)
    return F
t0 = time.time(); Pr, Ps = load(ref_run), load(src_run)
Fr, Fs = feats(ref_run, sorted(Pr)), feats(src_run, sorted(Ps))
D = {r: cv2.imread(str(H / ref_run / f'keyframes/kf_depth/{r}.png'), cv2.IMREAD_UNCHANGED).astype(float) / 1000 for r in Fr}
K = {('r', r): intr(ref_run, r) for r in Fr}; K.update({('s', s): intr(src_run, s) for s in Fs})
print(f'특징 추출 ref {len(Fr)}, src {len(Fs)} ({time.time()-t0:.0f}s)', flush=True)
fl = cv2.FlannBasedMatcher(dict(algorithm=1, trees=4), dict(checks=32))
acc = []; tried = 0
for s, (ps, ds) in Fs.items():
    for r, (pr, dr) in Fr.items():
        m12 = fl.knnMatch(ds, dr, k=2)
        good = {a.queryIdx: a.trainIdx for a, b in (x for x in m12 if len(x) == 2) if a.distance < 0.75 * b.distance}
        if len(good) < 30: continue
        back = {a.queryIdx: a.trainIdx for (a,) in (x[:1] for x in fl.knnMatch(dr, ds, k=1)) }
        pairs = [(q, t) for q, t in good.items() if back.get(t) == q]
        if len(pairs) < 30: continue
        tried += 1; dep = D[r]; Kr = K[('r', r)]; obj, img = [], []
        for q, t in pairs:
            u, v = pr[t]; z = dep[min(int(round(v)), 479), min(int(round(u)), 639)]
            if 0.5 < z < 12:
                obj.append((Pr[r] @ np.array([(u - Kr[0, 2]) * z / Kr[0, 0], (v - Kr[1, 2]) * z / Kr[1, 1], z, 1]))[:3]); img.append(ps[q])
        if len(obj) < 25: continue
        ok, rv, tv, inl = cv2.solvePnPRansac(np.array(obj), np.array(img), K[('s', s)], None, reprojectionError=3.0, iterationsCount=3000, confidence=0.999)
        if not ok or inl is None or len(inl) < 25: continue
        Rm, _ = cv2.Rodrigues(rv); Twc = np.eye(4); Twc[:3, :3] = Rm.T; Twc[:3, 3] = (-Rm.T @ tv).ravel()
        T = Twc @ np.linalg.inv(Ps[s]); tilt = np.degrees(np.arccos(np.clip(T[2, 2], -1, 1)))
        if tilt > 5: continue
        yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
        acc.append(dict(src=s, ref=r, mutual=len(pairs), pnp_inl=int(len(inl)), pnp_pts=len(obj), tilt=float(tilt), yaw=float(yaw), t=T[:3, 3].tolist(), T=T.tolist()))
        print(f'  채택 {s:4d}↔{r:4d}: 상호매칭 {len(pairs)}, PnP {len(inl)}/{len(obj)}, yaw {yaw:7.1f}°, 기울기 {tilt:.1f}°, 이동 ({T[0,3]:.1f},{T[1,3]:.1f},{T[2,3]:.1f})', flush=True)
json.dump(acc, open(out / 'accepted_pairs.json', 'w'), indent=1)
print(f'PnP 시도 {tried}쌍, 중력 검증 통과 {len(acc)}쌍 ({time.time()-t0:.0f}s)')
best = []
for x in acc:
    g = [y for y in acc if np.linalg.norm(np.subtract(x['t'], y['t'])[:2]) < 1.5 and abs((x['yaw'] - y['yaw'] + 180) % 360 - 180) < 5]
    if len(g) > len(best): best = g
print(f'가장 큰 일치 그룹 {len(best)}쌍: ' + ', '.join(f"{b['src']}↔{b['ref']}" for b in best))
if len(best) >= 3:
    yaw = np.degrees(np.arctan2(np.mean([np.sin(np.radians(b['yaw'])) for b in best]), np.mean([np.cos(np.radians(b['yaw'])) for b in best])))
    t = np.median([b['t'] for b in best], 0); tt = np.radians(yaw)
    T = np.eye(4); T[:2, :2] = [[np.cos(tt), -np.sin(tt)], [np.sin(tt), np.cos(tt)]]; T[:3, 3] = t
    np.savetxt(out / 'T_src_to_ref.txt', T); sp = np.std([b['t'] for b in best], 0)
    print(f'채택: yaw {yaw:.1f}°, 이동 ({t[0]:.2f},{t[1]:.2f},{t[2]:.2f}), 그룹 내 표준편차 ({sp[0]:.2f},{sp[1]:.2f},{sp[2]:.2f}) m')
else:
    print('일치 그룹 3쌍 미만 → 변환 채택 안 함')
