#!/usr/bin/env python3
"""두 맵의 키프레임 사이 직접 이미지 매칭으로 맵 간 변환 추정 (원본은 읽기만).
1) 모든 키프레임 RootSIFT → 모든 쌍 FLANN 매칭 + 비율 검사 + 기본행렬 RANSAC 인라이어 수
2) 상위 쌍: ref 키프레임 depth로 3D(ref 월드) ↔ src 2D → PnP RANSAC → src 카메라의 ref 월드 자세
   → T_src_to_ref = T_ref_world_cam · inv(T_src_world_cam)
3) 여러 쌍의 변환이 서로 일치하는지 확인
사용: xmap_match.py <ref_run> <src_run> <out_dir> [top_pairs=40]"""
import sys, re, json, time
from pathlib import Path
import numpy as np, cv2
ref_run, src_run, out = sys.argv[1], sys.argv[2], Path(sys.argv[3]); top = int(sys.argv[4]) if len(sys.argv) > 4 else 40
out.mkdir(parents=True, exist_ok=True); H = Path.home() / 'b_slam/exports'
def qR(qx, qy, qz, qw):
    return np.array([[1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
                     [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
                     [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)]])
def load(run):
    tr = np.loadtxt(H / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); P = {}
    for r in tr:
        T = np.eye(4); T[:3, :3] = qR(*r[4:8]); T[:3, 3] = r[1:4]; P[int(r[8])] = T
    return P
Kmat = None
def intr(run, i):
    t = (H / run / f'keyframes/kf_calib/{i}.yaml').read_text()
    a = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', t, re.S).group(1).replace('\n', ' ').split(',')]
    return np.array([[a[0], 0, a[2]], [0, a[4], a[5]], [0, 0, 1]])
sift = cv2.SIFT_create(nfeatures=1500)
def feats(run, ids):
    F = {}
    for i in ids:
        g = cv2.imread(str(H / run / f'keyframes/kf_rgb/{i}.jpg'), cv2.IMREAD_GRAYSCALE)
        kp, d = sift.detectAndCompute(g, None)
        if d is None or len(kp) < 50: continue
        d = d / (np.abs(d).sum(1, keepdims=True) + 1e-7); d = np.sqrt(d).astype(np.float32)  # RootSIFT
        F[i] = (np.array([k.pt for k in kp], np.float32), d)
    return F
t0 = time.time()
Pr, Ps = load(ref_run), load(src_run)
Fr, Fs = feats(ref_run, sorted(Pr)), feats(src_run, sorted(Ps))
print(f'특징 추출: ref {len(Fr)}장, src {len(Fs)}장 ({time.time()-t0:.0f}s)')
flann = cv2.FlannBasedMatcher(dict(algorithm=1, trees=4), dict(checks=32))
scores = []
for s, (ps, ds) in Fs.items():
    for r, (pr, dr) in Fr.items():
        m = flann.knnMatch(ds, dr, k=2)
        good = [a for a, b in (x for x in m if len(x) == 2) if a.distance < 0.75 * b.distance]
        if len(good) < 25: continue
        a = ps[[g.queryIdx for g in good]]; b = pr[[g.trainIdx for g in good]]
        Fm, mask = cv2.findFundamentalMat(a, b, cv2.FM_RANSAC, 2.0, 0.999)
        if mask is None: continue
        scores.append((int(mask.sum()), s, r))
scores.sort(reverse=True)
print(f'쌍 매칭: {len(scores)}쌍이 25개 이상 매칭 ({time.time()-t0:.0f}s). 상위 10: ' + ', '.join(f'{n}({s}↔{r})' for n, s, r in scores[:10]))
res = []
for n, s, r in scores[:top]:
    ps, ds = Fs[s]; pr, dr = Fr[r]
    m = flann.knnMatch(ds, dr, k=2)
    good = [a for a, b in (x for x in m if len(x) == 2) if a.distance < 0.75 * b.distance]
    dep = cv2.imread(str(H / ref_run / f'keyframes/kf_depth/{r}.png'), cv2.IMREAD_UNCHANGED).astype(float) / 1000
    K_r, K_s = intr(ref_run, r), intr(src_run, s)
    obj, img = [], []
    for g in good:
        u, v = pr[g.trainIdx]; z = dep[int(round(v)) % 480, int(round(u)) % 640]
        if 0.5 < z < 12:
            pc = np.array([(u - K_r[0, 2]) * z / K_r[0, 0], (v - K_r[1, 2]) * z / K_r[1, 1], z, 1])
            obj.append((Pr[r] @ pc)[:3]); img.append(ps[g.queryIdx])
    if len(obj) < 12: continue
    ok, rv, tv, inl = cv2.solvePnPRansac(np.array(obj), np.array(img), K_s, None, reprojectionError=4.0, iterationsCount=2000, confidence=0.999)
    if not ok or inl is None or len(inl) < 12: continue
    Rm, _ = cv2.Rodrigues(rv); Twc = np.eye(4); Twc[:3, :3] = Rm.T; Twc[:3, 3] = (-Rm.T @ tv).ravel()  # src 카메라의 ref 월드 자세
    T = Twc @ np.linalg.inv(Ps[s])
    tilt = np.degrees(np.arccos(np.clip(T[2, 2], -1, 1))); yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0]))
    res.append(dict(src=s, ref=r, F_inl=n, pnp_pts=len(obj), pnp_inl=int(len(inl)), tilt=tilt, yaw=yaw, t=T[:3, 3].tolist(), T=T.tolist()))
    print(f'  {s:4d}↔{r:4d}: F인라이어 {n:3d}, PnP {len(inl):3d}/{len(obj):3d}, yaw {yaw:7.1f}°, 기울기 {tilt:4.1f}°, 이동 ({T[0,3]:6.1f},{T[1,3]:6.1f},{T[2,3]:5.1f})')
json.dump(res, open(out / 'pairs.json', 'w'), indent=1)
# 일치 검사: 기울기 < 5° 인 것 중, 다른 쌍과 이동 1.5m·yaw 5° 이내로 몇 개가 모이는지
cand = [x for x in res if x['tilt'] < 5]
best = None
for x in cand:
    grp = [y for y in cand if np.linalg.norm(np.subtract(x['t'], y['t'])[:2]) < 1.5 and abs((x['yaw'] - y['yaw'] + 180) % 360 - 180) < 5]
    if best is None or len(grp) > len(best): best = grp
print(f'정상 기울기 쌍 {len(cand)}개, 가장 큰 일치 그룹 {len(best) if best else 0}개')
if best and len(best) >= 3:
    yaw = np.degrees(np.arctan2(np.mean([np.sin(np.radians(b['yaw'])) for b in best]), np.mean([np.cos(np.radians(b['yaw'])) for b in best])))
    t = np.median([b['t'] for b in best], 0); tt = np.radians(yaw)
    T = np.eye(4); T[:2, :2] = [[np.cos(tt), -np.sin(tt)], [np.sin(tt), np.cos(tt)]]; T[:3, 3] = t
    np.savetxt(out / 'T_src_to_ref.txt', T)
    sp = np.std([b['t'] for b in best], 0)
    print(f'채택 후보: yaw {yaw:.1f}°, 이동 ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f}), 그룹 내 이동 표준편차 ({sp[0]:.2f}, {sp[1]:.2f}, {sp[2]:.2f}) m → {out}/T_src_to_ref.txt')
else:
    print('일치하는 그룹이 3개 미만 → 변환 채택 안 함')
print(f'총 {time.time()-t0:.0f}s')
