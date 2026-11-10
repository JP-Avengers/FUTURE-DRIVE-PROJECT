"""소화전 찾기: (1) 손 촬영 fems5 키프레임(IT1wall 140~150초)에서 붉은 원기둥 화소 + depth → 세계 위치(ms·nr1 배치)
(2) v0.5 점군에서 그 근처 붉은 점 덩어리 (3) v0.5 차량 키프레임 중 그 근처를 본 것에서 붉은 덩어리 → depth로 위치
사용: hydrant.py <out_dir>"""
import sys, re, json, numpy as np, cv2, open3d as o3d
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial import cKDTree
O = Path(sys.argv[1]); O.mkdir(parents=True, exist_ok=True)
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'
T0 = 1794124668.845110149
def red_mask(bgr):
    h = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV); m = (((h[..., 0] < 8) | (h[..., 0] > 170)) & (h[..., 1] > 120) & (h[..., 2] > 70)).astype(np.uint8)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)); return m
def calib(path):
    return [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', path.read_text(), re.S).group(1).replace('\n', ' ').split(',')]
def blobs(bgr, dep, k, minpx=150):
    m = red_mask(bgr); n, lab, st, cen = cv2.connectedComponentsWithStats(m); out = []
    for i in range(1, n):
        x, y, w, h, a = st[i]
        if a < minpx or h < 1.3 * w: continue                    # 세로로 긴 붉은 덩어리(소화전 몸통)
        ys, xs = np.nonzero(lab == i); z = dep[ys, xs].astype(float) / 1000; ok = (z > 0.3) & (z < 12)
        if ok.sum() < 30: continue
        zz = np.median(z[ok]); u, v = np.median(xs[ok]), np.median(ys[ok])
        out.append(dict(u=float(u), v=float(v), z=float(zz), area=int(a), box=[int(x), int(y), int(w), int(h)], pc=np.array([(u - k[2]) / k[0] * zz, (v - k[5]) / k[4] * zz, zz])))
    return out
# (1) 손 촬영
res = {}
for tag, tf in (('ms', E / 'fems5/traj_camera_id_camera_poses.txt'), ('nr1', E / 'nr1/fems5c/traj_camera_id_camera_poses.txt')):
    cid = np.loadtxt(tf, comments='#', ndmin=2); T = np.load(E / 'align_fems5_ms_from_fecen.npy') if tag == 'ms' else np.eye(4)
    K = E / 'fems5/keyframes'; sel = np.where((cid[:, 0] > T0 + 135) & (cid[:, 0] < T0 + 160))[0]; pts = []
    for j in sel:
        r = cid[j]; i = int(r[8]); im = cv2.imread(str(K / f'kf_rgb/{i}.jpg')); dp = cv2.imread(str(K / f'kf_depth/{i}.png'), -1); k = calib(K / f'kf_calib/{i}.yaml')
        Rw = T[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix(); cw = T[:3, :3] @ r[1:4] + T[:3, 3]
        for b in blobs(im, dp, k):
            w = Rw @ b['pc'] + cw; pts.append(w); print(f'  [{tag}] kf{i} t+{r[0] - T0:.1f}s 붉은 덩어리 {b["area"]}px z={b["z"]:.2f}m → 세계 ({w[0]:+.2f},{w[1]:+.2f},{w[2]:+.2f})')
            if tag == 'ms': cv2.rectangle(im, tuple(b['box'][:2]), (b['box'][0] + b['box'][2], b['box'][1] + b['box'][3]), (0, 255, 0), 2); cv2.imwrite(str(O / f'hand_kf{i}.jpg'), im)
    if pts: res[tag] = np.median(np.array(pts), 0); print(f'손 촬영 소화전({tag}): ({res[tag][0]:+.2f},{res[tag][1]:+.2f},{res[tag][2]:+.2f}), 관측 {len(pts)}개, 흩어짐 {np.linalg.norm(np.array(pts)[:, :2] - res[tag][:2], axis=1).max():.2f}m')
if not res: raise SystemExit('손 촬영에서 소화전 못 찾음')
c0 = res['ms']
# (2) v0.5 점군 붉은 점
pc = o3d.io.read_point_cloud(str(P / 'cloud_3cm.ply')); X = np.asarray(pc.points); Cc = (np.asarray(pc.colors) * 255).astype(np.uint8)
hsv = cv2.cvtColor(Cc[None, :, ::-1], cv2.COLOR_BGR2HSV)[0]; red = ((hsv[:, 0] < 8) | (hsv[:, 0] > 170)) & (hsv[:, 1] > 120) & (hsv[:, 2] > 70)
near = np.linalg.norm(X[:, :2] - c0[:2], axis=1) < 8; R = X[near & red]
print(f'\nv0.5 점군: 손 촬영 위치 8m 안 점 {near.sum():,}, 그중 붉은 점 {len(R)}')
if len(R):
    lab = np.array(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(R)).cluster_dbscan(0.15, 5))
    for l in range(lab.max() + 1):
        q = R[lab == l]; print(f'   덩어리 {l}: {len(q)}점 중심 ({q[:, 0].mean():+.2f},{q[:, 1].mean():+.2f}) 높이 범위 {np.ptp(q[:, 2]):.2f}m')
# (3) v0.5 차량 키프레임
found = []
for s in ('bonnet_cw', 'bonnet_ccw', 'roof_cw', 'roof_ccw'):
    tr = np.loadtxt(P / f'trajectory_{s}_camera_optical.txt', comments='#', ndmin=2); D = P / f'keyframes_{s}'
    for r in tr:
        Rw = Rot.from_quat(r[4:8]).as_matrix(); c = r[1:4]; pcam = Rw.T @ (c0 - c)
        if not (1.0 < pcam[2] < 15) or np.linalg.norm(pcam[:2] / pcam[2]) > 0.75: continue      # 손 촬영 추정 위치가 시야 안
        i = int(r[8]); im = cv2.imread(str(D / f'rgb/{i}.jpg')); dpf = D / f'depth/{i}.png'
        if im is None or not dpf.exists(): continue
        dp = cv2.imread(str(dpf), -1); k = calib(D / f'calib/{i}.yaml')
        for b in blobs(im, dp, k, minpx=40):
            w = Rw @ b['pc'] + c; d = np.linalg.norm(w[:2] - c0[:2])
            if d < 6: found.append(dict(set=s, kf=i, world=w.round(2).tolist(), dist_to_hand=round(float(d), 2), z=round(b['z'], 2), area=b['area']))
            if d < 6:
                cv2.rectangle(im, tuple(b['box'][:2]), (b['box'][0] + b['box'][2], b['box'][1] + b['box'][3]), (0, 255, 0), 2)
                cv2.putText(im, f'{s} {i} d={d:.1f}m', (5, 20), 0, 0.6, (0, 255, 255), 2); cv2.imwrite(str(O / f'v05_{s}_{i}.jpg'), im)
print(f'\nv0.5 키프레임에서 손 촬영 위치 6m 안 붉은 세로 덩어리 {len(found)}개')
for f in found: print('  ', f)
if found:
    W = np.array([f['world'] for f in found]); med = np.median(W, 0); print(f'v0.5 소화전 추정(중앙): ({med[0]:+.2f},{med[1]:+.2f},{med[2]:+.2f}), 관측 흩어짐 최대 {np.linalg.norm(W[:, :2] - med[:2], axis=1).max():.2f}m')
    for tag, c in res.items(): print(f'  손 촬영({tag}) → v0.5 어긋남 ({med[0] - c[0]:+.2f},{med[1] - c[1]:+.2f}) = {np.linalg.norm(med[:2] - c[:2]):.2f}m')
json.dump(dict(hand={k: v.round(3).tolist() for k, v in res.items()}, v05=found), open(O / 'hydrant.json', 'w'), ensure_ascii=False, indent=1)
