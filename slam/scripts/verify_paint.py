"""정합 좌우 위치 검증: v0.5 컬러 점군의 바닥 주차선(밝은 바닥 점)을 키프레임 사진에 투영해 사진의 흰 선과 겹치는 비율을 잰다.
현재 정합 T 와, T 를 세계 x·y 로 ±0.6m·요 ±2° 흔든 경우를 비교 → 현재 T 가 국소 최적이면 정합이 맞는 것.
사용: verify_paint.py <run> <T.npy> <out_dir> [--kf 16] [--from 0 --to 1e9 (경로 m 범위)]"""
import sys, re, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, tf, O = a[1], a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
NK = int(opt('--kf', 16)); S0, S1 = opt('--from', 0), opt('--to', 1e9)
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'; T = np.load(tf)
ref = o3d.io.read_point_cloud(str(P / 'cloud_3cm.ply'))
ref.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=20))
X = np.asarray(ref.points); C = np.asarray(ref.colors).mean(1); N = np.abs(np.asarray(ref.normals)[:, 2])
paint = X[(C > 0.72) & (N > 0.85)]; floor_dark = X[(C < 0.5) & (N > 0.85)]
print(f'v0.5 바닥 주차선 점 {len(paint):,}')
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); K = E / run / 'keyframes'
tp = cid[:, 1:4]; s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(tp, axis=0), axis=1))]
idx = np.where((s >= S0) & (s <= S1))[0]; pick = cid[idx[np.linspace(0, len(idx) - 1, NK).astype(int)]]
shifts = [(dx, dy, dyaw) for dyaw in (-2, 0, 2) for dx in np.arange(-0.6, 0.61, 0.2) for dy in np.arange(-0.6, 0.61, 0.2)]
def Tshift(dx, dy, dyaw):        # 고른 키프레임 중심(세계)을 축으로 요 회전 + 수평 이동
    cen = (pick[:, 1:4] @ T[:3, :3].T + T[:3, 3]).mean(0); R = Rot.from_euler('z', dyaw, degrees=True).as_matrix()
    S = np.eye(4); S[:3, :3] = R; S[:3, 3] = cen - R @ cen + np.array([dx, dy, 0]); return S @ T
frames = []
for r in pick:
    i = int(r[8]); txt = (K / f'kf_calib/{i}.yaml').read_text()
    k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', txt, re.S).group(1).replace('\n', ' ').split(',')]
    im = cv2.imread(str(K / f'kf_rgb/{i}.jpg')); h, w = im.shape[:2]; g = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    th = cv2.subtract(g, cv2.morphologyEx(g, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (31, 31))))
    mask = cv2.dilate((th > 35).astype(np.uint8), np.ones((5, 5), np.uint8)); mask[: h // 3] = 0   # 위 1/3(하늘·차) 제외
    frames.append((i, k, im, mask, Rot.from_quat(r[4:8]).as_matrix(), r[1:4], s[np.argmin(np.linalg.norm(tp - r[1:4], axis=1))]))
def score(Tm, keep_overlay=False):
    hits = tot = 0; ov = []
    for i, k, im, mask, Ro, c, sm in frames:
        Rw = Tm[:3, :3] @ Ro; cw = Tm[:3, :3] @ c + Tm[:3, 3]
        pc = (paint - cw) @ Rw; z = pc[:, 2]; m = (z > 1) & (z < 10); pc = pc[m]
        u = (k[0] * pc[:, 0] / pc[:, 2] + k[2]).astype(int); v = (k[4] * pc[:, 1] / pc[:, 2] + k[5]).astype(int)
        h, w = mask.shape; ok = (u >= 0) & (u < w) & (v >= h // 3) & (v < h); u, v = u[ok], v[ok]
        if len(u) < 50: continue
        hits += mask[v, u].sum(); tot += len(u)
        if keep_overlay:
            o = im.copy(); o[mask > 0] = (o[mask > 0] * 0.5 + np.array([255, 0, 0]) * 0.5).astype(np.uint8); o[v, u] = (0, 255, 0)
            cv2.putText(o, f'{run} kf{i} {sm:.0f}m  hit {mask[v, u].mean():.0%}', (6, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2); ov.append(o)
    return (hits / tot if tot else 0), tot, ov
s0, n0, ov = score(T, True)
res = [(dx, dy, dyaw, *score(Tshift(dx, dy, dyaw))[:2]) for dx, dy, dyaw in shifts]
best = max(res, key=lambda t: t[3]); sc = np.array([t[3] for t in res])
lines = [f'{run} 경로 {S0:.0f}~{min(S1, s[-1]):.0f}m, 키프레임 {len(frames)}장',
         f'현재 정합: 주차선 점이 사진 흰 선 위 {s0:.1%} (투영 점 {n0:,})',
         f'흔든 49×3 경우: 중앙 {np.median(sc):.1%}, 최고 {best[3]:.1%} @ dx {best[0]:+.1f} dy {best[1]:+.1f} 요 {best[2]:+d}°',
         f'판정: ' + ('현재 정합이 국소 최적 근처(최고와 차이 < 2%p, 이동 ≤0.2m)' if (best[3] - s0 < 0.02 or (abs(best[0]) <= 0.2 and abs(best[1]) <= 0.2 and best[2] == 0)) else '현재 정합보다 나은 위치 있음 → 어긋남')]
print('\n'.join(lines)); (O / f'{run}_paint_{int(S0)}_{int(min(S1, 9999))}.txt').write_text('\n'.join(lines) + '\n', encoding='utf-8')
if ov:
    rows = [np.hstack(ov[j:j + 4] + [np.zeros_like(ov[0])] * (4 - len(ov[j:j + 4]))) for j in range(0, len(ov), 4)]
    cv2.imwrite(str(O / f'{run}_paint_check.jpg'), np.vstack(rows)[::2, ::2], [cv2.IMWRITE_JPEG_QUALITY, 80])
