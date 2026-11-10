"""동기 경로 그림(PDF 쪽 PNG)에서 손으로 그린 빨간 선을 맵 좌표로 뽑아, 묶음 맵 세션 궤적(배치판별)과 거리 비교.
원본 그리기판(route_drawing_sheet.png)을 SIFT+RANSAC으로 쪽 그림에 맞춰, 원본에 없던 빨간 화소 = 그린 선. 짧은 조각(빗금·점선)은 빼고 긴 선만.
사용: route_digitize.py <sheet.png> <pages_dir> <out_dir>"""
import sys, json, numpy as np, cv2
from pathlib import Path
from scipy.spatial import cKDTree
SHEET, PG, O = sys.argv[1], Path(sys.argv[2]), Path(sys.argv[3]); O.mkdir(parents=True, exist_ok=True)
X0, Y1, C, SC = -62, 26, 0.1, 2; MAPH = 1240                                   # route_sheet.py 와 같은 값
def s2w(u, v): return np.c_[X0 + u / SC * C, Y1 - v / SC * C]
def w2s(xy): return np.c_[(xy[:, 0] - X0) / C * SC, (Y1 - xy[:, 1]) / C * SC]
pages = {1: 's1_feit1', 2: 's3_fe422a', 3: 's5_feit1w', 4: 's4_fe422b', 5: 's2_fecen', 6: 's0_feone'}
SESS = {'s0_feone': (1794125601.8, 1794125934.5), 's1_feit1': (1794124670.4, 1794124862.7), 's2_fecen': (1794126172.0, 1794126316.2),
        's3_fe422a': (1794124958.7, 1794125118.4), 's4_fe422b': (1794126402.2, 1794126508.4), 's5_feit1w': (1794125227.8, 1794125358.0)}
E = Path.home() / 'b_slam/exports'; Tms = np.load(E / 'align_fems5_ms_from_fecen.npy')
tr = np.loadtxt(E / 'fems5/traj_optimized_poses.txt', comments='#', ndmin=2); P_ms = tr[:, 1:4] @ Tms[:3, :3].T + Tms[:3, 3]
trc = np.loadtxt(E / 'nr1/fems5c/traj_optimized_poses.txt', comments='#', ndmin=2); P_nr = trc[:, 1:4]
sheet = cv2.imread(SHEET); sg = cv2.cvtColor(sheet, cv2.COLOR_BGR2GRAY); sift = cv2.SIFT_create(6000)
ks, ds = sift.detectAndCompute(sg[:MAPH], None)
Q = np.load(Path.home() / 'b_slam/deliver/map_v0.5/outer/ground_ortho.npz')
res = {}
for pno, sess in pages.items():
    pg = cv2.imread(str(PG / f'p{pno:02d}.png')); gg = cv2.cvtColor(pg, cv2.COLOR_BGR2GRAY)
    kp, dp = sift.detectAndCompute(gg, None)
    m = cv2.BFMatcher().knnMatch(ds, dp, k=2); good = [a for a, b in m if a.distance < 0.75 * b.distance]
    A, inl = cv2.estimateAffinePartial2D(np.float32([ks[g.queryIdx].pt for g in good]), np.float32([kp[g.trainIdx].pt for g in good]), method=cv2.RANSAC, ransacReprojThreshold=3.0)
    sc_ = np.hypot(A[0, 0], A[1, 0])
    warp = cv2.warpAffine(sheet, A, (pg.shape[1], pg.shape[0]), flags=cv2.INTER_LINEAR)
    b, g, r = [pg[..., i].astype(int) for i in range(3)]; wb, wg, wr = [warp[..., i].astype(int) for i in range(3)]
    red = (r > 150) & (g < 110) & (b < 110); was_red = (wr > 140) & (wg < 120) & (wb < 120)
    newred = red & ~cv2.dilate(was_red.astype(np.uint8), np.ones((7, 7), np.uint8)).astype(bool)
    Ainv = cv2.invertAffineTransform(A); vv, uu = np.nonzero(newred)
    su = Ainv[0, 0] * uu + Ainv[0, 1] * vv + Ainv[0, 2]; sv = Ainv[1, 0] * uu + Ainv[1, 1] * vv + Ainv[1, 2]
    inmap = (su >= 0) & (su < sheet.shape[1]) & (sv >= 0) & (sv < MAPH); uu, vv, su, sv = uu[inmap], vv[inmap], su[inmap], sv[inmap]
    # 긴 선만: 쪽 그림 연결 요소의 바깥 길이(대각선) 기준
    mask = np.zeros(newred.shape, np.uint8); mask[vv, uu] = 1; mask = cv2.dilate(mask, np.ones((3, 3), np.uint8))
    n, lab, st, _ = cv2.connectedComponentsWithStats(mask); diag = np.hypot(st[:, 2], st[:, 3]) / sc_ * C / SC     # m 단위
    longc = np.where(diag > 6)[0]; longc = longc[longc > 0]; keep = np.isin(lab[vv, uu], longc)
    W = s2w(su[keep], sv[keep]); Wall = s2w(su, sv)
    t0, t1 = SESS[sess]; k = (tr[:, 0] >= t0) & (tr[:, 0] <= t1); kc = (trc[:, 0] >= t0) & (trc[:, 0] <= t1)
    tw = cKDTree(W); out = dict(page=pno, affine_scale=round(float(sc_), 3), ransac_inliers=int(inl.sum()), drawn_pts=int(keep.sum()), short_pts=int((~keep).sum()))
    for nm, P in (('ms', P_ms[k, :2]), ('nr1', P_nr[kc, :2])):
        d1, _ = tw.query(P); d2, _ = cKDTree(P).query(W)
        out[nm] = dict(traj_to_drawn_med=round(float(np.median(d1)), 2), traj_to_drawn_p90=round(float(np.percentile(d1, 90)), 2),
                       drawn_to_traj_med=round(float(np.median(d2)), 2), drawn_to_traj_p90=round(float(np.percentile(d2, 90)), 2),
                       start=np.round(P[0], 1).tolist(), end=np.round(P[-1], 1).tolist())
    res[sess] = out
    print(f"{sess:10s} p{pno} 맞춤 배율 {sc_:.3f} 안쪽 {int(inl.sum())} | 그린 선 점 {keep.sum()} (짧은 조각 {(~keep).sum()})")
    for nm in ('ms', 'nr1'):
        o = out[nm]; print(f"   {nm:3s} 궤적→그림 중앙 {o['traj_to_drawn_med']:.1f}m p90 {o['traj_to_drawn_p90']:.1f}m | 그림→궤적 중앙 {o['drawn_to_traj_med']:.1f}m p90 {o['drawn_to_traj_p90']:.1f}m | 시작 {o['start']} 끝 {o['end']}")
    # 그림: 그리기판 지도 위 그린 선(빨강)·짧은 조각(분홍)·ms 궤적(파랑)·nr1 궤적(노랑)
    img = sheet[:MAPH].copy(); img = (img * 0.55).astype(np.uint8)
    for xy, col, rr in ((Wall, (180, 120, 255), 1), (W, (0, 0, 255), 2), (P_ms[k, :2], (255, 120, 0), 2), (P_nr[kc, :2], (0, 230, 255), 2)):
        for q in w2s(xy).astype(int)[::(1 if col[2] == 255 and col[0] == 0 else 2)]: cv2.circle(img, tuple(q), rr, col, -1)
    for xy, tag in ((P_ms[k][0, :2], 'S'), (P_ms[k][-1, :2], 'E')):
        q = w2s(xy[None])[0].astype(int); cv2.putText(img, tag, tuple(q + 5), 0, 0.8, (255, 120, 0), 2)
    cv2.putText(img, f'{sess}: red=drawn  blue=ms(rigid)  yellow=nr1', (20, 40), 0, 1.0, (255, 255, 255), 2)
    cv2.imwrite(str(O / f'cmp_{sess}.jpg'), img[::2, ::2])
json.dump(res, open(O / 'route_compare.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
