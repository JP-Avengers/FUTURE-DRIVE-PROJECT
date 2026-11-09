"""손 촬영 회차 ↔ v0.5 바닥 정사 영상 구간별 정합(C안).
키프레임 사진의 바닥 화소를 바닥 높이 지도로 투영해 10cm 위에서 본 영상(BEV)을 만들고, 경로 15m 구간마다
v0.5 ground_ortho 와 선 무늬(톱햇) 상관이 최대인 이동·요를 찾는다(±R m, ±Y°). 앞 구간 보정을 다음 구간 초기값으로 이어감.
사용: fe_ortho_align.py <run> <T_init.npy> <out_dir> [--seg 15] [--R 5] [--Y 20] [--maxd 6] [--from 0]"""
import sys, re, json, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, tf, O = a[1], a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
SEG, RR, YY, MAXD, S0 = opt('--seg', 15), opt('--R', 5), opt('--Y', 20), opt('--maxd', 6), opt('--from', 0)
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'; T0 = np.load(tf)
F = np.load(P / 'floor_height.npz'); G, glo, gc = F['grid'].astype(np.float32), F['lo'], float(F['cell'])
Q = np.load(P / 'outer/ground_ortho.npz'); RGB, qlo, C = Q['rgb'], Q['lo'], float(Q['cell'])
def fh(x, y): return ndimage.map_coordinates(G, [(x - glo[0]) / gc - 0.5, (y - glo[1]) / gc - 0.5], order=1, mode='nearest')
def th(img, k=5):                                       # 밝은 선 강조(톱햇), 셀 단위
    return img - ndimage.grey_opening(img, size=(k, k))
ref = RGB.astype(np.float32).mean(-1); rv = RGB.sum(-1) > 0; ref_t = th(ref) * rv     # ref[ix, iy]
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); K = E / run / 'keyframes'
tp = cid[:, 1:4]; s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(tp, axis=0), axis=1))]
calib = {}
def kf_ground(r, T, step=3):
    """키프레임 하나 → 세계(T 적용) 바닥 점 xy 와 밝기(프레임 바닥 중앙값으로 정규화)"""
    i = int(r[8])
    if i not in calib:
        txt = (K / f'kf_calib/{i}.yaml').read_text()
        calib[i] = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', txt, re.S).group(1).replace('\n', ' ').split(',')]
    k = calib[i]; g = cv2.imread(str(K / f'kf_rgb/{i}.jpg'), cv2.IMREAD_GRAYSCALE).astype(np.float32); h, w = g.shape
    v, u = np.mgrid[h // 3:h:step, 0:w:step]; u = u.ravel(); v = v.ravel()
    Rw = T[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix(); cw = T[:3, :3] @ r[1:4] + T[:3, 3]
    d = np.c_[(u - k[2]) / k[0], (v - k[5]) / k[4], np.ones(len(u))] @ Rw.T; d /= np.linalg.norm(d, axis=1, keepdims=True)
    dn = d[:, 2] < -0.05; u, v, d = u[dn], v[dn], d[dn]
    z0 = fh(np.array([cw[0]]), np.array([cw[1]]))[0]; t = (z0 - cw[2]) / d[:, 2]
    for _ in range(2): p = cw + t[:, None] * d; t = (fh(p[:, 0], p[:, 1]) - cw[2]) / d[:, 2]
    p = cw + t[:, None] * d; ok = (t > 0.8) & (t < MAXD)
    val = g[v[ok], u[ok]]; val = val / (np.median(val) + 1e-6) * 100
    return p[ok, :2], val
def bev(pts, vals, lo, n):
    ix = ((pts[:, 0] - lo[0]) / C).astype(int); iy = ((pts[:, 1] - lo[1]) / C).astype(int)
    m = (ix >= 0) & (ix < n[0]) & (iy >= 0) & (iy < n[1]); ix, iy, vals = ix[m], iy[m], vals[m]
    sm = np.zeros(n, np.float32); ct = np.zeros(n, np.float32); np.add.at(sm, (ix, iy), vals); np.add.at(ct, (ix, iy), 1)
    img = np.where(ct > 0, sm / np.maximum(ct, 1), 0); return img, ct > 0
def Sm(dx, dy, yaw, cen):
    R = Rot.from_euler('z', yaw, degrees=True).as_matrix(); S = np.eye(4); S[:3, :3] = R; S[:3, 3] = cen - R @ cen + [dx, dy, 0]; return S
T = T0.copy(); res = []; ov = []
nseg = int(np.ceil((s[-1] - S0) / SEG))
for g_ in range(nseg):
    a0, a1 = S0 + g_ * SEG, S0 + (g_ + 1) * SEG; sel = np.where((s >= a0) & (s < a1))[0]
    if len(sel) < 3: continue
    pts, vals = [], []
    for j in sel: p_, v_ = kf_ground(cid[j], T); pts.append(p_); vals.append(v_)
    pts = np.vstack(pts); vals = np.concatenate(vals)
    if len(pts) < 2000: res.append(dict(seg=g_, s0=a0, note='바닥 점 부족')); continue
    cen = np.r_[np.median(pts, 0), 0.0]; half = np.ptp(pts, 0).max() / 2 + 1.0
    lo_p = cen[:2] - half; n_p = (int(2 * half / C) + 1,) * 2
    B, Bm = bev(pts, vals, lo_p, n_p); Bt = th(B) * Bm
    if Bm.mean() < 0.05: res.append(dict(seg=g_, s0=a0, note='BEV 빈약')); continue
    # 기준 정사 영상에서 ±RR 여유를 둔 창
    pad = int(RR / C); ox0 = int((lo_p[0] - qlo[0]) / C) - pad; oy0 = int((lo_p[1] - qlo[1]) / C) - pad
    W = np.zeros((n_p[0] + 2 * pad, n_p[1] + 2 * pad), np.float32)
    xs, ys = slice(max(ox0, 0), min(ox0 + W.shape[0], ref_t.shape[0])), slice(max(oy0, 0), min(oy0 + W.shape[1], ref_t.shape[1]))
    W[xs.start - ox0:xs.stop - ox0, ys.start - oy0:ys.stop - oy0] = ref_t[xs, ys]
    if (W > 0).mean() < 0.05: res.append(dict(seg=g_, s0=a0, note='정사 영상 밖')); continue
    best = (-9, 0, 0, 0); scores = []
    for yaw in np.arange(-YY, YY + 0.1, 1.0):              # 점을 세계 좌표에서 cen 축으로 직접 회전 → BEV 다시 만듦(부호 혼동 없음)
        Rz = Rot.from_euler('z', yaw, degrees=True).as_matrix()[:2, :2]; pr = (pts - cen[:2]) @ Rz.T + cen[:2]
        Br0, Bmr = bev(pr, vals, lo_p, n_p); Br = th(Br0) * Bmr
        if Bmr.sum() < 500: continue
        tpl = np.where(Bmr, Br - Br[Bmr].mean(), 0).astype(np.float32)
        r_ = cv2.matchTemplate(W, tpl, cv2.TM_CCORR_NORMED, mask=Bmr.astype(np.float32))
        r_ = np.nan_to_num(r_, nan=-1); mi = np.unravel_index(np.argmax(r_), r_.shape)
        scores.append((float(r_[mi]), yaw, mi))
        if r_[mi] > best[0]: best = (float(r_[mi]), yaw, (mi[0] - pad) * C, (mi[1] - pad) * C)
    sc = np.array([x[0] for x in scores]); r0 = [x for x in scores if x[1] == 0]
    # 두 번째 봉우리(최고에서 1m·3° 넘게 떨어진 것 중 최고)
    second = max([x[0] for x in scores if abs(x[1] - best[1]) > 3] or [np.nan])
    sc0 = cv2.matchTemplate(W, np.where(Bm, Bt - Bt[Bm].mean(), 0).astype(np.float32), cv2.TM_CCORR_NORMED, mask=Bm.astype(np.float32))[pad, pad]
    S = Sm(best[2], best[3], best[1], cen); Tn = S @ T
    res.append(dict(seg=g_, s0=round(a0, 1), kf=len(sel), corr_init=round(float(sc0), 3), corr_best=round(best[0], 3), second_peak=round(float(second), 3),
                    dx=round(best[2], 2), dy=round(best[3], 2), yaw=float(best[1])))
    print(f'구간 {a0:5.0f}~{a1:5.0f}m kf {len(sel):3d} | 초기 r {sc0:+.3f} → 최고 {best[0]:+.3f} (다른 봉우리 {second:+.3f}) @ dx {best[2]:+.1f} dy {best[3]:+.1f} 요 {best[1]:+.0f}°', flush=True)
    # 대조 그림: 보정 후 BEV | 같은 자리 정사 영상
    pts2 = (np.c_[pts, np.zeros(len(pts)), np.ones(len(pts))] @ S.T)[:, :2]
    B2, B2m = bev(pts2, vals, lo_p, n_p); crop = np.zeros(n_p, np.float32)
    ix0, iy0 = int((lo_p[0] - qlo[0]) / C), int((lo_p[1] - qlo[1]) / C)
    xs2, ys2 = slice(max(ix0, 0), min(ix0 + n_p[0], ref.shape[0])), slice(max(iy0, 0), min(iy0 + n_p[1], ref.shape[1]))
    crop[xs2.start - ix0:xs2.stop - ix0, ys2.start - iy0:ys2.stop - iy0] = ref[xs2, ys2]
    f8 = lambda x: np.clip(x / (np.percentile(x[x > 0], 99) + 1e-6) * 255, 0, 255).astype(np.uint8) if (x > 0).any() else x.astype(np.uint8)
    pair = np.hstack([f8(B2).T[::-1], np.full((n_p[1], 4), 255, np.uint8), f8(crop).T[::-1]])
    pair = cv2.resize(cv2.cvtColor(pair, cv2.COLOR_GRAY2BGR), None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
    cv2.putText(pair, f'{run} {a0:.0f}-{a1:.0f}m r={best[0]:+.2f}', (4, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1); ov.append(pair)
    np.save(O / f'T_seg{g_:02d}.npy', Tn)
    T = Tn                                                   # 다음 구간 초기값
(O / 'segments.json').write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding='utf-8')
if ov:
    hmax = max(x.shape[0] for x in ov); wmax = max(x.shape[1] for x in ov)
    tiles = [cv2.copyMakeBorder(x, 0, hmax - x.shape[0], 0, wmax - x.shape[1], cv2.BORDER_CONSTANT) for x in ov]
    rows = [np.hstack(tiles[j:j + 2] + [np.zeros_like(tiles[0])] * (2 - len(tiles[j:j + 2]))) for j in range(0, len(tiles), 2)]
    img = np.vstack(rows); sc_ = min(1.0, 1800 / img.shape[0]); cv2.imwrite(str(O / f'{run}_ortho_align.jpg'), cv2.resize(img, None, fx=sc_, fy=sc_))
print('→', O)
