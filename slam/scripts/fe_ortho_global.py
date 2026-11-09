"""손 촬영 회차 ↔ v0.5 바닥 정사 영상: 구간별 전역 후보 탐색 + 이웃 구간 일관성으로 후보 선택(C안 2단계).
1) 구간(경로 SEG m)마다 사진 바닥 BEV(20cm)를 만들어 정사 영상 전체에서 요 0~360°(2°) 상관 → 서로 떨어진 상위 K 후보
2) 후보마다 10cm·±0.4m·±2° 정밀화
3) 이웃 구간 후보 쌍의 상대 자세가 SLAM 상대 자세와 맞는 정도(벌점)와 상관 점수로 동적 계획 → 구간별 선택
사용: fe_ortho_global.py <run> <T_init.npy> <out_dir> [--seg 15] [--K 6] [--maxd 6]
T_init 은 높이·기울기만 쓰임(바닥 투영용). 수평 위치·요는 전역 탐색."""
import sys, re, json, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot
a = sys.argv; run, tf, O = a[1], a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
SEG, KK, MAXD = opt('--seg', 15), int(opt('--K', 6)), opt('--maxd', 6)
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'; T0 = np.load(tf)
F = np.load(P / 'floor_height.npz'); G, glo, gc = F['grid'].astype(np.float32), F['lo'], float(F['cell'])
Q = np.load(P / 'outer/ground_ortho.npz'); RGB, qlo, C1 = Q['rgb'], Q['lo'], float(Q['cell'])
def fh(x, y): return ndimage.map_coordinates(G, [(x - glo[0]) / gc - 0.5, (y - glo[1]) / gc - 0.5], order=1, mode='nearest')
def th(img, k): return img - ndimage.grey_opening(img, size=(k, k))
ref1 = RGB.astype(np.float32).mean(-1); rv1 = RGB.sum(-1) > 0
ref1_t = th(ref1, 5) * rv1                                               # 10cm
ref2 = cv2.resize(ref1, (ref1.shape[1] // 2, ref1.shape[0] // 2), interpolation=cv2.INTER_AREA)
rv2 = cv2.resize(rv1.astype(np.float32), (ref1.shape[1] // 2, ref1.shape[0] // 2), interpolation=cv2.INTER_AREA) > 0.99
ref2_t = (th(ref2, 3) * rv2).astype(np.float32); C2 = 2 * C1              # 20cm
VAL = {C1: rv1.astype(np.float32), C2: rv2.astype(np.float32)}           # 칸 크기별 정사 영상 유효 지도
cid = np.loadtxt(E / run / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2); K = E / run / 'keyframes'
tp = cid[:, 1:4]; s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(tp, axis=0), axis=1))]
calib = {}
def kf_ground(r, T, step=3):
    i = int(r[8])
    if i not in calib:
        txt = (K / f'kf_calib/{i}.yaml').read_text()
        calib[i] = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', txt, re.S).group(1).replace('\n', ' ').split(',')]
    k = calib[i]; g = cv2.imread(str(K / f'kf_rgb/{i}.jpg'), cv2.IMREAD_GRAYSCALE).astype(np.float32); h, w = g.shape
    v, u = np.mgrid[h // 3:h:step, 0:w:step]; u = u.ravel(); v = v.ravel()
    Rw = T[:3, :3] @ Rot.from_quat(r[4:8]).as_matrix(); cw = T[:3, :3] @ r[1:4] + T[:3, 3]
    d = np.c_[(u - k[2]) / k[0], (v - k[5]) / k[4], np.ones(len(u))] @ Rw.T; d /= np.linalg.norm(d, axis=1, keepdims=True)
    dn = d[:, 2] < -0.05; u, v, d = u[dn], v[dn], d[dn]
    # 바닥은 카메라 아래 국소 평면으로 근사(수평 위치가 미정이므로 높이 지도 대신 v0.5 바닥 중앙 높이 사용)
    t = (Z0 - cw[2]) / d[:, 2]; p = cw + t[:, None] * d; ok = (t > 0.8) & (t < MAXD)
    val = g[v[ok], u[ok]]; val = val / (np.median(val) + 1e-6) * 100
    return p[ok, :2], val
Z0 = float(np.median(G[np.isfinite(G)]))
def bev(pts, vals, lo, n, C):
    ix = ((pts[:, 0] - lo[0]) / C).astype(int); iy = ((pts[:, 1] - lo[1]) / C).astype(int)
    m = (ix >= 0) & (ix < n[0]) & (iy >= 0) & (iy < n[1]); ix, iy, vals = ix[m], iy[m], vals[m]
    sm = np.zeros(n, np.float32); ct = np.zeros(n, np.float32); np.add.at(sm, (ix, iy), vals); np.add.at(ct, (ix, iy), 1)
    return np.where(ct > 0, sm / np.maximum(ct, 1), 0).astype(np.float32), ct > 0
def match(pts, vals, cen, yaw, C, reft, k, region=None):
    """cen 축 yaw 회전한 BEV를 정사 영상(reft, 칸 C)에 맞춤 → 상관 지도와 BEV 원점(lo)"""
    Rz = Rot.from_euler('z', yaw, degrees=True).as_matrix()[:2, :2]; pr = (pts - cen) @ Rz.T + cen
    half = np.abs(pr - cen).max() + 0.5; lo = cen - half; n = (int(2 * half / C) + 1,) * 2
    B, Bm = bev(pr, vals, lo, n, C); Bt = th(B, k) * Bm
    if Bm.sum() < 300: return None
    tpl = np.where(Bm, Bt - Bt[Bm].mean(), 0).astype(np.float32)
    vref = VAL[C]; W, V, off = reft, vref, (0, 0)
    if region is not None:
        (x0, y0), (x1, y1) = region; W = reft[x0:x1, y0:y1]; V = vref[x0:x1, y0:y1]; off = (x0, y0)
        if W.shape[0] < n[0] or W.shape[1] < n[1]: return None
    M = Bm.astype(np.float32); nM = M.sum()
    num = cv2.matchTemplate(W, tpl, cv2.TM_CCORR)                            # 마스크 안 영 평균 템플릿 → W 의 상수항 무관
    Ws = cv2.matchTemplate(W, M, cv2.TM_CCORR); W2 = cv2.matchTemplate(W * W, M, cv2.TM_CCORR)
    cov = cv2.matchTemplate(V, M, cv2.TM_CCORR) / nM
    var = np.maximum(W2 - Ws * Ws / nM, 0); r_ = num / (np.sqrt(var) * np.linalg.norm(tpl) + 1e-3)
    r_[(cov < 0.6) | (var < 1e-2 * nM)] = -1
    return r_, lo, off
def to_T(yaw, cen, lo, cell_xy, C):
    """BEV 원점 lo 가 정사 영상 칸 cell_xy 로 가는 이동 → 수평 보정 S(세계)"""
    t = np.array([qlo[0] + cell_xy[0] * C, qlo[1] + cell_xy[1] * C]) - lo
    R = Rot.from_euler('z', yaw, degrees=True).as_matrix(); S = np.eye(4); S[:3, :3] = R
    S[:2, 3] = (cen - R[:2, :2] @ cen) + t; return S
# 구간 BEV 점(T0 기준) 준비
segs = []
for g_ in range(int(np.ceil(s[-1] / SEG))):
    sel = np.where((s >= g_ * SEG) & (s < (g_ + 1) * SEG))[0]
    if len(sel) < 3: continue
    pv = [kf_ground(cid[j], T0) for j in sel]; pts = np.vstack([x[0] for x in pv]); vals = np.concatenate([x[1] for x in pv])
    if len(pts) < 2000: continue
    cen = np.median(pts, 0); segs.append(dict(g=g_, sel=sel, pts=pts, vals=vals, cen=cen, kc=tp[sel].mean(0)))
print(f'{run}: 구간 {len(segs)}개 (경로 {s[-1]:.0f}m)', flush=True)
for sg in segs:
    cands = []
    for yaw in np.arange(0, 360, 2.0):
        m = match(sg['pts'], sg['vals'], sg['cen'], yaw, C2, ref2_t, 3)
        if m is None: continue
        r_, lo, _ = m
        for _ in range(3):                                           # 각도마다 상위 3 봉우리
            mi = np.unravel_index(np.argmax(r_), r_.shape); cands.append((float(r_[mi]), yaw, mi, lo))
            r_[max(mi[0] - 10, 0):mi[0] + 11, max(mi[1] - 10, 0):mi[1] + 11] = -1
    cands.sort(key=lambda x: -x[0]); keep = []
    for c in cands:                                                  # 서로 2m·10° 넘게 떨어진 상위 K
        S = to_T(c[1], sg['cen'], c[3], c[2], C2); xy = S[:2, :2] @ sg['cen'] + S[:2, 3]
        if all(np.linalg.norm(xy - k_['xy']) > 2 or abs((c[1] - k_['yaw'] + 180) % 360 - 180) > 10 for k_ in keep):
            keep.append(dict(score2=c[0], yaw=c[1], xy=xy))
        if len(keep) >= KK: break
    # 정밀화(10cm)
    for k_ in keep:
        best = None
        for dyaw in (-2, -1, 0, 1, 2):
            yaw = k_['yaw'] + dyaw; c0 = (k_['xy'] - qlo) / C1                     # 구간 중심이 갈 정사 영상 칸
            half = np.abs(sg['pts'] - sg['cen']).max() + 0.5; nb = int(2 * half / C1) + 1
            x0 = int(c0[0] - nb / 2 - 6); y0 = int(c0[1] - nb / 2 - 6)
            if x0 < 0 or y0 < 0: continue
            m = match(sg['pts'], sg['vals'], sg['cen'], yaw, C1, ref1_t.astype(np.float32), 5, region=((x0, y0), (x0 + nb + 12, y0 + nb + 12)))
            if m is None: continue
            r_, lo, off = m; mi = np.unravel_index(np.argmax(r_), r_.shape)
            if best is None or r_[mi] > best[0]: best = (float(r_[mi]), yaw, (mi[0] + off[0], mi[1] + off[1]), lo)
        if best is None: k_['score'] = k_['score2']; k_['S'] = None; continue
        S = to_T(best[1], sg['cen'], best[3], best[2], C1); k_.update(score=best[0], yaw=best[1] % 360, S=S, xy=S[:2, :2] @ sg['cen'] + S[:2, 3])
    sg['cands'] = [k_ for k_ in keep if k_.get('S') is not None]
    print(f"구간 {sg['g']*SEG:4.0f}m: " + ' | '.join(f"r {k_['score']:+.2f} 요 {k_['yaw']:5.1f} xy ({k_['xy'][0]:+5.1f},{k_['xy'][1]:+5.1f})" for k_ in sg['cands'][:4]), flush=True)
# 동적 계획: 점수 - λ·(이웃 상대 자세 불일치)
LAM_T, LAM_R = 0.04, 0.01                                             # 1m 불일치 = 상관 0.04, 1° = 0.01
def rel_err(Sa, Sb, ka, kb):
    pa = Sa[:3, :3] @ ka + Sa[:3, 3]; pb = Sb[:3, :3] @ kb + Sb[:3, 3]           # 두 구간 키프레임 중심(세계)
    d_slam = kb - ka; d_new = pb - pa
    ya = np.degrees(np.arctan2(Sa[1, 0], Sa[0, 0])); yb = np.degrees(np.arctan2(Sb[1, 0], Sb[0, 0]))
    return np.linalg.norm(d_new[:2] - (Sa[:2, :2] @ d_slam[:2])), abs((yb - ya + 180) % 360 - 180)
segs = [sg for sg in segs if sg['cands']]
best_cost = [np.array([c['score'] for c in segs[0]['cands']])]; back = []
for i in range(1, len(segs)):
    prev, cur = segs[i - 1], segs[i]; bc = []; bk = []
    for cb in cur['cands']:
        v = [best_cost[-1][j] + cb['score'] - LAM_T * rel_err(ca['S'] @ T0, cb['S'] @ T0, prev['kc'], cur['kc'])[0] - LAM_R * rel_err(ca['S'] @ T0, cb['S'] @ T0, prev['kc'], cur['kc'])[1]
             for j, ca in enumerate(prev['cands'])]
        bk.append(int(np.argmax(v))); bc.append(max(v))
    best_cost.append(np.array(bc)); back.append(bk)
pick = [int(np.argmax(best_cost[-1]))]
for bk in reversed(back): pick.append(bk[pick[-1]])
pick = pick[::-1]
out = []
for i, (sg, j) in enumerate(zip(segs, pick)):
    c = sg['cands'][j]; Tn = c['S'] @ T0; np.save(O / f"T_seg{sg['g']:02d}.npy", Tn)
    e = rel_err(segs[i - 1]['cands'][pick[i - 1]]['S'] @ T0, Tn, segs[i - 1]['kc'], sg['kc']) if i else (0, 0)
    rank = sorted([x['score'] for x in sg['cands']], reverse=True).index(c['score']) + 1
    out.append(dict(seg=sg['g'], s0=sg['g'] * SEG, kf=len(sg['sel']), pick_rank=rank, score=round(c['score'], 3),
                    best_other=round(max([x['score'] for k_, x in enumerate(sg['cands']) if k_ != j] or [np.nan]), 3),
                    yaw=round(c['yaw'], 1), x=round(float(c['xy'][0]), 2), y=round(float(c['xy'][1]), 2), link_err_m=round(float(e[0]), 2), link_err_deg=round(float(e[1]), 1)))
    print(f"선택 구간 {sg['g']*SEG:4.0f}m: 후보 {rank}위 r {c['score']:+.3f}(다른 최고 {out[-1]['best_other']:+.3f}) 요 {c['yaw']:.1f} | 이전 구간과 어긋남 {e[0]:.2f}m {e[1]:.1f}°")
(O / 'global.json').write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf-8')
# 그림: 정사 영상 위에 구간별 선택 BEV 점(색)
img = np.transpose(ref1, (1, 0))[::-1]; img = cv2.cvtColor(np.clip(img, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR); H = img.shape[0]
cols = [(0, 0, 255), (0, 200, 0), (255, 0, 0), (0, 200, 255), (255, 0, 255), (255, 255, 0)]
for i, (sg, j) in enumerate(zip(segs, pick)):
    S = sg['cands'][j]['S']; p = sg['pts'][::5] @ S[:2, :2].T + S[:2, 3]; v = sg['vals'][::5]; m = v > np.percentile(v, 90)
    ix = ((p[m, 0] - qlo[0]) / C1).astype(int); iy = H - 1 - ((p[m, 1] - qlo[1]) / C1).astype(int)
    ok = (ix >= 0) & (ix < img.shape[1]) & (iy >= 0) & (iy < H); img[iy[ok], ix[ok]] = cols[i % 6]
    q = ((sg['cands'][j]['xy'] - qlo) / C1).astype(int); cv2.putText(img, f"{sg['g']*SEG:.0f}", (int(q[0]), H - 1 - int(q[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.5, cols[i % 6], 2)
ys, xs = np.where(img.sum(2) > 0); img = img[ys.min():ys.max() + 1, xs.min():xs.max() + 1]
cv2.imwrite(str(O / f'{run}_global.jpg'), img)
print('→', O)
