"""대응점(국소 창) 측정과 2D 강체 재맞춤: 같은 세계 좌표의 손 촬영 BEV(fe_bev.py 원본)와 v0.5 정사 영상에서 창(S m)을 잘라
BEV 창을 ±R m(10cm) 옮겨가며 선 무늬(톱햇) 정규화 상관 최대 이동 = 그 지점의 어긋남.
품질 지표: 봉우리 선명도(최고 - 1m 밖 최고), 반복 무늬 정도(BEV 창 자기상관의 0.5m 밖 최고 = self2; 높으면 옆 칸에 붙을 위험).
대응점 = 선명(>= --sharp) · 탐색 경계 아님 · 반복 아님(self2 <= --aper) → RANSAC(--thr) → 안쪽 점으로 2D 강체.
출력: <run>_cp_local.json, <run>_cp_rigid2d.npy(맞춘 경우), <run>_cp_arrows.png(어긋남 화살표 ×3), [--tiles] 창별 그림.
사용: cp_local.py <bev_dir> <run> <out_dir> [--win 이름:x:y[:S] ...] [--auto 6] [--R 1.2] [--sharp 0.05] [--aper 1.0] [--thr 0.4] [--min_in 4] [--tiles] [--quiet]
주의: 주차칸 반복 간격 2.5m → --R 1.25 넘게 잡을 때는 --aper 로 반복 창을 빼야 옆 칸에 붙지 않음."""
import sys, json, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
a = sys.argv; D, run, O = Path(a[1]), a[2], Path(a[3]); O.mkdir(parents=True, exist_ok=True)
opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
R, SH, APER, THR, MIN_IN = opt('--R', 1.2), opt('--sharp', 0.05), opt('--aper', 1.0), opt('--thr', 0.4), int(opt('--min_in', 4)); C = 0.1
TILES, QUIET = '--tiles' in a, '--quiet' in a
wins = [w.split(':') for i, w in enumerate(a) if i > 0 and a[i - 1] == '--win']
Q = np.load(Path.home() / 'b_slam/deliver/map_v0.5/outer/ground_ortho.npz'); qlo = Q['lo']
ortho = np.ascontiguousarray(np.transpose(Q['rgb'], (1, 0, 2))[::-1][:, :, ::-1])            # 위 +y, BGR
b = np.load(D / f'{run}_bev.npz'); blo = b['lo']; bev = cv2.imread(str(D / f'{run}_bev_raw.png'))   # 격자·경로 없는 원본
def cut(img, lo, x0, y0, n):                     # 세계 (x0,y0)=창 왼쪽 아래, n칸 정사각
    H = img.shape[0]; c0 = int(round((x0 - lo[0]) / C)); r1 = H - int(round((y0 - lo[1]) / C)); out = np.zeros((n, n, 3), np.uint8)
    rs, cs = slice(max(r1 - n, 0), min(r1, H)), slice(max(c0, 0), min(c0 + n, img.shape[1]))
    if rs.stop > rs.start and cs.stop > cs.start: out[rs.start - (r1 - n):rs.stop - (r1 - n), cs.start - c0:cs.stop - c0] = img[rs, cs]
    return out
def th(g): g = g.astype(np.float32); return g - ndimage.grey_opening(g, size=(5, 5))
def ncc(Wt, Wv, tpl, M):                         # 마스크 정규화 상관 지도(덮임 50% 미만 = -1)
    nM = M.sum(); num = cv2.matchTemplate(Wt, tpl, cv2.TM_CCORR); Ws = cv2.matchTemplate(Wt, M, cv2.TM_CCORR); W2 = cv2.matchTemplate(Wt * Wt, M, cv2.TM_CCORR)
    cov = cv2.matchTemplate(Wv, M, cv2.TM_CCORR) / nM; var = np.maximum(W2 - Ws * Ws / nM, 0)
    r_ = num / (np.sqrt(var) * np.linalg.norm(tpl) + 1e-3); r_[cov < 0.5] = -1; return r_
if '--auto' in a:                                 # 손 촬영 바닥·정사 영상이 함께 덮고 양쪽에 선 무늬(톱햇>40 칸 1% 이상)가 있는 창 자동 선택
    S_ = opt('--auto', 6); n_ = int(S_ / C); k_ = 0
    for x in np.arange(-70, 25, S_ / 2):
        for y in np.arange(-40, 30, S_ / 2):
            Bw = cut(bev, blo, x - S_ / 2, y - S_ / 2, n_); Aw = cut(ortho, qlo, x - S_ / 2, y - S_ / 2, n_)
            bm, am = Bw.sum(2) > 0, Aw.sum(2) > 0
            if bm.mean() < 0.4 or am.mean() < 0.6 or (bm & am).mean() < 0.3: continue
            lb = (th(cv2.cvtColor(Bw, cv2.COLOR_BGR2GRAY)) > 40) & bm; la = (th(cv2.cvtColor(Aw, cv2.COLOR_BGR2GRAY)) > 40) & am
            if lb.mean() < 0.01 or la.mean() < 0.01: continue
            wins.append([f'a{k_:03d}', f'{x:.1f}', f'{y:.1f}', f'{S_:g}']); k_ += 1
    print(f'{run}: 자동 선택 창 {k_}개 ({S_:g}m, 탐색 ±{R:g}m)')
res = []; tiles = []
for w in wins:
    name, x, y = w[0], float(w[1]), float(w[2]); S = float(w[3]) if len(w) > 3 else 8.0
    n = int(S / C); p = int(round(R / C))
    A = cut(ortho, qlo, x - S / 2 - R, y - S / 2 - R, n + 2 * p); B = cut(bev, blo, x - S / 2, y - S / 2, n)
    Av = (A.sum(2) > 0).astype(np.float32); Bm = (B.sum(2) > 0).astype(np.float32)
    At, Bt = th(cv2.cvtColor(A, cv2.COLOR_BGR2GRAY)) * Av, th(cv2.cvtColor(B, cv2.COLOR_BGR2GRAY)) * Bm; nM = Bm.sum()
    if nM < 300 or Av.mean() < 0.3: res.append(dict(name=name, x=x, y=y, note='자료 부족')); continue
    tpl = np.where(Bm > 0, Bt - Bt[Bm > 0].mean(), 0).astype(np.float32)
    if np.linalg.norm(tpl) < 1e-3: res.append(dict(name=name, x=x, y=y, note='무늬 없음')); continue
    r_ = ncc(At, Av, tpl, Bm)
    mi = np.unravel_index(np.argmax(r_), r_.shape); best = float(r_[mi]); r0 = float(r_[p, p])
    dx, dy = (mi[1] - p) * C, -(mi[0] - p) * C                                                      # 이미지 행 아래 = -y
    yy, xx = np.mgrid[:r_.shape[0], :r_.shape[1]]; far = np.hypot(yy - mi[0], xx - mi[1]) > 10
    second = float(r_[far].max()) if far.any() else -1.0
    # 반복 무늬: BEV 창을 같은 BEV(여유 포함)에 대 보는 자기상관, 0.5m 밖 최고
    Bx = cut(bev, blo, x - S / 2 - R, y - S / 2 - R, n + 2 * p); Bxv = (Bx.sum(2) > 0).astype(np.float32); Bxt = th(cv2.cvtColor(Bx, cv2.COLOR_BGR2GRAY)) * Bxv
    rs = ncc(Bxt, Bxv, tpl, Bm); ys_, xs_ = np.mgrid[:rs.shape[0], :rs.shape[1]]; self2 = float(rs[np.hypot(ys_ - p, xs_ - p) > 5].max())
    edge = max(abs(dx), abs(dy)) >= R - 0.05
    res.append(dict(name=name, x=x, y=y, dx=round(dx, 2), dy=round(dy, 2), off_m=round(float(np.hypot(dx, dy)), 2), r_now=round(r0, 3), r_best=round(best, 3),
                    second=round(second, 3), self2=round(self2, 3), edge=bool(edge)))
    if not QUIET: print(f"{name:6s} ({x:+6.1f},{y:+6.1f}) dx {dx:+.2f} dy {dy:+.2f} | r {r0:+.2f}→{best:+.2f} 2nd {second:+.2f} self2 {self2:+.2f}{' 경계' if edge else ''}")
    if TILES:
        Ac = A[p:p + n, p:p + n].copy(); Bs = cut(bev, blo, x - S / 2 - dx, y - S / 2 - dy, n)
        def red(base, bimg):
            o = base.copy(); g = th(cv2.cvtColor(bimg, cv2.COLOR_BGR2GRAY)); o[(g > 40) & (bimg.sum(2) > 0)] = (0, 0, 255); return o
        row = [cv2.resize(t, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST) for t in (Ac, B, red(Ac, B), red(Ac, Bs))]
        for t in row:
            for k in range(0, t.shape[0], 15): cv2.line(t, (0, k), (t.shape[1] - 1, k), (60, 60, 60), 1); cv2.line(t, (k, 0), (k, t.shape[0] - 1), (60, 60, 60), 1)
        cv2.putText(row[0], f'{name} ortho', (4, 16), 0, 0.5, (0, 255, 255), 1); cv2.putText(row[1], f'hand self2 {self2:.2f}', (4, 16), 0, 0.5, (0, 255, 255), 1)
        cv2.putText(row[2], 'now', (4, 16), 0, 0.5, (0, 255, 255), 1); cv2.putText(row[3], f'shift {dx:+.1f},{dy:+.1f} r {best:.2f}', (4, 16), 0, 0.5, (0, 255, 255), 1)
        tiles.append(np.hstack([np.pad(t, ((2, 2), (2, 2), (0, 0)), constant_values=255) for t in row]))
(O / f'{run}_cp_local.json').write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding='utf-8')
if tiles:
    wmax = max(t.shape[1] for t in tiles); tiles = [np.pad(t, ((0, 0), (0, wmax - t.shape[1]), (0, 0))) for t in tiles]
    for k in range(0, len(tiles), 4): cv2.imwrite(str(O / f'{run}_cp_local_{k // 4}.jpg'), np.vstack(tiles[k:k + 4]), [cv2.IMWRITE_JPEG_QUALITY, 85])
def rig(P0, P1):
    c0, c1 = P0.mean(0), P1.mean(0); U, _, Vt = np.linalg.svd((P0 - c0).T @ (P1 - c1)); Rm = Vt.T @ U.T
    if np.linalg.det(Rm) < 0: Vt[-1] *= -1; Rm = Vt.T @ U.T
    return Rm, c1 - Rm @ c0
meas = [r for r in res if 'dx' in r]
good = [r for r in meas if r['r_best'] - r['second'] >= SH and not r['edge'] and r['self2'] <= APER]
n_per = sum(r['self2'] > APER for r in meas); n_edge = sum(r['edge'] for r in meas); n_blunt = sum(r['r_best'] - r['second'] < SH for r in meas)
print(f'{run}: 측정 {len(meas)} → 대응점 {len(good)} (제외: 반복 {n_per}, 경계 {n_edge}, 봉우리 흐림 {n_blunt}; 중복 가능)')
inl_names = set(); fit = None
if len(good) >= MIN_IN:
    P0 = np.array([[r['x'], r['y']] for r in good]); P1 = P0 + np.array([[r['dx'], r['dy']] for r in good]); rng = np.random.default_rng(0); best_in, be = None, 9
    for _ in range(3000):
        s = rng.choice(len(good), 2, replace=False)
        if np.linalg.norm(P0[s[0]] - P0[s[1]]) < 5: continue
        Rm, t = rig(P0[s], P1[s]); e = np.linalg.norm(P0 @ Rm.T + t - P1, axis=1); inl = e < THR
        if best_in is None or inl.sum() > best_in.sum() or (inl.sum() == best_in.sum() and e[inl].mean() < be): best_in, be = inl, e[inl].mean()
    span = np.ptp(P0[best_in], 0) if best_in is not None and best_in.any() else np.zeros(2)
    if best_in is not None and best_in.sum() >= MIN_IN and span.max() >= 6:
        Rm, t = rig(P0[best_in], P1[best_in]); e = np.linalg.norm(P0 @ Rm.T + t - P1, axis=1); before = np.linalg.norm(P1 - P0, axis=1)
        yaw = np.degrees(np.arctan2(Rm[1, 0], Rm[0, 0])); fit = (Rm, t)
        print(f'  RANSAC 안쪽 {best_in.sum()}/{len(good)} (퍼진 범위 {span[0]:.0f}×{span[1]:.0f}m) → 요 {yaw:+.2f}°, 이동 ({t[0]:+.2f},{t[1]:+.2f})')
        print(f'  안쪽 어긋남: 전 중앙 {np.median(before[best_in]):.2f}m (최대 {before[best_in].max():.2f}) → 후 중앙 {np.median(e[best_in]):.2f}m (최대 {e[best_in].max():.2f}) | 바깥 {int((~best_in).sum())}개 후 중앙 {np.median(e[~best_in]) if (~best_in).any() else float("nan"):.2f}m')
        np.save(O / f'{run}_cp_rigid2d.npy', np.r_[Rm.ravel(), t]); inl_names = {r['name'] for r, ii in zip(good, best_in) if ii}
    else: print(f'  강체 생략: 안쪽 {0 if best_in is None else int(best_in.sum())}개, 퍼진 범위 {span.max():.0f}m')
else: print('  강체 생략(대응점 부족)')
# 어긋남 화살표 지도(×3): 초록=RANSAC 안쪽, 빨강=대응점이나 바깥, 노랑=반복 무늬, 회색=흐림·경계
X0, Y0, Wd, Hd = -70, -40, 950, 700; img = (np.ascontiguousarray(cut(ortho, qlo, X0, Y0, max(Wd, Hd))[-Hd:, :Wd]) * 0.6).astype(np.uint8)
bm = cut(bev, blo, X0, Y0, max(Wd, Hd))[-Hd:, :Wd].sum(2) > 0; img[bm] = (img[bm] * 0.5 + np.array([80, 40, 0])).astype(np.uint8)
gn = {r['name'] for r in good}
for r in meas:
    u0, v0 = int((r['x'] - X0) / C), Hd - 1 - int((r['y'] - Y0) / C); u1, v1 = int(u0 + 3 * r['dx'] / C), int(v0 - 3 * r['dy'] / C)
    col = (0, 255, 0) if r['name'] in inl_names else (0, 0, 255) if r['name'] in gn else (0, 220, 255) if r['self2'] > APER else (150, 150, 150)
    cv2.arrowedLine(img, (u0, v0), (u1, v1), col, 2 if r['name'] in gn else 1, tipLength=0.25); cv2.circle(img, (u0, v0), 2, col, -1)
for xg in range(-70, 25, 10): cv2.putText(img, str(xg), (int((xg - X0) / C) + 2, 12), 0, 0.4, (255, 255, 255), 1)
for yg in range(-40, 30, 10): cv2.putText(img, str(yg), (2, Hd - 1 - int((yg - Y0) / C)), 0, 0.4, (255, 255, 255), 1)
cv2.putText(img, f'{run} R={R:g} green=inlier red=outlier yellow=periodic gray=blunt/edge (x3)', (100, Hd - 8), 0, 0.5, (255, 255, 255), 1)
cv2.imwrite(str(O / f'{run}_cp_arrows.png'), img)
