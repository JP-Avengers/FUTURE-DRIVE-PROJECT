"""여러 메시(건물 좌표)를 같은 잣대로 재서 Markdown 표 · 그림으로 — RealSense 맵 vs Polycam 맵 비교용.
 잣대(계산은 기존 도구와 같은 조건으로 다시 구현, 메시는 한 번만 읽음):
  - 검증 depth 일치(in_eval.py (1)): in1rL 키프레임 1/3(48장), 유효 0.4~3.5 m, 중앙 |차| · 5 cm 안 · 채운 비율
  - 강체 배치 검증(mc_rigidval.py 자세): in1r 원래 자세를 강체 하나로 놓은 판(rigid4 · rigid4zL · rigid6)과 in1rL 로 전체 142장, x 5 m 칸별
  - 로봇 시점 채움(in_eval.py (2)): 높이 0.45 m, y=0.15, 수평, x 2~44 m 3 m 간격 양방향 30곳, 12 m 안 · 화면 위 1/3
  - 복도 폭(in_width.py 전체 구간), 천장 덮음 · 벽 꼭대기(in_ceilcov.py), 통로 안에 떠 있는 면(in_clutter.py), 삼각형 수
 메시마다 결과를 ~/b_slam/agents/compare/mc_cache/ 에 저장(경로 · 크기 · 수정 시각이 같으면 다시 안 잼, --force 로 다시, --cache 로 폴더 바꿈)
 출력: <out>/compare_table.md, topviews.png(높이 색 + 음영), robot_views.jpg(x 11 · 32 m 앞 방향), rigidval_bins.png
사용: map_compare.py <out_dir> <이름=mesh.ply> [...] [--val in1rL] [--rigid ~/b_slam/agents/compare/rigidval] [--y 0.15] [--force] [--reg <이름>=<x_reg.json>]
     무거움(5M 삼각형 메시 하나에 ~2-3 GB): flock ~/b_slam/.heavy.lock 으로 돌릴 것"""
import sys, os, json, time, gc, hashlib, numpy as np, open3d as o3d, cv2
from pathlib import Path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mc_lib import load_mesh, scene_of, load_poses, depth_agree, agree_summary, EXPORTS
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

a = sys.argv; opt = lambda k, d: a[a.index(k) + 1] if k in a else d; HC, Y0, Y1 = 2.87, -1.10, 1.43
OUT = Path(a[1]); OUT.mkdir(parents=True, exist_ok=True); CACHE = Path(os.path.expanduser(opt('--cache', '~/b_slam/agents/compare/mc_cache'))); CACHE.mkdir(parents=True, exist_ok=True)
skip = {i + 1 for i, x in enumerate(a) if x in ('--val', '--rigid', '--y', '--reg', '--cache')}
MESHES = [x.split('=', 1) for i, x in enumerate(a[2:], 2) if '=' in x and i not in skip and not x.startswith('--')]
REGS = dict(x.split('=', 1) for i, x in enumerate(a) if i > 0 and a[i - 1] == '--reg')
VAL = opt('--val', 'in1rL'); RIG = Path(os.path.expanduser(opt('--rigid', '~/b_slam/agents/compare/rigidval'))); YL = float(opt('--y', 0.15))
t0 = time.time(); log = lambda *s: print(f'[{time.time() - t0:5.0f}s]', *s, flush=True)
KDIR = EXPORTS / VAL / 'keyframes'; PL = load_poses(EXPORTS / VAL / 'traj_camera_id_camera_poses.txt')
PLACE = {'in1rL': PL} | {p.stem.split('_', 1)[1]: load_poses(p) for p in sorted(RIG.glob('*_rigid*.txt'))}
XB = np.arange(-5, 55, 5.0); XL = {int(r[8]): r[1] for r in PL}      # 칸 나누기는 모든 배치에서 in1rL 카메라 x 로
CORR = {int(r[8]) for r in PL if 0 < r[1] < 46 and Y0 < r[2] < Y1}     # 복도 안 키프레임(in1rL 카메라가 x 0~46, 두 벽 선 사이 — 로비로 들어간 구간 제외)


def shade_hits(sc, rays, H, W, zcol=None):
    r = sc.cast_rays(o3d.core.Tensor(rays.astype(np.float32))); t = r['t_hit'].numpy(); hit = np.isfinite(t); n = r['primitive_normals'].numpy()
    lam = np.abs((n * rays[:, 3:]).sum(1)); g = (60 + 195 * lam).clip(0, 255)
    if zcol is None: img = np.repeat(g[:, None], 3, 1)
    else:
        z = rays[:, 2] + t * rays[:, 5]; c = cv2.applyColorMap(((z - zcol[0]) / (zcol[1] - zcol[0]) * 255).clip(0, 255).astype(np.uint8)[:, None], cv2.COLORMAP_TURBO)[:, 0].astype(float)
        img = c * (0.35 + 0.65 * lam[:, None])
    img[~hit] = 255; return img.reshape(H, W, 3).astype(np.uint8), hit.reshape(H, W), t.reshape(H, W)


def measure(name, path, hk):
    V, T, C = load_mesh(path); log(f'{name}: {Path(path).name} 꼭짓점 {len(V):,}, 삼각형 {len(T):,}'); R = dict(name=name, path=str(path), tris=int(len(T)), verts=int(len(V)),
                                                                                                   bbox=[V.min(0).tolist(), V.max(0).tolist()])
    # 형상 잣대(in_width · in_ceilcov · in_clutter 와 같은 조건)
    w = V[(V[:, 2] > 0.4) & (V[:, 2] < 1.4) & (np.abs(V[:, 1]) < 2.6)]; w = w[(w[:, 0] > -2) & (w[:, 0] < 48), 1]
    if len(w) >= 500:
        h, e = np.histogram(w, bins=np.arange(-2.2, 2.6, 0.01)); c = (e[:-1] + e[1:]) / 2; h = np.convolve(h, np.ones(3) / 3, 'same'); lo = c < 0
        i, j = int(np.argmax(np.where(lo, h, 0))), int(np.argmax(np.where(~lo, h, 0))); sub = lambda k: float((c[max(k - 4, 0):k + 5] * h[max(k - 4, 0):k + 5]).sum() / h[max(k - 4, 0):k + 5].sum())
        R['wall_y'] = [sub(i), sub(j)]; R['width'] = sub(j) - sub(i)
    cc = V[(np.abs(V[:, 2] - HC) < 0.15) & (V[:, 0] > 0) & (V[:, 0] < 46) & (V[:, 1] > Y0) & (V[:, 1] < Y1)]
    g = np.zeros((460, int((Y1 - Y0) / 0.1) + 1), bool); ij = ((cc[:, :2] - [0, Y0]) / 0.1).astype(int); g[ij[:, 0].clip(0, 459), ij[:, 1].clip(0, g.shape[1] - 1)] = True; R['ceil_cov'] = float(g.mean())
    tops = []
    for yw in (Y0, Y1):
        s = V[(np.abs(V[:, 1] - yw) < 0.12) & (V[:, 0] > 0) & (V[:, 0] < 46) & (V[:, 2] < HC - 0.1)]; b = (s[:, 0] / 0.5).astype(int)
        tt = [np.percentile(s[b == k, 2], 98) for k in np.unique(b) if (b == k).sum() >= 50]; tops.append(float(np.median(tt)) if tt else float('nan'))
    R['wall_top'] = tops
    ar = np.zeros(len(T)); ins = np.zeros(len(T), bool); cz = np.zeros(len(T))
    for s0 in range(0, len(T), 1000000):
        tt = T[s0:s0 + 1000000]; p0, p1, p2 = V[tt[:, 0]], V[tt[:, 1]], V[tt[:, 2]]; c = (p0 + p1 + p2) / 3; cz[s0:s0 + len(tt)] = c[:, 2]
        ins[s0:s0 + len(tt)] = (c[:, 0] > 0) & (c[:, 0] < 46) & ~((c[:, 0] > 18) & (c[:, 0] < 27)) & (c[:, 1] > Y0 + 0.25) & (c[:, 1] < Y1 - 0.25) & (c[:, 2] > 0.15) & (c[:, 2] < HC - 0.2)
        ar[s0:s0 + len(tt)] = 0.5 * np.linalg.norm(np.cross(p1 - p0, p2 - p0), axis=1)
    R['clutter_m2'] = float(ar[ins].sum()); R['clutter_lo_m2'] = float(ar[ins & (cz < 1.5)].sum()); R['area_m2'] = float(ar.sum()); del ar, ins, cz
    sc = scene_of(V, T); del V, T, C; gc.collect()
    # 검증 depth 일치(in_eval 과 같은 48장) + 배치별 전체 키프레임
    R['val48'] = agree_summary(depth_agree(sc, PL[::3], KDIR)); R['place'] = {}
    for pn, P in PLACE.items():
        res = depth_agree(sc, P, KDIR); bins = []
        for x0 in XB[:-1]:
            sel = [r for r in res if x0 <= XL[r[0]] < x0 + 5]
            if sel: bins.append(dict(x0=float(x0)) | agree_summary(sel))
        R['place'][pn] = dict(all=agree_summary(res), corr=agree_summary([r for r in res if r[0] in CORR]), bins=bins)
    log(f'   검증 48장: 중앙 {R["val48"]["med_cm"]:.1f} cm, 5 cm 안 {R["val48"]["in5"]:.0%}, 채움 {R["val48"]["fill"]:.0%} | ' + ', '.join(f'{k} {v["all"]["med_cm"]:.1f} cm/{v["all"]["in5"]:.0%}' for k, v in R['place'].items()))
    # 로봇 시점(in_eval (2))
    W_, H_ = 320, 200; fx = (W_ / 2) / np.tan(np.radians(40)); uu, vv = np.meshgrid(np.arange(W_) - W_ / 2 + 0.5, np.arange(H_) - H_ / 2 + 0.5); hits, ups, tiles = [], [], []
    for x in np.arange(2, 45.1, 3.0):
        for sgn in (1, -1):
            d = np.stack([np.full(uu.shape, sgn * fx), -sgn * uu, -vv], -1).reshape(-1, 3); d /= np.linalg.norm(d, axis=1, keepdims=True)
            img, hit, t = shade_hits(sc, np.c_[np.tile([x, YL, 0.45], (len(d), 1)), d], H_, W_); h = hit & (t < 12); hits.append(h.mean()); ups.append(h[:H_ // 3].mean())
            if sgn == 1 and (abs(x - 11) < 0.1 or abs(x - 32) < 0.1): img[~h] = (255, 200, 255); cv2.putText(img, f'x={x:.0f}', (W_ - 50, 14), 0, 0.45, (0, 0, 255), 1); tiles.append(img)
    R['robot_fill'] = float(np.mean(hits)); R['robot_top'] = float(np.mean(ups)); log(f'   로봇 시점 {R["robot_fill"]:.0%}, 위 1/3 {R["robot_top"]:.0%}')
    # 위에서 본 그림(z 1.9 m 에서 아래로, 4 cm/px, 높이 색)
    c4 = 0.04; x0, x1, y0, y1 = -4, 54, -5, 12; Wt, Ht = int((x1 - x0) / c4), int((y1 - y0) / c4); u, v = np.meshgrid(np.arange(Wt), np.arange(Ht))
    o = np.stack([x0 + (u + 0.5) * c4, y1 - (v + 0.5) * c4, np.full(u.shape, 1.9)], -1).reshape(-1, 3); top, _, _ = shade_hits(sc, np.c_[o, np.tile([0, 0, -1.0], (len(o), 1))], Ht, Wt, zcol=(-0.1, 1.9))
    for xg in range(0, 55, 5): cv2.putText(top, f'{xg}', (int((xg - x0) / c4), 16), 0, 0.5, (0, 0, 0), 1)
    cv2.imwrite(str(CACHE / f'{hk}_top.png'), top); cv2.imwrite(str(CACHE / f'{hk}_robot.png'), np.hstack(tiles)); del sc; gc.collect(); return R


rows = []
for name, path in MESHES:
    path = os.path.abspath(os.path.expanduser(path)); st = os.stat(path); hk = hashlib.md5(f'{path}|{st.st_size}|{int(st.st_mtime)}|{VAL}|{YL}|{RIG}'.encode()).hexdigest()[:12]; cf = CACHE / f'{hk}.json'
    if cf.exists() and '--force' not in a: R = json.loads(cf.read_text()); log(f'{name}: 저장된 결과 씀 ({cf.name})')
    else: R = measure(name, path, hk); cf.write_text(json.dumps(R, indent=1, ensure_ascii=False))
    R['name'] = name; R['hk'] = hk
    if name in REGS: R['reg'] = json.loads(Path(os.path.expanduser(REGS[name])).read_text())
    rows.append(R)

# ---------------------------------------------------------------- 표
pc = lambda x: '–' if x is None or (isinstance(x, float) and np.isnan(x)) else f'{x:.0%}'; cm = lambda x: '–' if x is None or np.isnan(x) else f'{x:.1f} cm'
L = [f'# 맵 형상 비교 ({time.strftime("%Y-%m-%d %H:%M")}, map_compare.py)', '', '모든 수치는 메시에서 계산한 값(계산값)입니다. 건물 좌표(바닥 z=0, 벽 선 y = -1.10 / +1.43 m, 천장 추정 2.87 m) 기준.', '',
     '## 1. 형상 잣대', '', '| 메시 | 삼각형 | 검증 depth 차 중앙 (in1rL 48장) | 5 cm 안 | 10 cm 안 | 검증 채움 | 로봇 시점 채움 | 화면 위 1/3 | 복도 폭 | 벽 꼭대기 중앙 -y / +y | 복도 천장 덮음 | 통로 안 떠 있는 면 (1.5 m 아래) |',
     '|---|---|---|---|---|---|---|---|---|---|---|---|']
for R in rows:
    v = R['val48']; L.append(f'| {R["name"]} | {R["tris"]:,} | {cm(v["med_cm"])} | {pc(v["in5"])} | {pc(v["in10"])} | {pc(v["fill"])} | {pc(R["robot_fill"])} | {pc(R["robot_top"])} | '
                             f'{R.get("width", float("nan")):.2f} m | {R["wall_top"][0]:.2f} / {R["wall_top"][1]:.2f} m | {pc(R["ceil_cov"])} | {R["clutter_m2"]:.2f} ㎡ ({R["clutter_lo_m2"]:.2f}) |')
L += ['', '- 검증 depth: 맵에 안 쓴 회차 in1rL 키프레임 1/3(48장)에서 메시를 렌더한 depth 와 실제 depth(0.4~3.5 m) 차이. in_eval.py 와 같은 조건.',
      '- 로봇 시점: 높이 0.45 m, y=0.15 m, 수평 시선(가로 80° · 세로 53°), x 2~44 m 3 m 간격 양방향 30곳에서 광선이 12 m 안에 메시에 닿는 비율.',
      '- 복도 폭: 높이 0.4~1.4 m 꼭짓점 y 분포의 두 봉우리 간격(x -2~48). 벽 꼭대기: 벽 선 ±0.12 m 꼭짓점 높이의 0.5 m 칸별 98% 값 중앙. 천장 덮음: 높이 2.87±0.15 m, 벽 선 사이 10 cm 칸.',
      '- 통로 안 떠 있는 면: 벽 선 0.25 m 안쪽, 바닥 0.15 m 위 ~ 천장 0.2 m 아래, x 0~46(로비 입구 18~27 제외)의 삼각형 넓이.', '']
regs = [R for R in rows if 'reg' in R]
if regs:
    L += ['## 2. 좌표 맞춤(mc_register.py)', '', '| 메시 | 고른 위 축(파일 좌표) | 위 후보 1·2위 점수 | 크기 비 남/우리: x 창 기울기 · 크기 허용 ICP · 단면 폭 | 크기 적용 | 벽 거리 잔차 중앙 | 5 cm 안 | 기준 벽 0.5 m 밖 | 주의 |', '|---|---|---|---|---|---|---|---|---|']
    for R in regs:
        g = R['reg']; rr = g['residual_after'] or g['residual']; hs = g['hypotheses']; sx = g['scale_check']['scale_x']; sa = '아니오' if g['scale_applied'] is None else '예 ({:.4f})'.format(g['scale_applied'])
        L.append(f'| {R["name"]} | {np.round(g["chosen_up_file"], 3).tolist()} | {hs[0]["score"]:.3f} · {hs[1]["score"]:.3f} | {"정하지 못함" if sx is None else f"{sx:.4f}"} · {g["scale_check"]["scale_icp"]:.4f} · {g["scale_check"].get("scale_width", float("nan")):.4f} | '
                 f'{sa} | {rr["wall_med_cm"]:.1f} cm | {pc(rr["wall_in5"])} | {pc(rr["wall_outside_50cm"])} | {g["note"] or ""} |')
    L += ['', '- 크기 비(남의 크기 / 우리 크기, 계산값): x 창 기울기 = 10 m 창마다 복도 방향 어긋남을 재서 RANSAC 직선의 기울기(문틀 · 벽 끝 같은 x 방향 특징이 있어야 정해짐, 못 정하면 정하지 못함). 크기 허용 ICP 는 벽이 x 와 나란해 1 쪽으로 치우침. 단면 폭 비의 1% 는 2.6 cm 라 벽면 재구성 방식 차이와 섞임 — 셋 다 참고값.', '- 벽 거리 잔차: 남의 벽 점(높이 0.3~1.9 m)에서 가장 가까운 기준 벽 점의 면까지 거리. 0.5 m 밖은 기준 맵에 없는 곳으로 보고 중앙값에서 뺌.', '']
pn = list(rows[0]['place'].keys()) if rows else []
L += ['## 3. 강체 배치 검증(독립 수치)', '', 'in1r 의 RTAB-Map 원래 자세를 강체 변환 하나로 건물 좌표에 놓고(mc_rigidval.py) 142장 전부로 같은 depth 일치를 잼. `in1rL` 은 키프레임마다 맵 벽에 맞춘 기존 배치(같은 142장, 참고용).',
      '`rigid4` = 요 + 3D 이동, `rigid4zL` = rigid4 에 높이만 in1rL 값(높이는 독립 아님), `rigid6` = 평균 자세 차로 롤 · 피치까지.', '',
      '| 메시 | 배치 | 전체 142장: depth 차 중앙 | 5 cm 안 | 채움 | 복도 안 키프레임만: 중앙 | 5 cm 안 | 장 수 |', '|---|---|---|---|---|---|---|---|']
for R in rows:
    for p in pn: q, k = R['place'][p]['all'], R['place'][p]['corr']; L.append(f'| {R["name"]} | {p} | {cm(q["med_cm"])} | {pc(q["in5"])} | {pc(q["fill"])} | {cm(k["med_cm"])} | {pc(k["in5"])} | {k["n"]} |')
L += ['', '복도 안 = in1rL 카메라가 x 0~46 m, 두 벽 선 사이(로비로 들어갔다 나온 구간과 양 끝 밖 제외).']
rj = RIG / 'rigidval.json'
if rj.exists():
    rv = json.loads(rj.read_text()); g = rv['rigid4']
    L += ['', f'자세 자체 비교(메시와 무관, mc_rigidval.py): rigid4 로 놓은 in1r 원래 위치 − in1rL 위치. 요 {g["yaw_pitch_roll_deg"][0]:.2f}°, 위치 차 중앙 {g["pos_resid_med_cm"]:.0f} cm · 최대 {g["pos_resid_max_cm"]:.0f} cm, '
          f'수평 길이 비(in1rL / 원래) {rv["scale_raw_to_placed_xy"]:.4f}, 원래 z 범위 {rv["raw_z_range"][0]:+.2f}~{rv["raw_z_range"][1]:+.2f} m', '',
          '| x 칸 | 장 수 | dx (복도 방향) | dy (가로) | dz (높이) | 요 차 |', '|---|---|---|---|---|---|']
    L += [f'| {q["x0"]:.0f}~{q["x0"] + 5:.0f} | {q["n"]} | {q["dx_cm"]:+.0f} cm | {q["dy_cm"]:+.0f} cm | {q["dz_cm"]:+.0f} cm | {q["dyaw_deg"]:+.2f}° |' for q in g['bins']]
L += ['', 'x 5 m 칸별 depth 차 중앙(cm) / 5 cm 안 비율 — 칸은 in1rL 카메라 x 기준, 괄호는 키프레임 수', '', '| x 칸 | ' + ' | '.join(f'{R["name"]} {p}' for R in rows for p in pn) + ' |', '|---|' + '---|' * (len(rows) * len(pn))]
for x0 in XB[:-1]:
    cells = []
    for R in rows:
        for p in pn:
            b = [q for q in R['place'][p]['bins'] if q['x0'] == x0]; cells.append('' if not b else f'{b[0]["med_cm"]:.1f} / {b[0]["in5"]:.0%} ({b[0]["n"]})')
    if any(cells): L.append(f'| {x0:.0f}~{x0 + 5:.0f} | ' + ' | '.join(cells) + ' |')
L += ['', '그림: `topviews.png`(위에서, 높이 색: 파랑 바닥 → 빨강 1.9 m), `robot_views.jpg`(로봇 시점 x 11 · 32 m, 분홍 = 12 m 안 빈 곳), `rigidval_bins.png`(칸별 depth 차)']
(OUT / 'compare_table.md').write_text('\n'.join(L) + '\n')
def label(img, s): img = img.copy(); cv2.putText(img, s, (6, img.shape[0] - 10), 0, 0.6, (255, 255, 255), 4); cv2.putText(img, s, (6, img.shape[0] - 10), 0, 0.6, (0, 0, 0), 1); return img
tops = [label(cv2.imread(str(CACHE / f'{R["hk"]}_top.png')), R['name']) for R in rows]; cv2.imwrite(str(OUT / 'topviews.png'), np.vstack(tops))
rv = [label(cv2.imread(str(CACHE / f'{R["hk"]}_robot.png')), R['name']) for R in rows]; wmax = max(x.shape[1] for x in rv)
cv2.imwrite(str(OUT / 'robot_views.jpg'), np.vstack([np.pad(x, ((0, 0), (0, wmax - x.shape[1]), (0, 0)), constant_values=255) for x in rv]), [cv2.IMWRITE_JPEG_QUALITY, 88])
fig, ax = plt.subplots(1, 2, figsize=(12, 4))
for i, R in enumerate(rows):
    for j, p in enumerate(pn):
        b = R['place'][p]['bins']; x = [q['x0'] + 2.5 for q in b]; ls = ['-', '--', ':', '-.'][j % 4]
        ax[0].plot(x, [q['med_cm'] for q in b], ls, color=f'C{i}', marker='o', ms=3, label=f'{R["name"]} {p}'); ax[1].plot(x, [q['in5'] * 100 for q in b], ls, color=f'C{i}', marker='o', ms=3)
ax[0].set_ylabel('median |depth diff| (cm)'); ax[1].set_ylabel('% within 5 cm'); [(q.set_xlabel('x of in1rL camera (m), 5 m bins'), q.grid(alpha=.3)) for q in ax]; ax[0].legend(fontsize=7)
fig.tight_layout(); fig.savefig(OUT / 'rigidval_bins.png', dpi=90); log('표', OUT / 'compare_table.md')
