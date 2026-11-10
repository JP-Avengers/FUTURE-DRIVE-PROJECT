"""수동 대응점 다듬기: 사람이 읽은 대략 어긋남(prior) 주변 ±R m만 국소 상관으로 다듬음(옆 칸에 붙을 수 없게 좁게).
입력 JSON 목록: {name, bev: BEV 이름(<bev_dir>/<bev>_bev.npz·_bev_raw.png), x, y(손 촬영 BEV 좌표의 지점), S(창 m), pdx, pdy(대략 어긋남),
               type: "point"|"line", n:[nx,ny](line일 때 법선), ref: "ortho"|BEV 이름(기본 ortho), R(기본 0.6), w(가중치, 기본 1)}
출력: 같은 목록에 dx, dy(최종 어긋남 = prior + 국소), r, r_prior, second, off_n(line: 법선 방향 어긋남) 추가 → <out>.json, 확인 그림 <out>_tiles_*.jpg
사용: cp_refine.py <bev_dir> <cps.json> <out_prefix>"""
import sys, json, numpy as np, cv2
from pathlib import Path
from scipy import ndimage
D, cps, OUT = Path(sys.argv[1]), json.load(open(sys.argv[2], encoding='utf-8')), sys.argv[3]; C = 0.1
Q = np.load(Path.home() / 'b_slam/deliver/map_v0.5/outer/ground_ortho.npz')
REF = {'ortho': (Q['lo'], np.ascontiguousarray(np.transpose(Q['rgb'], (1, 0, 2))[::-1][:, :, ::-1]))}
def load(nm):
    if nm not in REF: REF[nm] = (np.load(D / f'{nm}_bev.npz')['lo'], cv2.imread(str(D / f'{nm}_bev_raw.png')))
    return REF[nm]
def cut(img, lo, x0, y0, n):
    H = img.shape[0]; c0 = int(round((x0 - lo[0]) / C)); r1 = H - int(round((y0 - lo[1]) / C)); out = np.zeros((n, n, 3), np.uint8)
    rs, cs = slice(max(r1 - n, 0), min(r1, H)), slice(max(c0, 0), min(c0 + n, img.shape[1]))
    if rs.stop > rs.start and cs.stop > cs.start: out[rs.start - (r1 - n):rs.stop - (r1 - n), cs.start - c0:cs.stop - c0] = img[rs, cs]
    return out
def th(g): g = g.astype(np.float32); return g - ndimage.grey_opening(g, size=(5, 5))
def ncc(Wt, Wv, tpl, M):
    nM = M.sum(); num = cv2.matchTemplate(Wt, tpl, cv2.TM_CCORR); Ws = cv2.matchTemplate(Wt, M, cv2.TM_CCORR); W2 = cv2.matchTemplate(Wt * Wt, M, cv2.TM_CCORR)
    cov = cv2.matchTemplate(Wv, M, cv2.TM_CCORR) / nM; var = np.maximum(W2 - Ws * Ws / nM, 0)
    r_ = num / (np.sqrt(var) * np.linalg.norm(tpl) + 1e-3); r_[cov < 0.4] = -1; return r_
tiles = []
for c in cps:
    blo, bim = load(c['bev']); rlo, rim = load(c.get('ref', 'ortho')); R = c.get('R', 0.6); S = c['S']; n = int(round(S / C)); p = int(round(R / C))
    x, y, pdx, pdy = c['x'], c['y'], c['pdx'], c['pdy']
    B = cut(bim, blo, x - S / 2, y - S / 2, n); A = cut(rim, rlo, x + pdx - S / 2 - R, y + pdy - S / 2 - R, n + 2 * p)
    Bm = (B.sum(2) > 0).astype(np.float32); Av = (A.sum(2) > 0).astype(np.float32)
    Bt = th(cv2.cvtColor(B, cv2.COLOR_BGR2GRAY)) * Bm; At = th(cv2.cvtColor(A, cv2.COLOR_BGR2GRAY)) * Av
    if Bm.sum() < 100 or Av.mean() < 0.2: c.update(note='자료 부족'); print(c['name'], '자료 부족'); continue
    tpl = np.where(Bm > 0, Bt - Bt[Bm > 0].mean(), 0).astype(np.float32); r_ = ncc(At, Av, tpl, Bm)
    if c.get('type') == 'line':                                   # 선: 법선 방향 이동만 허용
        nx, ny = c['n']; yy, xx = np.mgrid[:r_.shape[0], :r_.shape[1]]; ddx, ddy = (xx - p) * C, -(yy - p) * C
        along = np.abs(ddx * (-ny) + ddy * nx); r_[along > 0.15] = -1
    mi = np.unravel_index(np.argmax(r_), r_.shape); ldx, ldy = (mi[1] - p) * C, -(mi[0] - p) * C
    yy, xx = np.mgrid[:r_.shape[0], :r_.shape[1]]; far = np.hypot(yy - mi[0], xx - mi[1]) > 4
    c.update(dx=round(pdx + ldx, 2), dy=round(pdy + ldy, 2), r=round(float(r_[mi]), 3), r_prior=round(float(r_[p, p]), 3),
             second=round(float(r_[far].max()) if far.any() else -1, 3), edge=bool(max(abs(ldx), abs(ldy)) >= R - 0.05))
    if c.get('type') == 'line': c['off_n'] = round(c['dx'] * c['n'][0] + c['dy'] * c['n'][1], 2)
    print(f"{c['name']:12s} {c['bev'][-8:]:>8s} {c.get('type','point'):5s} prior ({pdx:+.2f},{pdy:+.2f}) → ({c['dx']:+.2f},{c['dy']:+.2f})"
          f"{'  n방향 ' + format(c['off_n'], '+.2f') if 'off_n' in c else ''} | r {c['r_prior']:+.2f}→{c['r']:+.2f} 2nd {c['second']:+.2f}{' 경계' if c['edge'] else ''}")
    # 확인 그림: [기준(목표 자리) | 손 촬영(현재 자리) | 기준 위 손 촬영 선(빨강, 다듬은 어긋남 적용)]
    Ac = cut(rim, rlo, x + c['dx'] - S / 2, y + c['dy'] - S / 2, n)
    ov = Ac.copy(); g = th(cv2.cvtColor(B, cv2.COLOR_BGR2GRAY)); ov[(g > 35) & (Bm > 0)] = (0, 0, 255)
    row = [cv2.resize(t, (240, 240), interpolation=cv2.INTER_NEAREST) for t in (Ac, B, ov)]
    for t, lab in zip(row, (f"{c['name']} ref", 'hand', f"d=({c['dx']:+.1f},{c['dy']:+.1f}) r{c['r']:.2f}")): cv2.putText(t, lab, (4, 14), 0, 0.42, (0, 255, 255), 1)
    tiles.append(np.hstack([np.pad(t, ((2, 2), (2, 2), (0, 0)), constant_values=255) for t in row]))
json.dump(cps, open(OUT + '.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
for k in range(0, len(tiles), 6): cv2.imwrite(f'{OUT}_tiles_{k // 6}.jpg', np.vstack(tiles[k:k + 6]), [cv2.IMWRITE_JPEG_QUALITY, 85])
