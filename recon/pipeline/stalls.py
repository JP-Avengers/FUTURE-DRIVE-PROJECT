"""주차칸 다듬기 (v0.9, 11.11) — 구운 바닥 텍스처에서 주차선을 찾아 반듯한 주차칸으로 다시 그림. 보완(만든 것): 줄 위치 · 방향 · 간격은 측정, 선 모양은 만든 것.

왜: 키프레임 자세 오차(3~4 cm, 손 촬영 0.1~0.3 m)로 사진을 이어 붙인 주차선이 구불거리고 끊김. 주차칸은 규격(폭 2.5 m · 길이 5.0 m · 선 15 cm, B 실측)이라
줄마다 '기준선 + 같은 간격의 칸막이 선' 모형을 맞춰 다시 그리면 실제와 더 가까움. 찾은 주차칸은 차 모델을 칸에 맞춰 놓는 데도 씀 (cars.snap_to_stalls).

 1) 흰 페인트 화소 (둘레보다 밝고 무채색) → 직선 조각 (Hough) → 한 직선으로 묶음
 2) 긴 직선(기준선)마다 양옆으로 칸막이 선을 찾음: 기준선을 따라가며 옆으로 쏜 선 위의 페인트 비율 → 봉우리 = 칸막이. 기울어진 칸도 (skew)
    기준선 없는 줄 (칸막이만 보이는 줄) = 나란한 짧은 선들이 같은 간격으로 늘어선 묶음
 3) 줄마다 간격을 한 값으로 (stall_w_m 둘레에서 가장 잘 맞는 값), 칸막이 방향은 줄 공통. 빠진 칸막이는 양옆이 있으면 채움 (차에 가려 안 찍힌 선)
 4) 원래 페인트를 지울 자리(erase)와 새로 그릴 선(lines), 주차칸 사각형(stalls) 을 돌려줌 — 지우기는 floor_texture 의 아스팔트 채우기가 함

좌표: img[iy, ix] 는 iy 가 +y 방향 (floor_texture.finish 와 같음), xs · ys = 텍셀 중심 좌표.
"""
import json

import cv2
import numpy as np
from scipy import ndimage
from scipy.signal import find_peaks


def _paint_mask(img, on_floor, g, c):
    f = img.astype(np.float32)
    lum, sat = f.mean(-1), f.max(-1) - f.min(-1)
    loc = cv2.GaussianBlur(np.where(on_floor, lum, 0).astype(np.float32), (0, 0), 1.0 / g) / np.maximum(
        cv2.GaussianBlur(on_floor.astype(np.float32), (0, 0), 1.0 / g), 1e-3)
    p = (lum > c.get("paint_ratio", 1.22) * loc) & (sat < c.get("paint_sat", 0.16)) & (lum > c.get("paint_min_lum", 0.4)) & on_floor
    p = cv2.morphologyEx(p.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    n, lab, stt, _ = cv2.connectedComponentsWithStats(p)
    return np.isin(lab, 1 + np.nonzero(stt[1:, cv2.CC_STAT_AREA] * g * g > c.get("paint_min_m2", 0.05))[0])


def _merge_lines(segs, ang_tol=3.0, dist_tol=0.2, gap=1.0):
    """직선 조각 [x1, y1, x2, y2] → 직선 [dict(c 점, d 방향, s0, s1 범위, cover 덮인 길이)] (긴 것부터 붙임)."""
    lines = []
    order = np.argsort(-np.hypot(segs[:, 2] - segs[:, 0], segs[:, 3] - segs[:, 1]))
    for a in segs[order]:
        p, q = a[:2], a[2:]
        d = (q - p) / max(np.linalg.norm(q - p), 1e-9)
        for ln in lines:
            if abs(d @ ln["d"]) < np.cos(np.radians(ang_tol)):
                continue
            n = np.array([-ln["d"][1], ln["d"][0]])
            if max(abs((p - ln["c"]) @ n), abs((q - ln["c"]) @ n)) > dist_tol:
                continue
            s = sorted(((p - ln["c"]) @ ln["d"], (q - ln["c"]) @ ln["d"]))
            if s[0] > ln["s1"] + gap or s[1] < ln["s0"] - gap:
                continue
            ln["iv"].append(s)
            ln["s0"], ln["s1"] = min(ln["s0"], s[0]), max(ln["s1"], s[1])
            break
        else:
            lines.append(dict(c=p.copy(), d=d, s0=0.0, s1=float(np.linalg.norm(q - p)), iv=[[0.0, float(np.linalg.norm(q - p))]]))
    for ln in lines:
        iv = sorted(ln["iv"])
        cov, cur = 0.0, list(iv[0])
        for s0, s1 in iv[1:]:
            if s0 <= cur[1]:
                cur[1] = max(cur[1], s1)
            else:
                cov += cur[1] - cur[0]
                cur = [s0, s1]
        ln["cover"] = cov + cur[1] - cur[0]
    return lines


class _Sampler:
    def __init__(self, mask, x0, y0, g):
        self.m, self.x0, self.y0, self.g = mask, x0, y0, g

    def __call__(self, xy):
        ix = np.round((xy[..., 0] - self.x0) / self.g - 0.5).astype(int)
        iy = np.round((xy[..., 1] - self.y0) / self.g - 0.5).astype(int)
        ok = (ix >= 0) & (ix < self.m.shape[1]) & (iy >= 0) & (iy < self.m.shape[0])
        out = np.zeros(ix.shape, bool)
        out[ok] = self.m[iy[ok], ix[ok]]
        return out


def _refine(ln, P, g):
    """직선 둘레 0.25 m 안의 페인트 화소로 방향 · 위치를 다시 맞춤 (Hough 는 0.5° 단위)."""
    s = np.arange(ln["s0"], ln["s1"], g)
    n = np.array([-ln["d"][1], ln["d"][0]])
    o = np.arange(-0.25, 0.2501, g)
    pts = ln["c"] + s[:, None, None] * ln["d"] + o[None, :, None] * n
    hit = P(pts)
    if hit.sum() < 20:
        return ln
    xy = pts[hit]
    c = xy.mean(0)
    d = np.linalg.svd(xy - c, full_matrices=False)[2][0]
    if d @ ln["d"] < 0:
        d = -d
    t = (xy - c) @ d
    return dict(ln, c=c, d=d, s0=float(t.min()), s1=float(t.max()))


def _fit_grid(t, w, lo=0.94, hi=1.06):
    """봉우리 위치 t 를 간격 p (w × lo~hi) 격자에 맞춤 → (p, t0, 칸 번호 k, 잔차 rms)."""
    best = None
    for p in np.arange(w * lo, w * hi + 1e-9, 0.005):
        k = np.round((t - t[0]) / p)
        if len(np.unique(k)) < len(k):
            continue
        t0 = float(np.mean(t - k * p))
        r = float(np.sqrt(np.mean((t - t0 - k * p) ** 2)))
        if best is None or r < best[3]:
            best = (float(p), t0, k.astype(int), r)
    return best


def detect(img, on_floor, xs, ys, c):
    """→ dict(rows 줄, lines 새로 그릴 선 [[x1, y1, x2, y2]], stalls 주차칸 [[cx, cy, yaw, 폭, 길이, 줄, 입구 x, 입구 y]], erase 지울 자리 [iy, ix] 원래 해상도)."""
    cell = float(xs[1] - xs[0])
    k = max(1, int(round(c.get("work_cell_m", 0.02) / cell)))
    g = cell * k
    H, W = (img.shape[0] // k) * k, (img.shape[1] // k) * k
    small = cv2.resize(img[:H, :W], (W // k, H // k), interpolation=cv2.INTER_AREA)
    fl = on_floor[:H:k, :W:k]
    x0, y0 = float(xs[0] - cell / 2), float(ys[0] - cell / 2)
    paint = _paint_mask(small, fl, g, c)
    segs = cv2.HoughLinesP(paint.astype(np.uint8) * 255, 1, np.pi / 360, threshold=int(0.8 / g), minLineLength=int(1.2 / g), maxLineGap=int(0.5 / g))
    st = dict(paint_m2=round(float(paint.sum()) * g * g, 1), rows=0, stalls=0, separators=0, separators_added=0)
    empty = dict(rows=[], lines=[], stalls=[], erase=np.zeros(img.shape[:2], bool), stats=st)
    if segs is None:
        return empty
    segs = np.asarray(segs).reshape(-1, 4).astype(float)
    segs = np.c_[x0 + (segs[:, 0] + .5) * g, y0 + (segs[:, 1] + .5) * g, x0 + (segs[:, 2] + .5) * g, y0 + (segs[:, 3] + .5) * g]
    P = _Sampler(paint, x0, y0, g)
    F = _Sampler(fl, x0, y0, g)
    Fin = _Sampler(ndimage.binary_erosion(fl, iterations=int(c.get("edge_margin_m", 1.5) / g)), x0, y0, g)   # 바닥 가장자리에서 안쪽
    lines = [_refine(ln, P, g) for ln in _merge_lines(segs)]
    w, L = c.get("stall_w_m", 2.5), c.get("stall_l_m", 5.0)
    rows, used = [], np.zeros(len(lines), bool)
    rr = np.arange(0.5, min(L, 4.4), 2 * g)

    def profile(ln, side, skew):
        n = np.array([-ln["d"][1], ln["d"][0]]) * side
        a = np.radians(skew)
        sd = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]]) @ n
        t = np.arange(ln["s0"] - 0.3, ln["s1"] + 0.3, 2 * g)
        pts = ln["c"] + t[:, None, None] * ln["d"] + rr[None, :, None] * sd
        return t, P(pts).mean(1), sd

    for i in np.argsort([-ln["cover"] for ln in lines]):
        ln = lines[i]
        if ln["cover"] < c.get("baseline_min_m", 6.0) or (ln["s1"] - ln["s0"]) < c.get("baseline_min_m", 6.0):
            continue
        if Fin(ln["c"] + np.linspace(ln["s0"], ln["s1"], 9)[:, None] * ln["d"]).mean() < 0.5:
            continue
        for side in (1, -1):
            best = None
            for skew in np.arange(-c.get("max_skew_deg", 30), c.get("max_skew_deg", 30) + 0.1, 2.0):
                t, pr, sd = profile(ln, side, skew)
                pr = ndimage.uniform_filter1d(pr, max(1, int(0.1 / (2 * g))))
                pk, _ = find_peaks(pr, height=c.get("sep_min_cover", 0.3), distance=int(1.5 / (2 * g)))
                sc = float((pr[pk] ** 2).sum())
                if len(pk) >= 3 and (best is None or sc > best[0]):
                    best = (sc, skew, t[pk], sd)
            if best is None:
                continue
            for skew in np.arange(best[1] - 1.5, best[1] + 1.51, 0.5):           # 기울기 0.5° 로 다시
                t, pr, sd = profile(ln, side, skew)
                pr = ndimage.uniform_filter1d(pr, max(1, int(0.1 / (2 * g))))
                pk, _ = find_peaks(pr, height=c.get("sep_min_cover", 0.3), distance=int(1.5 / (2 * g)))
                sc = float((pr[pk] ** 2).sum())
                if len(pk) >= 3 and sc > best[0]:
                    best = (sc, skew, t[pk], sd)
            fit = _fit_grid(best[2], w)
            if fit is None or fit[3] > (c.get("grid_max_rms_m", 0.2) if len(best[2]) >= c.get("loose_min_n", 7) else c.get("grid_tight_rms_m", 0.12)):
                continue                                                          # 칸막이가 적은 줄은 더 엄격히 (모서리 부채꼴 선 등)
            rows.append(dict(c=ln["c"], d=ln["d"], sd=best[3], skew=float(best[1]), p=fit[0], t0=fit[1], k=fit[2], rms=fit[3],
                             base=[float(ln["s0"]), float(ln["s1"])], line=int(i)))
            used[i] = True

    # 칸막이로 설명된 짧은 선 표시
    def explained(ln):
        m = ln["c"] + (ln["s0"] + ln["s1"]) / 2 * ln["d"]
        for r in rows:
            if abs(ln["d"] @ r["sd"]) < np.cos(np.radians(8)):
                continue
            dn = np.array([-r["sd"][1], r["sd"][0]])
            A = np.c_[r["d"], r["sd"]]
            tt, rho = np.linalg.solve(A, m - r["c"])
            if -1.0 < rho < L + 1.0 and abs((tt - r["t0"]) / r["p"] - np.round((tt - r["t0"]) / r["p"])) * r["p"] < 0.4 \
                    and r["k"].min() - 0.5 < (tt - r["t0"]) / r["p"] < r["k"].max() + 0.5:
                return True
        return False

    # 기준선 없는 줄: 나란한 짧은 선들이 같은 간격으로 늘어선 묶음
    cand = [i for i, ln in enumerate(lines) if not used[i] and c.get("sep_len_range_m", [2.0, 6.5])[0] <= ln["s1"] - ln["s0"] <= c.get("sep_len_range_m", [2.0, 6.5])[1]
            and ln["cover"] > 0.6 * (ln["s1"] - ln["s0"]) and not explained(ln)]
    left = set(cand)
    while left:
        i = max(left, key=lambda j: lines[j]["cover"])
        left.discard(i)
        d = lines[i]["d"]
        n = np.array([-d[1], d[0]])
        mid = lambda j: lines[j]["c"] + (lines[j]["s0"] + lines[j]["s1"]) / 2 * lines[j]["d"]
        grp = [i]
        for j in sorted(left, key=lambda j: abs((mid(j) - mid(i)) @ n)):
            if abs(lines[j]["d"] @ d) < np.cos(np.radians(8)):
                continue
            near = min(grp, key=lambda q: abs((mid(j) - mid(q)) @ n))
            dr, ds = abs((mid(j) - mid(near)) @ n), abs((mid(j) - mid(near)) @ d)
            if w * 0.8 < dr < w * 2.3 and ds < 1.5:
                grp.append(j)
        if len(grp) < 3:
            continue
        left -= set(grp)
        dd = np.array([lines[j]["d"] * np.sign(lines[j]["d"] @ d) for j in grp]).mean(0)
        dd /= np.linalg.norm(dd)
        nn = np.array([-dd[1], dd[0]])
        s_lo = np.array([min((lines[j]["c"] + lines[j]["s0"] * lines[j]["d"]) @ dd, (lines[j]["c"] + lines[j]["s1"] * lines[j]["d"]) @ dd) for j in grp])
        s_hi = np.array([max((lines[j]["c"] + lines[j]["s0"] * lines[j]["d"]) @ dd, (lines[j]["c"] + lines[j]["s1"] * lines[j]["d"]) @ dd) for j in grp])
        rho = np.array([mid(j) @ nn for j in grp])
        o = np.argsort(rho)
        fit = _fit_grid(rho[o], w)
        if fit is None or fit[3] > c.get("free_row_max_rms_m", 0.08) or np.median(s_hi - s_lo) < c.get("free_row_min_len_m", 2.5):
            continue                                                              # 기준선이 없으면 더 엄격히 (난간 · 계단 같은 나란한 짧은 선)
        Lr = float(np.clip(np.median(s_hi - s_lo), 2.0, L))
        base_s = float(np.median(s_lo + s_hi) / 2 - Lr / 2)                       # 칸막이 가운데를 맞춤 (어느 쪽이 입구인지 모름)
        rows.append(dict(c=nn * 0.0 + dd * base_s, d=nn, sd=dd, skew=0.0, p=fit[0], t0=fit[1], k=fit[2], rms=fit[3], base=None, line=-1, length=Lr))

    out_lines, stalls, polys = [], [], []
    n_sep = n_add = 0
    for ri, r in enumerate(rows):
        Lr = r.get("length", L)
        ks = set(int(v) for v in r["k"])
        for kk in range(min(ks), max(ks) + 1):                                    # 빠진 칸막이: 양옆이 다 있고 max_gap 칸 안이면 채움
            if kk in ks:
                continue
            lo_k, hi_k = max(v for v in ks if v < kk), min(v for v in ks if v > kk)
            if hi_k - lo_k - 1 <= c.get("max_gap", 2):
                ks.add(kk)
                n_add += 1
        r["k_all"] = sorted(ks)
        n_sep += len(ks)
        for kk in r["k_all"]:
            a = r["c"] + (r["t0"] + kk * r["p"]) * r["d"]
            b = a + Lr * r["sd"]
            pts = a + np.linspace(0, Lr, 50)[:, None] * r["sd"]                    # 바닥 밖으로 나간 부분은 자름
            okf = F(pts)
            if okf.sum() < 5:
                continue
            last = np.nonzero(okf)[0].max()
            out_lines.append([*a, *pts[last]])
        if r["base"] is not None:
            t_lo = min(r["base"][0], r["t0"] + min(ks) * r["p"])
            t_hi = max(r["base"][1], r["t0"] + max(ks) * r["p"])
            out_lines.append([*(r["c"] + t_lo * r["d"]), *(r["c"] + t_hi * r["d"])])
        else:
            t_lo, t_hi = r["t0"] + min(ks) * r["p"], r["t0"] + max(ks) * r["p"]
        m = 0.45
        polys.append(np.array([r["c"] + (t_lo - m) * r["d"] - m * r["sd"], r["c"] + (t_hi + m) * r["d"] - m * r["sd"],
                               r["c"] + (t_hi + m) * r["d"] + (Lr + m) * r["sd"], r["c"] + (t_lo - m) * r["d"] + (Lr + m) * r["sd"]]))
        yaw = float(np.arctan2(r["sd"][1], r["sd"][0]))
        k_st = list(r["k_all"])
        if r["base"] is not None:      # 기준선이 이어진 데까지는 같은 간격의 칸이 더 있음 (칸막이가 차에 가려 안 찍힌 곳) — 선은 안 그리고 칸 자리만 (차 놓기용)
            k_st = list(range(min(k_st[0], int(np.ceil((r["base"][0] - r["t0"]) / r["p"] - 0.25))), max(k_st[-1], int(np.floor((r["base"][1] - r["t0"]) / r["p"] + 0.25))) + 1))
        for a_k, b_k in zip(k_st[:-1], k_st[1:]):
            if b_k - a_k != 1:
                continue
            mouth = r["c"] + (r["t0"] + (a_k + 0.5) * r["p"]) * r["d"]
            cen = mouth + Lr / 2 * r["sd"]
            stalls.append([float(cen[0]), float(cen[1]), yaw, float(r["p"]), float(Lr), ri, float(mouth[0]), float(mouth[1])])
    # 줄에 안 들어간 긴 선 (차로 선 · 줄 끝 선 등): 제자리에 반듯하게 다시 그림
    n_single = 0
    for i, ln in enumerate(lines):
        ext = ln["s1"] - ln["s0"]
        if used[i] or ln["cover"] < c.get("single_min_m", 2.5) or ln["cover"] < 0.75 * ext:
            continue
        a, b = ln["c"] + ln["s0"] * ln["d"], ln["c"] + ln["s1"] * ln["d"]
        mid = (a + b) / 2
        if not Fin(np.array([a, mid, b])).all():                                  # 바닥 가장자리(스캔이 번진 곳)의 선은 건드리지 않음
            continue
        inside = any(cv2.pointPolygonTest(pl.astype(np.float32), (float(q[0]), float(q[1])), False) >= 0 for pl in polys for q in (a, mid, b))
        if inside:                                                                # 줄 범위 안의 선은 줄 모형이 대신함
            continue
        n = np.array([-ln["d"][1], ln["d"][0]])
        out_lines.append([*a, *b])
        polys.append(np.array([a - 0.3 * ln["d"] - 0.3 * n, b + 0.3 * ln["d"] - 0.3 * n, b + 0.3 * ln["d"] + 0.3 * n, a - 0.3 * ln["d"] + 0.3 * n]))
        n_single += 1
    st["single_lines"] = n_single
    # 지울 자리 = 줄 범위 안의 원래 페인트 (조금 넓혀서)
    er = np.zeros(paint.shape, np.uint8)
    for pl in polys:
        px = np.c_[(pl[:, 0] - x0) / g - 0.5, (pl[:, 1] - y0) / g - 0.5]
        cv2.fillPoly(er, [np.round(px).astype(np.int32)], 1)
    bright = _paint_mask(small, fl, g, dict(c, paint_ratio=c.get("erase_ratio", 1.1), paint_min_lum=0.3, paint_min_m2=0.01))
    er = cv2.dilate((er.astype(bool) & (paint | bright)).astype(np.uint8), np.ones((3, 3), np.uint8), iterations=max(1, int(round(0.05 / g))))
    erase = np.zeros(img.shape[:2], bool)
    erase[:H, :W] = cv2.resize(er, (W, H), interpolation=cv2.INTER_NEAREST).astype(bool)
    col = np.percentile(small[paint], 85, axis=0) if paint.any() else np.array([0.8, 0.8, 0.8])   # 또렷하게 찍힌 선의 색 (가장자리는 아스팔트와 섞여 어두움)
    st["row_info"] = [dict(at=[round(float(v), 1) for v in r["c"] + r["t0"] * r["d"]], dir_deg=round(float(np.degrees(np.arctan2(r["sd"][1], r["sd"][0]))), 1),
                           n=len(r["k_all"]), baseline=r["base"] is not None) for r in rows]
    st.update(rows=len(rows), stalls=len(stalls), separators=n_sep, separators_added=n_add,
              row_pitch_m=[round(r["p"], 3) for r in rows], row_rms_cm=[round(100 * r["rms"], 1) for r in rows],
              row_skew_deg=[round(r["skew"], 1) for r in rows], erase_m2=round(float(er.sum()) * g * g, 1), paint_rgb=[round(float(v), 3) for v in col])
    return dict(rows=rows, lines=out_lines, stalls=stalls, erase=erase, stats=st, paint_rgb=col, paint=paint, grid=(x0, y0, g))


def _bilin(src, px, py):
    """src[iy, ix, 3] (uint8 · float) 를 화소 좌표 (px, py) 에서 읽음 → float32 0~1."""
    H, W = src.shape[:2]
    x0 = np.clip(np.floor(px).astype(int), 0, W - 2)
    y0 = np.clip(np.floor(py).astype(int), 0, H - 2)
    fx, fy = np.clip(px - x0, 0, 1)[..., None], np.clip(py - y0, 0, 1)[..., None]
    sc = 1 / 255.0 if src.dtype == np.uint8 else 1.0
    return ((src[y0, x0] * (1 - fx) + src[y0, x0 + 1] * fx) * (1 - fy) + (src[y0 + 1, x0] * (1 - fx) + src[y0 + 1, x0 + 1] * fx) * fy).astype(np.float32) * sc


def draw(img, det, xs, ys, on_floor, c, orig=None):
    """새 주차선을 img 에 그림 (제자리) → 그린 넓이 m².
    style photo (orig = 지우기 전 그림이 있을 때): 선은 반듯하고 또렷하게 긋되, 닳은 정도를 실제 사진에서 가져옴 —
      반듯한 선을 따라가며 원래 찍힌 선(옆으로 어긋난 만큼 중심 추적)이 둘레 아스팔트보다 얼마나 밝은지를 재서 그 자리의 페인트 진하기로 씀.
      실제로 흐려진 곳은 흐리게, 또렷한 곳은 또렷하게 남고 구불거림 · 끊김 · 겹침(자세 오차)만 없어짐. 가장자리는 수 mm 들쭉날쭉, 바닥 잔무늬가 비침.
      원래 선이 안 찍힌 구간(차에 가림 · 빠진 칸막이)은 잘 찍힌 선들의 닳은 무늬를 이어 씀.
    style clean: 한 가지 진하기 선 (line_w_m)."""
    cell = float(xs[1] - xs[0])
    x0, y0 = float(xs[0] - cell / 2), float(ys[0] - cell / 2)
    if orig is None or c.get("style", "photo") != "photo" or "paint" not in det:
        S = 4                                                                      # 부분 화소 정밀도
        a = np.zeros(img.shape[:2], np.uint8)
        hw = c.get("line_w_m", 0.15) / 2
        for x1, y1, x2, y2 in det["lines"]:
            p, q = np.array([x1, y1]), np.array([x2, y2])
            d = (q - p) / max(np.linalg.norm(q - p), 1e-9)
            n = np.array([-d[1], d[0]])
            poly = np.array([p - d * hw + n * hw, q + d * hw + n * hw, q + d * hw - n * hw, p - d * hw - n * hw])
            px = np.c_[(poly[:, 0] - x0) / cell - 0.5, (poly[:, 1] - y0) / cell - 0.5]
            cv2.fillPoly(a, [np.round(px * (1 << S)).astype(np.int32)], 255, lineType=cv2.LINE_AA, shift=S)
        alpha = cv2.GaussianBlur(a.astype(np.float32) / 255, (0, 0), 0.8) * c.get("opacity", 0.92) * on_floor
        lum = img.mean(-1)
        tex = lum / np.maximum(cv2.GaussianBlur(lum, (0, 0), 3.0), 1e-3)            # 바닥 잔무늬 (선에도 비치게)
        m = alpha > 0.003
        paint = np.asarray(det.get("paint_rgb", [0.8, 0.8, 0.8]), np.float32)[None, :] * np.clip(tex[m], 0.85, 1.15)[:, None]
        img[m] = np.clip(img[m] * (1 - alpha[m][:, None]) + paint * alpha[m][:, None], 0, 1)
        return float(m.sum()) * cell * cell
    gx0, gy0, g = det["grid"]
    P = _Sampler(det["paint"], gx0, gy0, g)
    hw_in, hw_out = c.get("strip_core_m", 0.11), c.get("strip_half_m", 0.24)        # 이 안은 사진 그대로 · 여기까지 부드럽게 섞음
    step = cell / 2                                                                 # 돌아간 선에서 빈 화소가 안 생기게 반 화소 간격
    rr = np.arange(-hw_out, hw_out + 1e-9, step)
    alpha_r = np.clip((hw_out - np.abs(rr)) / (hw_out - hw_in), 0, 1)
    alpha_r = (alpha_r * alpha_r * (3 - 2 * alpha_r)).astype(np.float32)
    ro = np.arange(-c.get("track_reach_m", 0.5), c.get("track_reach_m", 0.5) + 1e-9, g)
    jobs = []
    for x1, y1, x2, y2 in det["lines"]:
        a, b = np.array([x1, y1]), np.array([x2, y2])
        Ls = float(np.linalg.norm(b - a))
        if Ls < 0.2:
            continue
        d = (b - a) / Ls
        n = np.array([-d[1], d[0]])
        ss = np.arange(0, Ls, step)
        # 원래 선 중심이 옆으로 어긋난 정도 o(s): 옆 단면의 페인트 무게중심 → 중앙값 · 가우시안으로 다듬음. 못 찾은 긴 구간은 NaN
        hit = P(a + ss[:, None, None] * d + ro[None, :, None] * n)
        cnt = hit.sum(1)
        o = np.where(cnt >= 2, (hit * ro[None, :]).sum(1) / np.maximum(cnt, 1), np.nan)
        wide = cnt * g > c.get("track_max_w_m", 0.45)                                # 단면이 너무 넓으면 (다른 선과 만나는 곳) 그 자리 값은 안 믿음
        o[wide] = np.nan
        ok = np.isfinite(o)
        if ok.sum() >= 5:
            idx = np.arange(len(o))
            o_f = np.interp(idx, idx[ok], o[ok])
            o_f = ndimage.gaussian_filter1d(ndimage.median_filter(o_f, size=max(3, int(0.3 / step)) | 1), 0.1 / step)
            gap = ndimage.distance_transform_edt(~ok) * step                        # 가장 가까운 찾은 곳까지 거리
            have = gap < c.get("track_gap_m", 0.35)
        else:
            o_f, have = np.zeros(len(o)), np.zeros(len(o), bool)
        src = a + ss[:, None, None] * d + (o_f[:, None, None] + rr[None, :, None]) * n
        strip = _bilin(orig, (src[..., 0] - x0) / cell - 0.5, (src[..., 1] - y0) / cell - 0.5)
        jobs.append(dict(a=a, d=d, n=n, ss=ss, strip=strip, have=have, valid=float(have.mean())))
    # 선마다 '닳은 정도' wear(s): 실제 사진에서 선 가운데가 둘레 아스팔트보다 얼마나 밝은가 (0 = 다 지워짐 · 1 = 또렷한 페인트)
    core = np.abs(rr) < c.get("line_w_m", 0.15) * 0.3
    edge = np.abs(rr) > hw_out * 0.8
    LUM = np.array([0.299, 0.587, 0.114], np.float32)
    pl = float(np.asarray(det.get("paint_rgb", [0.8, 0.8, 0.8]), np.float32) @ LUM)
    for j in jobs:
        cl, el = (j["strip"][:, core] @ LUM).mean(1), (j["strip"][:, edge] @ LUM).mean(1)
        w_ = np.clip((cl - el) / np.maximum(pl - el, 0.05), 0, 1)
        j["wear"] = ndimage.gaussian_filter1d(w_, 0.03 / step)
    lib = [j["wear"][j["have"]] for j in jobs if j["valid"] >= 0.8 and len(j["ss"]) >= int(1.5 / step)]
    lib = np.concatenate(lib) if lib else None                                      # 안 찍힌 구간(차에 가림 · 빠진 칸막이)은 잘 찍힌 선들의 닳은 무늬를 이어 씀
    rng = np.random.default_rng(0)
    lum_img = img @ LUM
    tex = lum_img / np.maximum(cv2.GaussianBlur(lum_img, (0, 0), 3.0), 1e-3)         # 바닥 잔무늬 (선에도 비치게)
    del lum_img
    paint_rgb = np.asarray(det.get("paint_rgb", [0.8, 0.8, 0.8]), np.float32)
    hw = c.get("line_w_m", 0.15) / 2
    soft = c.get("edge_soft_m", 0.012)
    w_lo, w_hi = c.get("wear_range", [0.5, 1.0])
    drawn = np.zeros(img.shape[:2], bool)
    n_lib = n_real = 0
    for j in jobs:
        wear, have = j["wear"].copy(), j["have"]
        miss = ~have
        if miss.any() and lib is not None and len(lib):
            k0 = int(rng.integers(len(lib)))
            wear[miss] = lib[(k0 + np.arange(int(miss.sum()))) % len(lib)]
        elif miss.any():
            wear[miss] = 0.8
        wear = ndimage.gaussian_filter1d(wear, 0.02 / step)
        n_lib += int(miss.sum())
        n_real += int(have.sum())
        jit = ndimage.gaussian_filter1d(rng.normal(0, 1, len(wear)), 0.04 / step) * c.get("edge_jitter_m", 0.006) * (0.04 / step) ** 0.5   # 가장자리 들쭉날쭉 (± 수 mm)
        m = np.clip((hw + jit[:, None] - np.abs(rr)[None, :]) / soft + 0.5, 0, 1) * (w_lo + (w_hi - w_lo) * wear)[:, None] * c.get("opacity", 0.95)
        tgt = j["a"] + j["ss"][:, None, None] * j["d"] + rr[None, :, None] * j["n"]
        ix = np.round((tgt[..., 0] - x0) / cell - 0.5).astype(int)
        iy = np.round((tgt[..., 1] - y0) / cell - 0.5).astype(int)
        inb = (ix >= 0) & (ix < img.shape[1]) & (iy >= 0) & (iy < img.shape[0])
        ixc, iyc = np.clip(ix, 0, img.shape[1] - 1), np.clip(iy, 0, img.shape[0] - 1)
        al = (m * inb * on_floor[iyc, ixc]).astype(np.float32)
        acc = np.zeros(img.shape[:2], np.float32) if False else None
        # 같은 화소를 여러 표본이 덮으므로 (반 화소 간격) 화소마다 가장 큰 값으로
        flat = iyc.ravel() * img.shape[1] + ixc.ravel()
        o_ = np.argsort(al.ravel())
        uq, first = np.unique(flat[o_][::-1], return_index=True)
        amax = al.ravel()[o_][::-1][first]
        py_, px_ = uq // img.shape[1], uq % img.shape[1]
        sel = amax > 0.003
        py_, px_, amax = py_[sel], px_[sel], amax[sel][:, None]
        img[py_, px_] = np.clip(img[py_, px_] * (1 - amax) + paint_rgb[None, :] * np.clip(tex[py_, px_], 0.88, 1.12)[:, None] * amax, 0, 1)
        drawn[py_[amax[:, 0] > 0.3], px_[amax[:, 0] > 0.3]] = True
    det["stats"]["photo_strip"] = dict(wear_from_photo_pct=round(100 * n_real / max(n_real + n_lib, 1), 1), library=lib is not None)
    return float(drawn.sum()) * cell * cell


def save(path, det):
    json.dump(dict(stalls=det["stalls"], lines=[[float(v) for v in ln] for ln in det["lines"]], stats=det["stats"],
                   note="stalls = [cx, cy, yaw_rad(칸 길이 방향), 폭, 길이, 줄 번호, 기준선 쪽 가운데 x, y]"),
              open(path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)


if __name__ == "__main__":                                                         # 시험: python stalls.py <빌드 폴더> [그림 이름]
    import os
    import sys

    from PIL import Image
    Image.MAX_IMAGE_PIXELS = None
    d = sys.argv[1]
    mt = np.load(os.path.join(d, "floor_tex", "bake_meta.npz"))
    x0_, x1_, y0_, y1_ = mt["region"]
    cell_ = float(mt["cell"])
    im = np.asarray(Image.open(os.path.join(d, "floor_tex", "floor_color.jpg")).convert("RGB"))[::-1].astype(np.float32) / 255
    xs_, ys_ = x0_ + (np.arange(im.shape[1]) + 0.5) * cell_, y0_ + (np.arange(im.shape[0]) + 0.5) * cell_
    import open3d as o3d
    fv = np.asarray(o3d.io.read_triangle_mesh(os.path.join(d, "floor.ply")).vertices)
    fp = np.zeros((im.shape[0] // 5 + 1, im.shape[1] // 5 + 1), bool)
    fp[np.clip(((fv[:, 1] - y0_) / (5 * cell_)).astype(int), 0, fp.shape[0] - 1), np.clip(((fv[:, 0] - x0_) / (5 * cell_)).astype(int), 0, fp.shape[1] - 1)] = True
    onf = cv2.resize(ndimage.binary_closing(fp, iterations=2).astype(np.uint8), (im.shape[1], im.shape[0]), interpolation=cv2.INTER_NEAREST).astype(bool)
    cfg_ = {}
    det_ = detect(im, onf, xs_, ys_, cfg_)
    print(json.dumps(det_["stats"], ensure_ascii=False))
    out = im.copy()
    out[det_["erase"]] = out[det_["erase"]] * 0.3 + np.array([1, 0, 0], np.float32) * 0.7
    vis = (out[::-1][::2, ::2] * 255).astype(np.uint8).copy()
    out2 = im.copy()
    er = det_["erase"]
    blur = cv2.GaussianBlur(np.where(er[..., None], 0, out2), (0, 0), 12) / np.maximum(cv2.GaussianBlur((~er).astype(np.float32), (0, 0), 12), 1e-3)[..., None]
    out2[er] = blur[er]
    draw(out2, det_, xs_, ys_, onf, cfg_, orig=(im * 255).astype(np.uint8))
    print(det_["stats"].get("photo_strip"))
    vis2 = (np.clip(out2[::-1][::2, ::2], 0, 1) * 255).astype(np.uint8)
    name = sys.argv[2] if len(sys.argv) > 2 else "stalls_test"
    Image.fromarray(np.hstack([vis, vis2])[::2, ::2]).save(name + ".jpg", quality=88)
    Image.fromarray(vis2).save(name + "_after.jpg", quality=88)
    save(name + ".json", det_)
