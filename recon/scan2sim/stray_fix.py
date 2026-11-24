"""아이폰(Stray) 키프레임 자세의 높이 틀어짐을 바닥으로 바로잡음 — 깊이를 합치기(stray_mesh) 전에. B 의 단계.

    cd recon
    python -m scan2sim.stray_fix <맵 폴더> [--json <경로>] [--tilt]     # 재기만 함 (맵 폴더 · 메쉬는 그대로). 고쳐서 합치기는 stray_mesh 가 함

왜
    아이폰 자세(VIO)에는 되돌아온 자리를 맞추는 보정이 없어 긴 촬영에서 카메라 높이가 조금씩 틀어진다. 그대로 합치면 같은 바닥이
    두 높이로 찍혀 진단에 '바닥 두 겹' · '낮은 턱' 으로 나온다 (기준 촬영본 118 초: 키프레임 높이 오차 5~95% 약 ±5 cm, 최대 약 20 cm).

방법 (키프레임 자세의 높이만 고침 — 수평 위치 · 방향은 그대로)
    1. 키프레임마다 위를 향한 면(법선 z > 0.9 · 카메라보다 0.3~2.6 m 아래 · 깊이 3 m 안 — 바닥 · 턱 · 단 · 가구 윗면)을 20 cm 칸으로 나눠
       칸마다 넓이 가중 중앙 높이 y_kc 를 잼. 깊이는 키프레임 깊이 그림을 원래 크기(아이폰 256×192)로 모든 화소를 씀
       (960 처럼 키운 그림이면 256 근처가 되게 건너뜀 — step)
    2. y_kc = e_k + f_c 를 풂. e_k = 키프레임 k 자세의 높이 오차, f_c = 칸 c 의 실제 높이 (칸마다 따로 — 단 · 계단 · 턱을 평평하게 누르지 않음).
       같은 칸을 다른 때 본 키프레임끼리 높이가 다르면 그 차이만 e 로 감. 한 키프레임만 본 칸은 아무것도 정하지 않음.
       희소 가중 최소제곱 + Huber (2 cm, 6회, 3회째부터 10 cm 넘게 어긋난 관측은 버림 — 움직인 물체 · 가구 가장자리).
       e 는 시간으로 이어지게 (무작위 걸음 사전 q = 0.02 m/√s — 바닥을 못 본 키프레임은 앞뒤를 이어 메움), 바닥 근거가 있는 키프레임의
       e 중앙값 = 0 (가장 적게 옮김)
    3. 고친 자세 = 카메라 높이 − e_k. tilt=True 면 바닥 기울기 오차도 풀어 카메라 중심으로 돌림 (기본 끔 — 기준 촬영본에서 이득이 작았음)
    같은 입력이면 같은 결과 (난수 없음 · 스레드는 장마다 따로 계산하고 순서대로 모음).

관문 — 걸리면 자세를 그대로 둠 (info.applied = False)
    refused  바닥을 다른 키프레임과 10칸 넘게 같이 본 키프레임이 30% 미만 · 보정이 40 cm 넘음(틀어짐이 아니라 추적이 깨진 것) ·
             고친 뒤에도 바닥 높이 잔차 RMS 가 3 cm 넘음 (이 모델로 못 고침)
    skipped  보정 폭(2~98%)이 1.5 cm 미만 — 고칠 것 없음 (가상의 방처럼 자세가 정확한 촬영)
표시 — 고치되 알림 (info.flags)
    키프레임 5% 넘게가 주 바닥과 8 cm 넘게 다른 층에 서 있었음 (단 · 계단 — 층은 칸 높이로 따로 두므로 누르지 않음, 눈으로 확인) ·
    바닥을 못 본 구간이 10 초 넘음 (그 사이는 앞뒤를 이은 값) · 한 장만 튄 키프레임 (시간으로 9장 중앙에서 10 cm 넘게 — 목록만, 빼지 않음)

info (stray_mesh 가 <맵 폴더>/floor_fix.json 으로 씀 · tsdf_meta.json 의 pose_fix 에 요약)
    method · version · params · keyframes · applied · status(applied | skipped | refused | off | error) · reasons · flags ·
    t (키프레임 시각) · dz (키프레임마다 카메라 높이에서 뺀 값 m — 고치지 않았으면 0) · dz_estimate (고치지 않았을 때만, 추정값) · tilt ·
    frames_supported · frames_supported_pct · cells · observations · observations_dropped · resid_rms_cm · resid_med_cm ·
    dz_range_cm (2~98%) · dz_p5_p95_cm · dz_max_abs_cm · max_unsupported_s · jump_frames · jump_times · levels · seconds{observe, solve, total}
    apply_fix(시각, 자세, info) = 같은 보정을 키프레임이 아닌 프레임에도 (시각으로 이어 붙임) — stray_quality 가 안 쓴 프레임에 씀

측정 (11/24 B, 키프레임 깊이 256×192. 바닥 평탄도 = 메쉬에서 바닥 근처 위를 향한 면 · C 기준 띠 = 바닥 10~25 cm 아래 · 5~10 cm 위 넓이,
      두 겹 = 10 cm 칸에 4 cm 넘게 떨어진 두 층. 안 쓴 프레임 = stray_quality n=60 중앙 차이)
    기준 촬영본 118 초, 키프레임 1133장(기본): 고침 — 보정 5~95% −7.4~+4.8 cm · 최대 29.9 cm(한 장 튄 키프레임) · 잔차 1.96 cm · 6.0 초.
        C 기준 띠 14.6 → 7.8% · 바닥 두 겹 칸 19.8 → 12.9% · 안 쓴 프레임 2.03 → 1.89 cm · 벽 두 겹 22.7 → 21.9%.
        천장(보정에 안 쓴 면): 주 높이에서 5 cm 넘게 벗어난 넓이 50.7 → 37.4% 이지만 두 겹 칸 5.2 → 10.3% (나빠짐 — 원인 모름)
    같은 촬영 키프레임 681장: C 기준 띠 14.1 → 7.4% · 두 겹 22.4 → 11.0% · 안 쓴 프레임 1.77 → 1.84 cm (차이 없음) · 천장 두 겹 2.9 → 4.8%
    가상의 방 (자세가 정확): 보정 폭 0.3 cm → 건너뜀, mesh_raw.ply 가 바이트까지 같음 (턱 4.38 · 판 1.40 cm 그대로).
        같은 방에 높이 틀어짐을 넣으면 (8 cm 사인 + 12 초에 6 cm 계단) 장마다 되찾은 오차 중앙 0.05 cm · 95% 0.14 cm,
        턱 2.59 → 4.38 · 판 −0.90 → 1.39 cm (틀어짐 없을 때 4.38 · 1.40)
    tilt=True (기본 끔, 1133장): 두 겹 4.96% · C 기준 띠 3.0% · 안 쓴 프레임 1.78 cm · 천장 두 겹 10.1% — 바닥은 훨씬 좋아지나 더 확인 필요
"""
import argparse
import json
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

METHOD, VERSION = "floor_cells", 1
NATIVE_W = 256                                                  # 아이폰 깊이 원래 가로 크기 — 키운 깊이 그림은 이 근처가 되게 건너뜀
WORKERS = 2                                                     # 깊이를 읽고 칸으로 나누는 스레드 (cv2 · numpy 가 GIL 을 놓음)
DEFAULTS = dict(step=None, max_depth=3.0, cell=0.20, nz_min=0.9, band=(0.3, 2.6), min_views=2,
                q=0.02, sig0=0.01, sig_r=0.005, huber=0.02, iters=6,
                tilt=False, tilt_sigma_deg=1.0, tilt_q_deg=0.5)
GATES = dict(min_frames_frac=0.3, min_shared=10, max_abs_m=0.40, max_resid_cm=3.0, noop_range_m=0.015,
             level_step_m=0.08, level_flag_pct=5.0, gap_flag_s=10.0, jump_m=0.10)
SUMMARY_KEYS = ("method", "applied", "status", "reasons", "flags", "keyframes", "frames_supported_pct", "dz_p5_p95_cm", "dz_max_abs_cm",
                "resid_rms_cm", "max_unsupported_s", "stand_other_pct", "jump_frames", "seconds")


# ── 읽기 ─────────────────────────────────────────────────────────────────

def read_depth(path):
    """키프레임 깊이 그림(16비트 mm) → m (float32). 없거나 못 읽으면 None"""
    import cv2
    if not path:
        return None
    z = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    return None if z is None else z.astype(np.float32) / 1000.0


# ── 키프레임 하나: 위를 향한 면 → 칸별 높이 ─────────────────────────────

def frame_points(z, K, T, step=1, max_depth=3.0):
    """깊이(m) → 세계 점 · 세계 법선(카메라 쪽) · 면 넓이(m²), step 화소마다. K = (fx, fy, cx, cy) — 그림 크기 기준, 화소 중심 좌표"""
    fx, fy, cx, cy = K
    z = z[::step, ::step]
    h, w = z.shape
    X = (np.arange(w)[None, :] * step - cx) / fx * z
    Y = (np.arange(h)[:, None] * step - cy) / fy * z
    P = np.stack([X, Y, z], -1)
    ok = (z > 0) & (z <= max_depth)
    c = z[1:-1, 1:-1]
    m = ok[1:-1, 1:-1].copy()
    for nb in (z[1:-1, 2:], z[1:-1, :-2], z[2:, 1:-1], z[:-2, 1:-1]):
        m &= (nb > 0) & (np.abs(nb - c) < 0.08 * c + 0.02)      # 물체 가장자리 건너뛰기
    n = -np.cross(P[1:-1, 2:] - P[1:-1, :-2], P[2:, 1:-1] - P[:-2, 1:-1])
    nn = np.linalg.norm(n, axis=-1)
    m &= nn > 0
    Pc, n, zc = P[1:-1, 1:-1][m], n[m] / nn[m][:, None], c[m]
    inc = np.clip(np.abs((n * Pc).sum(1)) / np.linalg.norm(Pc, axis=1), 0.25, 1.0)
    area = zc ** 2 * step * step / (fx * fy) / inc
    R, t = T[:3, :3], T[:3, 3]
    return Pc @ R.T + t, n @ R.T, area


def frame_cells(Pw, nw, area, cam, cell=0.2, nz_min=0.9, band=(0.3, 2.6)):
    """위를 향한 면(카메라보다 band 만큼 아래)을 칸으로 → 열 (ix, iy, 넓이 가중 중앙 높이, 넓이, 카메라→칸 dx, dy). 면이 적으면 None"""
    h = cam[2] - Pw[:, 2]
    s = (nw[:, 2] > nz_min) & (h > band[0]) & (h < band[1])
    if s.sum() < 30:
        return None
    P, a = Pw[s], area[s]
    ij = np.floor(P[:, :2] / cell).astype(np.int64)
    key = ij[:, 0] * 2000003 + ij[:, 1]
    o = np.lexsort((P[:, 2], key))
    key, P, a, ij = key[o], P[o], a[o], ij[o]
    st = np.r_[0, np.nonzero(np.diff(key))[0] + 1]
    asum = np.add.reduceat(a, st)
    cw = np.cumsum(a)
    base = np.r_[0, cw[st[1:] - 1]]
    med = P[np.minimum(np.searchsorted(cw, base + 0.5 * asum), len(P) - 1), 2]
    dx = np.add.reduceat(a * (P[:, 0] - cam[0]), st) / asum
    dy = np.add.reduceat(a * (P[:, 1] - cam[1]), st) / asum
    keep = asum > 0.1 * cell * cell
    return np.c_[ij[st, 0], ij[st, 1], med, asum, dx, dy][keep]


def observe(depth, K, poses, step=None, max_depth=3.0, cell=0.2, nz_min=0.9, band=(0.3, 2.6), workers=WORKERS, **_):
    """키프레임마다 칸별 높이 → 한 표 (열: k, ix, iy, y, area, dx, dy).
    depth = 깊이 그림 경로 목록(None 이면 그 장 건너뜀) 또는 k → 깊이(m) 함수. K = (fx, fy, cx, cy) 하나 또는 장마다 (N×4)"""
    Kf = np.asarray(K, dtype=float)
    per = Kf.ndim == 2

    def one(k):
        z = depth(k) if callable(depth) else read_depth(depth[k])
        if z is None:
            return None
        s = int(step) if step else max(1, int(round(z.shape[1] / NATIVE_W)))
        Pw, nw, ar = frame_points(z, Kf[k] if per else Kf, poses[k], s, max_depth)
        c = frame_cells(Pw, nw, ar, poses[k][:3, 3], cell, nz_min, band)
        return None if c is None else np.c_[np.full(len(c), k), c]

    if callable(depth) or workers <= 1:                         # 받은 함수는 스레드에 안전한지 모름
        out = [one(k) for k in range(len(poses))]
    else:
        with ThreadPoolExecutor(workers) as ex:
            out = list(ex.map(one, range(len(poses))))          # map 은 순서를 지킴 — 결과가 실행마다 같음
    out = [c for c in out if c is not None]
    return np.concatenate(out) if out else np.zeros((0, 7))


# ── 풀이 ─────────────────────────────────────────────────────────────────

def solve(t, n_frames, obs, cell=0.2, min_views=2, q=0.02, sig0=0.01, sig_r=0.005, huber=0.02, iters=6,
          tilt=False, tilt_sigma_deg=1.0, tilt_q_deg=0.5, **_):
    """→ dict(e (n_frames,), g (n_frames, 2) 또는 None, cells (ix, iy, f, views), res · used · w0 (관측마다), shared (키프레임마다 쓴 칸 수)).
    두 키프레임 이상이 본 칸이 없으면 None"""
    from scipy import sparse
    from scipy.sparse.linalg import spsolve
    k = obs[:, 0].astype(int)
    key = obs[:, 1].astype(np.int64) * 2000003 + obs[:, 2].astype(np.int64)
    _, ci, cnt = np.unique(key, return_inverse=True, return_counts=True)
    good = cnt[ci] >= min_views                                 # 두 키프레임 이상이 본 칸만
    if not good.any():
        return None
    k, ci, o = k[good], ci[good], obs[good]
    _, ci = np.unique(ci, return_inverse=True)
    C, N = int(ci.max()) + 1, n_frames
    nt = 2 * N if tilt else 0
    nx = N + nt + C
    d = o[:, 5:7]
    order = np.argsort(ci, kind="stable")
    f0 = np.array([np.median(v) for v in np.split(o[order, 3], np.cumsum(np.bincount(ci, minlength=C))[:-1])])
    y = o[:, 3] - f0[ci]                                        # 칸마다 중앙값을 빼고 풂 — 0 사전이 절대 높이에 끌리지 않게
    r_h = np.hypot(d[:, 0], d[:, 1])
    w0 = np.minimum(1.0, o[:, 4] / (0.5 * cell * cell)) / (sig0 ** 2 + (sig_r * r_h) ** 2)
    m = len(y)
    rows, cols, vals = [np.arange(m), np.arange(m)], [k, N + nt + ci], [np.ones(m), np.ones(m)]
    if tilt:
        rows += [np.arange(m), np.arange(m)]
        cols += [N + 2 * k, N + 2 * k + 1]
        vals += [d[:, 0], d[:, 1]]
    A_d = sparse.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(m, nx))
    # 사전: e 시간 이음 (무작위 걸음) · e 0 근처 (σ 1 m — 게이지만 잡음) · 기울기 0 근처 · 시간 이음
    dt = np.maximum(np.diff(t), 1e-3)
    sw = 1.0 / (q * np.sqrt(dt) + 0.002)
    i = np.arange(N - 1)
    P_rows = [sparse.csr_matrix((np.r_[-sw, sw], (np.r_[i, i], np.r_[i, i + 1])), shape=(N - 1, nx)),
              sparse.csr_matrix((np.full(N, 1.0), (np.arange(N), np.arange(N))), shape=(N, nx))]
    if tilt:
        sg, sq = np.radians(tilt_sigma_deg), np.radians(tilt_q_deg)
        for a in (0, 1):
            P_rows.append(sparse.csr_matrix((np.full(N, 1 / sg), (np.arange(N), N + 2 * np.arange(N) + a)), shape=(N, nx)))
            P_rows.append(sparse.csr_matrix((np.r_[-np.full(N - 1, 1 / sq), np.full(N - 1, 1 / sq)],
                                             (np.r_[i, i], N + 2 * np.r_[i, i + 1] + a)), shape=(N - 1, nx)))
    A_p = sparse.vstack(P_rows).tocsr()
    ApA = (A_p.T @ A_p + sparse.eye(nx) * 1e-4).tocsc()        # 아주 약한 0 사전 — 관측이 모두 버려진 칸도 풀리게
    w = w0.copy()
    for it in range(iters):
        AtW = A_d.T.multiply(w)
        x = spsolve((AtW @ A_d).tocsc() + ApA, AtW @ y)
        res = y - A_d @ x
        s = np.abs(res)
        w = w0 * np.where(s < huber, 1.0, huber / np.maximum(s, 1e-9))
        if it >= 2:
            w[s > 5 * huber] = 0                                # 3회째부터 10 cm 넘게 어긋난 관측을 버림 (움직인 물체 · 가구 가장자리)
    used = w > 0
    cells = np.c_[o[np.unique(ci, return_index=True)[1], 1:3], x[N + nt:] + f0, np.bincount(ci[used], minlength=C)]
    return dict(e=x[:N], g=x[N:N + nt].reshape(N, 2) if tilt else None, cells=cells, res=res, used=used, w0=w0,
                shared=np.bincount(k[used], minlength=N))


# ── 층 · 고친 자세 ───────────────────────────────────────────────────────

def _levels(cells, cam, cell, step_m, near=0.8, hand=(0.9, 2.2)):
    """키프레임마다 '서 있던 바닥' = 카메라 near m 안 · 카메라보다 hand 만큼 아래인 칸 높이의 중앙 (책상 · 소파 윗면은 hand 밖이라 빠짐).
    그 높이가 주 층에서 step_m 넘게 다른 키프레임이 많으면 단 · 계단 위를 걸은 촬영. cam = 고친 카메라 위치 (n, 3)"""
    from scipy.spatial import cKDTree
    c = cells[cells[:, 3] >= 3]
    if not len(c):
        return dict(stand_frames=0)
    tr = cKDTree((c[:, :2] + 0.5) * cell)
    stand = np.full(len(cam), np.nan)
    for k, nb in enumerate(tr.query_ball_point(cam[:, :2], near)):
        if len(nb) >= 3:
            f = c[nb, 2]
            f = f[(cam[k, 2] - f > hand[0]) & (cam[k, 2] - f < hand[1])]
            if len(f) >= 3:
                stand[k] = np.median(f)
    s = stand[np.isfinite(stand)]
    if len(s) < 10:
        return dict(stand_frames=int(len(s)))
    h, e = np.histogram(s, bins=np.arange(s.min() - 0.02, s.max() + 0.03, 0.02))
    main = e[np.argmax(h)] + 0.01
    off = s - main
    hi, lo = off[off > step_m], off[off < -step_m]
    return dict(stand_frames=int(len(s)), stand_main_m=round(float(main), 3),
                stand_other_pct=round(100 * float((np.abs(off) > step_m).mean()), 1),
                stand_above_m=round(float(np.median(hi)), 3) if len(hi) else None, stand_below_m=round(float(np.median(lo)), 3) if len(lo) else None,
                cells_other_pct=round(100 * float((np.abs(c[:, 2] - main) > step_m).mean()), 1))   # 참고: 가구 윗면 · 턱 포함


def _tilt_rot(gx, gy):
    """보인 바닥 기울기 (gx, gy) 를 없애는 회전 (기울어진 법선 → 수직, 로드리게스)"""
    n = np.array([-gx, -gy, 1.0])
    n /= np.linalg.norm(n)
    ax = np.cross(n, [0, 0, 1.0])
    s = np.linalg.norm(ax)
    if s < 1e-12:
        return np.eye(3)
    a = ax / s
    th = np.arcsin(min(s, 1.0))
    Kx = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * Kx + (1 - np.cos(th)) * Kx @ Kx


def apply_fix(times, poses, info):
    """info 의 키프레임 보정을 시각으로 이어 아무 프레임(키프레임 · 안 쓴 프레임)에 적용 → 새 자세 (n, 4, 4).
    info.applied 가 아니면 그대로 (복사본). 키프레임 시각 밖은 처음 · 끝 값"""
    T = np.array(poses, dtype=float, copy=True)
    if not info or not info.get("applied"):
        return T
    tk = np.asarray(info["t"], dtype=float)
    times = np.atleast_1d(np.asarray(times, dtype=float))
    if info.get("tilt") is not None:
        g = np.asarray(info["tilt"], dtype=float)
        gx, gy = np.interp(times, tk, g[:, 0]), np.interp(times, tk, g[:, 1])
        for i in range(len(T)):
            T[i, :3, :3] = _tilt_rot(gx[i], gy[i]) @ T[i, :3, :3]
    T[:, 2, 3] -= np.interp(times, tk, np.asarray(info["dz"], dtype=float))
    return T


def not_applied(status, reason, **extra):
    """고치지 않은 info (보정을 껐음 · 오류 등) — floor_fix.json 에 같은 모양으로 남기려고"""
    return dict(method=METHOD, version=VERSION, applied=False, status=status, reasons=[reason], flags=[], **extra)


def summary(info):
    """tsdf_meta.json 의 pose_fix 에 넣을 짧은 요약 (t · dz 목록은 뺌)"""
    info = info if isinstance(info, dict) else {}
    s = {k: info.get(k) for k in SUMMARY_KEYS if k in info}
    s["applied"] = bool(info.get("applied"))
    s.setdefault("status", "applied" if s["applied"] else "unknown")
    lv = info.get("levels") or {}
    if "stand_other_pct" in lv:
        s["stand_other_pct"] = lv["stand_other_pct"]
    return s


def check_opts(kw):
    """floor_fix 에 줄 설정 이름 확인 — 모르는 이름이면 ValueError"""
    bad = sorted(set(kw) - set(DEFAULTS) - set(GATES) - {"workers"})
    if bad:
        raise ValueError(f"floor_fix 에 없는 설정: {bad} (설정: {sorted(DEFAULTS)} · 관문: {sorted(GATES)})")


def floor_fix(times, poses, depth, K, *, tilt=False, cell=0.2, q=0.02, log=print, **kw):
    """키프레임 시각 · 자세(카메라 광학 → 세계 Z 위, N×4×4) · 깊이(경로 목록 또는 k → 깊이 m 함수) · 내부 파라미터((fx, fy, cx, cy) 또는 N×4)
    → (고친 자세, info). 관문에 걸리면 자세를 그대로(복사본) 돌려주고 info.applied = False · status · reasons.
    kw: DEFAULTS · GATES 의 값을 바꿈 (예: noop_range_m=0 — 시험용으로 관문 끄기), workers = 스레드 수"""
    check_opts(kw)
    workers = kw.pop("workers", WORKERS)
    p = dict(DEFAULTS, tilt=bool(tilt), cell=float(cell), q=float(q))
    g = dict(GATES)
    for k, v in kw.items():
        (p if k in DEFAULTS else g)[k] = v
    t0 = time.time()
    times = np.asarray(times, dtype=float)
    poses = np.asarray(poses, dtype=float)
    N = len(times)
    info = dict(method=METHOD, version=VERSION, params={k: (list(v) if isinstance(v, tuple) else v) for k, v in {**p, **g}.items()},
                keyframes=N, applied=False, status="refused", reasons=[], flags=[], t=times.tolist())

    def done(out, why=None):
        if why:
            info["reasons"].append(why)
        info.setdefault("dz", [0.0] * N)
        info.setdefault("seconds", dict(total=round(time.time() - t0, 2)))
        log(f"[floor_fix] {dict(applied='고침', skipped='고칠 것 없음', refused='안 고침').get(info['status'], info['status'])} | 키프레임 {N}장"
            + (f" 중 바닥 근거 {info['frames_supported_pct']}% | 보정 5~95% {info['dz_p5_p95_cm']} cm · 최대 {info['dz_max_abs_cm']} cm | "
               f"잔차 {info['resid_rms_cm']} cm" if "dz_p5_p95_cm" in info else "") + f" | {info['seconds']['total']} s"
            + "".join(f"\n  - {m}" for m in info["reasons"] + info["flags"]))
        return out, info

    if poses.shape != (N, 4, 4):
        raise ValueError(f"자세 모양이 {poses.shape} — 시각 {N}개에 맞는 ({N}, 4, 4) 여야 함")
    if N < 2:
        return done(poses.copy(), f"키프레임이 {N}장 — 2장 이상이어야 고칠 수 있음")
    if np.any(np.diff(times) < 0):
        return done(poses.copy(), "키프레임 시각이 차례대로가 아님 — 고치지 않음")
    obs = observe(depth, K, poses, workers=workers, **p)
    t1 = time.time()
    S = solve(times, N, obs, **p) if len(obs) else None
    t2 = time.time()
    if S is None:
        return done(poses.copy(), "위를 향한 면(바닥)을 두 키프레임 이상이 같이 본 곳이 없음 — 고칠 근거가 없음")
    e, gk, shared = S["e"], S["g"], S["shared"]
    sup = shared >= g["min_shared"]
    e = e - np.median(e[sup] if sup.any() else e)
    from scipy.ndimage import median_filter                    # 시간으로 매끈한 e 와 크게 다른 키프레임 = 자세가 한 장만 튐 (목록만)
    jump = sup & (np.abs(e - median_filter(e, size=9, mode="nearest")) > g["jump_m"])
    u = S["used"]
    r = S["res"][u]
    resid_cm = 100 * float(np.sqrt(np.average(r ** 2, weights=S["w0"][u]))) if u.any() else float("nan")
    gap, i = 0.0, 0                                             # 바닥을 못 본 (공유 칸이 모자란) 구간 중 가장 긴 것 — 앞뒤 근거 있는 장 사이 시간
    while i < N:
        if sup[i]:
            i += 1
            continue
        j = i
        while j + 1 < N and not sup[j + 1]:
            j += 1
        gap = max(gap, float(times[min(j + 1, N - 1)] - times[max(i - 1, 0)]))
        i = j + 1
    rng = float(np.percentile(e, 98) - np.percentile(e, 2))
    lv = _levels(S["cells"], np.c_[poses[:, :2, 3], poses[:, 2, 3] - e], p["cell"], g["level_step_m"])
    info.update(frames_supported=int(sup.sum()), frames_supported_pct=round(100 * float(sup.mean()), 1),
                cells=int(len(S["cells"])), observations=int(u.sum()), observations_dropped=int((~u).sum()),
                resid_rms_cm=round(resid_cm, 2) if np.isfinite(resid_cm) else None, resid_med_cm=round(100 * float(np.median(np.abs(r))), 2) if len(r) else None,
                dz_range_cm=round(100 * rng, 1), dz_max_abs_cm=round(100 * float(np.max(np.abs(e))), 1),
                dz_p5_p95_cm=[round(100 * float(np.percentile(e, 5)), 1), round(100 * float(np.percentile(e, 95)), 1)],
                max_unsupported_s=round(gap, 1), jump_frames=np.nonzero(jump)[0].tolist(),
                jump_times=[round(float(times[j]), 6) for j in np.nonzero(jump)[0]], levels=lv)
    if gk is not None:
        tl = np.degrees(np.arctan(np.hypot(gk[:, 0], gk[:, 1])))
        info["tilt_deg_p50_p95"] = [round(float(np.median(tl)), 2), round(float(np.percentile(tl, 95)), 2)]
    # 관문
    refuse = []
    if sup.mean() < g["min_frames_frac"]:
        refuse.append(f"바닥을 다른 키프레임과 {g['min_shared']}칸 넘게 같이 본 키프레임이 {sup.mean():.0%} 뿐 (< {g['min_frames_frac']:.0%}) — 고칠 근거가 모자람")
    if info["dz_max_abs_cm"] > 100 * g["max_abs_m"]:
        refuse.append(f"높이 보정이 {info['dz_max_abs_cm']} cm (> {100 * g['max_abs_m']:.0f} cm) — 틀어짐이 아니라 추적이 깨진 것으로 봄, 다시 촬영 권함")
    if not resid_cm <= g["max_resid_cm"]:
        refuse.append(f"고친 뒤에도 바닥 높이가 키프레임끼리 {resid_cm:.1f} cm (RMS) 어긋남 (> {g['max_resid_cm']:g} cm) — 수평 틀어짐 · 움직인 바닥 등 이 모델로 못 고침")
    noop = rng < g["noop_range_m"]
    if lv.get("stand_other_pct", 0) > g["level_flag_pct"]:
        info["flags"].append(f"키프레임 {lv['stand_other_pct']}% 가 주 바닥과 {100 * g['level_step_m']:.0f} cm 넘게 다른 층에 서 있었음 "
                             f"(위 {lv.get('stand_above_m')} m · 아래 {lv.get('stand_below_m')} m) — 단 · 계단 위를 걸은 촬영일 수 있음. "
                             "층은 칸 높이로 따로 두므로 평평하게 누르지 않음 — 결과를 눈으로 확인")
    if gap > g["gap_flag_s"]:
        info["flags"].append(f"바닥을 못 본 구간이 최대 {gap:.0f} 초 — 그 사이 보정은 앞뒤를 이은 값")
    if jump.any():
        info["flags"].append(f"자세가 앞뒤와 {100 * g['jump_m']:.0f} cm 넘게 튄 키프레임 {int(jump.sum())}장 (jump_frames) — 고치지 않고 목록만 남김")
    info["reasons"] += refuse
    if noop:
        info["reasons"].append(f"고칠 것 없음: 높이 보정 폭(2~98%) {100 * rng:.1f} cm < {100 * g['noop_range_m']:.1f} cm")
    info["status"] = "refused" if refuse else "skipped" if noop else "applied"
    info["applied"] = info["status"] == "applied"
    if info["applied"]:
        info["dz"] = np.round(e, 5).tolist()                    # 0.01 mm 로 반올림한 값을 그대로 적용 — floor_fix.json 으로 다시 적용해도 같은 자세
        info["tilt"] = np.round(gk, 6).tolist() if gk is not None else None
    else:
        info["dz"], info["tilt"] = [0.0] * N, None
        info["dz_estimate"] = np.round(e, 5).tolist()
    out = apply_fix(times, poses, info)
    info["seconds"] = dict(observe=round(t1 - t0, 2), solve=round(t2 - t1, 2), total=round(time.time() - t0, 2))
    return done(out)


def make_pose_fix(map_root, log=print, **kw):
    """맵 폴더 → (times, poses) → (poses, info) 함수 — stray_mesh.build(pose_fix=…) 에 넣는 꼴.
    부를 때 맵 폴더의 키프레임 깊이 · 내부 파라미터를 직접 읽고, 받은 시각으로 키프레임을 짝지음 (짝이 없는 장은 깊이 없이 — 시간 이음으로만)"""
    check_opts(kw)
    map_root = os.path.abspath(map_root)

    def floor_fix_(times, poses):
        from .stray_mesh import read_map                       # 그림이 있는 줄만 · 내부 파라미터 한 종류 — stray_mesh 와 같은 읽기
        fr = read_map(map_root)
        tk, times = fr["times"], np.asarray(times, dtype=float)
        if len(tk) > 1:                                         # 시각이 가장 가까운 키프레임
            j = np.clip(np.searchsorted(tk, times), 1, len(tk) - 1)
            j = np.where(np.abs(tk[j - 1] - times) <= np.abs(tk[j] - times), j - 1, j)
        else:
            j = np.zeros(len(times), dtype=int)
        hit = np.abs(tk[j] - times) < 1e-4
        paths = [fr["depth"][jj] if h else None for jj, h in zip(j, hit)]
        return floor_fix(times, poses, paths, fr["intrinsic"][2:], log=log, **kw)

    floor_fix_.__name__ = "floor_fix"
    return floor_fix_


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map", help="맵 폴더 (stray.py 출력: keyframes/ · trajectory_camera_optical.txt)")
    ap.add_argument("--json", help="info 를 이 경로에 씀 (맵 폴더의 floor_fix.json 은 건드리지 않음)")
    ap.add_argument("--tilt", action="store_true", help="바닥 기울기 오차도 풂 (기본 끔)")
    a = ap.parse_args()
    from . import stray
    from .stray_mesh import read_map
    try:
        fr = read_map(a.map)
    except stray.CaptureError as e:
        sys.exit(f"[floor_fix] 실패: {e}")
    _, info = floor_fix(fr["times"], fr["poses"], fr["depth"], fr["intrinsic"][2:], tilt=a.tilt)
    if a.json:
        json.dump(info, open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
