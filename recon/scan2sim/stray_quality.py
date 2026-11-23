"""아이폰 LiDAR 원본(Stray Scanner)으로 만든 맵의 복원 정확도 — 맵을 만들 때 안 쓴 프레임의 깊이와 맵을 비교.

    cd recon
    python -m scan2sim.stray_quality <scan2sim 결과 폴더> [--n 30] [--json <경로>] [--capture <촬영 폴더 또는 zip>] [--drift]

무엇을 재나 (measure)
    stray.py 는 촬영 프레임 중 5 cm · 5° 넘게 움직인 장만 키프레임으로 골라 맵(TSDF)을 만든다. 고르지 않은 프레임은 맵에 안 들어갔으므로
    그 프레임의 자세에서 맵을 렌더한 깊이와 그 프레임이 실제로 잰 LiDAR 깊이(256×192 원본)를 비교하면 맵이 얼마나 맞는지 알 수 있다.
    - 안 쓴 프레임 n 장: 처음 skip_s(1 초)와 너무 빠른 장은 빼고, 시간 순으로 n 구간에 나눠 구간마다 한 장 (seed 로 고정 — 실행마다 같음)
    - 깊이: 신뢰도 min_conf(1) 이상 · 맵을 만들 때 쓴 깊이 한계(build/tsdf_meta.json 의 depth_trunc, 없으면 3 m) 이하
    - 내부 파라미터: odometry.csv 의 프레임별 값을 색 영상 크기 → 깊이 크기로 줄임 (화소 중심 기준)
    - 자세: odometry.csv (카메라 광학 → 세계, y 위) 에 stray.py 와 같은 Y_UP_TO_Z_UP 을 곱함 = 맵 폴더 세계 = mesh_raw.ply 좌표.
      visual.ply(최종 보이는 메쉬)는 씬 좌표라 build/transform.json 의 T_input_to_scene 을 한 번 더 곱해 따로 잼
    - 결과: 차이(잰 깊이 − 맵 깊이, 광축 방향 z) 절댓값의 중앙 · 90% · 2 cm 안 · 5 cm 안, 잰 깊이 중 맵에 닿은 비율, 프레임별 중앙값
    - 판정: 중앙 3 cm 이하 통과 · 3 cm 넘으면 주의 · 10 cm 넘으면 실패(자세 · 좌표가 어긋난 것). 맵에 닿은 비율 70% 미만도 주의
    - <결과 폴더>/input_map/stray_quality.json 에 씀

한계
    - 안 쓴 프레임은 키프레임 바로 옆(1/60 초)이라 자세가 독립이 아님 — 자세 흔들림 · 깊이 잡음 · 복셀(2 cm) 크기를 재는 것이고,
      긴 구간에 걸쳐 쌓이는 틀어짐(드리프트)은 거의 안 보임. 그건 drift() 가 따로 (되돌아온 촬영만)
    - 움직이는 물체(사람)는 차이로 잡힘. 유리 · 거울은 LiDAR 깊이 자체가 틀릴 수 있음

drift (기본으로 돌지 않음 — --drift 또는 drift() 를 따로 부를 때만)
    촬영이 출발한 자리 근처로 돌아왔을 때(처음과 끝 위치 1 m 안 · 바라보는 방향 30° 안)만 잼. 처음 구간(최대 30 초, 촬영 길이의 1/3 이하)의
    원본 깊이로만 TSDF 를 따로 만들고, 마지막 구간 프레임의 깊이를 (1) 그 메쉬와 비교하고 (2) 그 점들을 ICP 로 메쉬에 맞춰 옮겨야 하는 양을 잼.
    - 옮긴 양 = 처음과 끝 사이에 쌓인 틀어짐의 추정. 두 구간이 같은 곳을 충분히 겹쳐 봤을 때만 뜻이 있음 (ICP 맞은 비율 fitness 로 확인)
    - 중간 구간의 틀어짐 분포 · 되돌아오지 않는 촬영의 틀어짐은 모름
"""
import argparse
import json
import os
import time

import numpy as np

from . import stray
from .stray import _rot

PASS_CM, FAIL_CM, MIN_HIT_PCT = 3.0, 10.0, 70.0                  # 판정 기준 (중앙 차이 cm · 맵에 닿은 비율 %)
Y_UP_TO_Z_UP = stray.Y_UP_TO_Z_UP                               # 아이폰 세계(y 위) → 맵 폴더 세계(z 위). stray.py 와 같은 값


# ── 읽기 ───────────────────────────────────────────────────────────────

def _capture(out_dir, capture=None):
    """촬영 폴더 찾기: input_map/_unzip (zip 으로 받은 경우) → capture 인자 (폴더 또는 zip)."""
    work = os.path.join(out_dir, "input_map", "_unzip")
    if os.path.isdir(work):
        src = stray.find_capture(work)
        if src:
            return src
    if capture:
        src = stray.find_capture(os.path.abspath(os.path.expanduser(capture)), work)
        if src:
            return src
    raise SystemExit("촬영 원본을 찾지 못함 — input_map/_unzip 이 없으면 --capture <촬영 폴더 또는 zip> 을 줄 것")


def _load(src, meta):
    """odometry.csv → 시각 · 맵 폴더 세계(Z 위) 자세 · 프레임별 내부 파라미터(색 영상 크기) · 속력."""
    od = np.genfromtxt(os.path.join(src, "odometry.csv"), delimiter=",", skip_header=1, usecols=range(13))
    t, pos = od[:, 0], od[:, 2:5]
    T = np.repeat(np.eye(4)[None], len(od), axis=0)
    T[:, :3, :3] = [_rot(q) for q in od[:, 5:9]]
    T[:, :3, 3] = pos
    T = Y_UP_TO_Z_UP @ T                                        # stray.py 와 같은 세계 (모듈 값을 부를 때 읽음 — 점검에서 바꿔 볼 수 있게)
    step = np.r_[0, np.linalg.norm(np.diff(pos, axis=0), axis=1)]
    dt = np.r_[1, np.maximum(np.diff(t), 1e-3)]
    speed = np.convolve(step / dt, np.ones(5) / 5, mode="same")   # stray.py 와 같은 5장 평균 속력
    return dict(t=t, T=T, K=od[:, 9:13], speed=speed, color_w=(meta.get("color_size") or [1920, 1440])[0])


def _keyframe_rows(mp, t):
    """trajectory_camera_optical.txt 의 시각 → odometry.csv 의 줄 번호 (키프레임으로 쓴 장)."""
    tk = np.atleast_1d(np.loadtxt(os.path.join(mp, "trajectory_camera_optical.txt"), comments="#", usecols=0))
    j = np.clip(np.searchsorted(t, tk), 1, len(t) - 1)
    j = np.where(np.abs(t[j - 1] - tk) <= np.abs(t[j] - tk), j - 1, j)
    return set(j[np.abs(t[j] - tk) < 1e-3].tolist())


def _depth_trunc(bd):
    """맵을 만들 때 쓴 깊이 한계 (tsdf.py 가 남긴 설정). 없으면 3 m"""
    p = os.path.join(bd, "tsdf_meta.json")
    return float(json.load(open(p, encoding="utf-8"))["tsdf"]["depth_trunc"]) if os.path.exists(p) else 3.0


def _depth(src, i, trunc, min_conf):
    """원본 깊이(m) · 쓸 수 있는 화소. 없으면 None."""
    import cv2
    z = cv2.imread(os.path.join(src, "depth", f"{i:06d}.png"), cv2.IMREAD_UNCHANGED)
    if z is None:
        return None, None
    z = z.astype(np.float32) / 1000.0
    ok = (z > 0) & (z <= trunc)
    c = cv2.imread(os.path.join(src, "confidence", f"{i:06d}.png"), cv2.IMREAD_UNCHANGED)
    if c is not None:
        ok &= c >= min_conf
    return z, ok


def _intr(cap, i, w):
    """프레임 i 의 내부 파라미터를 색 영상 크기 → 깊이 크기 w 로. 화소 중심 기준: c' = (c + 0.5)·s − 0.5"""
    s = w / cap["color_w"]
    fx, fy, cx, cy = cap["K"][i]
    return fx * s, fy * s, (cx + 0.5) * s - 0.5, (cy + 0.5) * s - 0.5


def _rays(cap, i, z, ok):
    """쓸 수 있는 화소마다 광선 (원점 · 단위 방향, 세계) 과 z/거리 비, 잰 깊이."""
    v, u = np.nonzero(ok)
    fx, fy, cx, cy = _intr(cap, i, z.shape[1])
    d = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones(len(u))], 1)
    nr = np.linalg.norm(d, axis=1)
    T = cap["T"][i]
    return np.c_[np.broadcast_to(T[:3, 3], d.shape), (d / nr[:, None]) @ T[:3, :3].T], nr, z[v, u]


def _scene(mesh):
    import open3d as o3d
    sc = o3d.t.geometry.RaycastingScene()
    sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return sc


def _cast(sc, rays, nr):
    """광선 → 맵 깊이 (광축 방향 z). 안 닿으면 inf."""
    import open3d as o3d
    th = sc.cast_rays(o3d.core.Tensor(rays.astype(np.float32)))["t_hit"].numpy()
    return th / nr


def _stats(zm, zr):
    hit = np.isfinite(zr)
    d = (zm - zr)[hit]
    a = np.abs(d)
    if not len(a):
        return dict(rays=int(len(zm)), hit_pct=0.0)
    return dict(rays=int(len(zm)), hit_pct=round(100 * float(hit.mean()), 1),
                median_cm=round(100 * float(np.median(a)), 2), p90_cm=round(100 * float(np.percentile(a, 90)), 2),
                within_2cm_pct=round(100 * float((a < 0.02).mean()), 1), within_5cm_pct=round(100 * float((a < 0.05).mean()), 1),
                signed_median_cm=round(100 * float(np.median(d)), 2))   # + = 잰 깊이가 맵보다 멂


# ── 복원 정확도 ─────────────────────────────────────────────────────────

def verdict(st):
    """→ (pass|warn|fail, 이유 목록)"""
    if "median_cm" not in st:
        return "fail", ["안 쓴 프레임의 깊이가 맵에 하나도 닿지 않음 — 자세 · 좌표가 어긋났거나 맵이 비었음"]
    why, v = [], "pass"
    if st["median_cm"] > FAIL_CM:
        v = "fail"
        why.append(f"깊이 차이 중앙 {st['median_cm']} cm — {FAIL_CM:g} cm 를 넘음 (자세 · 좌표가 어긋난 것)")
    elif st["median_cm"] > PASS_CM:
        v = "warn"
        why.append(f"깊이 차이 중앙 {st['median_cm']} cm — {PASS_CM:g} cm 를 넘음 (자세가 흔들렸거나 움직인 물체가 많음)")
    if st["hit_pct"] < MIN_HIT_PCT:
        v = "fail" if v == "fail" else "warn"
        why.append(f"잰 깊이 중 맵에 닿은 것 {st['hit_pct']}% — {MIN_HIT_PCT:g}% 미만 (맵에 빈 곳이 많음)")
    return v, why


def measure(out_dir, n=30, seed=0, capture=None, min_conf=1, write=True):
    """scan2sim 결과 폴더(input_map/ · build/ 가 있는 곳) → 복원 정확도 dict. write 면 input_map/stray_quality.json 에도 씀."""
    import open3d as o3d
    t0 = time.time()
    out_dir = os.path.abspath(os.path.expanduser(out_dir))
    mp, bd = os.path.join(out_dir, "input_map"), os.path.join(out_dir, "build")
    meta = json.load(open(os.path.join(mp, "stray_meta.json"), encoding="utf-8"))
    sel = meta.get("select") or {}
    src = _capture(out_dir, capture)
    cap = _load(src, meta)
    t, N = cap["t"], len(cap["t"])
    kf = _keyframe_rows(mp, t)
    trunc = _depth_trunc(bd)
    start = int(np.searchsorted(t, t[0] + sel.get("skip_s", 1.0)))
    cand = np.array([i for i in range(start, N) if i not in kf and cap["speed"][i] <= sel.get("max_speed_mps", 1.5)
                     and os.path.exists(os.path.join(src, "depth", f"{i:06d}.png"))])
    if len(cand) == 0:
        raise SystemExit("비교할 안 쓴 프레임이 없음")
    rng = np.random.default_rng(seed)
    picks = sorted(int(rng.choice(b)) for b in np.array_split(cand, min(n, len(cand))) if len(b))

    raw_p, vis_p = os.path.join(bd, "mesh_raw.ply"), os.path.join(bd, "visual.ply")
    meshes = []                                                 # (이름, 파일, 입력 → 메쉬 좌표)
    if os.path.exists(raw_p):
        meshes.append(("mesh_raw", raw_p, np.eye(4)))
    if os.path.exists(vis_p):
        tp = os.path.join(bd, "transform.json")
        Ts = np.array(json.load(open(tp, encoding="utf-8"))["T_input_to_scene"]) if os.path.exists(tp) else np.eye(4)
        meshes.append(("visual", vis_p, Ts))
    if not meshes:
        raise SystemExit(f"맵 메쉬가 없음 ({raw_p} · {vis_p})")

    R, NR, ZM, FR = [], [], [], []
    for i in picks:
        z, ok = _depth(src, i, trunc, min_conf)
        if z is None or not ok.any():
            continue
        r, nr, zm = _rays(cap, i, z, ok)
        R.append(r), NR.append(nr), ZM.append(zm), FR.append((i, len(zm), float(ok.mean())))
    R, NR, ZM = np.concatenate(R), np.concatenate(NR), np.concatenate(ZM)
    cut = np.cumsum([f[1] for f in FR])[:-1]

    res = dict(frames=len(FR), frames_candidates=int(len(cand)), keyframes=len(kf), frames_total=N, depth_trunc_m=trunc, min_conf=min_conf,
               seed=seed, measured_valid_pct=round(100 * float(np.mean([f[2] for f in FR])), 1), meshes={})
    for name, path, Tm in meshes:
        m = o3d.io.read_triangle_mesh(path)
        if not len(m.triangles):
            continue
        Rm = Tm[:3, :3]
        rays = np.c_[R[:, :3] @ Rm.T + Tm[:3, 3], R[:, 3:] @ Rm.T]   # 입력 → 메쉬 좌표 (강체)
        zr = _cast(_scene(m), rays, NR)
        st = _stats(ZM, zr)
        st["file"] = os.path.relpath(path, out_dir)
        st["per_frame"] = [dict(frame=i, t_s=round(float(t[i] - t[0]), 2), rays=k, **{x: s.get(x) for x in ("hit_pct", "median_cm", "signed_median_cm")})
                           for (i, k, _), zm_, zr_ in zip(FR, np.split(ZM, cut), np.split(zr, cut)) for s in [_stats(zm_, zr_)]]
        res["meshes"][name] = st
    main_ = res["meshes"].get("mesh_raw") or res["meshes"].get("visual") or {}
    res["mesh"] = "mesh_raw" if "mesh_raw" in res["meshes"] else "visual"
    res.update({k: main_.get(k) for k in ("hit_pct", "median_cm", "p90_cm", "within_2cm_pct", "within_5cm_pct", "signed_median_cm")})
    res["verdict"], res["reasons"] = verdict(main_)
    res["summary"] = (f"복원 정확도: 안 쓴 프레임 {len(FR)}장의 깊이와 맵 차이 중앙 {res['median_cm']} cm · 2 cm 안 {res['within_2cm_pct']}% · "
                      f"맵에 닿음 {res['hit_pct']}%" if res.get("median_cm") is not None else "복원 정확도: 안 쓴 프레임의 깊이가 맵에 닿지 않음")
    res["thresholds"] = dict(pass_median_cm=PASS_CM, fail_median_cm=FAIL_CM, min_hit_pct=MIN_HIT_PCT)
    res["seconds"] = round(time.time() - t0, 1)
    if write:
        json.dump(res, open(os.path.join(mp, "stray_quality.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    return res


# ── 되돌아온 촬영의 틀어짐 (따로 부를 때만) ─────────────────────────────

def drift(out_dir, capture=None, window_s=30.0, max_gap_m=1.0, max_turn_deg=30.0, voxel=0.02, min_conf=1, write=True):
    """처음 구간의 깊이로 만든 TSDF 에 마지막 구간의 깊이를 맞춰 쌓인 틀어짐을 잼. 출발 자리로 돌아온 촬영만. → dict (applicable 이 False 면 reason)"""
    import open3d as o3d
    t0 = time.time()
    out_dir = os.path.abspath(os.path.expanduser(out_dir))
    mp, bd = os.path.join(out_dir, "input_map"), os.path.join(out_dir, "build")
    meta = json.load(open(os.path.join(mp, "stray_meta.json"), encoding="utf-8"))
    sel = meta.get("select") or {}
    src = _capture(out_dir, capture)
    cap = _load(src, meta)
    t, T, N = cap["t"], cap["T"], len(cap["t"])
    trunc = _depth_trunc(bd)
    a = int(np.searchsorted(t, t[0] + sel.get("skip_s", 1.0)))
    b = N - 1
    gap = float(np.linalg.norm(T[b, :3, 3] - T[a, :3, 3]))
    f0, f1 = T[a, :2, 2], T[b, :2, 2]                           # 바라보는 방향(광축)의 수평 성분
    turn = float(np.degrees(np.arccos(np.clip(f0 @ f1 / max(np.linalg.norm(f0) * np.linalg.norm(f1), 1e-9), -1, 1))))
    res = dict(start_end_gap_m=round(gap, 2), start_end_turn_deg=round(turn, 1))
    if gap > max_gap_m or turn > max_turn_deg:
        res.update(applicable=False, reason=f"출발 자리로 돌아오지 않은 촬영 (처음과 끝 거리 {gap:.2f} m · 방향 차 {turn:.0f}° — 기준 {max_gap_m:g} m · {max_turn_deg:g}° 안)")
    else:
        W = min(window_s, (t[b] - t[a]) / 3)
        ok_speed = cap["speed"] <= sel.get("max_speed_mps", 1.5)
        first = [i for i in range(a, N) if t[i] <= t[a] + W and ok_speed[i]]
        last = [i for i in range(a, N) if t[i] >= t[b] - W and ok_speed[i]]
        first = first[::max(1, len(first) // 300)]               # 최대 약 300장
        vol = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel, sdf_trunc=4 * voxel, color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)
        used = 0
        for i in first:
            z, ok = _depth(src, i, trunc, min_conf)
            if z is None:
                continue
            h, w = z.shape
            fx, fy, cx, cy = _intr(cap, i, w)
            zz = np.where(ok, z * 1000, 0).astype(np.uint16)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.zeros((h, w, 3), np.uint8)), o3d.geometry.Image(zz),
                                                                      depth_scale=1000.0, depth_trunc=trunc, convert_rgb_to_intensity=False)
            vol.integrate(rgbd, o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy), np.linalg.inv(T[i]))
            used += 1
        mesh = vol.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        lp = last[::max(1, len(last) // 30)]
        R_, NR, ZM, P = [], [], [], []
        for i in lp:
            z, ok = _depth(src, i, trunc, min_conf)
            if z is None or not ok.any():
                continue
            r, nr, zm = _rays(cap, i, z, ok)
            R_.append(r), NR.append(nr), ZM.append(zm), P.append(r[:, :3] + r[:, 3:] * (zm * nr)[:, None])
        if not len(mesh.triangles) or not R_:
            res.update(applicable=False, reason="처음 구간 메쉬 또는 마지막 구간 깊이가 비었음")
        else:
            R_, NR, ZM, P = np.concatenate(R_), np.concatenate(NR), np.concatenate(ZM), np.concatenate(P)
            st = _stats(ZM, _cast(_scene(mesh), R_, NR))
            tgt = o3d.geometry.PointCloud(mesh.vertices)
            tgt.normals = mesh.vertex_normals
            srcp = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P)).voxel_down_sample(voxel)
            reg = o3d.pipelines.registration.registration_icp(srcp, tgt, 0.15, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                                              o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
            M = reg.transformation
            off = float(np.linalg.norm(M[:3, 3]))
            rot = float(np.degrees(np.arccos(np.clip((np.trace(M[:3, :3]) - 1) / 2, -1, 1))))
            res.update(applicable=True, window_s=round(float(W), 1), frames_first=used, frames_last=len(lp), depth_vs_first=st,
                       icp=dict(offset_cm=round(100 * off, 1), offset_xyz_cm=np.round(100 * M[:3, 3], 1).tolist(), rotation_deg=round(rot, 2),
                                fitness=round(float(reg.fitness), 3), rmse_cm=round(100 * float(reg.inlier_rmse), 2)))
            reliable = reg.fitness >= 0.3
            res["summary"] = (f"되돌아온 자리 틀어짐: 처음 {W:.0f} 초로 만든 맵에 마지막 {W:.0f} 초 깊이를 맞추면 {100 * off:.1f} cm · {rot:.1f}° 옮겨짐 "
                              f"(깊이 차이 중앙 {st.get('median_cm')} cm" + ("" if reliable else f", 겹친 비율 {reg.fitness:.0%} 로 낮아 믿기 어려움") + ")")
    res["seconds"] = round(time.time() - t0, 1)
    if write:
        p = os.path.join(mp, "stray_quality.json")
        q = json.load(open(p, encoding="utf-8")) if os.path.exists(p) else {}
        q["drift"] = res
        json.dump(q, open(p, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", help="scan2sim 결과 폴더 (input_map/ · build/ 가 있는 곳)")
    ap.add_argument("--n", type=int, default=30, help="비교할 안 쓴 프레임 수")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--min-conf", type=int, default=1, choices=[0, 1, 2])
    ap.add_argument("--capture", help="촬영 원본 (input_map/_unzip 이 없을 때 — 폴더로 받은 경우)")
    ap.add_argument("--json", help="결과를 이 경로에도 씀")
    ap.add_argument("--drift", action="store_true", help="되돌아온 촬영의 틀어짐도 잼 (출발 자리로 돌아온 촬영만)")
    a = ap.parse_args()
    r = measure(a.out, a.n, a.seed, a.capture, a.min_conf)
    print(f"[품질] {r['summary']} | 90% {r['p90_cm']} cm · 5 cm 안 {r['within_5cm_pct']}% · 치우침 {r['signed_median_cm']} cm | 판정 {r['verdict']} | {r['seconds']} s")
    for k, s in r["meshes"].items():
        if k != r["mesh"]:
            print(f"       {k}: 중앙 {s.get('median_cm')} cm · 2 cm 안 {s.get('within_2cm_pct')}% · 맵에 닿음 {s.get('hit_pct')}%")
    for m in r["reasons"]:
        print("       - " + m)
    if a.drift:
        d = drift(a.out, a.capture)
        r["drift"] = d
        print("[품질] " + (d["summary"] if d.get("applicable") else "틀어짐: 재지 않음 — " + d["reason"]) + f" | {d['seconds']} s")
    if a.json:
        json.dump(r, open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
