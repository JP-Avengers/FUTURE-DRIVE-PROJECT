"""입력 판별: 무엇이 들어왔는지(GLB · B 형식 맵 폴더 · 녹화 파일)와 장면의 형태(천장 · 바닥)를 스스로 알아내 빌드 설정을 만든다."""
import glob
import os
import re

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
RECON = os.path.dirname(HERE)
TEMPLATES = os.path.join(RECON, "configs", "templates")


def kind_of(path):
    """→ 'glb' | 'map_folder' | 'stray' | 'bag' | None"""
    from . import stray
    if stray.find_capture(path):
        return "stray"
    if os.path.isfile(path):
        ext = os.path.splitext(path)[1].lower()
        return "glb" if ext in (".glb", ".gltf") else "bag" if ext in (".mcap", ".db3", ".bag") else None
    if os.path.isdir(path):
        if os.path.isdir(os.path.join(path, "keyframes")) and glob.glob(os.path.join(path, "trajectory*camera_optical.txt")):
            return "map_folder"
        if glob.glob(os.path.join(path, "*.mcap")) or os.path.exists(os.path.join(path, "metadata.yaml")):
            return "bag"
        g = glob.glob(os.path.join(path, "*.glb"))
        if len(g) == 1:
            return "glb"
    return None


def safe_name(path):
    s = re.sub(r"[^A-Za-z0-9_]", "_", os.path.splitext(os.path.basename(os.path.normpath(path)))[0]).strip("_") or "scene"
    return ("m_" + s if s[0].isdigit() else s).lower()


def profile_glb(path, up="y"):
    """GLB(또는 형상 메쉬 ply)를 읽어 장면 형태를 잰다 (수평을 맞춘 뒤): 범위 · 바닥 기울기와 평평함 · 천장 덮음. up = 파일에서 위쪽 축. → dict"""
    import sys
    sys.path.insert(0, os.path.join(RECON, "pipeline"))
    from glb_to_usd import find_floor, load_parts, merged
    parts = load_parts(path, up)
    w = merged(parts)
    M, st = find_floor(w, {})
    w.transform(M)
    w.compute_triangle_normals()
    v, t, n = np.asarray(w.vertices), np.asarray(w.triangles), np.asarray(w.triangle_normals)
    cen = v[t].mean(1)
    area = 0.5 * np.linalg.norm(np.cross(v[t[:, 1]] - v[t[:, 0]], v[t[:, 2]] - v[t[:, 0]]), axis=1)
    c = 0.25
    lo = v[:, :2].min(0)

    def cover(sel):                                             # 위에서 본 0.25 m 칸으로 덮은 넓이
        ij = np.unique(np.floor((cen[sel, :2] - lo) / c).astype(int), axis=0)
        return len(ij) * c * c, ij
    up = n[:, 2] > 0.9
    floor = up & (np.abs(cen[:, 2]) < 0.15)
    fa, fij = cover(floor)
    ceil = (n[:, 2] < -0.8) & (cen[:, 2] > 1.8)
    ca, cij = cover(ceil)
    over = len(set(map(tuple, fij)) & set(map(tuple, cij))) * c * c   # 바닥 위에 천장이 있는 넓이
    low = up & (cen[:, 2] > -0.6) & (cen[:, 2] < 0.6)
    z = cen[low, 2]
    zw = area[low]
    o = np.argsort(z)
    cw = np.cumsum(zw[o]) / zw.sum()
    spread = float(z[o][np.searchsorted(cw, 0.95)] - z[o][np.searchsorted(cw, 0.05)])
    # 가장 넓은 바닥 층(±7 cm)이 바닥 근처(±35 cm) 위 향한 면에서 차지하는 비율 — 왜 평평하지 않은지 가리는 데 씀.
    # 경사 · 굴곡이면 낮고, 평평한 바닥의 일부가 다른 높이로 찍힌 것(촬영 중 자세가 위아래로 틀어짐 — 방 촬영본 11.24: 퍼짐 0.217 m, 비율 84%)이면 높음.
    # 어느 쪽이든 충돌 바닥은 스캔 바닥 높이를 따라 깖: 평평한 한 장으로 깔면 5~12 cm 높게 찍힌 구역이 장애물로 남아 빈 바닥이 61 → 38 m² 로 줄었음
    near = up & (np.abs(cen[:, 2]) < 0.35)
    share = float(area[up & (np.abs(cen[:, 2]) < 0.07)].sum() / max(area[near].sum(), 1e-9))
    ext = np.ptp(v, axis=0)
    return dict(parts=len(parts), textured=sum(p[2] is not None for p in parts), triangles=int(len(t)), extent_m=np.round(ext, 2).tolist(),
                floor_tilt_deg=st["floor_tilt_deg"], floor_area_m2=round(fa, 1), floor_spread_m=round(spread, 3), floor_main_share_pct=round(100 * share, 1), floor_flat=bool(spread < 0.12),
                ceiling_area_m2=round(ca, 1), ceiling_over_floor_pct=round(100 * over / max(fa, 1e-6), 1), has_ceiling=bool(over / max(fa, 1e-6) > 0.3),
                wall_top_m=round(float(np.percentile(v[:, 2], 98)), 2))


def profile_map_folder(path):
    """B 형식 맵 폴더: 주 묶음 · 보조 묶음(keyframes_<이름> ↔ trajectory_<이름>_camera_optical.txt) · 형상 메쉬를 찾는다. → dict"""
    main = os.path.join(path, "trajectory_camera_optical.txt")
    extra = []
    for d in sorted(glob.glob(os.path.join(path, "keyframes_*"))):
        nm = os.path.basename(d)[len("keyframes_"):]
        if os.path.exists(os.path.join(path, f"trajectory_{nm}_camera_optical.txt")):
            extra.append(nm)
    mesh = sorted(glob.glob(os.path.join(path, "geometry*.ply")))
    n_main = len(glob.glob(os.path.join(path, "keyframes", "rgb", "*")))
    n_pose = sum(1 for ln in open(main, encoding="utf-8") if ln.strip() and not ln.startswith("#")) if os.path.exists(main) else 0
    return dict(main_keyframes=n_main, main_poses=n_pose, extra_sets=extra, geometry_mesh=os.path.basename(mesh[0]) if mesh else None,
                has_validation=os.path.isdir(os.path.join(path, "validation")))


def config_glb(path, name, build_dir, prof, up="y"):
    """GLB 용 빌드 설정(yaml 문자열). 실내 · 평평한 바닥이 기본, 바닥이 평평하지 않으면 스캔 바닥 높이 지도."""
    L = [f"# scan2sim 이 자동으로 만든 설정 — 입력 {os.path.basename(path)}",
         f"base: {os.path.join(TEMPLATES, 'polycam_glb.yaml').replace(os.sep, '/')}", f"name: {name}", f"out: {build_dir.replace(os.sep, '/')}",
         "input:", f"  root: {os.path.dirname(os.path.abspath(path)).replace(os.sep, '/')}", f"  mesh: {os.path.basename(path)}"] + ([f"  up: {up}"] if up != "y" else []) + [
         "orient:", "  origin: center",
         # 충돌 물체 삼각형 15만 (11.26, 통합 기록 R22 — B 측정: 10만 대비 물체 표면 ↔ 충돌 거리 95% 1.39 → 0.92 cm · 최대 24 → 14 cm, USD +1 MB, 시간 같음).
         # 라이다는 보이는 형상을 보고 지도는 충돌로 만들므로 둘이 가까운 쪽이 낫다. 스캔 바닥은 줄이기 전에 빼므로 바닥이 이 예산을 먹지 않음
         "collision:", "  tris: 150000", "  floor_margin_m: 0.0"]
    if prof["floor_flat"]:
        L += ["  floor_band_m: 0.05"]
    else:                                                       # 경사 · 굴곡 있는 바닥 (실외 등): 스캔으로 바닥 높이 지도
        L += ["  ground: scan", "  ground_cell_m: 0.5", "  ground_band_m: 0.2", "  floor_band_m: 0.08", "  low_h_m: 0.10", "  min_cluster_tris_after: 100", "  floor_fill_depth_m: 0.05"]
    if prof["has_ceiling"]:
        L += ["preview:", "  args: [--ray-top, 1.0]"]
    else:
        L += ["clean:", "  float_h_m: 0.3", "  float_max_tris: 20000"]
    return "\n".join(L) + "\n"


def config_map_folder(path, name, build_dir, prof):
    """B 형식 맵 폴더용 빌드 설정(yaml 문자열) — 실내 틀 + 찾은 묶음. 벽 처리 값은 복도에서 맞춘 것(다른 실내에서는 확인 안 됨)."""
    root = os.path.abspath(path).replace(os.sep, "/")
    L = [f"# scan2sim 이 자동으로 만든 설정 — 입력 {os.path.basename(os.path.normpath(path))}",
         f"base: {os.path.join(TEMPLATES, 'indoor_rtabmap.yaml').replace(os.sep, '/')}", f"name: {name}", f"out: {build_dir.replace(os.sep, '/')}",
         "input:", f"  root: {root}", "  keyframes: keyframes", "  poses: trajectory_camera_optical.txt"]
    if prof["geometry_mesh"]:
        L += ["geometry:", f"  mesh: {prof['geometry_mesh']}"]
    if prof["extra_sets"]:
        L += ["colorize:", "  extra_inputs:"]
        for nm in prof["extra_sets"]:
            L += [f"    - name: {nm}", f"      input: {{format: rtabmap_keyframes, root: {root}, keyframes: keyframes_{nm}, poses: trajectory_{nm}_camera_optical.txt, depth_scale: 1000.0}}"]
    L += ["walls:", "  dist_m: 0.08", "  min_normal_cos: 0.5", "  sheet: true", "  min_area_m2: 3.0", "  depth_tol_m: 0.08",
          "usd:", "  double_sided: [Visual, Wall0, Wall1, Wall2, Wall3, Wall4, Wall5, Wall6, Wall7, Wall8, Wall9, Wall10, Wall11]"]
    return "\n".join(L) + "\n"
