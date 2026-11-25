"""단계들: 입력 → 변환 → 지도 → 진단 → 묶음 → (주행 검증) → 보고서. 각 함수는 Context 를 받아 Result 를 돌려준다."""
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import time

import numpy as np

from . import detect
from .core import FAIL, PASS, SKIP, WARN, MARK, Result

RECON = detect.RECON
REPO = os.path.dirname(RECON)
PIPE = os.path.join(RECON, "pipeline")
TOOLS = os.path.join(RECON, "tools")
ROBOT_RADIUS_M = 0.3                                            # 진단 · 시작 자세에서 장애물과 띄우는 거리


def _call(cmd, log):
    with open(log, "a", encoding="utf-8") as fh:
        fh.write(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} {' '.join(cmd)}\n")
        fh.flush()
        return subprocess.run(cmd, stdout=fh, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1")).returncode


def s_input(ctx):
    """무엇이 들어왔는지 판별하고 장면 형태를 재서 빌드 설정을 만든다."""
    from . import stray
    ctx["name"] = detect.safe_name(ctx["input"])
    try:
        k = detect.kind_of(ctx["input"])
    except stray.CaptureError as e:                             # 망가진 촬영본 — B 의 입력 검사가 쓴 문장만 보임 (통합 기록 R4)
        ctx["kind"] = None
        return Result(FAIL, str(e))
    ctx["kind"] = k
    if k is None:
        return Result(FAIL, "입력을 알아볼 수 없음", messages=["받는 입력: GLB 파일 · B 형식 맵 폴더(keyframes/ + trajectory_camera_optical.txt) · 아이폰 LiDAR 원본(Stray Scanner 폴더 또는 zip)"])
    if k == "bag":
        return Result(FAIL, "녹화 파일(bag) 입력은 아직 연결되지 않음", messages=["녹화 파일 → 맵 폴더는 B 단계의 자리 (한 명령 스크립트가 나오면 여기에 붙임). 지금은 맵 폴더나 GLB 를 넣을 것"])
    build = ctx.path("build", ".keep")[:-len(".keep")].rstrip("/\\")
    sm, path, up = None, None, "y"
    if k == "stray":
        # 아이폰 원본 → 맵 폴더 형식(키프레임 + 자세) → 깊이를 합친 형상 메쉬(build/mesh_raw.ply) → 그 뒤는 GLB 와 같은 길.
        # 맵 폴더 경로(build_usd.py)로 보내지 않는 이유: 그 경로는 바닥을 새로 깔아서 스캔에 있던 바닥 결함(겹침 · 구멍 · 낮은 턱)이
        # 진단에 남지 않음 (B 통합 기록 R8, 가상의 방에서 3개 중 0개). 스캔 형상을 그대로 두는 GLB 경로는 3개 모두 잡음.
        # 메쉬 좌표는 맵 폴더 세계(Z 위) 그대로 — T_input_to_scene 의 입력 좌표가 전과 같게 (docs/CONTRACT.md 1장)
        mp = ctx.path("input_map", ".keep")[:-len(".keep")].rstrip("/\\")
        mj = os.path.join(mp, "stray_meta.json")
        fresh = not (ctx.get("resume") and os.path.exists(mj))  # 이번 실행에서 맵 폴더를 새로 만들었는가
        if not fresh:
            sm = json.load(open(mj, encoding="utf-8"))
        else:
            try:
                ctx["capture"] = stray.find_capture(ctx["input"], os.path.join(mp, "_unzip"))
                sm = stray.convert(ctx["capture"], mp)
            except stray.CaptureError as e:
                return Result(FAIL, str(e))
        path = os.path.join(build, "mesh_raw.ply")
        # --resume 이어도 맵 폴더를 새로 만들었거나 메쉬가 맵 폴더보다 오래됐으면 다시 만듦 — 예전 촬영의 메쉬를 그대로 쓰지 않게 (B 11.24)
        stale = fresh or not os.path.exists(path) or os.path.getmtime(path) < os.path.getmtime(mj)
        if stale:
            from . import stray_mesh                            # B 의 형상 메쉬 (통합 기록 R10): 깊이 규칙 · 자세 보정이 이 안에 있음. 파일 이름 · 좌표는 pipeline/tsdf.py 때와 같음
            try:
                with open(ctx.path("log.txt"), "a", encoding="utf-8") as fh:
                    # capture = 높이 보정 자동 고르기(fix "auto")가 원래 자세를 잴 촬영 원본 (통합 기록 R20). 폴더 입력은 input_map/_unzip 이 없어 넘겨야 잼
                    stray_mesh.build(mp, build, log=lambda s: print(s, file=fh, flush=True), capture=ctx.get("capture") or ctx["input"])
            except stray.CaptureError as e:                     # stray_mesh.MeshError 도 여기로 — 쓸 장 없음 · 해상도 불일치 · 메쉬 빔
                return Result(FAIL, "촬영본에서 형상을 만들지 못함", metrics=dict(stray=sm), messages=[str(e)])
            ctx["mesh_rebuilt"] = True                          # --resume 이어도 변환(USD · visual.ply · transform.json)을 다시 — 예전 메쉬 것이 남지 않게 (통합 기록 R15)
        ctx["source"], ctx["map_root"], ctx["kind"], k, up = "stray", mp, "glb", "glb", "z"
    if k == "glb":
        if path is None:
            path = ctx["input"] if os.path.isfile(ctx["input"]) else [os.path.join(ctx["input"], f) for f in os.listdir(ctx["input"]) if f.lower().endswith(".glb")][0]
        p = detect.profile_glb(path, up)
        cfg = detect.config_glb(path, ctx["name"], build, p, up)
        r = Result(PASS, ("아이폰 원본 → 형상 메쉬 · " if sm else "GLB · ") + f"{'천장 있음' if p['has_ceiling'] else '천장 없음'} · 바닥 {'평평' if p['floor_flat'] else '평평하지 않음'} · 범위 {p['extent_m'][0]:.0f}×{p['extent_m'][1]:.0f} m", metrics=p)
        if max(p["extent_m"]) > 300 or max(p["extent_m"]) < 1:
            r.fail(f"범위가 이상함 ({max(p['extent_m']):.1f} m) — 단위(m)를 확인")
        if p["floor_area_m2"] < 2:
            r.fail("바닥을 찾지 못함 — 위쪽 축 또는 스캔 범위 확인")
        if not p["floor_flat"] and p["floor_main_share_pct"] >= 75:   # 대부분은 한 높이인데 일부가 다른 높이 — 경사가 아니라 스캔이 어긋난 것 (75 는 자료 세 건으로 잡은 잠정값)
            r.warn(f"바닥의 {100 - p['floor_main_share_pct']:.0f}% 가 다른 높이에 찍힘 (높이 퍼짐 {p['floor_spread_m']} m) — 충돌 바닥은 스캔 바닥 높이를 따라 깖. 어디가 어긋났는지는 진단 단계에서 봄")
        elif not p["floor_flat"]:
            r.warn(f"바닥이 평평하지 않음 (높이 퍼짐 {p['floor_spread_m']} m) — 실외 · 경사로로 보고 스캔 바닥 높이 지도를 씀. 넓은 실외는 스캔 틀어짐 때문에 대상 밖")
        if p["textured"] == 0 and not sm:                       # 아이폰 원본은 원래 정점 색 — 결함이 아니므로 알리지 않음
            r.warn("사진 텍스처가 없음 — 정점 색으로만 보임")
        if sm:
            p["stray"] = sm
            if sm["path_length_m"] < 5 or max(sm["extent_m"][:2]) < 3:
                r.warn(f"촬영 범위가 좁음 (이동 {sm['path_length_m']} m · 카메라가 움직인 범위 {max(sm['extent_m'][:2]):.1f} m) — 로봇이 다닐 맵으로는 부족할 수 있음")
            if sm["valid_depth_pct"] < 50:
                r.warn(f"쓸 수 있는 깊이가 적음 ({sm['valid_depth_pct']}%) — 멀리 있거나 반사 · 유리가 많은 장면")
            if sm["speed_median_mps"] > 0.8:
                r.warn(f"촬영 속도가 빠름 (중앙값 {sm['speed_median_mps']} m/s) — 사진이 흐리고 깊이가 성김")
            for w in (sm.get("checks") or {}).get("warnings") or []:
                r.warn(w)
            # 자세 보정 요약 (B 의 stray_mesh 가 build/tsdf_meta.json 의 pose_fix 에 남김, 통합 기록 R11): 고쳤으면 알리기만, 못 고쳤거나 표시가 있으면 주의
            tm = os.path.join(build, "tsdf_meta.json")
            pf = (json.load(open(tm, encoding="utf-8")).get("pose_fix") if os.path.exists(tm) else None) or {}
            p["pose_fix"] = {k_: pf.get(k_) for k_ in ("method", "applied", "status", "reasons", "flags", "auto") if k_ in pf}   # auto = 높이 보정을 자동으로 고른 근거 (재지 못해 xy 로 간 까닭 포함, R20)
            if pf.get("applied"):
                r.messages.append("촬영 중 틀어진 자세를 고쳐서 합침" + (f" ({' · '.join(pf['reasons'])})" if pf.get("reasons") else ""))
            elif pf:
                r.warn("촬영 중 틀어진 자세를 고치지 않음 — 긴 구간은 틀어진 채일 수 있음" + (f" ({' · '.join(pf.get('reasons') or [])})" if pf.get("reasons") else ""))
            else:
                r.warn("아이폰 자세에는 되돌아온 자리를 맞추는 보정이 없음 — 긴 구간은 틀어질 수 있음 (얼마나인지는 재지 않음)")
            for w in pf.get("flags") or []:
                r.warn(w)
    else:
        root = ctx.get("map_root") or ctx["input"]
        p = detect.profile_map_folder(root)
        cfg = detect.config_map_folder(root, ctx["name"], build, p)
        r = Result(PASS, f"맵 폴더 · 주 묶음 {p['main_keyframes']}장 · 보조 묶음 {len(p['extra_sets'])}개 · 형상 메쉬 {'있음' if p['geometry_mesh'] else '없음'}", metrics=p)
        if p["main_keyframes"] == 0 or p["main_poses"] == 0:
            r.fail("주 묶음의 키프레임 또는 자세가 없음")
        elif p["main_keyframes"] < 0.95 * p["main_poses"] or p["main_poses"] < 0.95 * p["main_keyframes"]:
            r.warn(f"키프레임 {p['main_keyframes']}장과 자세 {p['main_poses']}줄의 수가 다름 — 범위가 빠졌을 수 있음")
    ctx["profile"] = p
    ctx["config"] = ctx.path("build", "auto_config.yaml")
    open(ctx["config"], "w", encoding="utf-8", newline="\n").write(cfg)
    return r


def s_convert(ctx):
    """기존 파이프라인으로 메쉬 · 충돌 · USD 를 만든다 (GLB: glb_to_usd.py · 맵 폴더: check_input.py + build_usd.py)."""
    py, log, out = sys.executable, ctx.path("log.txt"), os.path.dirname(ctx["config"])
    if ctx.get("resume") and not ctx.get("mesh_rebuilt") and os.path.exists(os.path.join(out, "report.json")):
        rc = 0                                                  # --resume: 이미 만든 빌드를 그대로 씀 (진단 · 묶음만 다시)
    elif ctx["kind"] == "map_folder":
        if _call([py, os.path.join(PIPE, "check_input.py"), "--config", ctx["config"]], log):
            ci = json.load(open(os.path.join(out, "check_input.json"), encoding="utf-8")) if os.path.exists(os.path.join(out, "check_input.json")) else {}
            return Result(FAIL, "입력 점검에서 오류", metrics=dict(check_input=ci.get("summary", ci)), messages=["자세한 내용: build/check_input.json · log.txt"])
        rc = _call([py, os.path.join(PIPE, "run.py"), "--config", ctx["config"], "--from", "tsdf", "--no-preview"], log)
    else:
        rc = _call([py, os.path.join(PIPE, "glb_to_usd.py"), "--config", ctx["config"]], log)
    rp = os.path.join(out, "report.json")
    if rc or not os.path.exists(rp):
        return Result(FAIL, f"빌드 실패 (종료 코드 {rc})", messages=["자세한 내용: log.txt"])
    rep = json.load(open(rp, encoding="utf-8"))
    u = rep["usd"]
    ctx["usd_scene"], ctx["usd_mesh"], ctx["build"] = u["scene"], u["mesh"], out
    ctx["prims"] = {k.split("/")[-1]: v for k, v in u["prims"].items()}
    r = Result(PASS, f"USD {u['mesh_mb']} MB · prim {len(ctx['prims'])}개", metrics=dict(mesh_mb=u["mesh_mb"], prims=ctx["prims"], collision=rep.get("collision") or rep.get("collider"), clean=rep.get("clean")))
    for p in u.get("problems") or []:
        r.fail("USD 규격: " + p)
    return r


def s_nav2(ctx):
    """충돌 메쉬로 Nav2 지도를 만든다 (D 의 sim/nav2/make_map.py 그대로)."""
    out = ctx.path("bundle", "nav2", ".keep")[:-len(".keep")]
    mm = os.path.join(REPO, "sim", "nav2", "make_map.py")
    if not os.path.exists(mm):
        # 지도가 없으면 진단 · 보정 · 묶음이 모두 못 돎(빈 바닥을 지도에서 찾음) — 건너뛰지 않고 여기서 멈춤 (통합 기록 R12: 전에는 다음 단계가 KeyError 로 죽었음)
        return Result(FAIL, "Nav2 지도를 만드는 도구가 없음", messages=["저장소의 sim/nav2/make_map.py 가 있어야 함 — recon/ 만 복사했다면 sim/ 도 함께 둘 것"])
    if _call([sys.executable, mm, ctx["usd_mesh"], out, "--route", ""], ctx.path("log.txt")) or not os.path.exists(os.path.join(out, "map.pgm")):
        return Result(FAIL, "Nav2 지도 생성 실패", messages=["자세한 내용: log.txt"])
    import yaml
    from PIL import Image
    y = yaml.safe_load(open(os.path.join(out, "map.yaml"), encoding="utf-8"))
    im = np.asarray(Image.open(os.path.join(out, "map.pgm")))
    free, occ = im >= 250, im < 50
    ctx["nav2"] = dict(dir=out, origin=y["origin"][:2], cell=y["resolution"], shape=im.shape)
    ctx["nav2_free"], ctx["nav2_occ"] = free, occ
    r = Result(PASS, f"{im.shape[1]}×{im.shape[0]} 칸 ({y['resolution']} m) · 빈 바닥 {free.sum() * y['resolution'] ** 2:.0f} m²",
               metrics=dict(cells=list(im.shape[::-1]), resolution=y["resolution"], free_m2=round(float(free.sum()) * y["resolution"] ** 2, 1), occupied_cells=int(occ.sum())))
    if free.sum() * y["resolution"] ** 2 < 4:
        r.fail("로봇이 다닐 빈 바닥이 4 m² 보다 작음")
    return r


def _free_xy(ctx, clearance, step=1):
    """Nav2 지도에서 장애물 · 미지 칸과 clearance 넘게 떨어진 빈 칸의 중심 (세계 좌표)과 그 거리."""
    from scipy import ndimage
    free, n = ctx["nav2_free"], ctx["nav2"]
    dist = ndimage.distance_transform_edt(free) * n["cell"]
    iy, ix = np.nonzero(dist >= clearance)
    iy, ix = iy[::step], ix[::step]
    xy = np.c_[n["origin"][0] + (ix + 0.5) * n["cell"], n["origin"][1] + (n["shape"][0] - 1 - iy + 0.5) * n["cell"]]
    return xy, dist[iy, ix]


def _defects(ctx, ground, xy, where, lay, ok, hi, two=None, h2=None, blocked_xy=None, top=30):
    """진단에 걸린 자리를 '결함 후보' 목록으로 — 어디에(씬 좌표 · 입력 원본 좌표) · 얼마나 크게 · 무슨 근거로. 화면에서 그 자리를 짚어 보이기 위한 것.
    <out>/defects_before.json (보정 전) · defects_after.json (보정 뒤 다시 진단) 으로 남김. → 종류별 개수"""
    from .checks import clusters
    n2 = ctx["nav2"]
    a1 = 4 * n2["cell"] ** 2                                    # 표본 하나가 대표하는 넓이 (빈 칸 4개마다 하나)
    D = []
    for c in clusters(np.asarray(where, float).reshape(-1, 2), n2, join_cells=int(round(0.5 / n2["cell"]))):   # 1 m 안이면 한 결함 — 얇은 판 · 턱의 가장자리가 여러 개로 쪼개지지 않게
        D.append(dict(type="double_floor", title="충돌 바닥 두 겹", severity="fail", fixable=True, **c,
                      basis=f"이 자리의 수직 광선 {c['samples']}개가 맨 위 충돌 면 아래 3 cm 안에서 면을 하나 더 만남 (전체 간격 중앙 {lay.get('double_gap_mm_median')} mm). 겹친 충돌 면은 물리 엔진을 멈추게 할 수 있음",
                      action="충돌 메쉬에서 바닥 바로 위의 낮은 조각을 지움 (보이는 형상은 그대로)"))
    if ok is not None:
        for c in clusters(xy[~ok], n2):
            if c["samples"] * a1 >= 0.25:
                glb, after = ctx.get("kind") == "glb", bool(ctx.get("corrected"))   # GLB · 아이폰 원본: 보정 단계가 둘레 바닥 색 판으로 덮음 (통합 기록 R9)
                D.append(dict(type="no_visible_floor", title="보이는 바닥 없음 (재촬영 후보)", severity="warn", fixable=glb and not after, **c,
                              basis=f"빈 바닥 표본 {c['samples']}개(약 {c['samples'] * a1:.1f} m²)에서 보이는 바닥 면을 찾지 못함. 충돌 바닥은 평평한 한 장이라 로봇은 지나가지만, "
                                    "카메라에는 받침 바닥(FloorFill, 맵 전체 바닥의 중앙값 한 가지 색 · 스캔 바닥보다 2~5 cm 아래)이 무늬 없는 자리로 보임",
                              action=("보정 단계에서 덮지 않음 — 4 m² 보다 넓거나, 둘레가 바닥이 아니거나, 아래에 면이 보이는 자리 (이유는 보정 단계 메시지). 다시 찍을 후보" if glb and after else
                                      "둘레 바닥 색의 판(FloorPatch)으로 덮음 — 찍힌 바닥이 아니므로 다시 찍으면 더 정확. 아래로 꺼진 자리 · 둘레가 바닥이 아닌 자리 · 4 m² 보다 넓은 자리는 덮지 않음" if glb
                                      else "고치지 않음 — 촬영 때 가려졌거나 찍히지 않은 자리. 다시 찍을 후보")))
        for c in clusters(xy[hi], n2):
            if c["extent_m"] >= 0.6:
                D.append(dict(type="low_bump", title="낮은 턱 의심", severity="warn", fixable=False, **c,
                              basis=f"보이는 바닥이 충돌 바닥보다 3~12 cm 높은 자리가 {c['extent_m']} m 이어짐 (표본 {c['samples']}개). 충돌 바닥에는 이 솟음이 없어 시뮬에서는 그냥 지나감",
                              action="고치지 않음 — 스캔만으로는 실제 턱인지 측정 잡음인지 가릴 수 없음. 현장 확인 후보"))
    if two is not None:
        for c in clusters(xy[two], n2):
            if c["samples"] * a1 >= 0.25:
                near = two & (np.linalg.norm(xy - c["center"], axis=1) <= c["radius_m"])
                D.append(dict(type="floor_two_levels", title="바닥이 두 높이로 찍힘 (재촬영 후보)", severity="warn", fixable=False, **c,
                              basis=f"빈 바닥 표본 {c['samples']}개(약 {c['samples'] * a1:.1f} m²)에서 보이는 바닥 아래 {100 * float(np.median(h2[near])):.0f} cm 쯤에 바닥 면이 한 겹 더 있음. "
                                    "촬영 중 자세가 위아래로 틀어지면 같은 바닥이 두 높이로 찍힘",
                              action="고치지 않음 — 어느 높이가 실제 바닥인지 스캔만으로는 알 수 없음. 다시 찍을 후보"))
    if blocked_xy is not None and len(blocked_xy):              # 촬영자가 걸어간 자리가 장애물로 찍힌 구간 (표본 = 경로 위 10 cm 마다)
        for c in clusters(np.asarray(blocked_xy, float), n2, join_cells=int(round(0.5 / n2["cell"]))):
            if c["samples"] >= 5:                               # 0.5 m 넘게
                D.append(dict(type="path_blocked", title="걸어간 길이 장애물로 찍힘 (재촬영)", severity="fail", fixable=False, **c,
                              basis=f"촬영자가 걸어간 길 약 {c['samples'] * 0.1:.1f} m 가 지도에서 장애물임. 그 자리는 실제로 비어 있었으므로, 같은 곳이 다른 높이 · 위치로 겹쳐 찍힌 형상이 길을 막은 것 (촬영 중 자세 틀어짐)",
                              action="고치지 않음 — 어느 쪽이 실제 형상인지 스캔만으로는 알 수 없음. 다시 찍을 것: 같은 길을 되돌아오지 말고 한 방향으로, 천천히"))
    bumps = [d for d in D if d["type"] == "low_bump"]
    for d in D:
        if d["type"] == "double_floor" and any(np.linalg.norm(np.subtract(d["center"], b["center"])) < b["radius_m"] + d["radius_m"] for b in bumps):
            d["near_low_bump"] = True
            d["basis"] += ". 낮은 턱 의심 자리와 붙어 있음 — 솟은 자리의 가장자리가 바닥과 3 cm 안에서 겹친 것일 수 있음"
    D.sort(key=lambda d: (d["severity"] != "fail", -d["samples"]))
    D = D[:top]
    tp = os.path.join(ctx["build"], "transform.json")
    T = np.array(json.load(open(tp, encoding="utf-8"))["T_input_to_scene"]) if os.path.exists(tp) else np.eye(4)
    Ti = np.linalg.inv(T)
    for k, d in enumerate(D):
        x, y = d.pop("center")
        z = float(ground.height(np.array([[x, y]]))[0])
        d["id"] = f"D{k + 1}"
        d["center_scene"] = [round(x, 3), round(y, 3), round(z, 3)]
        d["center_input"] = np.round(Ti @ [x, y, z, 1.0], 4)[:3].tolist()
    phase = "after" if ctx.get("corrected") else "before"
    json.dump(dict(phase=phase, T_input_to_scene=T.tolist(), up_input=np.round(Ti[:3, 2], 5).tolist(), floor_point_input=np.round(Ti[:3, 3], 4).tolist(), defects=D),
              open(ctx.path(f"defects_{phase}.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    cnt = {}
    for d in D:
        cnt[d["type"]] = cnt.get(d["type"], 0) + 1
    return cnt


def s_diagnose(ctx):
    """정적 진단 (논문 3.1 의 생각): 로봇이 지날 수 있는 바닥 전체에서 충돌 바닥 층수 · 보이는 바닥과 충돌 바닥의 높이 차 · 천장(조명)."""
    sys.path.insert(0, TOOLS)
    sys.path.insert(0, PIPE)
    from check_floor_layers import measure
    from mesh_utils import Ground
    import open3d as o3d
    if "nav2_free" not in ctx:                                  # --force 로 지도 없이 넘어온 경우
        return Result(FAIL, "Nav2 지도가 없어 진단할 수 없음", messages=["진단은 지도의 빈 바닥에서 표본을 뽑음 — 앞의 nav2 단계가 실패한 이유를 볼 것"])
    xy, _ = _free_xy(ctx, ROBOT_RADIUS_M, step=4)
    if len(xy) < 50:
        return Result(FAIL, "진단할 빈 바닥이 거의 없음")
    gp = os.path.join(ctx["build"], "ground.npz")
    ground = Ground.load(gp) if os.path.exists(gp) else Ground()
    zf = ground.height(xy)
    lay, where = measure(ctx["usd_mesh"], "Collision", xy, zf + 1.0)
    m = dict(samples=lay["samples"], floor_single_pct=lay["single_pct"], floor_double=lay["double_within_3cm"], floor_double_pct=lay["double_pct"], floor_under=lay["under_3_to_30cm"])
    r = Result(PASS, "", metrics=m)
    if lay["double_within_3cm"]:
        msg = f"충돌 바닥이 두 겹인 자리 {lay['double_within_3cm']}곳 ({lay['double_pct']}%, 간격 중앙 {lay['double_gap_mm_median']} mm) — 물리 엔진이 멈출 수 있음. 예: {np.round(where[:3], 2).tolist()}"
        if ctx.get("corrected"):
            r.fail(msg + " — 보정 뒤에도 남음")
        elif ctx.get("diagnose_only"):
            r.fail(msg)
        else:
            ctx.setdefault("needs", {})["double_floor"] = True
            r.warn(msg + " → 보정 단계에서 고침")
    if lay["under_pct"] > 1.0:
        r.warn(f"바닥 3~30 cm 아래에 면이 있는 자리 {lay['under_3_to_30cm']}곳 ({lay['under_pct']}%)")
    vp = os.path.join(ctx["build"], "visual.ply")               # 보이는 바닥 ↔ 충돌 바닥
    ok = hi = two = h2 = None
    if os.path.exists(vp):
        vis = o3d.io.read_triangle_mesh(vp)
        sc = o3d.t.geometry.RaycastingScene()
        sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(vis))
        pp, pid = os.path.join(ctx["build"], "floor_patch.ply"), -1
        # 판이 없는(보정 전) 장면은 보정 단계의 구멍 찾기(_patch_mask)가 다시 씀 — 큰 메쉬(방 하나 240만 삼각형)는 이 구조를 만드는 데만 수 초~수십 초
        # 이 구조에는 visual.ply 말고 넣지 말 것 — stray_quality.measure(visual_scene=…) 가 visual.ply 하나로 만든 구조라고 가정함 (통합 기록 R14)
        ctx["_vis_scene"] = None if os.path.exists(pp) else (vp, os.path.getmtime(vp), sc)
        if os.path.exists(pp):                                # 보정 뒤: 구멍을 덮은 판(FloorPatch)도 보이는 바닥 (통합 기록 R9)
            pid = sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(o3d.io.read_triangle_mesh(pp)))
        rays = np.c_[xy, zf + 1.0, np.zeros(len(xy)), np.zeros(len(xy)), -np.ones(len(xy))].astype(np.float32)
        a1 = sc.cast_rays(o3d.core.Tensor(rays))
        h, fake = a1["t_hit"].numpy(), a1["geometry_ids"].numpy().astype(np.int64) == pid   # fake = 판에 맞음 (찍힌 바닥이 아님 → 높이 차 통계에서 뺌)
        dz = 1.0 - h
        ok = np.isfinite(h) & (np.abs(dz) < 0.3)
        # 보이는 바닥이 두 높이로 겹친 자리: 맨 위 면 바로 아래에서 다시 쏴 3~30 cm 안에 위를 향한 면이 또 있음.
        # 촬영 중 자세가 위아래로 틀어지면 같은 바닥이 두 높이로 찍힘 (방 촬영본 11.24: 44 m 를 도는 동안 10~25 cm)
        r2 = rays.copy()
        r2[:, 2] = np.where(np.isfinite(h), zf + 1.0 - h - 1e-3, -1e6)
        a2 = sc.cast_rays(o3d.core.Tensor(r2))
        h2, n2z = a2["t_hit"].numpy(), a2["primitive_normals"].numpy()[:, 2]
        two = ok & np.isfinite(h2) & (h2 > 0.03) & (h2 < 0.30) & (np.abs(n2z) > 0.7) & (a2["geometry_ids"].numpy().astype(np.int64) != pid)   # 판은 스캔 바닥 테두리 밑에 깔려 있어 두 겹으로 안 셈
        m["floor_two_levels"] = dict(samples=int(two.sum()), pct=round(100 * float(two.mean()), 1), gap_cm_median=round(100 * float(np.median(h2[two])), 1) if two.any() else None)
        if two.mean() > 0.01:
            r.warn(f"보이는 바닥이 두 높이로 겹친 자리가 빈 바닥의 {m['floor_two_levels']['pct']}% (간격 중앙 {m['floor_two_levels']['gap_cm_median']} cm) — 촬영 중 자세가 위아래로 틀어진 흔적일 수 있음. "
                   "로봇 카메라에는 바닥이 어긋나 보이고, 벽 · 물체도 그만큼 어긋났을 수 있음")
        sk = ok & ~fake
        m.update(visible_floor_missing_pct=round(100 * float((~ok).mean()), 1),
                 visible_minus_collision_cm=dict(p5=round(100 * float(np.percentile(dz[sk], 5)), 1), median=round(100 * float(np.median(dz[sk])), 1), p95=round(100 * float(np.percentile(dz[sk], 95)), 1)) if sk.any() else None)
        if fake.any():
            m["floor_patch_samples"] = int(fake.sum())
        if sk.any() and max(abs(np.percentile(dz[sk], 5)), abs(np.percentile(dz[sk], 95))) > 0.05:
            r.warn(f"보이는 바닥과 충돌 바닥의 높이 차가 5 cm 를 넘는 곳이 있음 (5~95%: {m['visible_minus_collision_cm']['p5']} ~ {m['visible_minus_collision_cm']['p95']} cm)")
        if (~ok).mean() > 0.10:
            r.warn(f"빈 바닥의 {m['visible_floor_missing_pct']}% 에 보이는 바닥이 없음 (받침 바닥으로 메워짐)")
        from scipy import ndimage                              # 낮은 턱: 보이는 바닥이 충돌 바닥보다 3~12 cm 높은 자리 — 충돌에서는 평평하게 지나감
        n2, c2 = ctx["nav2"], ctx["nav2"]["cell"]
        ix = np.floor((xy[:, 0] - n2["origin"][0]) / c2).astype(int)
        iy = n2["shape"][0] - 1 - np.floor((xy[:, 1] - n2["origin"][1]) / c2).astype(int)
        bump = np.zeros(n2["shape"], bool)
        hi = ok & (dz > 0.03) & (dz < 0.12)
        bump[iy[hi], ix[hi]] = True
        lab, nb = ndimage.label(ndimage.binary_dilation(bump, iterations=2))   # 표본이 4칸마다라 이웃끼리 잇기
        runs = []
        for sl in ndimage.find_objects(lab):
            L = float(np.hypot(sl[0].stop - sl[0].start, sl[1].stop - sl[1].start) * c2)
            if L >= 0.6:
                cy, cx = (sl[0].start + sl[0].stop) / 2, (sl[1].start + sl[1].stop) / 2
                runs.append((round(L, 1), round(float(n2["origin"][0] + cx * c2), 1), round(float(n2["origin"][1] + (n2["shape"][0] - cy) * c2), 1)))
        runs.sort(reverse=True)
        m["low_bumps"] = dict(samples_over_3cm_pct=round(100 * float(hi.sum()) / max(int(ok.sum()), 1), 2), runs_over_0p6m=len(runs), longest=runs[:5])
        if runs:
            r.warn(f"보이는 바닥이 3~12 cm 솟은 자리가 0.6 m 넘게 이어진 곳 {len(runs)}군데 (가장 긴 것 {runs[0][0]} m, 위치 ({runs[0][1]}, {runs[0][2]})) — 문턱 · 턱일 수 있음. "
                   "충돌 바닥에는 이 솟음이 없어 시뮬에서는 그냥 지나감. 실제 턱이면 시뮬 결과가 실제보다 좋게 나옴")
    from .checks import obstacle_match                         # 보이는 물체 ↔ 충돌 물체 (라이다가 보는 것과 Nav2 지도가 같은가)
    om = obstacle_match(ctx["usd_mesh"], ground, ctx["nav2"], ctx["nav2_free"])
    m["obstacle_match"] = om
    # 다닐 수 있는 모양인가: 빈 바닥이 한 덩어리인지 · (아이폰 원본) 촬영자가 걸어간 길이 지도에서 빈 자리인지.
    # 왕복 촬영에서 자세가 틀어지면 같은 복도가 다른 높이 · 위치로 겹쳐 찍혀, 걸어간 자리가 장애물이 되고 빈 바닥이 끊김 (인문대 2층 촬영 11.25: 걸어간 길의 43% 가 장애물)
    from .checks import drivable
    walked, ctx["unusable"] = None, []
    tj, tp_ = os.path.join(ctx.get("map_root") or "", "trajectory_camera_optical.txt"), os.path.join(ctx["build"], "transform.json")
    if ctx.get("source") == "stray" and os.path.exists(tj) and os.path.exists(tp_):
        Ts = np.array(json.load(open(tp_, encoding="utf-8"))["T_input_to_scene"])
        cam = np.loadtxt(tj, usecols=(1, 2, 3), ndmin=2)
        walked = (cam @ Ts[:3, :3].T + Ts[:3, 3])[:, :2]
    dv = drivable(ctx["nav2"], ctx["nav2_free"], ctx["nav2_occ"], walked, ROBOT_RADIUS_M)
    blocked_xy = (dv.get("path") or {}).pop("blocked_xy", np.zeros((0, 2)))
    m["drivable"] = dv
    if dv.get("path"):
        pa = dv["path"]
        # 문턱 10% · 80% 는 촬영본 두 건으로 잡은 잠정값: 방 촬영본(한 바퀴) 과 복도 왕복 촬영본
        if pa["blocked_pct"] > 10:
            ctx["unusable"].append(f"촬영자가 걸어간 길 {pa['length_m']} m 가운데 {pa['blocked_m']} m ({pa['blocked_pct']}%)가 지도에서 장애물임 — 같은 곳이 다른 높이 · 위치로 겹쳐 찍힌 것. 이 맵으로는 주행할 수 없음")
        elif pa["blocked_pct"] > 2:
            r.warn(f"촬영자가 걸어간 길의 {pa['blocked_pct']}% ({pa['blocked_m']} m)가 지도에서 장애물임 — 그 자리는 실제로 비어 있었음. 겹쳐 찍힌 형상일 수 있음")
        if pa["main_piece_pct"] < 80:
            ctx["unusable"].append(f"촬영자가 걸어간 길이 지도에서 끊겨 있음 — 한 덩어리로 이어진 구간이 {pa['main_piece_pct']}% 뿐. 처음부터 끝까지 주행할 수 없음")
    if dv["main_share_pct"] < 60:
        r.warn(f"빈 바닥이 여러 조각으로 끊겨 있음 — 로봇이 다닐 수 있는 바닥(장애물과 {ROBOT_RADIUS_M} m 띄움) 중 가장 큰 덩어리가 {dv['main_share_pct']}% · 1 m² 넘는 조각 {dv['pieces']}개")
    if dv["turn_area_m2"] < 5.0:                                # sim/add_site.py 의 맵 판정이 "실패 예상"으로 보는 기준 (5.0 m² 미만, D docs/isaac_check.md) — 같은 수치 · 같은 문턱
        r.warn(f"로봇이 제자리에서 돌 수 있는 넓이가 {dv['turn_area_m2']} m² 뿐임 (5 m² 미만) — 주행 공간이 좁아 주행 실패가 예상됨 (sim 쪽 맵 판정의 '실패 예상'과 같은 기준). "
               f"빈 바닥은 {dv['free_m2']} m² 지만 장애물과 {dv['turn_radius_m']} m 넘게 떨어진 자리가 적음 — 책상 · 가구가 촘촘하거나 좁은 곳")
    for u in ctx["unusable"]:
        r.warn(u)
    if om["visible_cells"] > 200 and om["collision_cells"] > 200:
        # 문턱은 시험 세트 8개와 D 의 주행 결과로 잡은 잠정값: 반시계 주행이 실패한 주차장 v0.9 만 31.5 / 15.3%, 나머지는 73~100 / 99.7~100%
        if om["collision_in_visible_pct"] < 90 or om["visible_in_collision_pct"] < 60:
            r.warn(f"보이는 물체와 충돌 물체가 어긋남 — 보이는 칸 중 충돌이 있는 곳 {om['visible_in_collision_pct']}% · 충돌 칸 중 보이는 것이 있는 곳 {om['collision_in_visible_pct']}% (10 cm 안, 기준 60% · 90%). "
                   "라이다는 보이는 형상을 보고 지도는 충돌로 만들어 위치 추정이 흔들릴 수 있음")
    p = ctx["profile"]
    if "has_ceiling" not in p and os.path.exists(vp):          # 맵 폴더 입력: 만든 메쉬에서 천장을 알아냄
        from .checks import ceiling_over_floor
        p["ceiling_over_floor_pct"] = ceiling_over_floor(vp, ground, ctx["nav2"], ctx["nav2_free"])
        p["has_ceiling"] = p["ceiling_over_floor_pct"] > 30
    m["has_ceiling"] = bool(p.get("has_ceiling", False))
    cov = float(p.get("ceiling_over_floor_pct") or 0)
    # 그림자 설정은 천장이 일부만 찍혀도 넣음 — 천장 조각이 빛을 가려 그 아래가 어두워짐. 천장이 없는 장면에 넣어도 그림자만 없어질 뿐
    p["shadow_fix"] = m["shadow_fix"] = bool(m["has_ceiling"] or cov >= 10)
    if m["has_ceiling"]:
        r.messages.append("천장이 있음 — 씬 루트에 그림자 설정을 넣음 (안 넣으면 Isaac Sim 화면 밝기 0)")
    elif p["shadow_fix"]:
        r.messages.append(f"천장이 일부만 찍힘 (빈 바닥 위 {cov:.0f}%) — 그 아래가 어두워지지 않게 씬 루트에 그림자 설정을 넣음")
    if ctx.get("source") == "stray":                            # 아이폰 원본: 안 쓴 프레임 깊이 ↔ 맵 (B 의 stray_quality, 통합 기록 R3)
        from . import stray_quality
        try:
            q = ctx.get("stray_quality") or stray_quality.measure(ctx["out"], capture=ctx["input"], visual_scene=ctx.get("_vis_scene"))
            ctx["stray_quality"] = q
            m["stray_quality"] = {k: q.get(k) for k in ("frames", "median_cm", "p90_cm", "within_2cm_pct", "hit_pct", "verdict")}
            r.messages.append(q["summary"])
            for w in q.get("reasons") or []:
                r.warn(w)                                       # fail 이어도 주의로만 (맵은 만들어짐)
        except (SystemExit, Exception) as e:                    # measure 는 SystemExit 을 냄
            r.messages.append(f"복원 정확도를 재지 못함: {e}")
        # 되돌아온 자리 어긋남 (B 의 stray_revisit, 통합 기록 R19): 같은 곳을 다시 본 깊이끼리 맞춰 그 사이에 쌓인 자세 틀어짐을 직접 잼.
        # 복원 정확도는 키프레임 바로 옆 장과 견주어 이것을 못 봄 (복도 왕복 촬영본: 6.13 cm 로 주의에 그쳤지만 맵은 못 썼음).
        # 판정은 주의까지만 — "쓸 수 없음"은 걸어간 길이 실제로 막혔을 때(drivable)만
        from . import stray_revisit
        try:
            rv = ctx.get("stray_revisit") or stray_revisit.measure(ctx["out"], capture=ctx["input"])
            ctx["stray_revisit"] = rv
            keys = ("pairs", "reliable", "h_max_cm", "z_max_cm", "yaw_max_deg", "h_worst", "z_worst")
            m["revisit"] = dict(verdict=rv.get("verdict"), **{k: {kk: (rv[k].get("summary") or {}).get(kk) for kk in keys} for k in ("raw", "fixed") if rv.get(k)})
            for w in rv.get("reasons") or []:
                r.warn(w)
            if rv.get("verdict") == "fail":
                r.warn("되돌아온 자리가 크게 어긋나 있음 — 같은 곳이 두 번 겹쳐 찍혔을 수 있음. 다시 찍기를 권함 (긴 복도는 한 방향으로만)")
        except (SystemExit, Exception) as e:
            r.messages.append(f"되돌아온 자리 어긋남을 재지 못함: {e}")
    m["defects"] = _defects(ctx, ground, xy, where, lay, ok, hi, two, h2, blocked_xy)
    if m["defects"].get("no_visible_floor") and ctx.get("kind") == "glb" and not ctx.get("corrected") and not ctx.get("diagnose_only"):
        ctx.setdefault("needs", {})["no_visible_floor"] = True  # 보정 단계에서 둘레 바닥 색 판으로 덮음 (통합 기록 R9)
    r.summary = f"빈 바닥 표본 {lay['samples']:,} · 바닥 한 겹 {lay['single_pct']}% · 두 겹 {lay['double_within_3cm']}" + (f" · 보이는 바닥 없음 {m['visible_floor_missing_pct']}%" if "visible_floor_missing_pct" in m else "")
    return r


def _patch_mask(ctx, defects, top=0.10, max_m2=4.0, low_share=0.1, ring_share=0.5):
    """보이는 바닥 구멍을 덮을 칸 (Nav2 지도 격자, 통합 기록 R9). → (mask[H, W], 구멍 목록)
    빈 칸마다 바닥 위 top 에서 아래로 쏴 바닥 높이(± band)에 보이는 면이 없는 칸을 이어 구멍으로 → 보정 전 결함(no_visible_floor) 원에 걸친 구멍만.
    덮지 않음: 바닥보다 band 넘게 아래에 면이 보이는 칸이 low_share 넘음(꺼진 자리 · 계단) · 둘레 20 cm 띠의 보이는 바닥이 ring_share 미만 · max_m2 보다 넓음.
    덮는 구멍은 한 칸 넓혀 스캔 바닥 테두리 밑까지 — 빈 칸 안에서만 (장애물 · 벽 · 미지 칸에는 안 깖)."""
    sys.path.insert(0, PIPE)
    from mesh_utils import Ground
    from scipy import ndimage
    import open3d as o3d
    n, free = ctx["nav2"], ctx["nav2_free"]
    (H, W), c, (ox, oy) = n["shape"], n["cell"], n["origin"]
    band = 0.05 if ctx["profile"].get("floor_flat", True) else 0.08     # detect.config_glb 의 floor_band_m 과 같게
    gp = os.path.join(ctx["build"], "ground.npz")
    ground = Ground.load(gp) if os.path.exists(gp) else Ground()
    iy, ix = np.nonzero(free)
    xy = np.c_[ox + (ix + 0.5) * c, oy + (H - 1 - iy + 0.5) * c]
    vp = os.path.join(ctx["build"], "visual.ply")
    keep = ctx.get("_vis_scene")
    if keep and keep[0] == vp and keep[1] == os.path.getmtime(vp):   # 진단이 방금 만든 것 (같은 파일)
        sc = keep[2]
    else:
        sc = o3d.t.geometry.RaycastingScene()
        sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(o3d.io.read_triangle_mesh(vp)))
    h = sc.cast_rays(o3d.core.Tensor(np.c_[xy, ground.height(xy) + top, np.zeros((len(xy), 2)), -np.ones(len(xy))].astype(np.float32)))["t_hit"].numpy()
    fin, dz = np.isfinite(h), top - h                           # dz = 보이는 면 높이 − 바닥

    def grid(sel):
        g = np.zeros((H, W), bool)
        g[iy[sel], ix[sel]] = True
        return g
    floor, low = grid(fin & (np.abs(dz) < band)), grid(fin & (dz <= -band))
    lab, _ = ndimage.label(grid(~fin) | low, structure=np.ones((3, 3)))
    mask, out = np.zeros((H, W), bool), []
    for k, sl in enumerate(ndimage.find_objects(lab), 1):
        win = (slice(max(sl[0].start - 6, 0), min(sl[0].stop + 6, H)), slice(max(sl[1].start - 6, 0), min(sl[1].stop + 6, W)))
        comp = lab[win] == k
        py, px = np.nonzero(comp)
        pxy = np.c_[ox + (px + win[1].start + 0.5) * c, oy + (H - 1 - py - win[0].start + 0.5) * c]
        if not any((np.linalg.norm(pxy - d["center_scene"][:2], axis=1) <= d["radius_m"]).any() for d in defects):
            continue
        ring = ndimage.binary_dilation(comp, iterations=4) & ~ndimage.binary_dilation(comp) & free[win]
        g = dict(center=np.round(pxy.mean(0), 2).tolist(), area_m2=round(float(comp.sum()) * c * c, 2), low_pct=round(100 * float(low[win][comp].mean()), 1),
                 ring_floor_pct=round(100 * float(floor[win][ring].mean()), 1) if ring.any() else 0.0)
        g["reason"] = ("바닥보다 아래에 면이 보임 — 꺼진 자리 · 계단일 수 있음" if g["low_pct"] > 100 * low_share else
                       "둘레에 보이는 바닥이 적음 — 바닥 높이가 아닌 자리일 수 있음" if g["ring_floor_pct"] < 100 * ring_share else
                       f"{max_m2:g} m² 보다 넓음 — 다시 찍을 것" if g["area_m2"] > max_m2 else None)
        g["patched"] = g["reason"] is None
        if g["patched"]:
            mask[win] |= ndimage.binary_dilation(comp) & free[win]
        out.append(g)
    return mask, sorted(out, key=lambda g: -g["area_m2"])


def s_correct(ctx):
    """보정 (논문 3.2 의 1차 보정과 같은 생각): 진단에 걸린 항목만 고쳐 다시 만들고 다시 진단한다.
    두 겹 바닥 → 충돌 메쉬에서 바닥 위 5 cm 아래의 것을 지움 (바닥 한 장과 그보다 5 cm 넘게 솟은 형상만 남김).
    보이는 바닥 없음 (GLB · 아이폰 원본) → 그 구멍을 둘레 바닥 색의 판(FloorPatch)으로 덮음. 충돌은 그대로 (통합 기록 R9)."""
    needs = ctx.get("needs") or {}
    if not needs:
        return Result(SKIP, "진단에 걸린 항목이 없어 보정하지 않음")
    before = next(d for d in ctx["results"] if d["stage"] == "diagnose")["metrics"]
    cfg2 = os.path.join(os.path.dirname(ctx["config"]), "auto_config_corrected.yaml")
    glb = ctx["kind"] == "glb"
    lines = [f"# scan2sim 보정: 진단에 걸린 항목({', '.join(needs)})을 고친 설정", f"base: {os.path.basename(ctx['config'])}"]
    how, patch = [], None
    if needs.get("double_floor") and glb:
        lines += ["collision:", "  low_h_m: 0.05", "  min_cluster_tris_after: 100", ""]
        how.append("충돌 메쉬에서 바닥 위 5 cm 아래의 것을 지움 (보이는 메쉬는 그대로)")
    elif needs.get("double_floor"):                             # 맵 폴더 · 아이폰 원본: 바닥 한 장은 이미 깔려 있고, 걸리는 것은 그 위에 남은 낮은 조각 (틀의 기본 5 cm · 3 cm 를 넘긴 것)
        lines += ["single_floor:", "  collision_low_h_m: 0.10", "  collision_flat_h_m: 0.06", ""]
        how.append("충돌 메쉬에서 바닥 위 10 cm 안의 낮은 조각과 6 cm 안의 수평 면을 지움 (보이는 메쉬는 그대로)")
    if needs.get("no_visible_floor"):                           # 진단이 GLB 길일 때만 걸어 둠
        D = [d for d in json.load(open(ctx.path("defects_before.json"), encoding="utf-8"))["defects"] if d["type"] == "no_visible_floor"]
        mask, regions = _patch_mask(ctx, D)
        patch = dict(cells=int(mask.sum()), area_m2=round(float(mask.sum()) * ctx["nav2"]["cell"] ** 2, 2), regions=regions)
        if mask.any():
            mp = os.path.join(os.path.dirname(ctx["config"]), "floor_patch_mask.npz")
            np.savez_compressed(mp, mask=mask, origin=np.asarray(ctx["nav2"]["origin"], float), cell=float(ctx["nav2"]["cell"]))
            lines += ["floor_patch:", f"  mask_file: {mp.replace(os.sep, '/')}", "  depth_m: 0.005", ""]
            how.append(f"보이는 바닥 구멍 {sum(g['patched'] for g in regions)}곳 ({patch['area_m2']} m²)을 둘레 바닥 색의 판(FloorPatch, 바닥 0.5 cm 아래)으로 덮음 — 찍힌 바닥이 아님. 충돌 · Nav2 지도는 그대로")
        how += [f"덮지 않은 구멍 ({g['center'][0]}, {g['center'][1]}) {g['area_m2']} m² — {g['reason']}" for g in regions if not g["patched"]]
    if len(lines) == 2:
        return Result(SKIP, "고칠 수 있는 자리가 없어 보정하지 않음", metrics=dict(floor_patch=patch), messages=how)
    open(cfg2, "w", encoding="utf-8", newline=chr(10)).write(chr(10).join(lines))
    ctx["config"], ctx["corrected"], ctx["resume"] = cfg2, True, False   # 보정은 반드시 다시 만듦 (--resume 이어도)
    for nm, fn in (("convert", s_convert), ("nav2", s_nav2)):
        r = fn(ctx)
        if r.status == FAIL:
            return Result(FAIL, f"보정 뒤 {nm} 단계 실패: {r.summary}", messages=r.messages)
    r = s_diagnose(ctx)
    after = r.metrics
    ctx["diagnose_before"], ctx["diagnose_after"] = before, after
    S = [f"두 겹 바닥 {before.get('floor_double')} → {after.get('floor_double')} · 바닥 한 겹 {before.get('floor_single_pct')} → {after.get('floor_single_pct')}%"] if needs.get("double_floor") else []
    if patch:
        S.append(f"보이는 바닥 없음 {before['defects'].get('no_visible_floor', 0)} → {after['defects'].get('no_visible_floor', 0)}곳 (판 {patch['area_m2']} m²)")
    out = Result(r.status, " · ".join(S), metrics=dict(fixed=[k for k in needs if k != "no_visible_floor" or (patch and patch["cells"])], before=before, after=after, floor_patch=patch),
                 messages=how + r.messages)
    ctx["floor_patch"] = patch
    return out


def s_bundle(ctx):
    """Isaac Sim 에 올릴 묶음: 씬 루트 제안(천장이 있으면 그림자 설정) · 시작 자세 · 시험 경로 · 목록."""
    b = os.path.dirname(ctx["nav2"]["dir"].rstrip("/\\"))
    xy, d = _free_xy(ctx, ROBOT_RADIUS_M)
    k = int(np.argmax(d))                                       # 가장 넓은 자리에서 출발
    start = xy[k]
    far = xy[np.argmax(np.linalg.norm(xy - start, axis=1) * (d > 0.5 * d[k]))]
    yaw = math.degrees(math.atan2(far[1] - start[1], far[0] - start[0]))
    sys.path.insert(0, PIPE)
    from mesh_utils import Ground
    gp = os.path.join(ctx["build"], "ground.npz")
    z0 = float((Ground.load(gp) if os.path.exists(gp) else Ground()).height(start[None])[0])
    pose = dict(x=round(float(start[0]), 3), y=round(float(start[1]), 3), z=round(z0 + 0.022, 3), yaw_deg=round(yaw, 1), clearance_m=round(float(d[k]), 2),
                goal_xy=np.round(far, 3).tolist(), note="빈 바닥에서 장애물과 가장 먼 자리 · 가장 먼 빈 자리를 향함. 높이 = 바닥 + 0.022 (Nova Carter)")
    json.dump(pose, open(os.path.join(b, "start_pose.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    dp = os.path.join(ctx["build"], "drive_path.json")
    if os.path.exists(dp):
        P = np.array(json.load(open(dp, encoding="utf-8"))["poses"])
        with open(os.path.join(b, "route.csv"), "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["x", "y"])
            w.writerows(np.round(P[:, :2, 3], 3).tolist())
    usd_dst = os.path.join(b, "usd")
    shutil.rmtree(usd_dst, ignore_errors=True)
    shutil.copytree(os.path.dirname(ctx["usd_scene"]), usd_dst)
    root = ctx["name"][:1].upper() + ctx["name"][1:]
    vis = [p for p in ctx["prims"] if p not in ("Collision", "Boundary")]
    shadow = "".join(f'        over "{p}"\n        {{\n            bool primvars:doNotCastShadows = 1\n        }}\n\n' for p in vis) if ctx["profile"].get("shadow_fix", ctx["profile"].get("has_ceiling")) else ""
    usda = f'''#usda 1.0
(
    customLayerData = {{
        string map = "scan2sim 자동 생성 — 입력 {os.path.basename(os.path.normpath(ctx['input']))}"
        string start_pose = "빈 바닥에서 장애물과 가장 먼 자리 ({pose['x']}, {pose['y']}) · {pose['yaw_deg']} 도. 바닥 {z0:.3f} + 0.022"
        string lights = "돔 600 · 해 0{' · 천장이 있어 보이는 prim 에 doNotCastShadows (없으면 화면 밝기 0)' if shadow else ''}"
    }}
    defaultPrim = "World"
    doc = "scan2sim 이 만든 씬 루트 제안 (팀 시뮬 배치용) — sim/scenes/<장소>/ 에 두고, 맵 경로를 서버 위치에 맞출 것. 묶음만으로는 안 열림 (묶음만으로 여는 것은 scene_root.usda)"
    metersPerUnit = 1
    subLayers = [
        @../proxy/sensors.usda@,
        @../proxy/ros.usda@,
        @../proxy/robot.usda@,
        @../../../data/usd/{ctx['name']}/usd/{os.path.basename(ctx['usd_scene'])}@,
        @../proxy/world.usda@
    ]
    timeCodesPerSecond = 60
    upAxis = "Z"
)

over "World"
{{
    over "Nova_Carter_ROS"
    {{
        float xformOp:rotateZ = {pose['yaw_deg']}
        double3 xformOp:translate = ({pose['x']}, {pose['y']}, {pose['z']})
    }}

{f'    over "{root}"' + chr(10) + '    {' + chr(10) + shadow.rstrip(chr(10)) + chr(10) + '    }' + chr(10) + chr(10) if shadow else ''}    over "Lights"
    {{
        over "Dome"
        {{
            float inputs:intensity = 600
        }}

        over "Sun"
        {{
            float inputs:intensity = 0
        }}
    }}
}}
'''
    # 팀 시뮬 배치용(sim/scenes/<장소>/ 에 두고 로봇 · 센서 · ROS 층을 함께 거는 것)은 이름을 따로 — 묶음 밖 파일을 가리켜 혼자서는 안 열림 (D 11.24)
    open(os.path.join(b, "scene_root_sim.usda"), "w", encoding="utf-8", newline="\n").write(usda)
    # scene_root.usda: 묶음만으로 열리는 것 — 맵 + 돔 조명 + (천장이 있으면) 그림자 설정. 로봇은 없음. 맵 파일에는 조명이 없어 그대로 열면 화면이 검음
    over_map = (f'    over "{root}"' + chr(10) + '    {' + chr(10) + shadow.rstrip(chr(10)) + chr(10) + '    }' + chr(10) + chr(10)) if shadow else ""
    alone = f'''#usda 1.0
(
    customLayerData = {{
        string map = "scan2sim 자동 생성 — 입력 {os.path.basename(os.path.normpath(ctx['input']))}"
        string lights = "돔 600{' · 보이는 prim 에 doNotCastShadows (천장이 빛을 가려 화면이 검어지지 않게)' if shadow else ''}"
        string start_pose = "로봇을 놓을 자리는 start_pose.json ({pose['x']}, {pose['y']}, {pose['z']}) · {pose['yaw_deg']} 도"
    }}
    defaultPrim = "World"
    doc = "scan2sim 이 만든 씬 — 이 묶음만으로 열림 (맵 + 조명). 로봇 · 센서는 들어 있지 않음"
    metersPerUnit = 1
    subLayers = [
        @./usd/{os.path.basename(ctx['usd_scene'])}@
    ]
    upAxis = "Z"
)

over "World"
{{
{over_map}    def Xform "Lights"
    {{
        def DomeLight "Dome"
        {{
            float inputs:intensity = 600
        }}
    }}
}}
'''
    open(os.path.join(b, "scene_root.usda"), "w", encoding="utf-8", newline="\n").write(alone)
    files = sorted(os.path.relpath(os.path.join(dp_, f), b).replace(os.sep, "/") for dp_, _, fs in os.walk(b) for f in fs if f != ".keep")
    json.dump(dict(name=ctx["name"], input=os.path.basename(os.path.normpath(ctx["input"])), kind=ctx["kind"], files=files), open(os.path.join(b, "bundle.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    ctx["bundle"] = b
    r = Result(PASS, f"묶음 파일 {len(files)}개 · 시작 ({pose['x']}, {pose['y']}) {pose['yaw_deg']}° · 여유 {pose['clearance_m']} m", metrics=dict(start_pose=pose, files=len(files), shadow_overrides=len(vis) if shadow else 0))
    if d[k] < 0.5:
        r.warn(f"가장 넓은 자리의 여유가 {d[k]:.2f} m 뿐임 — 로봇이 다니기 좁음")
    try:                                                        # 화면용 GLB: 엔진이 줄여서 써 두고 서버는 주기만 함 (D 11.24). 못 만들어도 묶음은 그대로
        from . import preview
        r.metrics["preview"] = preview.write(ctx["usd_mesh"], ctx.path("preview.glb"))
    except Exception as e:
        r.metrics["preview"] = dict(error=f"{type(e).__name__}: {e}")
        r.messages.append(f"화면용 파일(preview.glb)을 만들지 못함: {type(e).__name__}: {e}")
    return r


def s_verify(ctx):
    """주행 검증 (논문 3.3): Isaac Sim 에서 Nav2 · E2E 모델로 주행 — D · E 단계의 자리."""
    return Result(SKIP, "Isaac Sim 이 필요함 — 묶음을 서버에 올려 D · E 가 주행으로 검증 (결과 파일이 오면 여기에 붙임)")


def s_report(ctx):
    """단계별 판정을 한 장으로."""
    res = ctx.get("results", [])
    worst = FAIL if any(d["status"] == FAIL for d in res) else WARN if any(d["status"] == WARN for d in res) else PASS
    unusable = ctx.get("unusable") or []                        # 단계는 끝까지 돌았지만(묶음 · 화면용 파일은 나옴) 이 맵으로는 주행할 수 없는 경우 — 진단이 남김
    if unusable and worst != FAIL:
        worst = FAIL
        verdict = "쓸 수 없음 — " + unusable[0]
    else:
        verdict = {PASS: "시뮬레이션에 올릴 수 있음 (주행 검증 전)", WARN: "올릴 수 있으나 아래 주의 항목을 볼 것", FAIL: "쓸 수 없음 — 아래 실패 단계 참고"}[worst]
    L = [f"# scan2sim 보고서 — {ctx.get('name', '?')}", "", f"- 입력: `{os.path.basename(os.path.normpath(ctx['input']))}` ({'아이폰 LiDAR 원본 → ' if ctx.get('source') == 'stray' else ''}{ctx.get('kind') or '알 수 없음'})", f"- 만든 때: {time.strftime('%Y-%m-%d %H:%M')}",
         f"- **판정: {verdict}**", "", "## 단계", "", "| 단계 | 판정 | 걸린 시간 | 요약 |", "| --- | --- | ---: | --- |"]
    L += [f"| {d['stage']} | {MARK[d['status']]} | {d['seconds']} s | {d['summary']} |" for d in res]
    notes = [(d["stage"], m) for d in res for m in d["messages"]]
    if notes:
        L += ["", "## 볼 것", ""] + [f"- **{s}**: {m}" for s, m in notes]
    dg = next((d for d in res if d["stage"] == "diagnose"), None)
    if ctx.get("diagnose_after"):
        b_, a_ = ctx["diagnose_before"], ctx["diagnose_after"]
        vb, va = b_.get("visible_minus_collision_cm") or {}, a_.get("visible_minus_collision_cm") or {}
        L += ["", "## 정적 진단 — 보정 전후", "", "| 항목 | 보정 전 | 보정 후 | 기준 |", "| --- | --- | --- | --- |",
              f"| 충돌 바닥 한 겹 (빈 바닥 표본 {a_.get('samples', 0):,}) | {b_.get('floor_single_pct')}% | {a_.get('floor_single_pct')}% | — |",
              f"| 두 겹 바닥 (3 cm 안) | {b_.get('floor_double')} | {a_.get('floor_double')} | 0 |",
              f"| 바닥 3~30 cm 아래 면 | {b_.get('floor_under')} | {a_.get('floor_under')} | 표본의 1% 아래 |",
              f"| 보이는 바닥 − 충돌 바닥 (5% / 중앙 / 95%, cm) | {vb.get('p5')} / {vb.get('median')} / {vb.get('p95')} | {va.get('p5')} / {va.get('median')} / {va.get('p95')} | ±5 cm 안 |",
              f"| 보이는 바닥이 없는 빈 바닥 | {b_.get('visible_floor_missing_pct')}% | {a_.get('visible_floor_missing_pct')}% | 10% 아래 |",
              f"| 천장 | {'있음' if a_.get('has_ceiling') else '없음'} | {'있음 → 그림자 설정 넣음' if a_.get('has_ceiling') else '없음'} | — |"]
    elif dg and dg["metrics"]:
        m = dg["metrics"]
        L += ["", "## 정적 진단", "", "| 항목 | 값 | 기준 |", "| --- | --- | --- |",
              f"| 충돌 바닥 한 겹 (빈 바닥 표본 {m.get('samples', 0):,}) | {m.get('floor_single_pct')}% | — |",
              f"| 두 겹 바닥 (3 cm 안) | {m.get('floor_double')} | 0 |",
              f"| 바닥 3~30 cm 아래 면 | {m.get('floor_under')} | 표본의 1% 아래 |"]
        if m.get("visible_minus_collision_cm"):
            v = m["visible_minus_collision_cm"]
            L += [f"| 보이는 바닥 − 충돌 바닥 (5% / 중앙 / 95%) | {v['p5']} / {v['median']} / {v['p95']} cm | ±5 cm 안 |", f"| 보이는 바닥이 없는 빈 바닥 | {m.get('visible_floor_missing_pct')}% | 10% 아래 |"]
        L += [f"| 천장 | {'있음 → 그림자 설정 넣음' if m.get('has_ceiling') else '없음'} | — |"]
    rv = ctx.get("stray_revisit")
    if rv and rv.get("verdict") not in (None, "skip"):
        def _rv(k, name):
            sm_ = (rv.get(k) or {}).get("summary") or {}
            if not sm_.get("pairs"):
                return None
            hw, zw = sm_.get("h_worst") or {}, sm_.get("z_worst") or {}
            return (f"- {name}: 수평 최대 {sm_.get('h_max_cm')} cm" + (f" ({hw.get('t_a')} 초 ↔ {hw.get('t_b')} 초)" if hw else "")
                    + f" · 높이 최대 {sm_.get('z_max_cm')} cm" + (f" ({zw.get('t_a')} 초 ↔ {zw.get('t_b')} 초)" if zw else "") + f" · 다시 본 쌍 {sm_.get('pairs')} (믿을 쌍 {sm_.get('reliable')})")
        L += ["", "## 되돌아온 자리 어긋남 (아이폰 원본)", "", f"- 판정: {MARK.get(rv.get('verdict'), rv.get('verdict'))} (기준: 높이 3 cm 이하 통과 · 15 cm 넘으면 실패, 수평 10 cm 이하 통과 · 30 cm 넘으면 실패)"]
        L += [x for x in (_rv("raw", "원래 자세"), _rv("fixed", "고친 자세 (맵을 만든 것)")) if x]
    sq = ctx.get("stray_quality")
    if sq and sq.get("median_cm") is not None:
        L += ["", "## 복원 정확도 (아이폰 원본)", "", f"- {sq['summary']} · 90% {sq.get('p90_cm')} cm → {MARK.get(sq.get('verdict'), sq.get('verdict'))} (기준: 중앙 3 cm 이하)"]
    elif sq:
        L += ["", "## 복원 정확도 (아이폰 원본)", "", f"- {sq['summary']}"]
    if ctx.get("bundle"):
        L += ["", "## 묶음", "", f"- 위치: `bundle/` — `usd/` · `nav2/` · `scene_root.usda` · `start_pose.json` · `route.csv`", "- Isaac Sim 에서 열어 본 것이 아님. 주행 검증은 서버에서."]
    L += ["", "## 이 맵이 가정한 것과 바꾼 것", "",
          "- **바닥은 평평하다고 가정함.** 스캔 바닥의 굴곡(대부분 측정 잡음)을 그대로 쓰지 않고 충돌 바닥을 한 장으로 깔았음. 실제로 울퉁불퉁한 바닥 · 계단은 대상이 아님",
          "- **바닥 위 5 cm 아래의 것은 충돌에서 뺌** (두 겹 바닥 보정을 했을 때). 낮은 문턱 · 턱이 있어도 시뮬에서는 걸리지 않음",
          "- 스캔 범위의 테두리에 보이지 않는 경계벽을 세움 (실제로는 없는 벽)",
          "- 스캔 바닥의 구멍은 한 가지 색 받침 바닥으로 메움",
          "- **판정의 뜻:** '올릴 수 있음' = 이 맵 때문에 시뮬레이터가 멈추거나 화면이 깨지지는 않는다는 것. 맵이 실제 공간과 같다는 뜻이 아님"]
    fp = ctx.get("floor_patch")
    if fp and fp["cells"]:
        L += [f"- **보이는 바닥 구멍 {sum(g['patched'] for g in fp['regions'])}곳 ({fp['area_m2']} m²)은 둘레 바닥 색의 판(`FloorPatch`)으로 덮음.** 찍힌 바닥이 아님 — 그 자리의 실제 무늬 · 물건은 없음 (위치: {[g['center'] for g in fp['regions'] if g['patched']]})"]
    lb = ((ctx.get("diagnose_after") or (dg or {}).get("metrics") or {}).get("low_bumps"))
    if lb:
        L += [f"- 보이는 바닥이 3~12 cm 솟은 자리: 빈 바닥 표본의 {lb['samples_over_3cm_pct']}% · 0.6 m 넘게 이어진 곳 {lb['runs_over_0p6m']}군데" + (f" (길이 m · 위치: {lb['longest']})" if lb["longest"] else "")]
    L += ["", "## 주의사항", "", "- 움직이는 물체(사람 · 차)는 스캔한 순간의 모습으로 굳어 있음", "- 유리 · 거울 · 광택 면은 형상이 틀릴 수 있음", "- 맵의 절대 크기는 줄자 실측과 대조하지 않았음", "- 넓은 실외는 대상이 아님 (스캔이 수십 cm 틀어짐)", ""]
    open(ctx.path("report.md"), "w", encoding="utf-8", newline="\n").write("\n".join(L))
    json.dump(dict(name=ctx.get("name"), verdict=worst, stages=res), open(ctx.path("report.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=str)
    return Result(worst if worst != FAIL else PASS, f"report.md — {verdict}")


STAGES = [("input", s_input), ("convert", s_convert), ("nav2", s_nav2), ("diagnose", s_diagnose), ("correct", s_correct), ("bundle", s_bundle), ("verify", s_verify), ("report", s_report)]
