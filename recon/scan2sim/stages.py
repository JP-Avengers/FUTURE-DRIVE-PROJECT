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
    k = detect.kind_of(ctx["input"])
    ctx["kind"] = k
    ctx["name"] = detect.safe_name(ctx["input"])
    if k is None:
        return Result(FAIL, "입력을 알아볼 수 없음", messages=["받는 입력: GLB 파일 · B 형식 맵 폴더(keyframes/ + trajectory_camera_optical.txt)"])
    if k == "bag":
        return Result(FAIL, "녹화 파일(bag) 입력은 아직 연결되지 않음", messages=["녹화 파일 → 맵 폴더는 B 단계의 자리 (한 명령 스크립트가 나오면 여기에 붙임). 지금은 맵 폴더나 GLB 를 넣을 것"])
    build = ctx.path("build", ".keep")[:-len(".keep")].rstrip("/\\")
    if k == "glb":
        path = ctx["input"] if os.path.isfile(ctx["input"]) else [os.path.join(ctx["input"], f) for f in os.listdir(ctx["input"]) if f.lower().endswith(".glb")][0]
        p = detect.profile_glb(path)
        cfg = detect.config_glb(path, ctx["name"], build, p)
        r = Result(PASS, f"GLB · {'천장 있음' if p['has_ceiling'] else '천장 없음'} · 바닥 {'평평' if p['floor_flat'] else '평평하지 않음'} · 범위 {p['extent_m'][0]:.0f}×{p['extent_m'][1]:.0f} m", metrics=p)
        if max(p["extent_m"]) > 300 or max(p["extent_m"]) < 1:
            r.fail(f"범위가 이상함 ({max(p['extent_m']):.1f} m) — 단위(m)를 확인")
        if p["floor_area_m2"] < 2:
            r.fail("바닥을 찾지 못함 — 위쪽 축 또는 스캔 범위 확인")
        if not p["floor_flat"]:
            r.warn(f"바닥이 평평하지 않음 (높이 퍼짐 {p['floor_spread_m']} m) — 실외 · 경사로로 보고 스캔 바닥 높이 지도를 씀. 넓은 실외는 스캔 틀어짐 때문에 대상 밖")
        if p["textured"] == 0:
            r.warn("사진 텍스처가 없음 — 정점 색으로만 보임")
    else:
        p = detect.profile_map_folder(ctx["input"])
        cfg = detect.config_map_folder(ctx["input"], ctx["name"], build, p)
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
    if ctx["kind"] == "map_folder":
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
        return Result(SKIP, "sim/nav2/make_map.py 가 없어 지도를 만들지 않음")
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


def s_diagnose(ctx):
    """정적 진단 (논문 3.1 의 생각): 로봇이 지날 수 있는 바닥 전체에서 충돌 바닥 층수 · 보이는 바닥과 충돌 바닥의 높이 차 · 천장(조명)."""
    sys.path.insert(0, TOOLS)
    sys.path.insert(0, PIPE)
    from check_floor_layers import measure
    from mesh_utils import Ground
    import open3d as o3d
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
        else:
            ctx.setdefault("needs", {})["double_floor"] = True
            r.warn(msg + " → 보정 단계에서 고침")
    if lay["under_pct"] > 1.0:
        r.warn(f"바닥 3~30 cm 아래에 면이 있는 자리 {lay['under_3_to_30cm']}곳 ({lay['under_pct']}%)")
    vp = os.path.join(ctx["build"], "visual.ply")               # 보이는 바닥 ↔ 충돌 바닥
    if os.path.exists(vp):
        vis = o3d.io.read_triangle_mesh(vp)
        sc = o3d.t.geometry.RaycastingScene()
        sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(vis))
        rays = np.c_[xy, zf + 1.0, np.zeros(len(xy)), np.zeros(len(xy)), -np.ones(len(xy))].astype(np.float32)
        h = sc.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
        dz = 1.0 - h
        ok = np.isfinite(h) & (np.abs(dz) < 0.3)
        m.update(visible_floor_missing_pct=round(100 * float((~ok).mean()), 1),
                 visible_minus_collision_cm=dict(p5=round(100 * float(np.percentile(dz[ok], 5)), 1), median=round(100 * float(np.median(dz[ok])), 1), p95=round(100 * float(np.percentile(dz[ok], 95)), 1)) if ok.any() else None)
        if ok.any() and max(abs(np.percentile(dz[ok], 5)), abs(np.percentile(dz[ok], 95))) > 0.05:
            r.warn(f"보이는 바닥과 충돌 바닥의 높이 차가 5 cm 를 넘는 곳이 있음 (5~95%: {m['visible_minus_collision_cm']['p5']} ~ {m['visible_minus_collision_cm']['p95']} cm)")
        if (~ok).mean() > 0.10:
            r.warn(f"빈 바닥의 {m['visible_floor_missing_pct']}% 에 보이는 바닥이 없음 (받침 바닥으로 메워짐)")
    p = ctx["profile"]
    m["has_ceiling"] = bool(p.get("has_ceiling", False))
    if m["has_ceiling"]:
        r.messages.append("천장이 있음 — 씬 루트에 그림자 설정을 넣음 (안 넣으면 Isaac Sim 화면 밝기 0)")
    r.summary = f"빈 바닥 표본 {lay['samples']:,} · 바닥 한 겹 {lay['single_pct']}% · 두 겹 {lay['double_within_3cm']}" + (f" · 보이는 바닥 없음 {m['visible_floor_missing_pct']}%" if "visible_floor_missing_pct" in m else "")
    return r


def s_correct(ctx):
    """보정 (논문 3.2 의 1차 보정과 같은 생각): 진단에 걸린 항목만 고쳐 다시 만들고 다시 진단한다.
    두 겹 바닥 → 충돌 메쉬에서 바닥 위 5 cm 아래의 것을 지움 (바닥 한 장과 그보다 5 cm 넘게 솟은 형상만 남김)."""
    needs = ctx.get("needs") or {}
    if not needs:
        return Result(SKIP, "진단에 걸린 항목이 없어 보정하지 않음")
    if ctx["kind"] != "glb":
        return Result(FAIL, "이 입력 종류의 자동 보정은 아직 없음", messages=[f"걸린 항목: {', '.join(needs)}"])
    before = next(d for d in ctx["results"] if d["stage"] == "diagnose")["metrics"]
    cfg2 = os.path.join(os.path.dirname(ctx["config"]), "auto_config_corrected.yaml")
    lines = [f"# scan2sim 보정: 진단에 걸린 항목({', '.join(needs)})을 고친 설정", f"base: {os.path.basename(ctx['config'])}",
             "collision:", "  low_h_m: 0.05", "  min_cluster_tris_after: 100", ""]
    open(cfg2, "w", encoding="utf-8", newline=chr(10)).write(chr(10).join(lines))
    ctx["config"], ctx["corrected"] = cfg2, True
    for nm, fn in (("convert", s_convert), ("nav2", s_nav2)):
        r = fn(ctx)
        if r.status == FAIL:
            return Result(FAIL, f"보정 뒤 {nm} 단계 실패: {r.summary}", messages=r.messages)
    r = s_diagnose(ctx)
    after = r.metrics
    ctx["diagnose_before"], ctx["diagnose_after"] = before, after
    out = Result(r.status, f"두 겹 바닥 {before.get('floor_double')} → {after.get('floor_double')} · 바닥 한 겹 {before.get('floor_single_pct')} → {after.get('floor_single_pct')}%",
                 metrics=dict(fixed=list(needs), before=before, after=after), messages=["충돌 메쉬에서 바닥 위 5 cm 아래의 것을 지움 (보이는 메쉬는 그대로)"] + r.messages)
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
    shadow = "".join(f'        over "{p}"\n        {{\n            bool primvars:doNotCastShadows = 1\n        }}\n\n' for p in vis) if ctx["profile"].get("has_ceiling") else ""
    usda = f'''#usda 1.0
(
    customLayerData = {{
        string map = "scan2sim 자동 생성 — 입력 {os.path.basename(os.path.normpath(ctx['input']))}"
        string start_pose = "빈 바닥에서 장애물과 가장 먼 자리 ({pose['x']}, {pose['y']}) · {pose['yaw_deg']} 도. 바닥 {z0:.3f} + 0.022"
        string lights = "돔 600 · 해 0{' · 천장이 있어 보이는 prim 에 doNotCastShadows (없으면 화면 밝기 0)' if shadow else ''}"
    }}
    defaultPrim = "World"
    doc = "scan2sim 이 만든 씬 루트 제안 — sim/scenes/<장소>/ 에 두고, 맵 경로를 서버 위치에 맞출 것. Isaac Sim 에서 열어 본 것이 아님"
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
    open(os.path.join(b, "scene_root.usda"), "w", encoding="utf-8", newline="\n").write(usda)
    files = sorted(os.path.relpath(os.path.join(dp_, f), b).replace(os.sep, "/") for dp_, _, fs in os.walk(b) for f in fs if f != ".keep")
    json.dump(dict(name=ctx["name"], input=os.path.basename(os.path.normpath(ctx["input"])), kind=ctx["kind"], files=files), open(os.path.join(b, "bundle.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    ctx["bundle"] = b
    r = Result(PASS, f"묶음 파일 {len(files)}개 · 시작 ({pose['x']}, {pose['y']}) {pose['yaw_deg']}° · 여유 {pose['clearance_m']} m", metrics=dict(start_pose=pose, files=len(files), shadow_overrides=len(vis) if shadow else 0))
    if d[k] < 0.5:
        r.warn(f"가장 넓은 자리의 여유가 {d[k]:.2f} m 뿐임 — 로봇이 다니기 좁음")
    return r


def s_verify(ctx):
    """주행 검증 (논문 3.3): Isaac Sim 에서 Nav2 · E2E 모델로 주행 — D · E 단계의 자리."""
    return Result(SKIP, "Isaac Sim 이 필요함 — 묶음을 서버에 올려 D · E 가 주행으로 검증 (결과 파일이 오면 여기에 붙임)")


def s_report(ctx):
    """단계별 판정을 한 장으로."""
    res = ctx.get("results", [])
    worst = FAIL if any(d["status"] == FAIL for d in res) else WARN if any(d["status"] == WARN for d in res) else PASS
    verdict = {PASS: "시뮬레이션에 올릴 수 있음 (주행 검증 전)", WARN: "올릴 수 있으나 아래 주의 항목을 볼 것", FAIL: "쓸 수 없음 — 아래 실패 단계 참고"}[worst]
    L = [f"# scan2sim 보고서 — {ctx.get('name', '?')}", "", f"- 입력: `{os.path.basename(os.path.normpath(ctx['input']))}` ({ctx.get('kind') or '알 수 없음'})", f"- 만든 때: {time.strftime('%Y-%m-%d %H:%M')}",
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
    if ctx.get("bundle"):
        L += ["", "## 묶음", "", f"- 위치: `bundle/` — `usd/` · `nav2/` · `scene_root.usda` · `start_pose.json` · `route.csv`", "- Isaac Sim 에서 열어 본 것이 아님. 주행 검증은 서버에서."]
    L += ["", "## 주의사항", "", "- 움직이는 물체(사람 · 차)는 스캔한 순간의 모습으로 굳어 있음", "- 유리 · 거울 · 광택 면은 형상이 틀릴 수 있음", "- 맵의 절대 크기는 줄자 실측과 대조하지 않았음", "- 넓은 실외는 대상이 아님 (스캔이 수십 cm 틀어짐)", ""]
    open(ctx.path("report.md"), "w", encoding="utf-8", newline="\n").write("\n".join(L))
    json.dump(dict(name=ctx.get("name"), verdict=worst, stages=res), open(ctx.path("report.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=str)
    return Result(worst if worst != FAIL else PASS, f"report.md — {verdict}")


STAGES = [("input", s_input), ("convert", s_convert), ("nav2", s_nav2), ("diagnose", s_diagnose), ("correct", s_correct), ("bundle", s_bundle), ("verify", s_verify), ("report", s_report)]
