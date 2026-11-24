"""아이폰 LiDAR 원본(Stray Scanner)을 바꾼 맵 폴더 → 깊이를 합친 형상 메쉬. B 의 단계 — stages.py 가 pipeline/tsdf.py 대신 부름.

    cd recon
    python -m scan2sim.stray_mesh <맵 폴더> --out <빌드 폴더> [--voxel 0.02] [--depth-trunc 3.0] [--image-step 1]
                                  [--fix floor+xy|floor|none] [--no-fix] [--tilt | --no-tilt]

입력 (stray.py 가 쓴 맵 폴더 = pipeline/io_rgbd.load_rtabmap_keyframes 가 읽는 모양)
    trajectory_camera_optical.txt   `시각 tx ty tz qx qy qz qw id` — 카메라 광학 좌표계 → 세계 (Z 가 위)
    keyframes/rgb/<id>.jpg · depth/<id>.png (16비트 mm, 색과 같은 크기) · calib/<id>.yaml

출력 (<빌드 폴더>/ — pipeline/tsdf.py 와 같은 이름 · 같은 좌표 · 같은 키)
    mesh_raw.ply     TSDF 메쉬 (정점 색). 좌표 = 맵 폴더 세계(Z 위) 그대로 — T_input_to_scene 의 입력 좌표 (docs/CONTRACT.md 1장)
    tsdf_meta.json   tsdf.py 와 같은 키: tsdf(설정 — stray_quality 가 depth_trunc 를 읽음) · frames_total · frames_used · seconds ·
                     vertices · triangles · far_triangles · up_hint_camera(build_usd.py 가 읽음).
                     더한 키: builder · map_root · poses_rows · frames_skipped · timing(단계별 초) · peak_rss_mb ·
                     pose_fix (자세 보정 요약 — stray_fix.summary: name · file · method "chain" · fix · order · applied · status · reasons ·
                     flags · keyframes · seconds{floor, xy, total} · steps[단계마다: kind · applied · status · reasons · flags · 높이는 dz_p5_p95_cm ·
                     dz_max_abs_cm · resid_rms_cm · frames_supported_pct · stand_other_pct · jump_frames · tilt_deg_p50_p95 · tilt_max_deg ·
                     tilt_big_frames · 수평은 axes_applied · manhattan · axes ·
                     shift_cm_p50_p95] · moved_m · moved_xy_m · moved_z_m · turned_deg{median, max})
더한 출력 (<맵 폴더>/ — 맵 폴더의 다른 파일은 건드리지 않음, trajectory_camera_optical.txt 도 원래 자세 그대로)
    pose_fix.json    자세 보정 info 전체 (stray_fix.py 머리말 "info" — steps 차례대로). 보정을 꺼도 status "off" 로 씀 — 예전 결과가 남아
                     stray_quality 가 안 쓴 프레임에 엉뚱한 보정을 하지 않게. stray_quality 는 applied 인 단계만 같은 보정(stray_fix.apply_fix)을
                     안 쓴 프레임에 적용. 11/24 af65f1b 판이 쓰던 floor_fix.json 이 있으면 지움 (그 판으로 만든 맵은 stray_quality 가 계속 읽음)

자세 보정 (기본 켬 — fix="floor+xy", --fix floor 는 높이만, --no-fix 로 끔)
    합치기 전에 키프레임 자세의 높이 틀어짐을 바닥으로, 이어서 수평 틀어짐을 벽으로 바로잡음 (stray_fix.fix_poses). 단계마다 관문에 걸리면
    (근거 부족 · 맨해튼이 아닌 방 · 고칠 것 없음 등) 그 단계는 자세를 그대로 두고 이유를 남김. 보정 중 오류가 나도 메쉬는 만듦 (그 단계 status "error").

pipeline/tsdf.py 와 같은 것
    자세 읽기(사원수 → 행렬, 같은 Open3D 함수) · 그림 없는 줄 빼기 · 내부 파라미터가 한 종류인지 · 해상도 확인 · 깊이 mm → m ·
    깊이 한계 · 칸 · sdf 폭 · Open3D ScalableTSDFVolume(색 RGB8) · up_hint_camera. 자세 보정을 끄거나 관문에서 건너뛰면 같은 메쉬
    (11/24 가상의 방 193장 · 기준 촬영본 681장: mesh_raw.ply 파일이 바이트까지 같음). 보정을 적용하면 자세가 바뀌므로 메쉬도 다름
다른 것
    - 그림을 cv2 로 읽고 다음 장을 미리 읽어 둠 (스레드 2개 · 4장 앞까지). cv2 는 읽는 동안 GIL 을 놓아 합치기와 겹침.
      화소 값은 Open3D 로 읽은 것과 같음 (JPEG · PNG 모두 확인). 미리 읽은 그림만 메모리에 둠 (장당 약 3.5 MB)
    - 원거리 볼륨(far_depth_trunc)이 없음 — stray.py 가 깊이를 5 m 에서 자르고 Stray 경로는 mesh_far_raw.ply 를 안 씀.
      예전 것이 남아 있으면 지움 (tsdf.py 가 끈 경우와 같음)
    - 설정 파일 없이 인자로 받음. frame_step 은 늘 1 (키프레임은 stray.py 가 이미 고름)
    - 합치기 전에 자세를 고침 (위 "자세 보정"). pose_fix= 로 다른 보정 함수를 주면 그것을 씀 — 돌려받은 info 는 pose_fix.json 에,
      요약과 자세가 옮겨진 양은 tsdf_meta.json 의 pose_fix 에
    - mesh_raw.ply 를 임시 이름으로 쓰고 끝에 바꿔 놓음 — 중간에 멈춰도 덜 쓴 파일을 --resume 이 다시 쓰지 않게.
      tsdf_meta.json 을 먼저 쓰므로 mesh_raw.ply 가 있으면 tsdf_meta.json 도 있음
    - 쓸 장이 없음 · 해상도가 다름 · 메쉬가 빔 → MeshError (한국어 문장, stray.CaptureError 의 한 종류)
"""
import argparse
import collections
import json
import os
import re
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from . import stray, stray_fix

FIX_FILE = "pose_fix.json"                                      # 맵 폴더에 쓰는 자세 보정 info (단계 묶음 — stray_quality 가 읽음)
OLD_FIX_FILE = "floor_fix.json"                                 # 11/24 af65f1b 판이 쓰던 높이 info — 새로 만들면 지움 (stray_quality 는 예전 맵에서 계속 읽음)
READERS, AHEAD = 2, 4                                           # 그림을 미리 읽는 스레드 · 장 수 (11/24 기준 촬영본 200장: 스레드 없이 14.8 · 2개 10.5 ms/장, tsdf.py 18.7)
DEPTH_SCALE = 1000.0                                            # 깊이 PNG 값 / 1000 = m (stray.py 가 mm 로 씀)


class MeshError(stray.CaptureError):
    """형상 메쉬를 만들 수 없음 — 사용자에게 보일 문구. stray.CaptureError 를 받는 곳에서 같이 받힘"""


def _calib(path):
    """calib/<id>.yaml (OpenCV 형식) → (w, h, fx, fy, cx, cy). io_rgbd.read_calib_yaml 과 같은 읽기"""
    s = open(path, encoding="utf-8").read()
    k = [float(x) for x in re.search(r"camera_matrix:.*?data:\s*\[([^\]]+)\]", s, re.S).group(1).replace("\n", " ").split(",")]
    w, h = (int(re.search(rf"{key}:\s*(\d+)", s).group(1)) for key in ("image_width", "image_height"))
    return w, h, k[0], k[4], k[2], k[5]


def read_map(map_root):
    """맵 폴더 → dict(times, poses (N×4×4, 카메라 광학 → 세계), ids, color, depth (그림 경로), intrinsic (w, h, fx, fy, cx, cy), rows).
    그림(rgb · depth)이 없는 줄은 뺌 (io_rgbd 와 같음). 그림은 읽지 않음"""
    import open3d as o3d
    tp = os.path.join(map_root, "trajectory_camera_optical.txt")
    kf = os.path.join(map_root, "keyframes")
    if not os.path.isfile(tp) or not os.path.isdir(kf):
        raise MeshError(f"맵 폴더가 아님 ({map_root}) — keyframes/ 와 trajectory_camera_optical.txt 가 있어야 함")
    times, poses, ids, color, depth, calib, rows = [], [], [], [], [], set(), 0
    for ln, line in enumerate(open(tp, encoding="utf-8"), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        rows += 1
        try:
            v = [float(x) for x in line.split()]
            i = int(v[8])
        except (ValueError, IndexError):
            raise MeshError(f"trajectory_camera_optical.txt {ln}번째 줄을 읽지 못함 — `시각 tx ty tz qx qy qz qw id` 9칸이어야 함")
        c, d = os.path.join(kf, "rgb", f"{i}.jpg"), os.path.join(kf, "depth", f"{i}.png")
        if not (os.path.exists(c) and os.path.exists(d)):
            continue
        T = np.eye(4)                                           # io_rgbd.tum_to_matrix 와 같은 계산 (같은 Open3D 함수 — 메쉬가 바이트까지 같게)
        T[:3, :3] = o3d.geometry.get_rotation_matrix_from_quaternion([v[7], v[4], v[5], v[6]])
        T[:3, 3] = v[1:4]
        times.append(v[0]), poses.append(T), ids.append(i), color.append(c), depth.append(d)
        cp = os.path.join(kf, "calib", f"{i}.yaml")
        if not os.path.exists(cp):
            raise MeshError(f"키프레임 {i} 의 내부 파라미터(keyframes/calib/{i}.yaml)가 없음")
        calib.add(_calib(cp))
    if not ids:
        raise MeshError(f"쓸 키프레임이 없음 — 자세 {rows}줄 중 keyframes/rgb · depth 그림이 둘 다 있는 줄이 없음")
    if len(calib) != 1:
        raise MeshError(f"키프레임마다 내부 파라미터가 다름 ({len(calib)}종) — 한 종류만 지원")
    return dict(times=np.array(times), poses=np.array(poses), ids=ids, color=color, depth=depth, intrinsic=calib.pop(), rows=rows)


def _json_safe(o):
    return o.tolist() if hasattr(o, "tolist") else str(o)


def _peak_rss_mb():
    """이 프로세스가 지금까지 쓴 가장 큰 메모리 (MB, 리눅스). 윈도우처럼 잴 수 없으면 None"""
    try:
        import resource
    except ImportError:
        return None
    return round(resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024, 1)


def _write_json(path, obj):
    """임시 이름으로 쓰고 바꿔 놓음 — 중간에 멈춰도 덜 쓴 파일이 남지 않게"""
    tmp = path + ".writing"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1, default=_json_safe)
    os.replace(tmp, path)


def _fix_poses(fr, poses, fix, fix_opts, pose_fix, log):
    """합치기 전 자세 보정 → (새 자세, info, 이름). fix = stray_fix.FIXES 의 이름 ("floor+xy" · "floor") · None = 끔 ·
    pose_fix = 다른 보정 함수 (info 를 그대로 씀)"""
    if pose_fix is not None:
        new, info = pose_fix(fr["times"].copy(), poses.copy())
        name = getattr(pose_fix, "__name__", type(pose_fix).__name__)
        if not isinstance(info, dict):
            info = dict(applied=None, status="custom", reasons=[], flags=[], info=info)
        return new, {k: v for k, v in info.items() if not str(k).startswith("_")}, name
    if fix in (None, False, "none", "off"):
        log("[mesh] 자세 보정 끔 — 자세 그대로 합침")
        return poses, stray_fix.chain_info([], fix="none", status="off", reason="자세 보정을 끔 (fix=None · --no-fix)", keyframes=len(poses)), None
    if fix not in stray_fix.FIXES:
        raise ValueError(f"fix 는 {sorted(stray_fix.FIXES)} 중 하나 또는 None 이어야 함: {fix!r}")
    steps = stray_fix.FIXES[fix]
    per = stray_fix.split_opts(steps, fix_opts)                 # 설정이 잘못되면 ValueError — 조용히 넘기지 않음
    new, info = stray_fix.fix_poses(fr["times"], poses, fr["depth"], fr["intrinsic"][2:], steps, per, log=log, fix=fix)
    return new, info, fix                                       # 단계마다 오류는 fix_poses 가 받아 status "error" 로 남김 — 메쉬는 계속 만듦


def build(map_root, build_dir, voxel=0.02, depth_trunc=3.0, sdf_trunc_voxels=4, image_step=1, pose_fix=None, log=print,
          fix=stray_fix.DEFAULT_FIX, fix_opts=None):
    """맵 폴더 → <build_dir>/mesh_raw.ply · tsdf_meta.json (+ <map_root>/pose_fix.json). → meta dict (tsdf_meta.json 과 같음). 만들 수 없으면 MeshError.
    fix = 합치기 전 자세 보정: "floor+xy" (기본 — 높이를 바닥으로 고친 뒤 수평을 벽으로, stray_fix.fix_poses) · "floor" (높이만) · None (끔, 자세 그대로).
    fix_opts = 단계별 설정 {"floor": {...}, "xy": {...}} (예: {"floor": dict(tilt=True)}). 예전 꼴 dict(tilt=True) 는 높이 설정으로 받음.
    pose_fix(times, poses) → (poses, info): 다른 보정 함수 (주면 fix 보다 먼저 씀). 자세는 N×4×4, 카메라 광학 → 세계.
    image_step = 그림을 이 화소마다 하나씩만 읽음 (2 = 절반 크기 — 빠르지만 메쉬가 조금 달라짐, docs/C_TRACK.md)"""
    import cv2
    import open3d as o3d
    if voxel <= 0 or depth_trunc <= 0 or sdf_trunc_voxels <= 0 or int(image_step) < 1:
        raise ValueError(f"설정이 잘못됨: voxel {voxel} · depth_trunc {depth_trunc} · sdf_trunc_voxels {sdf_trunc_voxels} · image_step {image_step}")
    t0 = time.time()
    tm = {}
    map_root, build_dir = os.path.abspath(map_root), os.path.abspath(build_dir)
    os.makedirs(build_dir, exist_ok=True)
    fr = read_map(map_root)
    poses, n = fr["poses"], len(fr["ids"])
    tm["read_map_s"] = round(time.time() - t0, 2)

    t = time.time()
    new, info, name = _fix_poses(fr, poses, fix, fix_opts, pose_fix, log)
    new = np.asarray(new, dtype=float)
    if new.shape != poses.shape or not np.all(np.isfinite(new)):
        raise MeshError(f"자세 보정이 잘못된 자세를 돌려줌 (모양 {new.shape}, 받은 것 {poses.shape}) — 보정 없이 다시 만들 것 (--no-fix)")
    dp = new[:, :3, 3] - poses[:, :3, 3]
    moved, moved_xy, moved_z = np.linalg.norm(dp, axis=1), np.linalg.norm(dp[:, :2], axis=1), np.abs(dp[:, 2])
    turn = np.degrees(np.arccos(np.clip((np.einsum("nij,nij->n", new[:, :3, :3], poses[:, :3, :3]) - 1) / 2, -1, 1)))
    _write_json(os.path.join(map_root, FIX_FILE), info)
    if os.path.exists(os.path.join(map_root, OLD_FIX_FILE)):
        os.remove(os.path.join(map_root, OLD_FIX_FILE))          # 예전 판이 쓴 높이 info — 남으면 이 메쉬와 다른 보정으로 읽힐 수 있음
    dist = lambda x, nd: dict(median=round(float(np.median(x)), nd), max=round(float(x.max()), nd))
    pf = dict(name=name, file=FIX_FILE, **stray_fix.summary(info),
              moved_m=dist(moved, 4), moved_xy_m=dist(moved_xy, 4), moved_z_m=dist(moved_z, 4), turned_deg=dist(turn, 3))
    if info.get("applied") is None:                             # 다른 보정 함수가 applied 를 안 알려 줌 — 자세가 움직였는지로
        pf["applied"] = bool(moved.max() > 0 or turn.max() > 0)
    poses = new
    tm["pose_fix_s"] = round(time.time() - t, 2)

    w, h, fx, fy, cx, cy = fr["intrinsic"]
    step = int(image_step)
    intr = o3d.camera.PinholeCameraIntrinsic(len(range(0, w, step)), len(range(0, h, step)), fx / step, fy / step, cx / step, cy / step)
    p = dict(frame_step=1, voxel=voxel, sdf_trunc_voxels=sdf_trunc_voxels, depth_trunc=depth_trunc, far_depth_trunc=0, image_step=step)
    log(f"[mesh] 키프레임 {n}장 (자세 {fr['rows']}줄) | 칸 {voxel * 100:g} cm | 깊이 {depth_trunc} m 까지" + (f" | 그림 1/{step} 크기" if step > 1 else ""))

    def load(k):                                                # 스레드에서 읽음 — 장마다 해상도 확인 (tsdf.py 는 첫 장만)
        c = cv2.imread(fr["color"][k], cv2.IMREAD_COLOR)
        d = cv2.imread(fr["depth"][k], cv2.IMREAD_UNCHANGED)
        if c is None or d is None:
            raise MeshError(f"키프레임 {fr['ids'][k]} 의 그림을 읽지 못함 — 파일이 깨졌을 수 있음")
        if c.shape[:2] != (h, w) or d.shape[:2] != (h, w) or d.dtype != np.uint16:
            raise MeshError(f"키프레임 {fr['ids'][k]}: 해상도 불일치 — 내부 파라미터 {(h, w)}, 색 {c.shape[:2]}, 깊이 {d.shape[:2]} {d.dtype} (깊이는 색에 맞춘 16비트 mm 여야 함)")
        c = cv2.cvtColor(c, cv2.COLOR_BGR2RGB)
        if step > 1:
            c, d = c[::step, ::step], d[::step, ::step]
        return np.ascontiguousarray(c), np.ascontiguousarray(d)

    vol = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel, sdf_trunc=sdf_trunc_voxels * voxel,
                                                       color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    t = time.time()
    wait = 0.0
    with ThreadPoolExecutor(READERS) as ex:
        q = collections.deque(ex.submit(load, k) for k in range(min(AHEAD, n)))
        for k in range(n):
            tw = time.time()
            c, d = q.popleft().result()
            wait += time.time() - tw
            if k + AHEAD < n:
                q.append(ex.submit(load, k + AHEAD))
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(c), o3d.geometry.Image(d), depth_scale=DEPTH_SCALE,
                                                                      depth_trunc=depth_trunc, convert_rgb_to_intensity=False)
            vol.integrate(rgbd, intr, np.linalg.inv(poses[k]))  # 자세(카메라 → 세계)의 역 = extrinsic
            if (k + 1) % 100 == 0:
                log(f"  {k + 1}/{n} ({time.time() - t:.0f}s)")
    tm["integrate_s"], tm["read_wait_s"] = round(time.time() - t, 2), round(wait, 2)
    te = time.time()
    mesh = vol.extract_triangle_mesh()
    del vol
    tm["extract_s"] = round(time.time() - te, 2)
    sec = time.time() - t
    if not len(mesh.triangles):
        raise MeshError(f"깊이를 {n}장 합쳤지만 메쉬가 비었음 — 깊이가 {depth_trunc} m 안에 거의 없거나 자세가 틀렸음")

    up = -poses[:, :3, 1].mean(axis=0)                          # 카메라 화면 아래(+y) 방향 평균의 반대 = 대략적인 위쪽 (tsdf.py 와 같음)
    meta = dict(config=None, builder="scan2sim.stray_mesh", map_root=map_root, tsdf=p, frames_total=n, frames_used=n,
                seconds=round(sec, 1), vertices=len(mesh.vertices), triangles=len(mesh.triangles), far_triangles=0,
                up_hint_camera=(up / np.linalg.norm(up)).tolist(), poses_rows=fr["rows"], frames_skipped=fr["rows"] - n,
                frame="맵 폴더 세계 (Z 위) — trajectory_camera_optical.txt 와 같은 좌표", pose_fix=pf)
    path = os.path.join(build_dir, "mesh_raw.ply")
    tmp = os.path.join(build_dir, "mesh_raw.writing.ply")
    tw = time.time()
    o3d.io.write_triangle_mesh(tmp, mesh)
    far = os.path.join(build_dir, "mesh_far_raw.ply")
    if os.path.exists(far):
        os.remove(far)                                          # 예전 결과가 섞이지 않게 (tsdf.py 와 같음)
    tm["write_s"] = round(time.time() - tw, 2)
    tm["total_s"] = round(time.time() - t0, 2)
    meta.update(timing=tm, peak_rss_mb=_peak_rss_mb())
    json.dump(meta, open(os.path.join(build_dir, "tsdf_meta.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=_json_safe)
    os.replace(tmp, path)
    log(f"[mesh] 삼각형 {meta['triangles']:,} | {sec:.0f}s (그림 기다림 {wait:.1f}s · 메쉬 꺼내기 {tm['extract_s']}s) → {path}")
    return meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map", help="맵 폴더 (stray.py 출력: keyframes/ · trajectory_camera_optical.txt)")
    ap.add_argument("--out", required=True, help="빌드 폴더 (mesh_raw.ply · tsdf_meta.json 을 씀)")
    ap.add_argument("--voxel", type=float, default=0.02, help="칸 크기 m (기본 0.02)")
    ap.add_argument("--depth-trunc", type=float, default=3.0, help="이보다 먼 깊이는 버림 m (기본 3.0)")
    ap.add_argument("--sdf-trunc-voxels", type=int, default=4, help="sdf 폭 = 칸 × 이 값 (기본 4)")
    ap.add_argument("--image-step", type=int, default=1, help="그림을 이 화소마다 하나씩만 읽음 (기본 1 = 다 읽음, 2 = 절반 크기)")
    ap.add_argument("--fix", choices=sorted(stray_fix.FIXES) + ["none"], default=stray_fix.DEFAULT_FIX,
                    help=f"합치기 전 자세 보정 (기본 {stray_fix.DEFAULT_FIX} = 높이 → 수평, floor = 높이만, none = 끔)")
    ap.add_argument("--no-fix", action="store_true", help="자세 보정(stray_fix)을 끔 = --fix none — 자세 그대로 합침 (pipeline/tsdf.py 와 같은 메쉬)")
    ap.add_argument("--tilt", action=argparse.BooleanOptionalAction, default=None,
                    help=f"높이 보정에서 바닥 기울기 오차도 풂 (기본 {'켬' if stray_fix.DEFAULTS['tilt'] else '끔'})")
    a = ap.parse_args()
    fix = None if a.no_fix or a.fix == "none" else a.fix
    opts = {"floor": dict(tilt=a.tilt)} if fix and a.tilt is not None else None
    try:
        build(a.map, a.out, a.voxel, a.depth_trunc, a.sdf_trunc_voxels, a.image_step, fix=fix, fix_opts=opts)
    except (stray.CaptureError, ValueError) as e:
        sys.exit(f"[mesh] 실패: {e}")


if __name__ == "__main__":
    main()
