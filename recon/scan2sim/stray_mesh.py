"""아이폰 LiDAR 원본(Stray Scanner)을 바꾼 맵 폴더 → 깊이를 합친 형상 메쉬. B 의 단계 — stages.py 가 pipeline/tsdf.py 대신 부름.

    cd recon
    python -m scan2sim.stray_mesh <맵 폴더> --out <빌드 폴더> [--voxel 0.02] [--depth-trunc 5.0] [--image-step 1]
                                  [--fix auto|xy|floor+xy|floor|none] [--no-fix] [--tilt | --no-tilt] [--capture <촬영 폴더 · zip>]

입력 (stray.py 가 쓴 맵 폴더 = pipeline/io_rgbd.load_rtabmap_keyframes 가 읽는 모양)
    trajectory_camera_optical.txt   `시각 tx ty tz qx qy qz qw id` — 카메라 광학 좌표계 → 세계 (Z 가 위)
    keyframes/rgb/<id>.jpg · depth/<id>.png (16비트 mm, 색과 같은 크기) · calib/<id>.yaml

출력 (<빌드 폴더>/ — pipeline/tsdf.py 와 같은 이름 · 같은 좌표 · 같은 키)
    mesh_raw.ply     TSDF 메쉬 (정점 색). 좌표 = 맵 폴더 세계(Z 위) 그대로 — T_input_to_scene 의 입력 좌표 (docs/CONTRACT.md 1장)
    tsdf_meta.json   tsdf.py 와 같은 키: tsdf(설정 — stray_quality 가 depth_trunc 를 읽음) · frames_total · frames_used · seconds ·
                     vertices · triangles · far_triangles · up_hint_camera(build_usd.py 가 읽음).
                     더한 키: builder · map_root · poses_rows · frames_skipped · timing(단계별 초) · peak_rss_mb ·
                     pose_fix (자세 보정 요약 — stray_fix.summary: name · file ("pose_fix.json" — 빌드 폴더) · sha256 (그 파일 바이트의 sha256) ·
                     method "chain" · fix · order · applied · status · reasons ·
                     flags · keyframes · seconds{floor, xy, total} · steps[단계마다: kind · applied · status · reasons · flags · 높이는 dz_p5_p95_cm ·
                     dz_max_abs_cm · resid_rms_cm · frames_supported_pct · stand_other_pct · jump_frames · tilt_deg_p50_p95 · tilt_max_deg ·
                     tilt_big_frames · 수평은 axes_applied · manhattan · axes ·
                     shift_cm_p50_p95] · moved_m · moved_xy_m · moved_z_m · turned_deg{median, max} ·
                     auto (fix="auto" 일 때만 — 아래 "자세 보정"))
                     timing 에 auto_s (자동 판단에 쓴 초 — auto 가 아니면 0)
    pose_fix.json    자세 보정 info 전체 (stray_fix.py 머리말 "info" — steps 차례대로, auto 면 auto 도). 보정을 꺼도 status "off" 로 씀.
                     stray_quality 는 이 파일의 sha256 이 tsdf_meta.json 의 pose_fix.sha256 과 같을 때만 applied 인 단계의 보정(stray_fix.apply_fix)을
                     안 쓴 프레임에 적용 — 다른 보정으로 다시 만들다 멈춘 빌드에 엉뚱한 보정을 하지 않게
    쓰는 차례: 합치기 → tsdf_meta.json → mesh_raw.ply → pose_fix.json (모두 임시 이름으로 쓰고 바꿔 놓음). 합치기 전에는 아무것도 쓰지 않음
더한 출력 (<맵 폴더>/ — 되면. 맵 폴더의 다른 파일은 건드리지 않음, trajectory_camera_optical.txt 도 원래 자세 그대로)
    pose_fix.json    빌드 폴더의 것과 같은 바이트 (예전에 여기서 읽던 것을 위해). 쓸 수 없으면 (읽기 전용 맵 폴더) 건너뛰고 한 줄 남김.
                     11/24 af65f1b 판이 쓰던 floor_fix.json 이 있으면 지움 (그 판으로 만든 맵은 stray_quality 가 계속 읽음)

자세 보정 (기본 fix="auto" — 엔진이 부르는 길. --fix xy 는 수평만, floor+xy 는 높이 → 수평, floor 는 높이만, --no-fix 로 끔)
    합치기 전에 키프레임 자세의 수평 틀어짐을 벽으로 (높이 단계를 켜면 그 전에 높이 틀어짐을 바닥으로) 바로잡음 (stray_fix.fix_poses).
    높이 단계는 11/24 밤 hf 깊이 규칙 · 5 m 로 다시 재 보니 방 촬영본에서 천장을 나쁘게 하고 바닥 이득이 작아 늘 켜지는 않음 (stray_fix.py 머리말 "측정").
    auto (11/25 결정): 합치기 전에 촬영 원본의 원래 자세(ARKit)로 되돌아온 자리 어긋남을 재서 (stray_revisit.measure 의 raw — 3~13 초 ·
      0.9 GB 이하) 믿을 쌍이 있고 높이 어긋남 최대가 AUTO_FLOOR_Z_CM (15 cm = stray_revisit 의 높이 실패 기준) 을 넘으면 floor+xy,
      아니면 xy (= stray_fix.DEFAULT_FIX — 그때 메쉬는 --fix xy 와 바이트까지 같음). 촬영 원본을 못 찾거나 재다 오류가 나면 xy 와 이유 —
      auto 때문에 빌드가 멈추지는 않음. 근거: 복도 왕복 922acd4755 (원래 자세 높이 어긋남 44 cm) 엔진 막힌 길 xy 35.8% → floor+xy 0.0%
      (안 쓴 프레임 6.13 → 3.49 cm, 천장 나빠지지 않음) · 방 한 바퀴 6bc1ae021f (11 cm) 는 xy 그대로.
      촬영 원본 찾기 (find_capture): capture= (--capture) 를 주면 그것만, 아니면 <맵 폴더>/_unzip/<stray_meta.json 의 source> → _unzip 아래
      3 겹 안의 <source> 폴더 (macOS zip 의 outer/cap 꼴) → <맵 폴더>/_unzip (엔진 · stray CLI 가 zip 을 푼 곳). 이름이 source 와 다르거나
      odometry.csv 줄 수가 frames_total 과 다르면 받지 않음. 촬영 폴더를 그대로 넣어 만든 맵은 _unzip 이 없어 --capture 를 줘야 잼 (안 주면 xy).
      프레임 간격 = max(0.1 초, 촬영 초 / 3000) — 긴 촬영도 3000장 근처 (14 분 촬영 ≈ 1.4 GB · 20 초). 6bc · 922 는 0.1 그대로.
      남기는 것: pose_fix.auto = {chosen · threshold_cm · measured · source · verdict · pairs · reliable · z_max_cm · h_max_cm · every_s · reason ·
      seconds (· unused_opts)}, fix · name = 고른 이름. 높이 단계를 켰으면 flags 맨 앞에 한 줄 (엔진이 주의로 띄움 — 켜서 고쳤는지 · 옆 어긋남이 남는지)
    --tilt / --no-tilt 는 높이 단계의 설정 — --fix 를 주지 않고 쓰면 재지 않고 높이 단계를 켜 floor+xy 로 돎 (예전과 같음).
      --fix auto 와 같이 주면 auto 가 높이 단계를 켤 때만 씀. --fix xy · --no-fix 와 같이 주면 오류. build(fix="auto", fix_opts=…) 도 같음 —
      고른 보정에 없는 단계의 설정은 쓰지 않고 auto.unused_opts 에 남김 (설정 이름이 틀리면 재기 전에 ValueError)
    단계마다 관문에 걸리면 (근거 부족 · 맨해튼이 아닌 방 · 고칠 것 없음 등) 그 단계는 자세를 그대로 두고 이유를 남김.
    보정 중 오류가 나도 메쉬는 만듦 (그 단계 status "error").

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
    - 합치기 전에 자세를 고침 (위 "자세 보정"). pose_fix= 로 다른 보정 함수를 주면 그것을 씀 — 돌려받은 info 는 <빌드 폴더>/pose_fix.json 에,
      요약과 자세가 옮겨진 양은 tsdf_meta.json 의 pose_fix 에
    - mesh_raw.ply · tsdf_meta.json 을 임시 이름으로 쓰고 끝에 바꿔 놓음 — 중간에 멈춰도 덜 쓴 파일을 --resume 이 다시 쓰지 않게.
      tsdf_meta.json 을 먼저 쓰므로 mesh_raw.ply 가 있으면 tsdf_meta.json 도 있음
    - 쓸 장이 없음 · 해상도가 다름 · 메쉬가 빔 → MeshError (한국어 문장, stray.CaptureError 의 한 종류)
"""
import argparse
import collections
import hashlib
import json
import os
import re
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

from . import stray, stray_fix

FIX_FILE = "pose_fix.json"                                      # 빌드 폴더에 쓰는 자세 보정 info (단계 묶음 — stray_quality 가 sha256 을 확인해 읽음). 맵 폴더에도 복사 (되면)
OLD_FIX_FILE = "floor_fix.json"                                 # 11/24 af65f1b 판이 쓰던 높이 info — 새로 만들면 지움 (stray_quality 는 예전 맵에서 계속 읽음)
READERS, AHEAD = 2, 4                                           # 그림을 미리 읽는 스레드 · 장 수 (11/24 기준 촬영본 200장: 스레드 없이 14.8 · 2개 10.5 ms/장, tsdf.py 18.7)
DEPTH_SCALE = 1000.0                                            # 깊이 PNG 값 / 1000 = m (stray.py 가 mm 로 씀)
AUTO = "auto"                                                   # build(fix=…) 기본 — 되돌아온 자리 높이 어긋남을 재서 아래 둘 중 고름 (머리말 "자세 보정")
AUTO_BASE = stray_fix.DEFAULT_FIX                               # "xy" — 어긋남이 작거나 잴 수 없을 때
AUTO_FLOOR = stray_fix.with_floor(stray_fix.DEFAULT_FIX)        # "floor+xy" — 높이 어긋남이 클 때
AUTO_FLOOR_Z_CM = 15.0                                          # 원래 자세의 되돌아온 자리 높이 어긋남(믿을 쌍 중 최대)이 이보다 크면 높이 단계를 켬
                                                                # = stray_revisit.FAIL_Z_CM (모듈은 auto 를 쓸 때만 읽음 — 같은 값인지는 tests/stray_edge 가 봄)
AUTO_SIDE_NOTE_CM = 10.0                                        # 옆 어긋남이 이보다 크면 표시에 "벽이 남을 수 있음"을 덧붙임 (= stray_revisit.PASS_H_CM)
AUTO_EVERY_S, AUTO_MAX_FRAMES = 0.1, 3000                       # 되돌아온 자리를 잴 프레임 간격 (= stray_revisit.EVERY_S) · 그 프레임 수 상한 — 긴 촬영은
                                                                # 간격을 넓힘 (촬영 초 / 3000). 3000장 ≈ 1.4 GB · 20 초 (14 분 촬영) — 922 · 6bc 는 0.1 그대로
AUTO_NEST = 3                                                   # 촬영 원본을 _unzip 아래 이만큼 깊이까지 찾음 (macOS zip 의 outer/cap 꼴)
_FLOOR_STATUS = dict(refused="관문에서 거부", error="오류")       # 높이 단계를 켰는데 고치지 못한 까닭 (표시 문장)


class MeshError(stray.CaptureError):
    """형상 메쉬를 만들 수 없음 — 사용자에게 보일 문구. stray.CaptureError 를 받는 곳에서 같이 받힘"""


class NoCapture(Exception):
    """auto 가 잴 촬영 원본을 찾지 못함 (이유 문장) — choose_fix 가 받아 xy 로"""


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


def _json_bytes(obj):
    return json.dumps(obj, ensure_ascii=False, indent=1, default=_json_safe).encode("utf-8")


def _write_bytes(path, data):
    """임시 이름으로 쓰고 바꿔 놓음 — 중간에 멈춰도 덜 쓴 파일이 남지 않게"""
    tmp = path + ".writing"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def _write_json(path, obj):
    _write_bytes(path, _json_bytes(obj))


def _copy_to_map(map_root, data, log):
    """맵 폴더에도 같은 info (예전에 input_map/pose_fix.json 을 읽던 곳을 위해 — 없어도 됨). 예전 판의 floor_fix.json 은 지움.
    맵 폴더에 쓸 수 없으면 (읽기 전용 · 남이 만든 맵) 건너뜀 — stray_quality 는 build/pose_fix.json 을 sha256 으로 확인해 읽음"""
    try:
        _write_bytes(os.path.join(map_root, FIX_FILE), data)
        old = os.path.join(map_root, OLD_FIX_FILE)
        if os.path.exists(old):
            os.remove(old)                                      # 11/24 af65f1b 판이 쓴 높이 info — 남으면 sha256 없는 예전 길로 읽힐 수 있음
    except OSError as e:
        log(f"[mesh] 맵 폴더에 {FIX_FILE} 을 쓰지 못함 — 건너뜀 (빌드 폴더의 것을 씀): {type(e).__name__}: {e}")


def _odo_rows(src):
    """촬영 폴더의 odometry.csv 데이터 줄 수 (빈 줄 · 머리글 뺌 = stray_meta.json 의 frames_total). 못 읽으면 None"""
    try:
        with open(os.path.join(src, "odometry.csv"), encoding="utf-8", errors="replace") as f:
            return sum(1 for s in f if s.strip()) - 1
    except OSError:
        return None


def _read_meta(map_root):
    """<맵 폴더>/stray_meta.json → dict (없거나 깨졌으면 {})"""
    try:
        with open(os.path.join(map_root, "stray_meta.json"), encoding="utf-8") as f:
            meta = json.load(f)
    except (OSError, ValueError):
        return {}
    return meta if isinstance(meta, dict) else {}


def _nested(work, name, depth=AUTO_NEST):
    """work 아래 depth 겹 안에서 이름이 name 인 폴더 → 목록 (얕은 것부터 · 이름 차례). __MACOSX · '.' 으로 시작하는 것 (._ 파일 포함) 은 건너뜀.
    stray.find_capture 가 zip 안 촬영 폴더를 그 경로 그대로 풀므로 (예: outer/cap → _unzip/outer/cap) 그 자리를 찾음"""
    out, level = [], [work]
    for _ in range(depth):
        nxt = []
        for d in level:
            try:
                subs = sorted(os.listdir(d))
            except OSError:
                continue
            for s in subs:
                p = os.path.join(d, s)
                if s.startswith((".", "__MACOSX")) or not os.path.isdir(p):
                    continue
                if s == name:
                    out.append(p)
                nxt.append(p)
        level = nxt
    return out


def find_capture(map_root, capture=None):
    """맵 폴더 → 그 맵을 만든 촬영 원본 (auto 가 되돌아온 자리를 잴 것). 못 찾으면 NoCapture (이유 문장). 풀지 않고 찾기만 함.
    capture (폴더 · zip) 를 주면 그것만 봄. 아니면 <맵 폴더>/_unzip/<source> → _unzip 아래 AUTO_NEST 겹 안의 <source> 폴더 (얕은 것부터 —
    macOS zip 의 outer/cap 꼴) → _unzip 그 자체 (zip 맨 위에 촬영 파일이 있던 경우). stray_quality._capture 는 _unzip/<source> → _unzip →
    capture 차례라 다름 (여기서는 준 capture 가 먼저).
    source · frames_total = <맵 폴더>/stray_meta.json. 폴더 이름이 source 와 다르거나 odometry.csv 줄 수가 frames_total 과 다르면 받지 않음
    (다른 촬영). zip 은 풀어야 알 수 있어 준 것을 믿음 (stray_revisit 이 임시 폴더에 풀어 잼)"""
    meta = _read_meta(map_root)
    source, total = meta.get("source"), meta.get("frames_total")
    if capture:
        tries = [os.path.abspath(os.path.expanduser(capture))]
    elif source:
        work, name = os.path.join(map_root, "_unzip"), os.path.basename(str(source))
        direct = os.path.join(work, name)
        tries = [direct] + [p for p in _nested(work, name) if p != direct] + [work]
    else:
        raise NoCapture("맵 폴더에 stray_meta.json 의 source 가 없음 — 아이폰 원본으로 만든 맵이 아니면 capture 를 줄 것")
    why = []
    for p in tries:
        if not os.path.exists(p):
            continue
        try:
            src = stray.find_capture(p)
        except stray.CaptureError as e:
            why.append(f"{p}: {e}")
            continue
        if not src:
            why.append(f"{p}: 촬영 폴더 · zip 이 아님")
            continue
        if os.path.isfile(src):                                 # zip
            return src
        name = os.path.basename(os.path.normpath(src))
        if source and name != source:
            why.append(f"'{name}' 은 이 맵의 촬영('{source}')이 아님")
            continue
        rows = _odo_rows(src)
        if total is not None and rows != total:
            why.append(f"{src}: odometry.csv {rows}줄 ≠ 맵의 frames_total {total} — 다른 촬영")
            continue
        return src
    raise NoCapture("촬영 원본을 찾지 못함 — " + (" / ".join(why) if why else f"{tries[0]} 가 없음" if capture else
                                                  "맵 폴더에 _unzip/ 이 없음 (촬영 폴더를 그대로 넣어 만든 맵이면 capture= · --capture 로 줄 것)"))


def _capture_seconds(src, meta):
    """촬영 길이 (초) — stray_meta.json 의 duration_s, 없으면 촬영 폴더 odometry.csv 의 처음 · 끝 시각. 모르면 None (zip)"""
    d = meta.get("duration_s")
    if isinstance(d, (int, float)) and not isinstance(d, bool):
        return float(d)
    if os.path.isdir(src):
        try:
            with open(os.path.join(src, "odometry.csv"), encoding="utf-8", errors="replace") as f:
                rows = [s for s in f.read().splitlines()[1:] if s.strip()]
            return float(rows[-1].split(",")[0]) - float(rows[0].split(",")[0])
        except (OSError, ValueError, IndexError):
            pass
    return None


def choose_fix(map_root, capture=None, log=print):
    """fix="auto" → (쓸 보정 이름, auto dict — pose_fix.auto). 촬영 원본의 원래 자세로 되돌아온 자리 어긋남을 재서 (stray_revisit.measure 의 raw)
    믿을 쌍이 있고 높이 어긋남 최대 z_max_cm 가 AUTO_FLOOR_Z_CM 를 넘으면 AUTO_FLOOR ("floor+xy"), 아니면 AUTO_BASE ("xy").
    프레임 간격 = max(AUTO_EVERY_S, 촬영 초 / AUTO_MAX_FRAMES) — 긴 촬영도 메모리 · 시간이 늘지 않게 (auto.every_s 에 남김).
    촬영 원본을 못 찾거나 재다 오류가 나면 (measure 의 SystemExit 도) AUTO_BASE 와 이유 — 빌드를 멈추지 않음. 같은 입력이면 같은 고름"""
    t = time.time()
    auto = dict(chosen=AUTO_BASE, threshold_cm=AUTO_FLOOR_Z_CM, measured=False, source=None, verdict=None, pairs=None, reliable=None,
                z_max_cm=None, h_max_cm=None, every_s=None)
    try:
        src = find_capture(map_root, capture)
        sec = _capture_seconds(src, _read_meta(map_root))
        auto["every_s"] = every = round(max(AUTO_EVERY_S, sec / AUTO_MAX_FRAMES), 3) if sec else AUTO_EVERY_S
        from . import stray_revisit                             # auto 를 쓸 때만 읽음 — 읽다 깨져도 빌드는 xy 로 계속
        raw = stray_revisit.measure(src, write=False, every_s=every)["raw"]
        sm = raw["summary"]
        z, rel, pairs = sm.get("z_max_cm"), int(sm.get("reliable") or 0), int(sm.get("pairs") or 0)
        auto.update(measured=True, source=os.path.basename(os.path.normpath(src)), verdict=raw.get("verdict"), pairs=pairs, reliable=rel,
                    z_max_cm=z, h_max_cm=sm.get("h_max_cm"))
        if rel and z is not None and z > AUTO_FLOOR_Z_CM:
            auto["chosen"] = AUTO_FLOOR
            why = f"되돌아온 자리 높이 어긋남 {z:g} cm > {AUTO_FLOOR_Z_CM:g} cm (믿을 쌍 {rel}/{pairs})"
        elif not pairs:
            why = "다시 지나간 곳이 없어 높이 어긋남을 잴 수 없음"
        elif not rel:
            why = f"다시 지나간 곳 {pairs}군데를 맞추지 못해 높이 어긋남을 모름"
        elif z is None:
            why = f"믿을 쌍 {rel}군데에 같이 본 바닥이 없어 높이 어긋남을 모름"
        else:
            why = f"되돌아온 자리 높이 어긋남 {z:g} cm ≤ {AUTO_FLOOR_Z_CM:g} cm (믿을 쌍 {rel}/{pairs})"
    except (Exception, SystemExit) as e:                       # 못 찾음 · 재다 오류 · measure 의 SystemExit ("촬영 폴더가 아님") — 기본 보정으로
        why = (str(e) if isinstance(e, NoCapture) else f"되돌아온 자리를 재지 못함 ({type(e).__name__}: {e})")[:300]
        auto["chosen"] = AUTO_BASE
    auto.update(reason=f"{why} → {auto['chosen']}", seconds=round(time.time() - t, 2))
    log(f"[mesh] 자세 보정 auto: {auto['reason']} ({auto['seconds']}s)")
    return auto["chosen"], auto


def _auto_flag(auto, info):
    """auto 가 높이 단계를 켰으면 사용자에게 보일 표시 한 줄 (엔진이 pose_fix.flags 를 주의로 띄움) → 문장 · 안 켰으면 None"""
    if auto.get("chosen") != AUTO_FLOOR:
        return None
    head = f"되돌아온 자리의 높이 어긋남 {auto['z_max_cm']:g} cm (> {AUTO_FLOOR_Z_CM:g} cm)"
    fl = next((s for s in info.get("steps") or [] if isinstance(s, dict) and s.get("kind") == "floor"), {})
    if not fl.get("applied"):
        st = fl.get("status")
        if st == "skipped":                                     # 두 재기가 엇갈림 (바닥으로는 고칠 폭이 1.5 cm 미만)
            return f"{head} — 높이 보정을 켰지만 고칠 것이 없다고 나옴 — 바닥이 두 높이로 찍혔는지 눈으로 확인"
        return f"{head} — 높이 보정을 켰지만 고치지 못함 ({_FLOOR_STATUS.get(st, st)}) — 바닥이 두 높이로 찍혔을 수 있음"
    h = auto.get("h_max_cm")
    side = f". 옆으로 어긋난 벽(최대 {h:g} cm)은 남을 수 있음" if h is not None and h > AUTO_SIDE_NOTE_CM else ""
    return f"{head} — 높이 보정을 켜서 합침{side}"


def _fix_poses(fr, poses, fix, fix_opts, pose_fix, log):
    """합치기 전 자세 보정 → (새 자세, info, 이름). fix = stray_fix.FIXES 의 이름 ("xy" · "floor+xy" · "floor") · None = 끔 ·
    pose_fix = 다른 보정 함수 (info 를 그대로 씀). "auto" 는 build 가 먼저 이름으로 바꿔 줌 (choose_fix)"""
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
        raise ValueError(f"fix 는 {[AUTO] + sorted(stray_fix.FIXES)} 중 하나 또는 None 이어야 함: {fix!r}")
    steps = stray_fix.FIXES[fix]
    per = stray_fix.split_opts(steps, fix_opts)                 # 설정이 잘못되면 ValueError — 조용히 넘기지 않음
    new, info = stray_fix.fix_poses(fr["times"], poses, fr["depth"], fr["intrinsic"][2:], steps, per, log=log, fix=fix)
    return new, info, fix                                       # 단계마다 오류는 fix_poses 가 받아 status "error" 로 남김 — 메쉬는 계속 만듦


def build(map_root, build_dir, voxel=0.02, depth_trunc=5.0, sdf_trunc_voxels=4, image_step=1, pose_fix=None, log=print,
          fix=AUTO, fix_opts=None, capture=None):
    """맵 폴더 → <build_dir>/mesh_raw.ply · tsdf_meta.json · pose_fix.json (+ 되면 <map_root>/pose_fix.json). → meta dict (tsdf_meta.json 과 같음).
    만들 수 없으면 MeshError.
    fix = 합치기 전 자세 보정: "auto" (기본 — 되돌아온 자리 높이 어긋남을 재서 "floor+xy" · "xy" 중 고름, choose_fix · 머리말 "자세 보정") ·
    stray_fix.FIXES 의 이름 "xy" (수평을 벽으로) · "floor+xy" (높이를 바닥으로 고친 뒤 수평을 벽으로) · "floor" (높이만) · None (끔, 자세 그대로).
    stray_fix.fix_poses 가 차례대로 돎. capture = auto 가 잴 촬영 원본 (폴더 · zip — 안 주면 find_capture 가 맵 폴더에서 찾음)
    fix_opts = 단계별 설정 {"floor": {...}, "xy": {...}} (예: {"floor": dict(tilt=True)} — fix 에 floor 단계가 있어야 함, 없으면 ValueError.
    auto 면 고른 보정에 있는 단계의 설정만 씀). 예전 꼴 dict(tilt=True) 는 높이 설정으로 받음.
    pose_fix(times, poses) → (poses, info): 다른 보정 함수 (주면 fix 보다 먼저 씀 — auto 도 재지 않음). 자세는 N×4×4, 카메라 광학 → 세계.
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

    auto = None
    if pose_fix is None and fix == AUTO:
        per = stray_fix.split_opts(stray_fix.FIXES[AUTO_FLOOR], fix_opts)   # 설정 이름이 틀리면 재기 전에 ValueError
        fix, auto = choose_fix(map_root, capture, log)
        steps = stray_fix.FIXES[fix]
        unused = sorted(k for k in per if k not in steps)
        if unused:
            auto["unused_opts"] = unused
            log(f"[mesh] 자세 보정 auto: {fix} 에 없는 단계의 설정 {unused} 은 쓰지 않음")
        fix_opts = {k: v for k, v in per.items() if k in steps}
    tm["auto_s"] = auto["seconds"] if auto else 0
    t = time.time()
    new, info, name = _fix_poses(fr, poses, fix, fix_opts, pose_fix, log)
    if auto:
        info["auto"] = auto
        flag = _auto_flag(auto, info)
        if flag:
            info["flags"] = [flag] + list(info.get("flags") or [])
    new = np.asarray(new, dtype=float)
    if new.shape != poses.shape or not np.all(np.isfinite(new)):
        raise MeshError(f"자세 보정이 잘못된 자세를 돌려줌 (모양 {new.shape}, 받은 것 {poses.shape}) — 보정 없이 다시 만들 것 (--no-fix)")
    dp = new[:, :3, 3] - poses[:, :3, 3]
    moved, moved_xy, moved_z = np.linalg.norm(dp, axis=1), np.linalg.norm(dp[:, :2], axis=1), np.abs(dp[:, 2])
    turn = np.degrees(np.arccos(np.clip((np.einsum("nij,nij->n", new[:, :3, :3], poses[:, :3, :3]) - 1) / 2, -1, 1)))
    fix_bytes = _json_bytes(info)                               # 메쉬를 다 쓴 뒤 build/pose_fix.json 으로 — 그 sha256 을 tsdf_meta.json 에
    dist = lambda x, nd: dict(median=round(float(np.median(x)), nd), max=round(float(x.max()), nd))
    pf = dict(name=name, file=FIX_FILE, sha256=hashlib.sha256(fix_bytes).hexdigest(), **stray_fix.summary(info),
              moved_m=dist(moved, 4), moved_xy_m=dist(moved_xy, 4), moved_z_m=dist(moved_z, 4), turned_deg=dist(turn, 3))
    if info.get("applied") is None:                             # 다른 보정 함수가 applied 를 안 알려 줌 — 자세가 움직였는지로
        pf["applied"] = bool(moved.max() > 0 or turn.max() > 0)
    if auto:
        pf["auto"] = auto
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
    _write_json(os.path.join(build_dir, "tsdf_meta.json"), meta)    # 차례: tsdf_meta.json → mesh_raw.ply → pose_fix.json (모두 바꿔 놓기)
    os.replace(tmp, path)                                       # 멈춘 자리에 따라 pose_fix.json 이 예전 것이면 sha256 이 달라 stray_quality 가 적용하지 않음
    _write_bytes(os.path.join(build_dir, FIX_FILE), fix_bytes)
    _copy_to_map(map_root, fix_bytes, log)
    log(f"[mesh] 삼각형 {meta['triangles']:,} | {sec:.0f}s (그림 기다림 {wait:.1f}s · 메쉬 꺼내기 {tm['extract_s']}s) → {path}")
    return meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map", help="맵 폴더 (stray.py 출력: keyframes/ · trajectory_camera_optical.txt)")
    ap.add_argument("--out", required=True, help="빌드 폴더 (mesh_raw.ply · tsdf_meta.json 을 씀)")
    ap.add_argument("--voxel", type=float, default=0.02, help="칸 크기 m (기본 0.02)")
    ap.add_argument("--depth-trunc", type=float, default=5.0, help="이보다 먼 깊이는 버림 m (기본 5.0 — stray.py 가 키프레임 깊이를 이미 규칙대로 골라 둠)")
    ap.add_argument("--sdf-trunc-voxels", type=int, default=4, help="sdf 폭 = 칸 × 이 값 (기본 4)")
    ap.add_argument("--image-step", type=int, default=1, help="그림을 이 화소마다 하나씩만 읽음 (기본 1 = 다 읽음, 2 = 절반 크기)")
    ap.add_argument("--fix", choices=[AUTO] + sorted(stray_fix.FIXES) + ["none"], default=None,
                    help=f"합치기 전 자세 보정 (기본 {AUTO}). auto = 되돌아온 자리 높이 어긋남이 {AUTO_FLOOR_Z_CM:g} cm 넘으면 {AUTO_FLOOR}, 아니면 {AUTO_BASE} · "
                         "xy = 수평(벽)만 · floor = 높이(바닥)만 · floor+xy = 높이 → 수평 · none = 끔")
    ap.add_argument("--no-fix", action="store_true", help="자세 보정(stray_fix)을 끔 = --fix none — 자세 그대로 합침 (pipeline/tsdf.py 와 같은 메쉬)")
    ap.add_argument("--tilt", action=argparse.BooleanOptionalAction, default=None,
                    help=f"높이 보정에서 바닥 기울기 오차도 풂 (높이 단계의 기본 {'켬' if stray_fix.DEFAULTS['tilt'] else '끔'}). "
                         f"--fix 없이 주면 재지 않고 높이 단계를 켬 ({AUTO_FLOOR}) · --fix auto 와 같이 주면 auto 가 높이 단계를 켤 때만 씀")
    ap.add_argument("--capture", help="auto 가 되돌아온 자리를 잴 촬영 원본 (폴더 · zip). 기본은 <맵 폴더>/_unzip 에서 찾음 — "
                                      "촬영 폴더를 그대로 넣어 만든 맵이면 줄 것")
    a = ap.parse_args()
    if a.no_fix or a.fix == "none":
        fix = None
    elif a.fix:
        fix = a.fix
    elif a.tilt is not None:                                    # 예전과 같이 --tilt 만 주면 높이 단계를 늘 켬 (재지 않음)
        fix = AUTO_FLOOR
        print(f"[mesh] {'--tilt' if a.tilt else '--no-tilt'} → --fix 없이 주면 재지 않고 높이 단계를 켜 {fix} 로 돎")
    else:
        fix = AUTO
    if a.capture and fix != AUTO:
        ap.error(f"--capture 는 --fix auto (기본) 에서만 씀 — 지금 보정 {fix or '끔'}")
    if fix != AUTO:
        try:
            fix = stray_fix.tilt_fix(fix, a.tilt, explicit=True, log=print, who="mesh")
        except ValueError as e:
            ap.error(str(e))
    opts = {"floor": dict(tilt=a.tilt)} if fix and a.tilt is not None else None
    try:
        build(a.map, a.out, a.voxel, a.depth_trunc, a.sdf_trunc_voxels, a.image_step, fix=fix, fix_opts=opts, capture=a.capture)
    except (stray.CaptureError, ValueError) as e:
        sys.exit(f"[mesh] 실패: {e}")


if __name__ == "__main__":
    main()
