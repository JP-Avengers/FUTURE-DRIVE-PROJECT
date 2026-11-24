"""아이폰 LiDAR 원본(Stray Scanner 형식) → 맵 폴더 형식. 맵 폴더 경로(pipeline/build_usd.py)에 그대로 넣기 위한 형식 변환.

    cd recon
    python -m scan2sim.stray <촬영 폴더 또는 zip> --out <맵 폴더>

입력 (촬영 한 번 = 폴더 하나)
    rgb.mp4            색 영상 (1920×1440)
    depth/<n>.png      깊이, 16비트 mm (256×192)
    confidence/<n>.png 깊이 신뢰도 0 · 1 · 2
    odometry.csv       timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy — 프레임마다 자세
    camera_matrix.csv  3×3 (색 영상 크기 기준)

출력 (io_rgbd.load_rtabmap_keyframes 가 읽는 모양)
    keyframes/rgb/<id>.jpg · depth/<id>.png (mm, 색에 맞춘 크기) · calib/<id>.yaml
    trajectory_camera_optical.txt   `시각 tx ty tz qx qy qz qw id` — 카메라 광학 좌표계 → 세계 (Z 가 위)
    stray_meta.json                 무엇을 얼마나 골랐는지 (깊이 규칙은 depth_rule)

깊이 고르기 (conf_rule, 기본 "hf" — 11/24 기준 촬영본에서 잰 것)
    신뢰도 2 는 max_depth_m(5 m)까지 씀. 신뢰도 1 은 near_conf1_m(3 m) 안이고 세계 높이가 바닥 + floor_margin_m(10 cm) 보다 높을 때만 씀.
    신뢰도 0 은 버림. 멀리 · 비스듬히 본 바닥의 신뢰도 1 깊이가 바닥을 두 겹으로 만들었음 (1.5 m 넘는 곳부터). 기준 촬영본(키프레임 1133장)에서
    예전 규칙 · 3 m 대비: 바닥 7~20 cm 아래 겹 9.3 → 0.8 m² · 보이는 바닥 두 높이 15.8 → 3.2% · 천장 덮음 28 → 56% · 보이는 바닥 없음 0.8 → 6.9%.
    신뢰도 2 만 쓰면 어두운 물체 약 19 m² 를 잃어서 바닥보다 높은 신뢰도 1 은 남김 (Nav2 장애물 40.5 → 41.1 m²).
    바닥 높이는 먼저 한 번 훑어 정함: 고른 키프레임 10장마다 1장의 신뢰도 2 화소 세계 높이 → 2 cm 칸 분포에서 가장 많은 칸의 30% 이상인
    가장 낮은 칸 → 그 위 10 cm 안의 봉우리 (pipeline/glb_to_usd.find_floor 와 같은 생각). 바닥을 못 찾으면 (바닥 근처 신뢰도 2 화소가 적거나
    카메라보다 30 cm 넘게 낮지 않음) 높이 조건 없이 "dist" (신뢰도 1 은 3 m 안 · 그 밖은 2 만)로 쓰고 주의를 남김.
    신뢰도 그림이 없으면 3 m 안의 깊이를 모두 씀. conf_rule="min" = 예전 규칙 (신뢰도 min_conf 이상 · max_depth_m 까지).
    쓴 규칙 · 값 · 바닥 높이는 stray_meta.json 의 depth_rule — stray_quality 가 안 쓴 프레임에 같은 규칙을 씀 (depth_keep)

좌표: odometry.csv 의 자세는 카메라 광학 좌표계(x 오른쪽 · y 아래 · z 앞) → 세계이고 세계는 y 가 위 (샘플에서 프레임 사이 깊이를 겹쳐 확인,
어긋남 중앙값 1~2.5 cm). 그래서 카메라 쪽은 그대로 두고 세계만 y 위 → z 위로 돌림.
"""
import argparse
import json
import os
import shutil
import sys
import zipfile

import numpy as np

Y_UP_TO_Z_UP = np.array([[1.0, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]])   # (x, y, z) → (x, −z, y)

CALIB = """%YAML:1.0
---
camera_name: "{id}"
image_width: {w}
image_height: {h}
camera_matrix:
   rows: 3
   cols: 3
   data: [ {fx:.6f}, 0., {cx:.6f}, 0., {fy:.6f}, {cy:.6f}, 0., 0., 1. ]
distortion_coefficients:
   rows: 1
   cols: 5
   data: [ 0., 0., 0., 0., 0. ]
distortion_model: plumb_bob
"""


NEED = ("rgb.mp4", "odometry.csv", "camera_matrix.csv", "depth")
ZIP_HINT = "Stray Scanner 의 촬영 폴더 하나(rgb.mp4 · odometry.csv · camera_matrix.csv · depth/ · confidence/)를 통째로 zip 으로 묶어 올릴 것"
MIN_ROWS, MIN_S = 100, 2.0                                      # 이보다 짧은 촬영은 맵이 안 됨
COUNT_TOL, VIDEO_MIN = 2, 0.9                                   # 깊이 · 영상 장 수가 자세 줄 수와 다를 때: 2장까지는 주의, 영상은 90 % 밑이면 실패
QUAT_TOL = 0.01                                                 # 자세 사원수 길이가 1 에서 벗어나도 되는 정도
MAX_KEYFRAMES = 2000                                            # 긴 촬영은 간격을 넓혀 이 수 안으로 — 맵 빌드 시간이 키프레임 수 × (바닥 + 벽 수)에 비례 (11/23 시간 측정)
ZIP_MAX_FILES, ZIP_MAX_BYTES = 100_000, 6 * 1024 ** 3           # zip 폭탄 막기 (샘플 17 초 = 1682개 · 50 MB)
CONF_RULES = ("hf", "min")                                      # 깊이 고르기 규칙 (머리말 "깊이 고르기")
FLOOR_EVERY, FLOOR_MIN_FRAMES = 10, 30                          # 바닥 높이 훑기: 고른 키프레임 10장마다 1장 (짧은 촬영은 30장이 되게 촘촘히)
FLOOR_BIN_M, FLOOR_PEAK_FRAC, FLOOR_BAND_M = 0.02, 0.3, 0.05    # 높이 칸 · 바닥 칸 문턱(가장 많은 칸 대비) · 바닥 화소로 셀 폭(±)
FLOOR_MIN_PX, FLOOR_MIN_SHARE, FLOOR_BELOW_CAM_M = 5000, 0.03, 0.3   # 이보다 적거나 낮지 않으면 바닥을 못 찾은 것 → dist 규칙


class CaptureError(Exception):
    """촬영 자료가 잘못됨 — 사용자에게 보일 문구. SystemExit 가 아니라서 scan2sim 의 core.run 이 단계 실패로 받아 보고서에 남김"""


def is_stray(path):
    return os.path.isdir(path) and all(os.path.exists(os.path.join(path, f)) for f in NEED)


def _looks_stray(path):
    """필수 4개 중 2개 이상 있으면 촬영 폴더로 봄 — 빠진 것은 convert 의 validate 가 무엇인지 알려 줌"""
    return os.path.isdir(path) and sum(os.path.exists(os.path.join(path, f)) for f in NEED) >= 2


def _zip_captures(names):
    """zip 안의 촬영 폴더들 (필수 4개 중 2개 이상이 같은 폴더에 있음, '' = 맨 위). macOS 가 덧붙이는 __MACOSX/ · ._ 파일은 뺌"""
    got = {}
    for n in names:
        p = n.replace("\\", "/").split("/")
        if p[0] == "__MACOSX" or p[-1].startswith("._"):
            continue
        if len(p) >= 2 and p[-2] == "depth":
            got.setdefault("/".join(p[:-2]), set()).add("depth")
        elif p[-1] in NEED:
            got.setdefault("/".join(p[:-1]), set()).add(p[-1])
    return sorted(d for d, s in got.items() if len(s) >= 2)


def find_capture(path, work=None):
    """촬영 폴더를 찾음. zip 이면 work 에 풀고, 한 겹 아래에 들어 있으면 그 폴더. → 폴더 경로 또는 None
    촬영 폴더가 여러 개 · zip 이 깨졌거나 너무 큼 · 폴더 밖 경로 → CaptureError"""
    if os.path.isfile(path) and path.lower().endswith(".zip"):
        try:
            with zipfile.ZipFile(path) as z:
                info = z.infolist()
        except (zipfile.BadZipFile, OSError) as e:
            raise CaptureError(f"zip 파일을 열 수 없음 — 깨졌거나 올리기가 끝나지 않았을 수 있음 ({e})")
        caps = _zip_captures([i.filename for i in info])
        if work is None or not caps:                              # 풀지 않고 판별만
            return path if caps else None
        if len(caps) > 1:
            raise CaptureError(f"zip 안에 촬영 폴더가 여러 개임 ({', '.join(c or '(맨 위)' for c in caps[:5])}{' …' if len(caps) > 5 else ''}) — 한 번에 촬영 하나만: {ZIP_HINT}")
        size = sum(i.file_size for i in info)
        if len(info) > ZIP_MAX_FILES or size > ZIP_MAX_BYTES:
            raise CaptureError(f"zip 이 너무 큼 (파일 {len(info)}개 · 풀면 {size / 1024 ** 3:.2f} GB, 한도 {ZIP_MAX_FILES}개 · {ZIP_MAX_BYTES / 1024 ** 3:.0f} GB) — {ZIP_HINT}")
        os.makedirs(work, exist_ok=True)
        root = os.path.abspath(work)
        for i in info:                                            # 폴더 밖으로 나가는 경로는 풀지 않음
            if not os.path.abspath(os.path.join(root, i.filename)).startswith(root + os.sep):
                raise CaptureError(f"zip 안에 폴더 밖을 가리키는 경로가 있음: {i.filename}")
        with zipfile.ZipFile(path) as z:
            z.extractall(root)
        return os.path.normpath(os.path.join(root, caps[0]))      # 이름으로 고른 폴더 (work 에 예전에 푼 것이 남아 있어도 헷갈리지 않음)
    if _looks_stray(path):
        return path
    if os.path.isdir(path):
        subs = [d for d in sorted(os.listdir(path)) if not d.startswith((".", "__MACOSX")) and _looks_stray(os.path.join(path, d))]
        if len(subs) > 1:
            raise CaptureError(f"폴더 안에 촬영 폴더가 여러 개임 ({', '.join(subs[:5])}{' …' if len(subs) > 5 else ''}) — 촬영 하나만 넣을 것")
        return os.path.join(path, subs[0]) if subs else None
    return None


def _read_odometry(p):
    """odometry.csv 를 줄마다 확인하며 읽음 → (앞 13열 배열, 각 줄의 파일 줄 번호, 오류 목록)"""
    rows, line_no, errs = [], [], []
    with open(p, encoding="utf-8", errors="replace") as f:
        lines = f.read().splitlines()
    for ln, s in enumerate(lines[1:], 2):                         # 1줄 = 머리글
        if not s.strip():
            continue
        c = s.split(",")
        if len(c) < 13:
            errs.append(f"odometry.csv {ln}번째 줄: 열이 {len(c)}개뿐 (timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy 13개가 있어야 함)")
        else:
            try:
                v = [float(x) for x in c[:13]]
            except ValueError:
                errs.append(f"odometry.csv {ln}번째 줄: 숫자가 아닌 값이 있음 ({s.strip()[:60]!r})")
            else:
                if np.all(np.isfinite(v)):
                    rows.append(v)
                    line_no.append(ln)
                else:
                    errs.append(f"odometry.csv {ln}번째 줄: NaN · 무한대 값이 있음")
        if len(errs) >= 3:
            errs.append("odometry.csv: 그 뒤 줄은 확인하지 않음 — 파일이 깨졌을 수 있음")
            break
    return np.array(rows, dtype=float).reshape(-1, 13), line_no, errs


def validate(src):
    """촬영 폴더를 변환 전에 점검 (영상은 머리 정보만 읽음, 깊이는 파일 수 · 첫 장만). → (오류 목록, 주의 목록, 정보 dict)"""
    import cv2
    errs, warns, info = [], [], {}
    miss = [f + ("/" if f == "depth" else "") for f in NEED if not (os.path.isdir if f == "depth" else os.path.isfile)(os.path.join(src, f))]
    if miss:
        return [f"촬영 폴더에 필요한 파일이 없음: {', '.join(miss)} — {ZIP_HINT}"], warns, info

    od, line_no, e = _read_odometry(os.path.join(src, "odometry.csv"))
    errs += e
    n = len(od)
    info["odometry_rows"] = n
    if not e:
        if n == 0:
            errs.append("odometry.csv 에 자세 줄이 없음")
        else:
            t = od[:, 0]
            info["duration_s"] = round(float(t[-1] - t[0]), 2)
            bad = np.nonzero(np.diff(t) <= 0)[0]
            if bad.size:
                b = bad[0] + 1
                errs.append(f"odometry.csv {line_no[b]}번째 줄: 시각이 앞 줄보다 커지지 않음 ({t[b]:.6f} ≤ {t[b - 1]:.6f}, 이런 곳 {bad.size}군데)")
            qn = np.linalg.norm(od[:, 5:9], axis=1)
            bad = np.nonzero(np.abs(qn - 1) > QUAT_TOL)[0]
            if bad.size:
                errs.append(f"odometry.csv {line_no[bad[0]]}번째 줄: 자세 사원수(qx, qy, qz, qw)의 길이가 1 이 아님 ({qn[bad[0]]:.3f}, 이런 줄 {bad.size}개)")
            if n < MIN_ROWS or t[-1] - t[0] < MIN_S:
                errs.append(f"촬영이 너무 짧음 ({t[-1] - t[0]:.1f} 초 · {n}장) — {MIN_S:.0f} 초 · {MIN_ROWS}장 이상 찍어야 함")
            if not np.array_equal(od[:, 1], np.arange(n)):
                warns.append("odometry.csv 의 frame 번호가 0, 1, 2 … 로 이어지지 않음 — 깊이 · 영상과 줄 순서로 짝지음")

    def count(sub, required):
        d = os.path.join(src, sub)
        k = sum(f.endswith(".png") for f in os.listdir(d)) if os.path.isdir(d) else None
        info[f"{sub}_files"] = k
        if not k:
            if required:
                errs.append(f"{sub}/ 에 깊이 파일(.png)이 없음 — 촬영이 덜 저장됐거나 폴더 일부만 묶였음 (옛 Stray Scanner 의 .npy 깊이는 못 읽음)")
            else:
                warns.append(f"{sub}/ 가 {'없음' if k is None else '비었음'} — 신뢰도로 거르지 않고 깊이를 모두 씀")
        elif n and k != n:
            m = f"{sub}/ 의 파일 {k}개와 자세(odometry.csv) {n}줄의 수가 {abs(k - n)}개 다름"
            (warns if abs(k - n) <= COUNT_TOL else errs).append(m + ("" if abs(k - n) <= COUNT_TOL else f" — {COUNT_TOL}개까지만 받음. 촬영이 덜 저장됐거나 다른 촬영과 섞였음"))
        return k
    if count("depth", True):
        p = os.path.join(src, "depth", "000000.png")
        z = cv2.imread(p, cv2.IMREAD_UNCHANGED) if os.path.exists(p) else None
        if z is not None and z.dtype != np.uint16:
            errs.append(f"depth/000000.png 가 16비트 깊이(mm)가 아님 ({z.dtype})")
    count("confidence", False)

    cap = cv2.VideoCapture(os.path.join(src, "rgb.mp4"))
    W0, H0 = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    nv, fps = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)), cap.get(cv2.CAP_PROP_FPS)
    ok = cap.isOpened() and W0 > 0 and H0 > 0
    cap.release()
    if not ok:
        errs.append("rgb.mp4 를 열 수 없음 — 깨졌거나 올리기가 끝나지 않았을 수 있음")
    else:
        info.update(video_size=[W0, H0], video_frames=nv, video_fps=round(fps, 2))
        if abs(W0 / H0 - 4 / 3) > 0.01:
            errs.append(f"rgb.mp4 크기가 4:3 이 아님 ({W0}×{H0}) — Stray Scanner 원본 영상(1920×1440)이 아니거나 다시 인코딩됨")
        if nv <= 0:
            warns.append("rgb.mp4 의 장 수를 머리 정보에서 읽지 못함 — 읽히는 데까지만 씀")
        elif n and nv < n - COUNT_TOL:
            if nv < VIDEO_MIN * n:
                errs.append(f"rgb.mp4 가 {nv}장뿐 (자세 {n}줄의 {100 * nv / n:.0f} %) — 영상이 잘렸거나 다른 촬영의 영상임")
            else:
                warns.append(f"rgb.mp4 가 자세보다 {n - nv}장 적음 ({nv} / {n}) — 뒤쪽 자세는 버림")
        elif n and nv > n + COUNT_TOL:
            warns.append(f"rgb.mp4 가 자세보다 {nv - n}장 많음 ({nv} / {n}) — 뒤쪽 영상은 버림")

    try:
        K = np.loadtxt(os.path.join(src, "camera_matrix.csv"), delimiter=",")
        assert K.shape == (3, 3) and np.all(np.isfinite(K))
    except (ValueError, AssertionError):
        errs.append("camera_matrix.csv 를 읽지 못함 — 3×3 숫자 행렬이어야 함")
    else:
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        if fx <= 0 or fy <= 0:
            errs.append(f"camera_matrix.csv 의 초점거리가 0 이하임 (fx {fx:g} · fy {fy:g})")
        if ok and not (0 < cx < W0 and 0 < cy < H0):
            errs.append(f"camera_matrix.csv 의 주점(cx {cx:g} · cy {cy:g})이 영상({W0}×{H0}) 밖임")
    return errs, warns, info


def _rot(q):
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _quat(R):
    """회전 행렬 → (qx, qy, qz, qw)"""
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1)
        q = [(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k])
        q = [0, 0, 0, (R[k, j] - R[j, k]) / s]
        q[i], q[j], q[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    return np.array(q)


def select(pos, R, speed, move_m, turn_deg, max_speed, start=0):
    """키프레임 고르기: 앞에 고른 장에서 move_m 넘게 움직였거나 turn_deg 넘게 돌았을 때. 너무 빨리 움직인 장(흐림)은 뺌.
    start = 처음 몇 장을 건너뜀 (촬영을 시작한 직후에는 자세가 튐 — 샘플에서 첫 0.35 초에 0.8 m)."""
    keep, fast = [start], 0
    cos_turn = np.cos(np.radians(turn_deg))
    for i in range(start + 1, len(pos)):
        k = keep[-1]
        if np.linalg.norm(pos[i] - pos[k]) < move_m and (np.trace(R[k].T @ R[i]) - 1) / 2 > cos_turn:
            continue
        if speed[i] > max_speed:
            fast += 1
            continue
        keep.append(i)
    return keep, fast


def world_z(z, T, K, sel):
    """깊이 그림(mm)의 sel 화소마다 세계 높이 (m, Z 위). T = 카메라 광학 → 세계, K = 그 그림 크기의 (fx, fy, cx, cy) — 화소 중심 기준"""
    fx, fy, cx, cy = K
    v, u = np.nonzero(sel)
    zz = z[v, u] / 1000.0
    return T[2, 0] * (u - cx) / fx * zz + T[2, 1] * (v - cy) / fy * zz + T[2, 2] * zz + T[2, 3]


def depth_keep(z, c, rule, T=None, K=None):
    """깊이(mm) · 신뢰도(없으면 None) → 쓸 화소 (bool). rule = stray_meta.json 의 depth_rule (T · K 는 hf 일 때만 씀).
    hf: 신뢰도 2 는 max_depth_m 까지 · 1 은 near_conf1_m 안이고 바닥 + floor_margin_m 보다 높을 때만 · dist: hf 에서 높이 조건을 뺀 것 ·
    min: 신뢰도 min_confidence 이상 · max_depth_m 까지 (예전 규칙). 신뢰도 그림이 없으면 min 은 max_depth_m · 다른 규칙은 near_conf1_m 까지 모두"""
    name = rule["name"]
    ok = z > 0
    if c is None:
        return ok & (z <= 1000 * rule["max_depth_m" if name == "min" else "near_conf1_m"])
    ok &= z <= 1000 * rule["max_depth_m"]
    if name == "min":
        return ok & (c >= rule["min_confidence"])
    near = ok & (c == 1) & (z <= 1000 * rule["near_conf1_m"])
    if name == "hf" and near.any():
        near[near] = world_z(z, T, K, near) > rule["floor_z_m"] + rule["floor_margin_m"]
    return (ok & (c >= 2)) | near


def depth_rule(src, frames, pose, K, conf_rule="hf", min_conf=1, max_depth_m=5.0, near_conf1_m=3.0, floor_margin_m=0.10):
    """깊이 고르기 규칙 → (rule dict = stray_meta.json 의 depth_rule, 주의 문장 또는 None). hf 면 바닥 높이를 먼저 훑음:
    frames(고른 키프레임 줄 번호)의 FLOOR_EVERY 장마다 신뢰도 2 화소의 세계 높이 (pose(i) = 카메라 광학 → 세계, K = 깊이 그림 크기)"""
    import cv2
    if conf_rule not in CONF_RULES:
        raise ValueError(f"conf_rule 은 {CONF_RULES} 중 하나: {conf_rule!r}")
    if conf_rule == "min":
        return dict(name="min", min_confidence=int(min_conf), max_depth_m=max_depth_m), None
    rule = dict(name="hf", max_depth_m=max_depth_m, near_conf1_m=near_conf1_m, floor_margin_m=floor_margin_m, floor_z_m=None)
    if not os.path.isdir(os.path.join(src, "confidence")):       # validate 가 이미 주의를 남김
        rule["name"] = "no_confidence"
        return rule, None
    step = max(1, min(FLOOR_EVERY, len(frames) // FLOOR_MIN_FRAMES))
    hs, n = [], 0
    for i in frames[::step]:
        dp, cp = (os.path.join(src, d, f"{i:06d}.png") for d in ("depth", "confidence"))
        z, c = (cv2.imread(p, cv2.IMREAD_UNCHANGED) if os.path.exists(p) else None for p in (dp, cp))
        if z is None or c is None or c.shape != z.shape:
            continue
        n += 1
        m = (c >= 2) & (z > 0) & (z <= 1000 * max_depth_m)
        if m.any():
            hs.append(world_z(z, pose(i), K, m).astype(np.float32))
    h = np.concatenate(hs) if hs else np.zeros(0, np.float32)
    floor, near = None, 0
    if len(h):
        cnt, edges = np.histogram(h, np.arange(float(h.min()), float(h.max()) + 2 * FLOOR_BIN_M, FLOOR_BIN_M))
        k = int(np.argmax(cnt >= FLOOR_PEAK_FRAC * cnt.max()))    # 가장 많은 칸의 30% 이상인 가장 낮은 칸 → 그 위 10 cm 안의 봉우리
        k += int(np.argmax(cnt[k:k + 5]))
        floor = float(0.5 * (edges[k] + edges[k + 1]))
        near = int((np.abs(h - floor) <= FLOOR_BAND_M).sum())
    cam = float(np.median([pose(i)[2, 3] for i in frames])) if len(frames) else float("nan")
    rule["floor"] = dict(frames=n, step=step, conf2_px=int(len(h)), floor_px=near, floor_px_pct=round(100 * near / max(len(h), 1), 1),
                         floor_z_est_m=None if floor is None else round(floor, 3), camera_above_floor_m=None if floor is None else round(cam - floor, 2))
    why = ("신뢰도 2 깊이가 없음" if floor is None else
           f"바닥 근처(±{100 * FLOOR_BAND_M:.0f} cm) 신뢰도 2 화소가 {near}개 · {rule['floor']['floor_px_pct']}% 뿐" if near < FLOOR_MIN_PX or near < FLOOR_MIN_SHARE * len(h) else
           f"찾은 바닥이 카메라보다 {cam - floor:.2f} m 아래뿐" if cam - floor < FLOOR_BELOW_CAM_M else None)
    if why:
        rule["name"], rule["fallback_reason"] = "dist", why
        return rule, (f"바닥 높이를 찾지 못함 ({why}) — 신뢰도 1 깊이를 높이와 상관없이 {near_conf1_m:g} m 안에서 씀. "
                      "멀리 본 바닥이 두 겹으로 찍힐 수 있음")
    rule["floor_z_m"] = round(floor, 3)                         # 규칙은 반올림한 값으로 — stray_quality 가 같은 값을 읽음
    return rule, None


def convert(src, out, width=None, min_conf=1, move_m=0.05, turn_deg=5.0, max_speed=1.5, max_depth_m=5.0, skip_s=1.0, log=print, max_keyframes=None,
            conf_rule="hf", near_conf1_m=3.0, floor_margin_m=0.10):
    """촬영 폴더 src → 맵 폴더 out. → 요약 dict (stray_meta.json 과 같음). 자료가 잘못됐으면 CaptureError (무엇이 잘못됐는지 한국어로)
    conf_rule = 깊이 고르기 (머리말): "hf" (기본 — max_depth_m · near_conf1_m · floor_margin_m) · "min" (예전 — 신뢰도 min_conf 이상)"""
    import cv2
    if not src or not os.path.isdir(src):
        raise CaptureError(f"Stray Scanner 촬영 폴더를 찾지 못함 — {ZIP_HINT}")
    errs, warns, info = validate(src)
    if errs:
        raise CaptureError("촬영 자료에 문제가 있어 변환하지 않음: " + " / ".join(errs))
    for w in warns:
        log(f"[stray] 주의: {w}")
    od = np.genfromtxt(os.path.join(src, "odometry.csv"), delimiter=",", skip_header=1, usecols=range(13))
    K = np.loadtxt(os.path.join(src, "camera_matrix.csv"), delimiter=",")
    cap = cv2.VideoCapture(os.path.join(src, "rgb.mp4"))
    W0, H0 = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if len(od) < 2 or W0 == 0:
        raise CaptureError("촬영 자료를 읽지 못함 (odometry.csv 가 비었거나 rgb.mp4 를 열 수 없음)")
    dfirst = sorted(f for f in os.listdir(os.path.join(src, "depth")) if f.endswith(".png"))[0]
    z0 = cv2.imread(os.path.join(src, "depth", dfirst), cv2.IMREAD_UNCHANGED)
    DW, DH = z0.shape[1], z0.shape[0]                              # 깊이 원래 크기 (아이폰 256×192)
    if width is None:                                              # 기본: 깊이 원래 크기로 저장 — 깊이 정보가 이 크기라 키워도 형상은 같고 합치기만 느려짐 (11/24 측정: 메쉬 차이 중앙 0.4 mm, 합치기 약 2배 빠름)
        W, H = DW, DH
    else:
        W, H = width, int(round(H0 * width / W0))
    sx, sy = W / W0, H / H0
    fx, fy = K[0, 0] * sx, K[1, 1] * sy
    cx, cy = (K[0, 2] + 0.5) * sx - 0.5, (K[1, 2] + 0.5) * sy - 0.5   # 화소 중심 기준 (stray_quality._intr 와 같음). 256 에서 c·s 로 하면 0.4 px 어긋남
    MID = (960, int(round(H0 * 960 / W0)))                         # 색을 크게 줄일 때는 두 번에 나눠 줄임 (한 번에 1920 → 256 보다 2배 빠르고 색 차이 0.2/255)

    t, pos = od[:, 0], od[:, 2:5]
    R = np.array([_rot(q) for q in od[:, 5:9]])
    step = np.r_[0, np.linalg.norm(np.diff(pos, axis=0), axis=1)]
    dt = np.r_[1, np.maximum(np.diff(t), 1e-3)]
    speed = np.convolve(step / dt, np.ones(5) / 5, mode="same")   # 프레임 5장 평균 속력 (m/s)
    start = int(min(np.searchsorted(t, t[0] + skip_s), len(od) - 2))
    keep, fast = select(pos, R, speed, move_m, turn_deg, max_speed, start)
    cap_n, widen = MAX_KEYFRAMES if max_keyframes is None else max_keyframes, 1.0
    if cap_n and len(keep) > cap_n:                                # 긴 촬영: 움직임 · 회전 간격을 같이 넓혀 상한에 가장 가깝게 (이분 탐색 — 1.2배씩 넓히면 상한보다 15% 넘게 적게 고를 수 있었음)
        lo, hi = 1.0, 1.0
        while hi < 32:
            hi *= 1.5
            k2, f2 = select(pos, R, speed, move_m * hi, turn_deg * hi, max_speed, start)
            if len(k2) <= cap_n:
                break
        best = (hi, k2, f2)
        for _ in range(12):
            mid = (lo + hi) / 2
            k2, f2 = select(pos, R, speed, move_m * mid, turn_deg * mid, max_speed, start)
            if len(k2) <= cap_n:
                hi, best = mid, (mid, k2, f2)
            else:
                lo = mid
        widen, keep, fast = best
    if widen > 1:
        warns.append(f"촬영이 길어 키프레임 간격을 {widen:.2f}배 넓힘 ({move_m * widen * 100:.0f} cm · {turn_deg * widen:.1f}°, 상한 {cap_n}장) — 처리 시간을 줄이려는 것")
        log(f"[stray] {warns[-1]}")
    keep_set = set(keep)

    def pose(i):                                                # 카메라 광학 → 세계 (Z 위) — 깊이를 고르기 전에 씀 (hf 규칙의 높이)
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R[i], pos[i]
        return Y_UP_TO_Z_UP @ T
    s_dx, s_dy = DW / W0, DH / H0                                # 깊이 그림 크기의 내부 파라미터 (화소 중심 기준) — 기본(키프레임 = 깊이 원래 크기)이면 calib 에 쓰는 값과 같음
    Kd = (K[0, 0] * s_dx, K[1, 1] * s_dy, (K[0, 2] + 0.5) * s_dx - 0.5, (K[1, 2] + 0.5) * s_dy - 0.5)
    rule, why = depth_rule(src, keep, pose, Kd, conf_rule, min_conf, max_depth_m, near_conf1_m, floor_margin_m)
    if why:
        warns.append(why)
        log(f"[stray] 주의: {why}")

    for sub in ("rgb", "depth", "calib"):
        d = os.path.join(out, "keyframes", sub)
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
    rows, conf_hist, conf_used, valid, used, i = [], np.zeros(3), np.zeros(3), [], [], 0
    while True:                                                   # 영상은 차례로만 읽음 (건너뛰어 읽으면 빈 장이 나옴)
        ok = cap.grab()
        if not ok or i >= len(od):
            break
        if i in keep_set:
            ok, bgr = cap.retrieve()
            dp = os.path.join(src, "depth", f"{i:06d}.png")
            z = cv2.imread(dp, cv2.IMREAD_UNCHANGED) if ok and os.path.exists(dp) else None
            if z is not None:
                T = pose(i)
                cp = os.path.join(src, "confidence", f"{i:06d}.png")
                c = cv2.imread(cp, cv2.IMREAD_UNCHANGED) if os.path.exists(cp) else None   # 없으면 cv2 가 장마다 경고를 찍음
                k_px = depth_keep(z, c, rule, T, Kd)
                ok_px = (z > 0) & (z <= max_depth_m * 1000)              # 쓸 수 있는 깊이 = 신뢰도 1 이상 · max_depth_m 안 (규칙과 상관없이 — 촬영 상태)
                if c is not None:
                    conf_hist += np.bincount(c.ravel(), minlength=3)[:3]
                    conf_used += np.bincount(c[k_px].ravel(), minlength=3)[:3]
                    ok_px &= c >= 1
                valid.append(float(ok_px.mean()))
                used.append(float(k_px.mean()))
                z = np.where(k_px, z, 0).astype(np.uint16)
                kid = len(rows) + 1
                if W < MID[0] < bgr.shape[1]:
                    bgr = cv2.resize(bgr, MID, interpolation=cv2.INTER_AREA)
                cv2.imwrite(os.path.join(out, "keyframes", "rgb", f"{kid}.jpg"), cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 92])
                if z.shape[1] != W or z.shape[0] != H:                 # 키울 때는 가까운 값 그대로 — 섞으면 물체 가장자리에 없는 점이 생김
                    z = cv2.resize(z, (W, H), interpolation=cv2.INTER_NEAREST)
                cv2.imwrite(os.path.join(out, "keyframes", "depth", f"{kid}.png"), z)
                open(os.path.join(out, "keyframes", "calib", f"{kid}.yaml"), "w", encoding="utf-8").write(CALIB.format(id=kid, w=W, h=H, fx=fx, fy=fy, cx=cx, cy=cy))
                rows.append((t[i], *T[:3, 3], *_quat(T[:3, :3]), kid))
        i += 1
    cap.release()
    if len(rows) < 2:
        raise CaptureError("쓸 수 있는 장이 2장 미만임 — 깊이 파일 이름(depth/000000.png …)이 자세 줄과 맞지 않거나 카메라가 거의 움직이지 않음")
    with open(os.path.join(out, "trajectory_camera_optical.txt"), "w", encoding="utf-8") as f:
        f.write("# timestamp x y z qx qy qz qw id  (카메라 광학 좌표계 -> 세계. 세계 = 아이폰이 잡은 좌표를 Z 위로 돌린 것, m)" + chr(10))
        for r in rows:
            f.write(" ".join(f"{v:.6f}" for v in r[:8]) + f" {r[8]}" + chr(10))

    P = np.array([r[1:4] for r in rows])
    fl = od[:, 9]
    meta = dict(source=os.path.basename(os.path.normpath(src)), format="stray_scanner",
                frames_total=int(len(od)), frames_read=int(i), keyframes=len(rows), dropped_fast=int(fast),
                duration_s=round(float(t[-1] - t[0]), 1), path_length_m=round(float(step.sum()), 2),
                extent_m=np.round(np.ptp(P, axis=0), 2).tolist(), camera_height_range_m=[round(float(P[:, 2].min()), 2), round(float(P[:, 2].max()), 2)],
                speed_median_mps=round(float(np.median(speed)), 2), speed_max_mps=round(float(speed.max()), 2),
                color_size=[W0, H0], depth_size_in=[DW, DH], keyframe_size=[W, H],
                intrinsics=dict(fx=round(fx, 3), fy=round(fy, 3), cx=round(cx, 3), cy=round(cy, 3)),
                focal_variation_pct=round(float(100 * np.ptp(fl) / np.median(fl)), 2),   # 프레임마다 초점거리가 조금 다름 — 한 값(camera_matrix.csv)으로 씀
                confidence_pct=np.round(100 * conf_hist / max(conf_hist.sum(), 1), 1).tolist(),
                confidence_used_pct=np.round(100 * conf_used / np.maximum(conf_hist, 1), 1).tolist(),   # 신뢰도 0 · 1 · 2 화소 중 맵에 쓴 비율
                min_confidence=rule.get("min_confidence", 1),           # 예전 키 (가장 낮게 쓴 신뢰도) — 규칙은 depth_rule
                depth_rule=rule,
                valid_depth_pct=round(100 * float(np.mean(valid)), 1), used_depth_pct=round(100 * float(np.mean(used)), 1),
                select=dict(move_m=move_m, turn_deg=turn_deg, max_speed_mps=max_speed, max_depth_m=max_depth_m, skip_s=skip_s,
                            max_keyframes=cap_n, widen=round(widen, 3), move_m_used=round(move_m * widen, 4), turn_deg_used=round(turn_deg * widen, 2)),
                checks=dict(warnings=warns, info=info))                # validate 결과 — 주의는 보고서에 띄울 수 있게
    json.dump(meta, open(os.path.join(out, "stray_meta.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    rs = (f"신뢰도 {rule['min_confidence']} 이상 · {max_depth_m:g} m 안" if rule["name"] == "min" else
          f"신뢰도 2 는 {max_depth_m:g} m · 1 은 {near_conf1_m:g} m 안" + (f" · 바닥({rule['floor_z_m']} m) + {100 * floor_margin_m:g} cm 위만" if rule["name"] == "hf" else
                                                                         " (바닥 높이 조건 없음)" if rule["name"] == "dist" else " (신뢰도 그림 없음 — 모두)"))
    log(f"[stray] {meta['frames_total']}장 중 {meta['keyframes']}장 고름 (빨라서 뺀 것 {fast}) | 이동 {meta['path_length_m']} m · {meta['duration_s']} s | "
        f"깊이 규칙 {rule['name']}: {rs} | 쓸 수 있는 깊이 {meta['valid_depth_pct']}% 중 {meta['used_depth_pct']}% 씀 → {out}")
    return meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=None, help="키프레임 가로 크기 (세로는 비율대로). 기본은 깊이 원래 크기(256×192). 사진 텍스처를 구울 때만 960 등으로")
    ap.add_argument("--conf-rule", default="hf", choices=CONF_RULES,
                    help="깊이 고르기: hf (기본 — 신뢰도 2 는 --max-depth 까지 · 1 은 3 m 안이고 바닥 + 10 cm 위만) · min (예전 — --min-conf 이상)")
    ap.add_argument("--min-conf", type=int, default=1, choices=[0, 1, 2], help="--conf-rule min 일 때: 이 신뢰도 밑의 깊이는 버림")
    ap.add_argument("--max-depth", type=float, default=5.0, help="이보다 먼 깊이는 버림 m (기본 5)")
    ap.add_argument("--move-m", type=float, default=0.05)
    ap.add_argument("--turn-deg", type=float, default=5.0)
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    try:
        src = find_capture(os.path.abspath(a.input), os.path.join(out, "_unzip"))
        if not src:
            sys.exit("Stray Scanner 촬영 폴더가 아님 (rgb.mp4 · odometry.csv · camera_matrix.csv · depth/ 가 있어야 함)")
        convert(src, out, a.width, a.min_conf, a.move_m, a.turn_deg, max_depth_m=a.max_depth, conf_rule=a.conf_rule)
    except CaptureError as e:
        sys.exit(f"[stray] 실패: {e}")


if __name__ == "__main__":
    main()
