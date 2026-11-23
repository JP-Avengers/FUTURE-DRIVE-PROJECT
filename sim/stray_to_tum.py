#!/usr/bin/env python3
"""Stray Scanner 녹화 폴더(아이폰 LiDAR) → RGB-D + 자세 폴더(tum 모양).

    python3 sim/stray_to_tum.py <녹화 폴더> <출력 폴더> [--width 960]

입력 — Stray Scanner 의 docs/format.md 기준:
    rgb.mp4                 HEVC 동영상. 없으면 rgb/<frame>.jpg|png (미리 풀어 둔 프레임)를 씀
    depth/<frame>.png       16비트 mm, 256x192, RGB 프레임마다 한 장
    confidence/<frame>.png  0 · 1 · 2 (클수록 믿을 만함)
    odometry.csv            timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy, ...
    camera_matrix.csv       3x3 (odometry.csv 에 내부 파라미터 칸이 없을 때만 씀)
    imu.csv                 그대로 복사

출력 — recon/pipeline/io_rgbd.py 의 tum 형식. sim/add_site.py 가 그대로 받음:
    rgb/<frame>.jpg · depth/<frame>.png   깊이는 16비트 mm, 사진과 같은 크기(가장 가까운 화소로 늘림)
    rgb.txt · depth.txt                   `시각 파일`
    poses.txt                             `시각 tx ty tz qx qy qz qw` — 카메라 광학 좌표계(x 오른쪽 · y 아래 · z 앞) → Z-up 맵
    intrinsics.txt                        `가로,세로,fx,fy,cx,cy` (add_site.py --intrinsics 에 그대로)
    convert_report.json · imu.csv

자세 규약: 형식 문서에 축 방향이 없음. ARKit 의 camera.transform 그대로라고 가정함(--convention arkit):
카메라 → 세계, 카메라 축은 x 오른쪽 · y 위 · z 뒤, 세계는 y 가 중력 반대 방향. 가정이 맞는지는 이웃한
프레임의 깊이 점들이 세계 좌표에서 겹치는지(겹침 잔차)와 바닥 기울기로 확인함. --convention auto 는
후보 8가지를 모두 재서 고름.

끝 줄은 STRAY_OK · STRAY_WARN · STRAY_FAIL. 실패면 종료 코드 3.
"""
import argparse
import csv
import json
import math
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

FLIP_YZ = np.diag([1.0, -1.0, -1.0, 1.0])     # ARKit 카메라(y 위 · z 뒤) <-> 광학(y 아래 · z 앞)
Y_UP_TO_Z_UP = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)   # 세계 y 위 → z 위
CONVENTIONS = {"arkit": (False, "arkit", "yup"), "optical": (False, "optical", "zup")}
SIZES_4_3 = [(1920, 1440), (3840, 2880), (1440, 1080), (1280, 960), (960, 720), (640, 480), (4032, 3024)]
OK_CM, WARN_CM = 5.0, 15.0                    # 겹침 잔차(중앙값) 기준
OK_TILT, WARN_TILT = 3.0, 10.0                # 바닥 기울기 기준(도)


class Stop(Exception):
    pass


def quat_to_R(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_to_quat(R):
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s
    if R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s


def read_odometry(path):
    """odometry.csv → [{t, frame, T(4x4, 적힌 그대로), K(fx,fy,cx,cy) 또는 None}]."""
    rows = []
    with open(path, newline="", encoding="utf-8") as f:
        lines = [l for l in csv.reader(f) if l and l[0].strip()]
    if not lines:
        raise Stop(f"odometry.csv 가 비어 있음: {path}")
    head = [h.strip().lower() for h in lines[0]]
    names = ["timestamp", "frame", "x", "y", "z", "qx", "qy", "qz", "qw", "fx", "fy", "cx", "cy"]
    if "timestamp" in head and "qw" in head:
        col = {h: i for i, h in enumerate(head)}
        body = lines[1:]
    else:                                      # 머리줄이 없으면 자리 순서로 읽음
        col = {h: i for i, h in enumerate(names)}
        body = lines
    for l in body:
        g = lambda k: l[col[k]].strip()
        T = np.eye(4)
        T[:3, :3] = quat_to_R(*(float(g(k)) for k in ("qx", "qy", "qz", "qw")))
        T[:3, 3] = [float(g(k)) for k in ("x", "y", "z")]
        K = None
        if all(k in col and col[k] < len(l) and g(k) for k in ("fx", "fy", "cx", "cy")):
            K = [float(g(k)) for k in ("fx", "fy", "cx", "cy")]
        rows.append(dict(t=float(g("timestamp")), frame=f"{int(float(g('frame'))):06d}", T=T, K=K))
    return rows


def convert_pose(T, invert, cam, world):
    """적힌 자세 → 카메라 광학 좌표계 → Z-up 맵."""
    T = np.linalg.inv(T) if invert else T
    if cam == "arkit":
        T = T @ FLIP_YZ
    if world == "yup":
        T = Y_UP_TO_Z_UP @ T
    return T


def scale_K(K, s):
    """화소 가운데가 정수 좌표인 규약에서 크기를 s 배 했을 때의 fx,fy,cx,cy."""
    fx, fy, cx, cy = K
    return [fx * s, fy * s, (cx + 0.5) * s - 0.5, (cy + 0.5) * s - 0.5]


def resize_nearest(a, w, h):
    ys = ((np.arange(h) + 0.5) * a.shape[0] / h).astype(int).clip(0, a.shape[0] - 1)
    xs = ((np.arange(w) + 0.5) * a.shape[1] / w).astype(int).clip(0, a.shape[1] - 1)
    return a[ys][:, xs]


def load_depth(src, frame, min_conf, max_mm):
    """깊이(mm, uint16). 신뢰도가 낮거나 너무 먼 화소는 0."""
    d = np.array(Image.open(os.path.join(src, "depth", frame + ".png"))).astype(np.uint16)
    cp = os.path.join(src, "confidence", frame + ".png")
    if os.path.isfile(cp):
        c = np.array(Image.open(cp))
        if c.shape != d.shape:
            raise Stop(f"confidence 와 depth 크기가 다름: {frame} {c.shape} · {d.shape}")
        d[c < min_conf] = 0
    if max_mm > 0:
        d[d > max_mm] = 0
    return d


def backproject(d, K, T, step=3, far=4.0):
    """깊이 한 장 → 세계 좌표 점들(N,3)."""
    z = d[::step, ::step].astype(np.float64) / 1000.0
    v, u = np.mgrid[0:d.shape[0]:step, 0:d.shape[1]:step]
    m = (z > 0.2) & (z < far)
    P = np.stack([(u[m] - K[2]) * z[m] / K[0], (v[m] - K[3]) * z[m] / K[1], z[m]], 1)
    return P @ T[:3, :3].T + T[:3, 3]


def overlap_residual(src, kept, Kd, poses, min_conf, max_mm, pairs=10, gap=2):
    """이웃한(gap 칸 떨어진) 두 프레임의 점들이 세계 좌표에서 얼마나 겹치는지 — 가장 가까운 점까지 거리의 중앙값(cm)."""
    from scipy.spatial import cKDTree
    if len(kept) <= gap:
        return None, []
    idx = np.unique(np.linspace(0, len(kept) - gap - 1, min(pairs, len(kept) - gap)).astype(int))
    out = []
    for i in idx:
        A = backproject(load_depth(src, kept[i]["frame"], min_conf, max_mm), Kd, poses[i])
        B = backproject(load_depth(src, kept[i + gap]["frame"], min_conf, max_mm), Kd, poses[i + gap])
        if len(A) < 200 or len(B) < 200:
            continue
        dist, _ = cKDTree(B).query(A, k=1)
        out.append(float(np.median(dist)) * 100.0)
    return (float(np.median(out)) if out else None), out


def floor_check(src, kept, Kd, poses, min_conf, max_mm, frames=40):
    """바닥 면을 맞춰 기울기(도)와 카메라 높이(m)를 냄. 세계가 Z-up 이면 기울기가 0 에 가까움."""
    idx = np.unique(np.linspace(0, len(kept) - 1, min(frames, len(kept))).astype(int))
    P = np.concatenate([backproject(load_depth(src, kept[i]["frame"], min_conf, max_mm), Kd, poses[i], step=6) for i in idx])
    if len(P) < 2000:
        return None
    cam = np.array([poses[i][:3, 3] for i in idx])
    lo = P[P[:, 2] < np.percentile(P[:, 2], 35)]
    rng = np.random.default_rng(0)
    best_n, best = 0, None
    for _ in range(400):
        s = lo[rng.choice(len(lo), 3, replace=False)]
        n = np.cross(s[1] - s[0], s[2] - s[0])
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        inl = np.abs((lo - s[0]) @ n) < 0.03
        if inl.sum() > best_n:
            best_n, best = int(inl.sum()), inl
    if best is None:
        return None
    Q = lo[best]
    c0 = Q.mean(0)
    n = np.linalg.svd(Q - c0, full_matrices=False)[2][-1]
    if n[2] < 0:
        n = -n
    return dict(tilt_deg=float(np.degrees(np.arccos(np.clip(n[2], -1, 1)))),
                camera_height_m=float(np.median((cam - c0) @ n)),
                floor_points_pct=float(100.0 * best_n / len(lo)))


def forward_score(poses):
    """걷는 방향과 카메라가 보는 방향(광학 z 축)이 얼마나 같은지(-1 ~ 1). 앞을 보고 앞으로 걸었으면 양수.

    평평한 벽 · 바닥뿐인 복도에서는 점들이 면을 따라 미끄러져도 겹침 잔차가 작게 나오므로, 축이 뒤집힌 것은 이것으로 가림.
    """
    s = []
    for A, B in zip(poses[:-1], poses[1:]):
        d = B[:3, 3] - A[:3, 3]
        n = np.linalg.norm(d)
        if n >= 0.02:
            s.append(float(d @ A[:3, 2]) / n)
    return float(np.mean(s)) if len(s) >= 5 else None


def height_spread(poses):
    """카메라 높이(z)의 흔들림 ÷ 수평 이동의 퍼짐. 손에 들고 걸으면 높이는 거의 일정하므로 작아야 함.

    방 안에서는 벽이 '가장 낮은 면'으로 잡혀 바닥 기울기만으로는 위 방향이 틀린 것을 못 가리는 경우가 있어 함께 봄.
    """
    P = np.array([T[:3, 3] for T in poses])
    s = P.std(0)
    if max(s[0], s[1]) < 0.5:              # 거의 제자리에서 찍은 자료는 판단하지 않음
        return None
    return float(s[2] / max(s[0], s[1]))


def yaw_fraction(poses):
    """카메라 회전 가운데 z 축 둘레(좌우로 돌리기)가 차지하는 몫(0 ~ 1). 손에 든 촬영은 좌우 회전이 대부분이라 커야 함."""
    acc, total = np.zeros(3), 0.0
    for A, B in zip(poses[:-1], poses[1:]):
        R = B[:3, :3] @ A[:3, :3].T
        ang = math.acos(max(-1.0, min(1.0, (np.trace(R) - 1) / 2)))
        if ang < 1e-4:
            continue
        w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) * ang / (2 * math.sin(ang))
        acc += w * w
        total += ang
    return float(acc[2] / acc.sum()) if total > math.radians(20) and acc.sum() > 0 else None


def guess_rgb_size(K):
    """내부 파라미터의 중심으로 사진 크기를 짐작함(영상을 풀기 전 · --check-only 용)."""
    return min(SIZES_4_3, key=lambda s: abs(s[0] / 2 - K[2]) + abs(s[1] / 2 - K[3]))


def frame_source(src, wanted, out_w=0, out_h=0):
    """(백엔드 이름, (프레임 번호, PIL.Image) 생성기). 영상은 저장된 방향 그대로(자동 회전 없이) 풂."""
    mp4, fdir = os.path.join(src, "rgb.mp4"), os.path.join(src, "rgb")
    want, last = set(wanted), wanted[-1]

    def from_dir():
        for i in wanted:
            for ext in (".jpg", ".jpeg", ".png"):
                p = os.path.join(fdir, f"{i:06d}{ext}")
                if os.path.isfile(p):
                    yield i, Image.open(p).convert("RGB")
                    break
            else:
                raise Stop(f"풀어 둔 프레임이 없음: {fdir}/{i:06d}.jpg")

    if not os.path.isfile(mp4):
        if os.path.isdir(fdir):
            return "frames", from_dir()
        raise Stop(f"rgb.mp4 도 rgb/ 폴더도 없음: {src}")
    try:
        import av

        def from_av():
            with av.open(mp4) as c:
                st = c.streams.video[0]
                st.thread_type = "AUTO"
                for i, fr in enumerate(c.decode(st)):
                    if i in want:
                        yield i, fr.to_image()
                    if i >= last:
                        break
        return "pyav", from_av()
    except ImportError:
        pass
    try:
        import cv2

        def from_cv2():
            cap = cv2.VideoCapture(mp4)
            cap.set(cv2.CAP_PROP_ORIENTATION_AUTO, 0)
            i = 0
            while cap.grab():
                if i in want:
                    ok, bgr = cap.retrieve()
                    if ok:
                        yield i, Image.fromarray(bgr[:, :, ::-1].copy())
                if i >= last:
                    break
                i += 1
            cap.release()
        return "opencv", from_cv2()
    except ImportError:
        pass
    if shutil.which("ffmpeg"):
        def from_ffmpeg():
            with tempfile.TemporaryDirectory(prefix="stray_rgb_") as tmp:
                vf = ["-vf", f"scale={out_w}:{out_h}"] if out_w else []
                subprocess.run(["ffmpeg", "-v", "error", "-noautorotate", "-i", mp4, "-vsync", "0", *vf, "-q:v", "2",
                                "-start_number", "0", os.path.join(tmp, "%06d.jpg")], check=True)
                for i in wanted:
                    p = os.path.join(tmp, f"{i:06d}.jpg")
                    if not os.path.isfile(p):
                        raise Stop(f"영상에 {i}번 프레임이 없음")
                    yield i, Image.open(p).convert("RGB")
        return "ffmpeg", from_ffmpeg()
    if os.path.isdir(fdir):
        return "frames", from_dir()
    raise Stop("rgb.mp4 를 풀 방법이 없음 — 이 파이썬에 PyAV(pip install av) 나 OpenCV 를 깔거나, ffmpeg 를 깔거나, "
               "프레임을 rgb/<frame>.jpg 로 미리 풀어 둘 것")


def thin(rows, every, min_move, min_rot_deg, max_frames):
    """쓸 프레임 고르기: --every 가 있으면 N 장마다, 없으면 그만큼 움직였거나 돌았을 때."""
    if every > 0:
        kept = rows[::every]
    else:
        kept, last = [rows[0]], rows[0]["T"]
        for r in rows[1:]:
            d = np.linalg.norm(r["T"][:3, 3] - last[:3, 3])
            c = (np.trace(last[:3, :3].T @ r["T"][:3, :3]) - 1) / 2
            if d >= min_move or math.degrees(math.acos(max(-1.0, min(1.0, c)))) >= min_rot_deg:
                kept.append(r)
                last = r["T"]
    if max_frames and len(kept) > max_frames:
        kept = [kept[i] for i in np.unique(np.linspace(0, len(kept) - 1, max_frames).astype(int))]
    return kept


def main():
    ap = argparse.ArgumentParser(description="Stray Scanner 녹화 폴더 → tum 모양 RGB-D + 자세 폴더")
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--width", type=int, default=960, help="출력 사진 가로(세로는 비율대로). 0 이면 원본 크기")
    ap.add_argument("--min-move", type=float, default=0.05, help="이만큼(m) 움직였으면 프레임을 씀")
    ap.add_argument("--min-rot", type=float, default=5.0, help="이만큼(도) 돌았으면 프레임을 씀")
    ap.add_argument("--every", type=int, default=0, help="N 장마다 한 장(주면 --min-move · --min-rot 대신 씀)")
    ap.add_argument("--max-frames", type=int, default=0, help="쓸 프레임 수 상한(고르게 솎음)")
    ap.add_argument("--min-confidence", type=int, default=2, choices=(0, 1, 2), help="이보다 낮은 신뢰도의 깊이는 버림")
    ap.add_argument("--max-depth", type=float, default=5.0, help="이보다 먼(m) 깊이는 버림. 0 이면 끔")
    ap.add_argument("--convention", default="arkit", choices=("arkit", "optical", "auto"),
                    help="자세 규약. arkit = camera.transform 그대로(카메라 y 위 · z 뒤, 세계 y 위), optical = 이미 광학 좌표계 → Z-up, auto = 재서 고름")
    ap.add_argument("--no-gravity", action="store_true", help="세계가 중력에 맞춰져 있지 않은 자료 — 바닥 기울기를 안 봄(add_site.py 는 --up camera)")
    ap.add_argument("--jpeg-quality", type=int, default=95)
    ap.add_argument("--check-only", action="store_true", help="파일을 쓰지 않고 자세 규약 · 겹침 · 바닥만 확인")
    ap.add_argument("--force", action="store_true", help="확인이 실패여도 출력을 남기고 종료 코드 0")
    a = ap.parse_args()
    src, dst = os.path.abspath(os.path.expanduser(a.src)), os.path.abspath(os.path.expanduser(a.dst))
    warn = []

    for need in ("odometry.csv", "depth"):
        if not os.path.exists(os.path.join(src, need)):
            raise Stop(f"Stray Scanner 녹화 폴더가 아님 — {need} 가 없음: {src}")
    rows = read_odometry(os.path.join(src, "odometry.csv"))
    have = {os.path.splitext(f)[0] for f in os.listdir(os.path.join(src, "depth")) if f.endswith(".png")}
    if not have:
        raise Stop("depth/ 에 PNG 가 없음(.npy 로 저장하는 옛 판은 지원하지 않음)")
    n_all = len(rows)
    rows = [r for r in rows if r["frame"] in have]
    if len(rows) < n_all:
        warn.append(f"깊이가 없는 프레임 {n_all - len(rows)}장을 뺌")
    if len(rows) < 3:
        raise Stop("쓸 수 있는 프레임이 3장보다 적음")
    if not os.path.isdir(os.path.join(src, "confidence")):
        warn.append("confidence/ 가 없어 깊이를 모두 믿음")
    kept = thin(rows, a.every, a.min_move, a.min_rot, a.max_frames)

    # 내부 파라미터 (사진 크기 기준)
    Ks = np.array([r["K"] for r in kept if r["K"]])
    if len(Ks):
        K = [float(v) for v in np.median(Ks, 0)]
        spread = float(100.0 * (Ks[:, 0].max() - Ks[:, 0].min()) / K[0])
    else:
        M = np.loadtxt(os.path.join(src, "camera_matrix.csv"), delimiter=",")
        K, spread = [float(M[0, 0]), float(M[1, 1]), float(M[0, 2]), float(M[1, 2])], 0.0
        warn.append("odometry.csv 에 내부 파라미터 칸이 없어 camera_matrix.csv 를 씀")
    if spread > 1.0:
        warn.append(f"프레임마다 초점 거리가 {spread:.1f} % 달라짐 — 중앙값 하나만 씀")
    dh, dw = np.array(Image.open(os.path.join(src, "depth", kept[0]["frame"] + ".png"))).shape
    rgb_w, rgb_h = guess_rgb_size(K)
    max_mm = int(a.max_depth * 1000)

    # 자세 규약
    def all_poses(conv):
        return [convert_pose(r["T"], *conv) for r in kept]

    def measure(conv, size):
        Kd = scale_K(K, dw / size[0])
        P = all_poses(conv)
        res, each = overlap_residual(src, kept, Kd, P, a.min_confidence, max_mm)
        return res, each, (None if a.no_gravity else floor_check(src, kept, Kd, P, a.min_confidence, max_mm))

    def penalty(res, fwd, floor, hs, yf, natural):
        """자세 규약 후보의 벌점 — 겹침 잔차, 걷는 방향, 바닥(기울기 · 카메라 높이), 높이 흔들림, 좌우 회전 몫을 함께 봄.

        natural: 카메라 축과 세계 축이 한 벌인 조합(ARKit 카메라 + y 위, 광학 + z 위). 나머지가 비기면 이쪽을 고름.
        """
        p = 0.0 if natural else 0.25
        p += 4.0 if res is None or res > WARN_CM else (1.0 if res > OK_CM else 0.0)
        if yf is not None and yf < 0.34 and not a.no_gravity:
            p += 1.0
        if fwd is not None:
            p += 2.0 if fwd < -0.2 else (0.5 if fwd < 0.2 else 0.0)
        if not a.no_gravity:
            if floor is None:
                p += 1.0
            else:
                p += 2.0 if floor["tilt_deg"] > WARN_TILT else (1.0 if floor["tilt_deg"] > OK_TILT else 0.0)
                p += 0.0 if 0.2 <= floor["camera_height_m"] <= 2.6 else 2.0
            if hs is not None and hs > 0.6:
                p += 1.5
        return p

    tried = {}
    if a.convention == "auto":
        best = None
        for inv in (False, True):
            for cam in ("arkit", "optical"):
                for world in (("zup",) if a.no_gravity else ("yup", "zup")):
                    c = (inv, cam, world)
                    res, _, floor = measure(c, (rgb_w, rgb_h))
                    P = all_poses(c)
                    fwd, hs, yf = forward_score(P), height_spread(P), yaw_fraction(P)
                    p = penalty(res, fwd, floor, hs, yf, (cam == "arkit") == (world == "yup"))
                    tried[f"{'w2c' if inv else 'c2w'}/{cam}/{world}"] = dict(
                        penalty=p, overlap_cm=None if res is None else round(res, 2), forward=None if fwd is None else round(fwd, 2),
                        height_spread=None if hs is None else round(hs, 2), yaw_fraction=None if yf is None else round(yf, 2),
                        tilt_deg=None if not floor else round(floor["tilt_deg"], 1),
                        camera_height_m=None if not floor else round(floor["camera_height_m"], 2))
                    key = (p, 1e9 if res is None else res)
                    if best is None or key < best[0]:
                        best = (key, c)
        if best[0][1] >= 1e9:
            raise Stop("겹침을 잴 수 없음(깊이가 거의 비어 있음)")
        conv = best[1]
    else:
        conv = CONVENTIONS[a.convention]
        if a.no_gravity and conv[2] == "yup":
            conv = (conv[0], conv[1], "zup")           # 중력을 모르면 세계 축은 그대로 둠
    conv_name = f"{'w2c' if conv[0] else 'c2w'}/{conv[1]}/{conv[2]}"

    # 사진 쓰기 (영상을 풀어 실제 크기를 알게 됨)
    backend, n_written = "none", 0
    out_w = out_h = 0
    if not a.check_only:
        for d in ("rgb", "depth"):
            os.makedirs(os.path.join(dst, d), exist_ok=True)
        wanted = [int(r["frame"]) for r in kept]
        gw = a.width if a.width else rgb_w
        backend, it = frame_source(src, wanted, gw, round(gw * dh / dw))
        by_no = {int(r["frame"]): r for r in kept}
        for no, im in it:
            if n_written == 0:
                if backend != "ffmpeg" or not a.width:
                    rgb_w, rgb_h = im.size
                if abs(rgb_w / rgb_h - dw / dh) > 0.02:
                    raise Stop(f"사진({rgb_w}x{rgb_h})과 깊이({dw}x{dh})의 가로세로 비가 다름 — 영상이 돌려져 풀렸는지 확인할 것")
                out_w = a.width if a.width else rgb_w
                out_h = round(out_w * rgb_h / rgb_w)
            if im.size != (out_w, out_h):
                im = im.resize((out_w, out_h), Image.LANCZOS)
            f = by_no[no]["frame"]
            im.save(os.path.join(dst, "rgb", f + ".jpg"), quality=a.jpeg_quality)
            d = resize_nearest(load_depth(src, f, a.min_confidence, max_mm), out_w, out_h)
            Image.fromarray(d).save(os.path.join(dst, "depth", f + ".png"))
            n_written += 1
        if n_written != len(kept):
            raise Stop(f"사진을 {len(kept)}장 써야 하는데 {n_written}장만 풀림 — 영상 프레임 수와 odometry.csv 줄 수가 다른지 볼 것")
        gs = guess_rgb_size(K)
        if abs(gs[0] - rgb_w) > 0.15 * rgb_w:
            warn.append(f"내부 파라미터의 중심({K[2]:.0f}, {K[3]:.0f})이 사진 크기 {rgb_w}x{rgb_h} 와 안 맞음 — 다른 해상도 기준일 수 있음")
    else:
        out_w = a.width if a.width else rgb_w
        out_h = round(out_w * rgb_h / rgb_w)

    # 확인 (깊이 원본 크기에서)
    poses = all_poses(conv)
    res, each, floor = measure(conv, (rgb_w, rgb_h))
    Ko = scale_K(K, out_w / rgb_w)
    path_len = float(sum(np.linalg.norm(poses[i + 1][:3, 3] - poses[i][:3, 3]) for i in range(len(poses) - 1)))
    valid = float(np.mean([100.0 * np.count_nonzero(load_depth(src, r["frame"], a.min_confidence, max_mm)) / (dw * dh)
                           for r in kept[::max(1, len(kept) // 30)]]))
    level = 0
    if res is None:
        level, why = 2, "겹침을 잴 수 없음"
    elif res > WARN_CM:
        level, why = 2, f"겹침 잔차 {res:.1f} cm — 자세 규약이 틀렸거나 자세가 크게 어긋남"
    elif res > OK_CM:
        level, why = 1, f"겹침 잔차 {res:.1f} cm"
    else:
        why = f"겹침 잔차 {res:.1f} cm"
    if floor is None and not a.no_gravity:
        level = max(level, 1)
        warn.append("바닥 면을 못 찾음")
    elif floor:
        if floor["tilt_deg"] > WARN_TILT:
            level = max(level, 2)
            why += f" · 바닥 기울기 {floor['tilt_deg']:.1f}도 — 세계가 Z-up 이 아님"
        elif floor["tilt_deg"] > OK_TILT:
            level = max(level, 1)
            why += f" · 바닥 기울기 {floor['tilt_deg']:.1f}도"
        else:
            why += f" · 바닥 기울기 {floor['tilt_deg']:.1f}도"
    if floor and not 0.2 <= floor["camera_height_m"] <= 2.6:
        level = max(level, 2)
        why += f" · 카메라 높이 {floor['camera_height_m']:.2f} m — 바닥이 카메라 아래 0.2 ~ 2.6 m 에 없음(축이 뒤집혔을 수 있음)"
    fwd = forward_score(poses)
    if fwd is not None and fwd < -0.2:
        level = max(level, 1)
        warn.append(f"걷는 방향과 카메라가 보는 방향이 반대임({fwd:.2f}) — 뒤로 걸었거나 자세 규약이 틀림")
    hs = None if a.no_gravity else height_spread(poses)
    yf = None if a.no_gravity else yaw_fraction(poses)
    if hs is not None and hs > 0.6:
        level = max(level, 1)
        warn.append(f"카메라 높이 변화가 수평 이동과 비슷함({hs:.2f}) — 위 방향이 틀렸을 수 있음")
    # 좌우 회전 몫(yf)은 보고서에만 남김: 곧게 걷는 복도에서는 회전 자체가 작아 판정에 쓰면 헛경고가 남 (auto 후보 고르기에만 씀)
    if valid < 30:
        level = max(level, 1)
        warn.append(f"깊이가 찬 화소가 {valid:.0f} % 뿐임")
    if warn and level == 0:
        level = 1
    verdict = ("STRAY_OK", "STRAY_WARN", "STRAY_FAIL")[level]

    report = dict(source=src, frames_in_odometry=n_all, frames_with_depth=len(rows), frames_kept=len(kept),
                  thinning=dict(every=a.every, min_move_m=a.min_move, min_rot_deg=a.min_rot, max_frames=a.max_frames),
                  video_backend=backend, rgb_size=[rgb_w, rgb_h], depth_size=[dw, dh], out_size=[out_w, out_h],
                  intrinsics_rgb=K, intrinsics_out=Ko, focal_spread_pct=spread, convention=conv_name, tried=tried,
                  overlap_residual_cm=res, overlap_each_cm=each, floor=floor, forward_score=fwd, height_spread=hs, yaw_fraction=yf, path_length_m=path_len,
                  depth_valid_pct=valid, min_confidence=a.min_confidence, max_depth_m=a.max_depth,
                  warnings=warn, verdict=verdict, why=why)

    if not a.check_only:
        with open(os.path.join(dst, "rgb.txt"), "w") as fr, open(os.path.join(dst, "depth.txt"), "w") as fd, \
                open(os.path.join(dst, "poses.txt"), "w") as fp:
            fp.write("# timestamp tx ty tz qx qy qz qw — camera optical frame -> Z-up map\n")
            for r, T in zip(kept, poses):
                fr.write(f"{r['t']:.6f} rgb/{r['frame']}.jpg\n")
                fd.write(f"{r['t']:.6f} depth/{r['frame']}.png\n")
                q = R_to_quat(T[:3, :3])
                fp.write(f"{r['t']:.6f} " + " ".join(f"{v:.9f}" for v in (*T[:3, 3], *q)) + "\n")
        intr = f"{out_w},{out_h}," + ",".join(f"{v:.4f}" for v in Ko)
        open(os.path.join(dst, "intrinsics.txt"), "w").write(intr + "\n")
        json.dump(report, open(os.path.join(dst, "convert_report.json"), "w"), ensure_ascii=False, indent=1)
        if os.path.isfile(os.path.join(src, "imu.csv")):
            shutil.copyfile(os.path.join(src, "imu.csv"), os.path.join(dst, "imu.csv"))

    print(f"[stray] 프레임 {n_all}장 중 {len(kept)}장을 씀 · 경로 {path_len:.1f} m · 영상 풀기 {backend}")
    print(f"[stray] 사진 {rgb_w}x{rgb_h} → {out_w}x{out_h} · 깊이 {dw}x{dh} · 깊이가 찬 화소 {valid:.0f} %")
    print(f"[stray] 자세 규약 {conv_name}" + (" · 후보 벌점 " + ", ".join(f"{k}={v['penalty']:g}" for k, v in tried.items()) if tried else ""))
    if floor:
        print(f"[stray] 바닥 기울기 {floor['tilt_deg']:.2f}도 · 카메라 높이 {floor['camera_height_m']:.2f} m")
    for w in warn:
        print(f"[stray] 주의: {w}")
    if not a.check_only:
        up = "camera" if a.no_gravity else "z"
        print(f"[stray] 다음: python3 sim/add_site.py {dst} --site <장소> --ver <판> --poses poses.txt "
              f"--intrinsics {out_w},{out_h}," + ",".join(f"{v:.4f}" for v in Ko) + f" --up {up} --route <경로>")
    print(f"{verdict} {why}")
    if level == 2 and not a.force:
        sys.exit(3)


if __name__ == "__main__":
    try:
        main()
    except Stop as e:
        print(f"STRAY_FAIL {e}")
        sys.exit(3)
