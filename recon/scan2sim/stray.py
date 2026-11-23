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
    stray_meta.json                 무엇을 얼마나 골랐는지

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


def is_stray(path):
    return os.path.isdir(path) and all(os.path.exists(os.path.join(path, f)) for f in ("rgb.mp4", "odometry.csv", "camera_matrix.csv", "depth"))


def find_capture(path, work=None):
    """촬영 폴더를 찾음. zip 이면 work 에 풀고, 한 겹 아래에 들어 있으면 그 폴더. → 폴더 경로 또는 None"""
    if os.path.isfile(path) and path.lower().endswith(".zip"):
        if work is None:
            with zipfile.ZipFile(path) as z:                      # 풀지 않고 판별만
                names = z.namelist()
            return path if any(n.endswith("odometry.csv") for n in names) and any(n.endswith("rgb.mp4") for n in names) else None
        os.makedirs(work, exist_ok=True)
        root = os.path.abspath(work)
        with zipfile.ZipFile(path) as z:
            for n in z.namelist():                                # 폴더 밖으로 나가는 경로는 풀지 않음
                if not os.path.abspath(os.path.join(root, n)).startswith(root + os.sep):
                    raise SystemExit(f"zip 안에 폴더 밖을 가리키는 경로가 있음: {n}")
            z.extractall(root)
        path = root
    if is_stray(path):
        return path
    if os.path.isdir(path):
        for d in sorted(os.listdir(path)):
            if is_stray(os.path.join(path, d)):
                return os.path.join(path, d)
    return None


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


def convert(src, out, width=960, min_conf=1, move_m=0.05, turn_deg=5.0, max_speed=1.5, max_depth_m=5.0, skip_s=1.0, log=print):
    """촬영 폴더 src → 맵 폴더 out. → 요약 dict (stray_meta.json 과 같음)"""
    import cv2
    od = np.genfromtxt(os.path.join(src, "odometry.csv"), delimiter=",", skip_header=1, usecols=range(13))
    K = np.loadtxt(os.path.join(src, "camera_matrix.csv"), delimiter=",")
    cap = cv2.VideoCapture(os.path.join(src, "rgb.mp4"))
    W0, H0 = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if len(od) < 2 or W0 == 0:
        raise SystemExit("촬영 자료를 읽지 못함 (odometry.csv 가 비었거나 rgb.mp4 를 열 수 없음)")
    s = width / W0
    W, H = width, int(round(H0 * s))
    fx, fy, cx, cy = K[0, 0] * s, K[1, 1] * s, K[0, 2] * s, K[1, 2] * s

    t, pos = od[:, 0], od[:, 2:5]
    R = np.array([_rot(q) for q in od[:, 5:9]])
    step = np.r_[0, np.linalg.norm(np.diff(pos, axis=0), axis=1)]
    dt = np.r_[1, np.maximum(np.diff(t), 1e-3)]
    speed = np.convolve(step / dt, np.ones(5) / 5, mode="same")   # 프레임 5장 평균 속력 (m/s)
    start = int(min(np.searchsorted(t, t[0] + skip_s), len(od) - 2))
    keep, fast = select(pos, R, speed, move_m, turn_deg, max_speed, start)
    keep_set = set(keep)

    for sub in ("rgb", "depth", "calib"):
        d = os.path.join(out, "keyframes", sub)
        shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
    rows, conf_hist, valid, i = [], np.zeros(3), [], 0
    while True:                                                   # 영상은 차례로만 읽음 (건너뛰어 읽으면 빈 장이 나옴)
        ok = cap.grab()
        if not ok or i >= len(od):
            break
        if i in keep_set:
            ok, bgr = cap.retrieve()
            dp = os.path.join(src, "depth", f"{i:06d}.png")
            z = cv2.imread(dp, cv2.IMREAD_UNCHANGED) if ok and os.path.exists(dp) else None
            if z is not None:
                c = cv2.imread(os.path.join(src, "confidence", f"{i:06d}.png"), cv2.IMREAD_UNCHANGED)
                if c is not None:
                    conf_hist += np.bincount(c.ravel(), minlength=3)[:3]
                    z = np.where(c >= min_conf, z, 0)
                z = np.where(z > max_depth_m * 1000, 0, z).astype(np.uint16)
                valid.append(float((z > 0).mean()))
                kid = len(rows) + 1
                cv2.imwrite(os.path.join(out, "keyframes", "rgb", f"{kid}.jpg"), cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 92])
                cv2.imwrite(os.path.join(out, "keyframes", "depth", f"{kid}.png"), cv2.resize(z, (W, H), interpolation=cv2.INTER_NEAREST))   # 가까운 값 그대로 — 섞으면 물체 가장자리에 없는 점이 생김
                open(os.path.join(out, "keyframes", "calib", f"{kid}.yaml"), "w", encoding="utf-8").write(CALIB.format(id=kid, w=W, h=H, fx=fx, fy=fy, cx=cx, cy=cy))
                T = np.eye(4)
                T[:3, :3], T[:3, 3] = R[i], pos[i]
                T = Y_UP_TO_Z_UP @ T
                rows.append((t[i], *T[:3, 3], *_quat(T[:3, :3]), kid))
        i += 1
    cap.release()
    if len(rows) < 2:
        raise SystemExit("쓸 수 있는 장이 2장 미만임")
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
                color_size=[W0, H0], depth_size_in=[256, 192], keyframe_size=[W, H],
                intrinsics=dict(fx=round(fx, 3), fy=round(fy, 3), cx=round(cx, 3), cy=round(cy, 3)),
                focal_variation_pct=round(float(100 * np.ptp(fl) / np.median(fl)), 2),   # 프레임마다 초점거리가 조금 다름 — 한 값(camera_matrix.csv)으로 씀
                confidence_pct=np.round(100 * conf_hist / max(conf_hist.sum(), 1), 1).tolist(), min_confidence=min_conf,
                valid_depth_pct=round(100 * float(np.mean(valid)), 1),
                select=dict(move_m=move_m, turn_deg=turn_deg, max_speed_mps=max_speed, max_depth_m=max_depth_m, skip_s=skip_s))
    json.dump(meta, open(os.path.join(out, "stray_meta.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    log(f"[stray] {meta['frames_total']}장 중 {meta['keyframes']}장 고름 (빨라서 뺀 것 {fast}) | 이동 {meta['path_length_m']} m · {meta['duration_s']} s | "
        f"쓸 수 있는 깊이 {meta['valid_depth_pct']}% → {out}")
    return meta


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("--out", required=True)
    ap.add_argument("--width", type=int, default=960, help="키프레임 가로 크기 (세로는 비율대로)")
    ap.add_argument("--min-conf", type=int, default=1, choices=[0, 1, 2], help="이 신뢰도 밑의 깊이는 버림")
    ap.add_argument("--move-m", type=float, default=0.05)
    ap.add_argument("--turn-deg", type=float, default=5.0)
    a = ap.parse_args()
    out = os.path.abspath(a.out)
    src = find_capture(os.path.abspath(a.input), os.path.join(out, "_unzip"))
    if not src:
        sys.exit("Stray Scanner 촬영 폴더가 아님 (rgb.mp4 · odometry.csv · camera_matrix.csv · depth/ 가 있어야 함)")
    convert(src, out, a.width, a.min_conf, a.move_m, a.turn_deg)


if __name__ == "__main__":
    main()
