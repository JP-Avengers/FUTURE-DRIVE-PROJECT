"""시험용 가짜 아이폰 LiDAR 촬영(Stray Scanner 형식) — make_room.py 의 방 안을 가상 카메라로 걸으며 찍은 것처럼 만든다.

    cd recon
    python scan2sim/samples/make_room.py --out ~/webapp_runs/samples
    python scan2sim/samples/make_stray.py --room ~/webapp_runs/samples --out ~/webapp_runs/samples [--seconds 25] [--fps 30] [--seed 0]
    python -m scan2sim ~/webapp_runs/samples/synth_room_s0.zip --out ~/webapp_runs/samples/run_stray

만드는 것 (zip 안에 촬영 폴더 하나 — 실제 촬영과 같은 모양)
    rgb.mp4              1920×1440 색 영상 (mp4v). 광선으로 --render-width 크기를 그린 뒤 키움 — 삼각형 색(정점 색 평균) × 간단한 음영
    depth/NNNNNN.png     256×192 16비트 mm (Z 깊이). 맞은 면이 없으면 0. 잡음: 부드러운 잡음 + 화소 잡음 (깊이에 비례)
    confidence/NNNNNN.png 0 · 1 · 2 — 2.5 m 안 = 2, 4 m 안 = 1, 그보다 멀거나 · 물체 가장자리 · 깊이 없음 = 0, 비스듬히 본 면은 1 로 낮춤
    odometry.csv         timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy (1920×1440 기준 내부 변수)
    camera_matrix.csv    3×3
    imu.csv              timestamp, a_x, a_y, a_z, alpha_x, alpha_y, alpha_z — 100 Hz, 궤적을 미분해 만든 대략의 값 (카메라 광학 좌표계, 가속도 g 단위 · 중력 포함)

자세 규약 (실제 Stray Scanner 자료에서 B · C 가 확인한 것과 같게)
    자세 = 카메라 → 세계, 카메라 축은 OpenCV (x 오른쪽 · y 아래 · z 앞), 세계는 +Y 위, 쿼터니언 순서 qx, qy, qz, qw.
    세계 좌표 = room.glb 의 glTF 좌표 그대로 (Y 위, m). 그래서 맞힌 결과를 room_truth.json 의 center_input 과 바로 견줄 수 있음.

걷는 길
    방 가운데를 도는 모서리 둥근 사각형 (x ±1.9 · y ±0.9 m, 모서리 반지름 0.5) 을 반시계로 --speed (기본 0.42 m/s) 로 걸음 — 25 초면 한 바퀴.
    처음 0.5 초는 서 있음. 카메라 높이 1.35 m, 아래로 약 32° 숙이고 좌우로 ±50° 둘러봄.
    실제 촬영처럼 처음 0.3 초의 자세는 튐 (기록된 자세만 0.7 m 어긋났다가 제자리로 — 영상 · 깊이는 실제 자리에서 그림).
    scan2sim (stray.py) 은 처음 1 초를 건너뛰므로 이 구간은 쓰이지 않음.

출력: <out>/<이름>.zip (기본 이름 synth_room_s<seed>) · <out>/stray_truth.json. --keep-folder 면 풀린 폴더도 남김. 같은 --seed 면 같은 내용.
"""
import argparse
import json
import os
import shutil
import sys
import time
import zipfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from make_room import Y_UP_TO_Z_UP  # noqa: E402

W0, H0 = 1920, 1440                                             # 색 영상 크기
WD, HD = 256, 192                                               # 깊이 크기
FX = FY = 1335.0                                                # 아이폰 광각과 비슷한 값 (실제 자료 1333~1338)
CX, CY = 959.5, 719.5
LOOP = dict(a=1.9, b=0.9, r=0.5)                                # 걷는 길: 반폭 · 반높이 · 모서리 반지름 (만든 좌표 = Z 위)
CAM_H = 1.35
R_Y2Z = Y_UP_TO_Z_UP[:3, :3]                                    # Y 위 세계 → Z 위 (만든 좌표)


def load_room(path):
    """room.glb → (정점 (Z 위), 삼각형, 정점 색 0~1)"""
    import trimesh
    sc = trimesh.load(path, force="scene", process=False)
    V, F, C, n = [], [], [], 0
    for node in sc.graph.nodes_geometry:
        T, g = sc.graph[node]
        m = sc.geometry[g]
        V.append(trimesh.transform_points(np.asarray(m.vertices, float), T))
        F.append(np.asarray(m.faces) + n)
        vc = m.visual.vertex_colors if m.visual.kind == "vertex" else np.full((len(m.vertices), 4), 180)
        C.append(np.asarray(vc)[:, :3] / 255.0)
        n += len(m.vertices)
    return np.vstack(V) @ R_Y2Z.T, np.vstack(F), np.vstack(C)


def loop_path(step=0.005):
    """모서리 둥근 사각형을 반시계로 촘촘히 → (점 (N,2), 누적 길이 (N,))"""
    a, b, r = LOOP["a"], LOOP["b"], LOOP["r"]
    segs = []

    def line(p, q):
        n = max(2, int(np.hypot(q[0] - p[0], q[1] - p[1]) / step))
        segs.append(np.linspace(p, q, n, endpoint=False))

    def arc(c, t0, t1):
        n = max(2, int(r * np.radians(abs(t1 - t0)) / step))
        t = np.radians(np.linspace(t0, t1, n, endpoint=False))
        segs.append(np.c_[c[0] + r * np.cos(t), c[1] + r * np.sin(t)])
    line((0, -b), (a - r, -b)); arc((a - r, -b + r), -90, 0)
    line((a, -b + r), (a, b - r)); arc((a - r, b - r), 0, 90)
    line((a - r, b), (-(a - r), b)); arc((-(a - r), b - r), 90, 180)
    line((-a, b - r), (-a, -b + r)); arc((-(a - r), -b + r), 180, 270)
    line((-(a - r), -b), (0, -b))
    P = np.vstack(segs + [np.array([[0, -b]])])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    return P, s


class Walk:
    """시각 t → 카메라 → 세계 자세 (만든 좌표 = Z 위, 카메라 축 OpenCV)."""

    def __init__(self, speed, still=0.5):
        self.P, self.s = loop_path()
        self.L, self.speed, self.still = self.s[-1], speed, still

    def pose(self, t):
        d = (self.speed * max(0.0, t - self.still)) % self.L
        x = np.interp(d, self.s, self.P[:, 0])
        y = np.interp(d, self.s, self.P[:, 1])
        x2 = np.interp((d + 0.05) % self.L, self.s, self.P[:, 0])
        y2 = np.interp((d + 0.05) % self.L, self.s, self.P[:, 1])
        heading = np.arctan2(y2 - y, x2 - x)
        yaw = heading + np.radians(50) * np.sin(2 * np.pi * t / 6.5)          # 좌우로 둘러봄
        pitch = np.radians(32 + 8 * np.sin(2 * np.pi * t / 4.3 + 1.0))        # 아래로 숙임
        roll = np.radians(2.0 * np.sin(2 * np.pi * t / 3.1))
        f = np.array([np.cos(pitch) * np.cos(yaw), np.cos(pitch) * np.sin(yaw), -np.sin(pitch)])
        xr = np.array([np.sin(yaw), -np.cos(yaw), 0.0])                      # 오른쪽 (수평)
        yd = np.cross(f, xr)                                                  # 아래
        c, s_ = np.cos(roll), np.sin(roll)
        xr, yd = c * xr + s_ * yd, -s_ * xr + c * yd
        T = np.eye(4)
        T[:3, :3] = np.stack([xr, yd, f], 1)
        T[:3, 3] = [x, y, CAM_H + 0.01 * np.sin(2 * np.pi * 1.8 * t)]        # 걸음에 따라 1 cm 출렁임
        return T


def pixel_dirs(w, h):
    """w×h 영상의 화소 가운데 → 카메라 좌표 방향 (z = 1 이라 t_hit 가 곧 Z 깊이). 내부 변수는 1920×1440 기준을 비율대로."""
    s = w / W0
    fx, fy, cx, cy = FX * s, FY * s, (CX + 0.5) * s - 0.5, (CY + 0.5) * s - 0.5
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    return np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u, float)], -1).reshape(-1, 3)


def cast(scene, T, dirs):
    import open3d as o3d
    d = dirs @ T[:3, :3].T
    rays = np.c_[np.broadcast_to(T[:3, 3], d.shape), d].astype(np.float32)
    r = scene.cast_rays(o3d.core.Tensor(rays))
    return r, d


def quat(R):
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(R).as_quat()                    # (qx, qy, qz, qw)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--room", required=True, help="room.glb 가 든 폴더 (make_room.py 의 --out)")
    ap.add_argument("--out", required=True, help="zip 을 쓸 폴더 (레포 밖)")
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--speed", type=float, default=0.42, help="걷는 빠르기 (m/s)")
    ap.add_argument("--render-width", type=int, default=640, help="색 영상을 이 가로 크기로 그린 뒤 1920 으로 키움 (클수록 느림 — 960 이면 약 2배)")
    ap.add_argument("--name", help="촬영 폴더 · zip 이름 (기본 synth_room_s<seed>)")
    ap.add_argument("--keep-folder", action="store_true", help="zip 말고 풀린 폴더도 남김")
    a = ap.parse_args()
    import cv2
    import open3d as o3d

    t_start = time.time()
    rng = np.random.default_rng(a.seed)
    room = os.path.abspath(os.path.expanduser(a.room))
    glb = room if room.lower().endswith(".glb") else os.path.join(room, "room.glb")
    out = os.path.abspath(os.path.expanduser(a.out))
    name = a.name or f"synth_room_s{a.seed}"
    cap_dir = os.path.join(out, name)
    shutil.rmtree(cap_dir, ignore_errors=True)
    os.makedirs(os.path.join(cap_dir, "depth"))
    os.makedirs(os.path.join(cap_dir, "confidence"))

    V, F, C = load_room(glb)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.core.Tensor(V.astype(np.float32)), o3d.core.Tensor(F.astype(np.uint32)))
    TC = (C[F].mean(1) * 255).astype(np.float32)                # 삼각형 색 = 세 정점 색 평균 (0~255)

    walk = Walk(a.speed)
    n = int(round(a.seconds * a.fps))
    ts = np.arange(n) / a.fps
    poses = [walk.pose(t) for t in ts]
    clear = scene.compute_distance(o3d.core.Tensor(np.array([P[:3, 3] for P in poses], np.float32))).numpy()
    if clear.min() < 0.3:
        raise SystemExit(f"카메라 길이 물체와 {clear.min():.2f} m 까지 가까움 — LOOP 를 고칠 것")

    wr = a.render_width
    hr = int(round(wr * H0 / W0))
    dir_c, dir_d = pixel_dirs(wr, hr), pixel_dirs(WD, HD)
    inv_c = 1.0 / np.linalg.norm(dir_c, axis=1)                 # 방향 길이는 회전해도 같음
    vw = cv2.VideoWriter(os.path.join(cap_dir, "rgb.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), a.fps, (W0, H0))
    if not vw.isOpened():
        raise SystemExit("cv2.VideoWriter 를 열 수 없음 (mp4v)")
    jump = rng.normal(size=3)
    jump = 0.7 * jump / np.linalg.norm(jump)                    # 처음 0.3 초 자세 튐 (만든 좌표, m)
    t0 = 3600.0 + a.seed
    rows, prev_q, conf_n, valid = [], None, np.zeros(3), []
    for i, (t, T) in enumerate(zip(ts, poses)):
        r, d = cast(scene, T, dir_c)                            # 색: 삼각형 색 × (0.55 + 0.45 |cos|)
        k = np.nonzero(np.isfinite(r["t_hit"].numpy()))[0]
        nrm = r["primitive_normals"].numpy()[k]
        cosang = np.abs((nrm * d[k]).sum(1)) * inv_c[k]
        img = np.full((len(d), 3), 8, np.uint8)
        img[k] = (TC[r["primitive_ids"].numpy()[k]] * (0.55 + 0.45 * cosang)[:, None]).clip(0, 255).astype(np.uint8)
        bgr = np.ascontiguousarray(img.reshape(hr, wr, 3)[..., ::-1])
        vw.write(cv2.resize(bgr, (W0, H0), interpolation=cv2.INTER_LINEAR) if wr != W0 else bgr)

        r, d = cast(scene, T, dir_d)                            # 깊이 · 신뢰도
        z = r["t_hit"].numpy().reshape(HD, WD)
        nrm = r["primitive_normals"].numpy().reshape(HD, WD, 3)
        ok = np.isfinite(z)
        zf = np.where(ok, z, 0.0)
        smooth = cv2.resize(rng.normal(size=(6, 8)), (WD, HD), interpolation=cv2.INTER_CUBIC)
        zn = zf + (0.002 + 0.002 * zf) * smooth + 0.001 * zf * rng.normal(size=zf.shape)
        dm = np.where(ok, np.round(zn * 1000), 0).clip(0, 65535).astype(np.uint16)
        cosang = np.abs((nrm * d.reshape(HD, WD, 3)).sum(-1)) / np.linalg.norm(d.reshape(HD, WD, 3), axis=-1)
        kern = np.ones((3, 3), np.uint8)
        zr = cv2.dilate(zf.astype(np.float32), kern) - cv2.erode(np.where(ok, zf, 99.0).astype(np.float32), kern)
        conf = np.where(zf < 2.5, 2, np.where(zf < 4.0, 1, 0)).astype(np.uint8)
        conf[(cosang < 0.25) & (conf == 2)] = 1
        conf[(zr > 0.1 * np.maximum(zf, 0.5)) | ~ok] = 0
        cv2.imwrite(os.path.join(cap_dir, "depth", f"{i:06d}.png"), dm, [cv2.IMWRITE_PNG_COMPRESSION, 9])
        cv2.imwrite(os.path.join(cap_dir, "confidence", f"{i:06d}.png"), conf, [cv2.IMWRITE_PNG_COMPRESSION, 9])
        conf_n += np.bincount(conf.ravel(), minlength=3)[:3]
        valid.append(float((conf >= 1).mean()))

        Tw = T.copy()                                           # 기록할 자세: Y 위 세계 (glTF 좌표)
        if t < 0.3:
            Tw[:3, 3] += jump * (1 - t / 0.3)
        Rw, pw = R_Y2Z.T @ Tw[:3, :3], R_Y2Z.T @ Tw[:3, 3]
        q = quat(Rw)
        if prev_q is not None and np.dot(q, prev_q) < 0:
            q = -q
        prev_q = q
        rows.append(f"{t0 + t:.6f}, {i:06d}, " + ", ".join(f"{v:.7f}" for v in (*pw, *q)) + f", {FX:.4f}, {FY:.4f}, {CX:.4f}, {CY:.4f}")
    vw.release()
    with open(os.path.join(cap_dir, "odometry.csv"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write("timestamp, frame, x, y, z, qx, qy, qz, qw, fx, fy, cx, cy\n" + "\n".join(rows) + "\n")
    with open(os.path.join(cap_dir, "camera_matrix.csv"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write(f"{FX}, 0.0, {CX}\n0.0, {FY}, {CY}\n0.0, 0.0, 1.0")

    dt = 0.01                                                   # imu.csv: 궤적 미분 (100 Hz)
    lines = []
    for t in np.arange(0, a.seconds, dt):
        Pm, P0, Pp = walk.pose(max(t - dt, 0)), walk.pose(t), walk.pose(t + dt)
        acc = (Pp[:3, 3] - 2 * P0[:3, 3] + Pm[:3, 3]) / dt ** 2 + [0, 0, 9.81]
        f = P0[:3, :3].T @ acc / 9.81
        dR = P0[:3, :3].T @ Pp[:3, :3]
        om = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * dt)
        lines.append(f"{t0 + t:.6f}, " + ", ".join(f"{v:.6f}" for v in (*f, *om)))
    with open(os.path.join(cap_dir, "imu.csv"), "w", encoding="utf-8", newline="\n") as fh:
        fh.write("timestamp, a_x, a_y, a_z, alpha_x, alpha_y, alpha_z\n" + "\n".join(lines) + "\n")

    zp = os.path.join(out, name + ".zip")                       # zip: 촬영 폴더 하나가 맨 위에 (실제 내보내기와 같게)
    with zipfile.ZipFile(zp, "w") as z:
        for dp, _, fs in sorted(os.walk(cap_dir)):
            for fn in sorted(fs):
                p = os.path.join(dp, fn)
                arc = os.path.relpath(p, out).replace(os.sep, "/")
                info = zipfile.ZipInfo(arc, date_time=(2026, 1, 1, 0, 0, 0))   # 날짜를 고정해 같은 seed 면 같은 zip
                info.compress_type = zipfile.ZIP_DEFLATED if fn.endswith(".csv") else zipfile.ZIP_STORED
                z.writestr(info, open(p, "rb").read())
    if not a.keep_folder:
        shutil.rmtree(cap_dir)

    P = np.array([R_Y2Z.T @ T[:3, 3] for T in poses])
    truth_p = os.path.join(os.path.dirname(glb), "room_truth.json")
    truth = dict(file=os.path.basename(zp), room=os.path.basename(glb), seed=a.seed, frames=n, fps=a.fps, seconds=a.seconds,
                 world=("세계 좌표 = room.glb 의 glTF 좌표 (Y 위, m). odometry.csv 의 자세 = 카메라(OpenCV 축) → 이 세계. "
                        "scan2sim 결과와 견줄 때: 세계 점 p → Y 위를 Z 위로 (x, −z, y) → build/transform.json 의 T_input_to_scene"),
                 pose_jump=dict(seconds=0.3, offset_m_world=np.round(R_Y2Z.T @ jump, 3).tolist(), note="기록된 자세만 어긋남, 영상 · 깊이는 실제 자리"),
                 walk=dict(loop_scene_m=LOOP, speed_mps=a.speed, still_s=walk.still, path_length_m=round(float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum()), 2),
                           camera_height_m=CAM_H, min_clearance_m=round(float(clear.min()), 2)),
                 intrinsics=dict(fx=FX, fy=FY, cx=CX, cy=CY, size=[W0, H0], depth_size=[WD, HD], render_width=wr),
                 depth_noise="부드러운 잡음 (0.2 cm + 0.2 %·z) × N(0,1) 격자 8×6 을 키운 것 + 화소 잡음 0.1 %·z",
                 confidence_pct=np.round(100 * conf_n / conf_n.sum(), 1).tolist(), conf_ge1_pct=round(100 * float(np.mean(valid)), 1),
                 defects=json.load(open(truth_p, encoding="utf-8"))["defects"] if os.path.exists(truth_p) else None)
    json.dump(truth, open(os.path.join(out, "stray_truth.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[make_stray] {n}장 ({a.seconds:g} s · {a.fps:g} fps) · 길 {truth['walk']['path_length_m']} m · 신뢰도 1 이상 {truth['conf_ge1_pct']}% · "
          f"{os.path.getsize(zp) / 1e6:.1f} MB · {time.time() - t_start:.0f} s → {zp}")


if __name__ == "__main__":
    main()
