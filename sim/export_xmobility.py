#!/usr/bin/env python3
"""교사 녹화(MCAP 묶음 폴더) → X-Mobility 학습 형식(parquet, IsaacSimDataset) 변환.

형식은 공개 샘플(HF nvidia/X-Mobility, goal_0032_0hz.pqt)에 맞춤:
  camera_image    JPEG bytes — 640×480 원본을 512×320(로더 INPUT_IMAGE_SIZE)으로 줄여 저장
  driving_command 6차원 [v, 0, 0, 0, 0, ω] — /teacher/cmd_vel(깨끗한 교사 명령, /cmd_vel 아님)
  ego_speed       /chassis/odom 선속도
  route_poses     40 = 20점 × (x, y), 로봇 좌표계 — 로봇 위치 + 기준선 앞쪽 0.5 m 간격 19점
                  (tools/xm_route.py 추론 때 /route 와 같은 모양). 구간 시나리오는 목표 s 에서 끊고 끝점 반복
  path            10 = 5점 × (x, y), 로봇 좌표계 — 실제로 지나간 미래 위치 t + k·0.84 s(k = 0..4).
                  간격은 공개 샘플(0.5 m/s 에서 0.42 m 간격, 멈출 때 줄어듦)에서 추정
의미 분할 열은 없음 → 학습 설정에서 ENABLE_SEMANTIC=False.

샘플링 5 Hz(카메라 20 Hz 중 헤더 시각으로 고름 — 녹화 수신 시각은 최대 0.5 s 흔들려서 쓰지 않음). /teacher/status 의 ph == FOLLOW 인 구간만 쓰고, 끊긴 곳(바퀴 사이 정지 ·
WAIT · STOP)마다 run 파일을 나눔 — 로더가 한 파일 안의 연속 행을 시퀀스로 묶기 때문.
위치: /ground_truth/tf(World → nova_carter)가 있으면 그것, 없으면(raw_v1) /chassis/odom + 씬 시작 자세.

사용: python export_xmobility.py <묶음 폴더> <출력 폴더> [--scenario 이름] [--split train]
  → <출력>/<split>/<scenario>/run_<묶음>_<NN>.pqt + <출력>/index/<묶음>.json
실행 환경(server207): source /opt/ros/jazzy/setup.bash 후 pandas · pyarrow 있는 python(~/xmobility_test/venv).
"""
import argparse
import bisect
import csv
import glob
import io
import json
import math
import os
import struct

import numpy as np
import pandas as pd
import rosbag2_py
import yaml
from PIL import Image as PILImage
from rclpy.serialization import deserialize_message

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CAM = "/front_camera/image_raw"
DEFAULT_START = (-1.756, -7.621, -96.0)   # r3 씬 시작 자세 = 기준선 출발점(raw_v1 9묶음 모두 같음)
HZ = 5.0
OUT_W, OUT_H = 512, 320
JPEG_Q = 90
N_ROUTE, ROUTE_SPACING = 20, 0.5
N_PATH, PATH_DT = 5, 0.84
GAP_S = 0.3             # 고른 프레임 사이가 이보다 벌어지면 run 을 나눔(정상 0.2 s)
MIN_RUN = 8             # 이보다 짧은 run 은 버림(시퀀스 4 × 2)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Poly:
    """닫힌 폴리라인 — tools/xm_route.py 의 Poly 와 같음(point_at · project)."""

    def __init__(self, pts):
        self.pts = pts + [pts[0]]
        self.n = len(self.pts) - 1
        self.cum = [0.0]
        for i in range(self.n):
            self.cum.append(self.cum[-1] + math.hypot(self.pts[i + 1][0] - self.pts[i][0], self.pts[i + 1][1] - self.pts[i][1]))
        self.L = self.cum[-1]

    def point_at(self, s):
        s %= self.L
        i = max(0, min(self.n - 1, bisect.bisect_right(self.cum, s) - 1))
        seg = self.cum[i + 1] - self.cum[i]
        t = 0.0 if seg < 1e-9 else (s - self.cum[i]) / seg
        return (self.pts[i][0] + t * (self.pts[i + 1][0] - self.pts[i][0]),
                self.pts[i][1] + t * (self.pts[i + 1][1] - self.pts[i][1]))

    def project(self, x, y, hint=None, back=1.0, fwd=10.0):
        best = None
        for i in range(self.n):
            ax, ay = self.pts[i]
            dx, dy = self.pts[i + 1][0] - ax, self.pts[i + 1][1] - ay
            l2 = dx * dx + dy * dy
            t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
            s = (self.cum[i] + t * math.sqrt(l2)) % self.L
            d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            if hint is not None:
                dl = (s - hint + self.L / 2) % self.L - self.L / 2
                if not -back <= dl <= fwd:
                    continue
            if best is None or d < best[1]:
                best = (s, d)
        return best if best else self.project(x, y)


def load_csv(path):
    rows = [r for r in csv.DictReader(open(path)) if r["run"] == "0"]
    return [(float(r["x"]), float(r["y"])) for r in rows]


def open_reader(mcap, topics):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=mcap, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return r


def to_robot(px, py, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = px - x, py - y
    return c * dx + s * dy, -s * dx + c * dy


class Track:
    """시각 정렬된 (t, x, y, yaw) — 선형 보간(yaw 는 감싸서)."""

    def __init__(self, rows):
        rows.sort()
        a = np.array(rows, dtype=np.float64)
        self.t, self.x, self.y = a[:, 0], a[:, 1], a[:, 2]
        self.yaw = np.unwrap(a[:, 3])

    def at(self, t):
        return (float(np.interp(t, self.t, self.x)), float(np.interp(t, self.t, self.y)),
                wrap(float(np.interp(t, self.t, self.yaw))))


def last_le(ts, vals, t):
    i = bisect.bisect_right(ts, t) - 1
    return vals[i] if i >= 0 else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bundle")
    ap.add_argument("out")
    ap.add_argument("--scenario", default=None, help="출력 하위 폴더 이름(기본: operator 로 정함)")
    ap.add_argument("--split", default="train")
    a = ap.parse_args()

    bundle = a.bundle.rstrip("/")
    name = os.path.basename(bundle)
    meta = yaml.safe_load(open(os.path.join(bundle, "meta.yaml")))
    mcap = sorted(glob.glob(os.path.join(bundle, "*.mcap")))[0]
    tmeta = meta.get("teacher", {})
    direction = tmeta.get("direction", meta.get("direction", "cw"))
    route_rel = str(tmeta.get("route", "")).replace("csv:", "")
    route_csv = os.path.join(bundle, "teacher", os.path.basename(route_rel))
    if not os.path.exists(route_csv):
        route_csv = os.path.join(REPO, route_rel)
    pts = load_csv(route_csv)
    if direction == "ccw" and os.path.basename(route_csv).endswith("_cw.csv"):
        pts = [pts[0]] + pts[1:][::-1]      # sim/teacher.py 의 r.reversed() 와 같은 뜻: 같은 출발점, 반대 방향
    route = Poly(pts)

    # 구간 시나리오면 목표 s(그 시나리오 경로 CSV 기준) — 여기서 경로를 끊음
    goal_s, start = None, DEFAULT_START
    scen_rel = tmeta.get("scenario")
    if scen_rel:
        sc = yaml.safe_load(open(os.path.join(bundle, "teacher", os.path.basename(scen_rel))))
        if "s" in (sc.get("goal") or {}):
            goal_s = float(sc["goal"]["s"]) % route.L
        st = sc.get("start") or {}
        if "x" in st:
            start = (float(st["x"]), float(st["y"]), float(st["yaw_deg"]))
    scenario = a.scenario or ("r3_auto" if meta.get("operator") == "auto" else
                              ("r3_nav2_seg" if goal_s is not None else "r3_nav2_obs"))

    # 1차: 카메라 외 토픽
    status_t, status_v, cmd_t, cmd_v = [], [], [], []
    odom, gt, spd_t, spd_v = [], [], [], []
    rd = open_reader(mcap, ["/teacher/status", "/teacher/cmd_vel", "/chassis/odom", "/ground_truth/tf"])
    while rd.has_next():
        topic, data, bt = rd.read_next()
        if topic == "/teacher/status":
            d = json.loads(deserialize_message(data, String).data)
            status_t.append(float(d["t"]))
            status_v.append(d)
        elif topic == "/teacher/cmd_vel":
            m = deserialize_message(data, TwistStamped)
            cmd_t.append(stamp(m.header))
            cmd_v.append((m.twist.linear.x, m.twist.angular.z))
        elif topic == "/chassis/odom":
            m = deserialize_message(data, Odometry)
            t = stamp(m.header)
            p = m.pose.pose.position
            odom.append((t, p.x, p.y, quat_yaw(m.pose.pose.orientation)))
            spd_t.append(t)
            spd_v.append(m.twist.twist.linear.x)
        else:
            for tr in deserialize_message(data, TFMessage).transforms:
                if tr.header.frame_id == "World" and tr.child_frame_id == "nova_carter":
                    q = tr.transform.rotation
                    gt.append((stamp(tr.header), tr.transform.translation.x, tr.transform.translation.y, quat_yaw(q)))
    for lst in (status_t, cmd_t, spd_t):
        assert lst == sorted(lst), "시각이 정렬되지 않음"

    if gt:
        pose_src, track = "ground_truth", Track(gt)
    else:
        X0, Y0, P0 = start[0], start[1], math.radians(start[2])
        c, s = math.cos(P0), math.sin(P0)
        track = Track([(t, X0 + c * ox - s * oy, Y0 + s * ox + c * oy, wrap(P0 + oyaw)) for t, ox, oy, oyaw in odom])
        pose_src = "odom+start"
    t_end = track.t[-1]

    # 2차: 카메라 5 Hz
    rows, grid, s_hint = [], None, None
    rd = open_reader(mcap, [CAM])
    while rd.has_next():
        _, data, _ = rd.read_next()
        sec, nsec = struct.unpack_from("<iI", data, 4)     # CDR: 캡슐 헤더 4 B 다음이 header.stamp
        t = sec + nsec * 1e-9
        if grid is not None and t < grid - 0.01:
            continue
        grid = (grid + 1.0 / HZ) if grid is not None and t < grid + 0.5 / HZ else t + 1.0 / HZ
        st = last_le(status_t, status_v, t)
        cmd = last_le(cmd_t, cmd_v, t)
        if st is None or cmd is None or st.get("ph") != "FOLLOW" or t + (N_PATH - 1) * PATH_DT > t_end or t < track.t[0]:
            s_hint = None
            continue
        m = deserialize_message(data, Image)
        assert m.encoding == "rgb8" and abs(stamp(m.header) - t) < 1e-6, m.encoding
        img = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
        buf = io.BytesIO()
        PILImage.fromarray(img).resize((OUT_W, OUT_H), PILImage.BILINEAR).save(buf, "JPEG", quality=JPEG_Q)

        x, y, yaw = track.at(t)
        s_now, cte = route.project(x, y, s_hint, back=1.0, fwd=10.0)
        s_hint = s_now
        reach = None if goal_s is None else (goal_s - s_now) % route.L
        rp = [(x, y)]
        for k in range(1, N_ROUTE):
            ds = k * ROUTE_SPACING if reach is None else min(k * ROUTE_SPACING, reach)
            rp.append(route.point_at(s_now + ds))
        route_poses = np.array([to_robot(px, py, x, y, yaw) for px, py in rp], np.float64).reshape(-1)
        path = np.array([to_robot(*track.at(t + k * PATH_DT)[:2], x, y, yaw) for k in range(N_PATH)], np.float64).reshape(-1)
        rows.append(dict(
            acqtime=t,
            camera_image=buf.getvalue(),
            driving_command=np.array([cmd[0], 0, 0, 0, 0, cmd[1]], np.float64),
            ego_speed=float(np.interp(t, spd_t, spd_v)),
            route_poses=route_poses,
            path=path,
            route_command=int(st.get("cmd", 0)),   # /route_command 와 같은 값(0 직진 · 1 좌 · 2 우)
            map_pose=np.array([x, y, yaw], np.float64),
            route_s=s_now,
            cte=cte,
            teacher_noise=float(st.get("nw", 0.0)),
            obstacle_dist=float(st["od"]) if st.get("od") is not None else float("nan"),
            lap=int(st.get("lap", 0)),
        ))

    # 끊긴 곳마다 run 나눔
    runs, cur = [], []
    for r in rows:
        if cur and r["acqtime"] - cur[-1]["acqtime"] > GAP_S:
            runs.append(cur)
            cur = []
        cur.append(r)
    if cur:
        runs.append(cur)
    runs = [r for r in runs if len(r) >= MIN_RUN]
    odir = os.path.join(a.out, a.split, scenario)
    os.makedirs(odir, exist_ok=True)
    files = []
    for i, r in enumerate(runs):
        fn = os.path.join(odir, f"run_{name}_{i:02d}.pqt")
        pd.DataFrame(r).to_parquet(fn, engine="pyarrow", index=False)
        files.append(dict(file=os.path.relpath(fn, a.out), rows=len(r), t0=r[0]["acqtime"], t1=r[-1]["acqtime"]))

    # 위치 점검: 쓴 위치 vs /teacher/status 의 지도 x · y
    dev = [math.hypot(track.at(t)[0] - d["x"], track.at(t)[1] - d["y"]) for t, d in zip(status_t[::20], status_v[::20])
           if "x" in d and track.t[0] <= t <= t_end]
    info = dict(bundle=name, note=meta.get("note"), operator=meta.get("operator"), direction=direction,
                scenario=scenario, split=a.split, route_csv=os.path.relpath(route_csv, REPO) if route_csv.startswith(REPO) else route_csv,
                goal_s=goal_s, pose_src=pose_src, samples=sum(len(r) for r in runs), runs=len(runs),
                dropped_short=sum(1 for _ in rows) - sum(len(r) for r in runs),
                pose_vs_status_m=dict(median=float(np.median(dev)) if dev else None, max=float(np.max(dev)) if dev else None),
                files=files)
    os.makedirs(os.path.join(a.out, "index"), exist_ok=True)
    json.dump(info, open(os.path.join(a.out, "index", name + ".json"), "w"), ensure_ascii=False, indent=1)
    print(f"{name} {scenario}/{a.split} samples={info['samples']} runs={len(runs)} pose={pose_src} "
          f"dev_med={info['pose_vs_status_m']['median']:.3f} max={info['pose_vs_status_m']['max']:.3f}", flush=True)


if __name__ == "__main__":
    main()
