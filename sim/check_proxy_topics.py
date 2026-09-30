#!/usr/bin/env python3
"""Proxy 씬 점검 — 씬이 Play 된 뒤 다른 터미널에서: sim/run_proxy.sh check [초]

규격서(docs/interfaces/topics.md) 토픽이 다 있는지, /cmd_vel 구독 · 발행 수, RTF, 카메라 · odom 주기를
[초] 동안(기본 10, 실제 시간) 재서 항목별 OK/NG 를 찍는다. 주기는 메시지 헤더 시각(시뮬 시간) 기준이다.
시스템 python3 + ROS Jazzy(run_proxy.sh 가 source). Proxy 계획 11장 측정표에 옮겨 적을 값: RTF · 카메라 Hz · odom Hz.
LiDAR 는 끈 것이 정상이다 — 켜고 띄웠다면 이것도 FD_LIDAR=1 로 부른다. 끝 줄 CHECK_SUMMARY 는 grep 용.
"""
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2

SPEC = {  # 규격서 2장 — 시뮬이 내는 토픽
    "/front_camera/image_raw": "sensor_msgs/msg/Image",
    "/front_camera/camera_info": "sensor_msgs/msg/CameraInfo",
    "/chassis/odom": "nav_msgs/msg/Odometry",
    "/chassis/imu": "sensor_msgs/msg/Imu",
    "/tf": "tf2_msgs/msg/TFMessage",
    "/clock": "rosgraph_msgs/msg/Clock",
}
LIDAR = "/front_3d_lidar/lidar_points"
IGNORE = {"/cmd_vel", "/parameter_events", "/rosout", LIDAR}
CAM_MIN_HZ = 10.0   # 규격서 — 목표 20, 최소 10 (시뮬 시간)
RTF_MIN = 0.8       # Proxy 계획 11장 — 사람이 몰 때의 임시 기준

def qos_for(node, topic):
    """발행자와 같은 신뢰성으로 받는다 — 영상(921 KB)을 best-effort 로 받으면 조각이 빠져 주기가 낮게 잡힌다(9.30)."""
    pubs = node.get_publishers_info_by_topic(topic)
    rel = ReliabilityPolicy.RELIABLE if pubs and all(
        p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in pubs) else ReliabilityPolicy.BEST_EFFORT
    return QoSProfile(reliability=rel, history=HistoryPolicy.KEEP_LAST, depth=50)


class Rec:
    """받은 메시지의 헤더 시각(시뮬)과 받은 시각(실제)."""

    def __init__(self):
        self.sim, self.wall, self.first = [], [], None

    def add(self, t, msg=None):
        self.sim.append(t)
        self.wall.append(time.monotonic())
        if self.first is None:
            self.first = msg

    def rates(self):
        """{hz: 메시지 Hz(시뮬), uniq_hz: 고유 시각 Hz, dup: 중복 비율, gap: 최대 간격 s(시뮬), wall_hz: 실제 Hz}"""
        if len(self.sim) < 3:
            return None
        uniq = sorted(set(self.sim))
        span, wall = uniq[-1] - uniq[0], self.wall[-1] - self.wall[0]
        if span <= 0 or wall <= 0:
            return None
        return {"hz": len(self.sim) / span, "uniq_hz": len(uniq) / span, "dup": 1 - len(uniq) / len(self.sim),
                "gap": max(b - a for a, b in zip(uniq, uniq[1:])), "wall_hz": len(self.sim) / wall}


def hdr(m):
    return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    want_lidar = os.environ.get("FD_LIDAR", "0") == "1"
    rclpy.init()
    node = rclpy.create_node("fd_check_proxy")
    t = time.monotonic()
    while time.monotonic() - t < 5:  # 발견(discovery) 대기 — 규격서 토픽이 다 보이면 바로 넘어감
        rclpy.spin_once(node, timeout_sec=0.2)
        if all(k in dict(node.get_topic_names_and_types()) for k in SPEC):
            break
    names = dict(node.get_topic_names_and_types())

    rec = {k: Rec() for k in ("clock", "image", "info", "odom", "imu", "lidar")}
    subs = (("/clock", Clock, lambda m: rec["clock"].add(m.clock.sec + m.clock.nanosec * 1e-9)),
            ("/front_camera/image_raw", Image, lambda m: rec["image"].add(hdr(m), m)),
            ("/front_camera/camera_info", CameraInfo, lambda m: rec["info"].add(hdr(m))),
            ("/chassis/odom", Odometry, lambda m: rec["odom"].add(hdr(m))),
            ("/chassis/imu", Imu, lambda m: rec["imu"].add(hdr(m))),
            (LIDAR, PointCloud2, lambda m: rec["lidar"].add(hdr(m))))
    for topic, mtype, cb in subs:
        node.create_subscription(mtype, topic, cb, qos_for(node, topic))
    print(f"점검 {dur:.0f} s — 도메인 {os.environ.get('ROS_DOMAIN_ID', '?')}", flush=True)
    done = Future()
    node.create_timer(dur, lambda: done.set_result(True))
    rclpy.spin_until_future_complete(node, done)
    pub, sub = node.count_publishers("/cmd_vel"), node.count_subscribers("/cmd_vel")

    rows = []  # (항목, 값, 판정 True/False/None=참고)
    missing = [k for k in SPEC if k not in names]
    wrong = [k for k in SPEC if k in names and SPEC[k] not in names[k]]
    rows.append(("규격서 토픽", f"{len(SPEC) - len(missing)}/{len(SPEC)}"
                 + (f" · 없음 {missing}" if missing else "") + (f" · 타입 다름 {wrong}" if wrong else ""),
                 not missing and not wrong))
    rows.append(("/cmd_vel 구독(씬) · 발행", f"구독 {sub} · 발행 {pub} — 발행은 조작 노드 하나뿐이어야 함", sub == 1 and pub <= 1))
    img = rec["image"].first
    rows.append(("카메라 영상", f"{img.width}×{img.height} {img.encoding} frame={img.header.frame_id}" if img else "메시지 없음",
                 bool(img) and (img.width, img.height, img.encoding) == (640, 480, "rgb8")))
    r = rec["image"].rates()
    rows.append(("카메라 Hz (시뮬 · 실제)", f"{r['hz']:.1f} · {r['wall_hz']:.1f} · 최대 간격 {r['gap']:.3f} s" if r else "—",
                 bool(r) and r["hz"] >= CAM_MIN_HZ))
    r = rec["info"].rates()
    rows.append(("camera_info Hz (시뮬)", f"{r['hz']:.1f} · 최대 간격 {r['gap']:.3f} s" if r else "—", bool(r)))
    for key, label in (("odom", "odom"), ("imu", "imu"), ("clock", "/clock")):
        r = rec[key].rates()
        rows.append((f"{label} Hz (시뮬, 중복 포함 · 고유 시각)",
                     f"{r['hz']:.1f} · {r['uniq_hz']:.1f} · 중복 {r['dup'] * 100:.0f} % · 최대 간격 {r['gap'] * 1000:.1f} ms"
                     if r else "—", bool(r)))
    c = rec["clock"]
    rtf = (c.sim[-1] - c.sim[0]) / (c.wall[-1] - c.wall[0]) if len(c.sim) > 2 and c.wall[-1] > c.wall[0] else None
    rows.append(("RTF (시뮬 ÷ 실제)", f"{rtf:.2f} — 모니터 모드 기준 {RTF_MIN} 이상" if rtf is not None else "—",
                 rtf is not None and rtf >= RTF_MIN))
    n_lidar = len(rec["lidar"].sim)
    rows.append(("3D LiDAR", f"{'켬' if want_lidar else '끔'}이 기대값 — 메시지 {n_lidar}개", (n_lidar > 0) == want_lidar))
    extra = sorted(k for k in names if k not in SPEC and k not in IGNORE)
    rows.append(("규격 밖 토픽 (참고)", ", ".join(extra) or "없음", None))

    w = max(len(a) for a, _, _ in rows)
    for a, b, ok in rows:
        mark = "  -- " if ok is None else (" OK  " if ok else " NG  ")
        print(f"{mark}{a.ljust(w)}  {b}")
    n_ng = sum(1 for _, _, ok in rows if ok is False)
    cam = rec["image"].rates()
    odom = rec["odom"].rates()
    print(f"CHECK_SUMMARY ng={n_ng} rtf={rtf:.2f}" if rtf is not None else f"CHECK_SUMMARY ng={n_ng} rtf=-", end="")
    print(f" cam_hz={cam['hz']:.1f}" if cam else " cam_hz=-", end="")
    print(f" odom_hz={odom['hz']:.1f}" if odom else " odom_hz=-", end="")
    print(f" cmd_vel=sub{sub}/pub{pub} lidar_msgs={n_lidar}")
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(1 if n_ng else 0)


if __name__ == "__main__":
    main()
