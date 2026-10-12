#!/usr/bin/env python3
"""교사 데이터 원본 녹화 점검 — sim/record_teacher.sh 가 녹화를 닫으면 부른다. 따로도 쓴다:

    source /opt/ros/jazzy/setup.bash && python3 sim/qc_teacher.py <녹화 폴더> [...]

시뮬 시간(메시지 헤더 · 녹화 시각) 기준. 항목마다 OK / NG / 참고를 찍고, 끝 줄은 QC_RESULT.
  카메라   장 수 · 평균 Hz · 최대 간격. 간격 0.5 s 초과 또는 10 Hz 미만이면 NG (규격서: 목표 20 · 최소 10 Hz, 9.30 샘플에서 0.48 s 한 번 빔)
  /clock   뒤로 간 적이 있으면 NG — Stop → Play 가 한 파일에 섞임(10.1), 파일을 나눠야 함
  odom     같은 헤더 시각 비율(규격서 주의: 25~29 %) · 고유 시각 Hz · 달린 거리
  /cmd_vel 개수 · Hz · |v| · |ω| 최댓값 — 상한(v 0.8 · ω 0.7, 규격서) 넘으면 NG, 하나도 없으면 NG
  자동 교사(sim/teacher.py) 녹화면 더 본다 — 라벨 /teacher/cmd_vel(헤더 시각 Hz 10 미만 · 상한 초과면 NG),
  /teacher/status(바퀴 · 단계 비율 · 주행 중 잡음 · 이탈 0.3 m 넘는 시간 · 최대 이탈), /route_command 지시별 비율
"""
import json
import os
import struct
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

V_MAX, W_MAX = 0.8, 0.7        # 규격서 2장 속도 상한
CAM_GAP_NG, CAM_GAP_WARN = 0.5, 0.15
CAM_HZ_MIN = 10.0


def stamp_cdr(data):
    """헤더가 맨 앞인 메시지(Image · CameraInfo · Odometry · Imu)와 Clock 의 시각을 CDR 바이트에서 바로 읽는다(이미지 전체를 풀지 않으려고)."""
    if data[1] == 1:  # CDR little-endian
        sec, nsec = struct.unpack_from('<iI', data, 4)
    else:
        sec, nsec = struct.unpack_from('>iI', data, 4)
    return sec + nsec * 1e-9


def check(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    Odom = get_message(types.get('/chassis/odom', 'nav_msgs/msg/Odometry'))
    Twist = get_message(types.get('/cmd_vel', 'geometry_msgs/msg/Twist'))
    TwistS = get_message('geometry_msgs/msg/TwistStamped')
    Str = get_message('std_msgs/msg/String')
    U8 = get_message('std_msgs/msg/UInt8')
    lab_t, lab_vmax, lab_wmax = [], 0.0, 0.0
    status, rcmd = [], {}

    cam, cam_rx, clock, odom_t, cmd = [], [], [], [], []
    dist, last_xy, moving = 0.0, None, 0
    vmax = wmax = 0.0
    counts = {}
    while reader.has_next():
        topic, data, t = reader.read_next()
        counts[topic] = counts.get(topic, 0) + 1
        if topic == '/front_camera/image_raw':
            cam.append(stamp_cdr(data))
            cam_rx.append(t * 1e-9)
        elif topic == '/clock':
            clock.append(stamp_cdr(data))
        elif topic == '/chassis/odom':
            m = deserialize_message(data, Odom)
            odom_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            p = m.pose.pose.position
            if last_xy is not None:
                dist += ((p.x - last_xy[0]) ** 2 + (p.y - last_xy[1]) ** 2) ** 0.5
            last_xy = (p.x, p.y)
        elif topic == '/cmd_vel':
            m = deserialize_message(data, Twist)
            cmd.append(t * 1e-9)
            vmax, wmax = max(vmax, abs(m.linear.x)), max(wmax, abs(m.angular.z))
            moving += abs(m.linear.x) > 0.05 or abs(m.angular.z) > 0.05
        elif topic == '/teacher/cmd_vel':
            m = deserialize_message(data, TwistS)
            lab_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            lab_vmax, lab_wmax = max(lab_vmax, abs(m.twist.linear.x)), max(lab_wmax, abs(m.twist.angular.z))
        elif topic == '/teacher/status':
            try:
                status.append(json.loads(deserialize_message(data, Str).data))
            except ValueError:
                pass
        elif topic == '/route_command':
            v = deserialize_message(data, U8).data
            rcmd[v] = rcmd.get(v, 0) + 1

    size = sum(os.path.getsize(os.path.join(path, f)) for f in os.listdir(path) if f.endswith('.mcap'))
    lines, ng = [], False

    def out(item, ok, text):
        nonlocal ng
        ng |= ok == 'NG'
        lines.append(f'QC {item:<8} {ok:<4} {text}')

    # /clock — 시뮬 시간 길이, 뒤로 간 적
    if len(clock) < 2:
        out('clock', 'NG', '/clock 이 거의 없음 — 씬이 Play 중이었는지')
        dur = 0.0
    else:
        back = sum(b < a for a, b in zip(clock, clock[1:]))
        dur = clock[-1] - clock[0]
        out('clock', 'NG' if back else 'OK', f'시뮬 {dur:.1f} s' + (f' · 뒤로 간 곳 {back}번 — Stop → Play 섞임, 나눠서 다시 녹화' if back else ''))

    # 카메라
    if len(cam) < 2:
        out('camera', 'NG', f'{len(cam)}장 — 카메라가 안 들어옴')
    else:
        gaps = [b - a for a, b in zip(cam, cam[1:])]
        span = cam[-1] - cam[0]
        hz = (len(cam) - 1) / span if span > 0 else 0.0
        g = max(gaps)
        at = cam[gaps.index(g)] - cam[0]
        n_warn = sum(x > CAM_GAP_WARN for x in gaps)
        ok = 'NG' if (g > CAM_GAP_NG or hz < CAM_HZ_MIN) else 'OK'
        out('camera', ok, f'{len(cam)}장 · {hz:.1f} Hz · 최대 간격 {g:.3f} s(시작 뒤 {at:.1f} s) · {CAM_GAP_WARN} s 넘는 간격 {n_warn}곳')
        # 헤더 시각은 이어져도 녹화기에 늦게 도착한 구간 — 학습 샘플링은 헤더 시각을 쓰므로 참고로만
        rx = [b - a for a, b in zip(cam_rx, cam_rx[1:])]
        if rx:
            out('cam_rx', '참고', f'수신 시각 최대 간격 {max(rx):.3f} s(시작 뒤 {cam_rx[rx.index(max(rx))] - cam_rx[0]:.1f} s)')

    # odom
    if len(odom_t) < 2:
        out('odom', 'NG', 'odom 이 거의 없음')
    else:
        dup = sum(b == a for a, b in zip(odom_t, odom_t[1:]))
        uniq = len(set(odom_t))
        span = odom_t[-1] - odom_t[0]
        out('odom', '참고', f'{len(odom_t)}개 · 같은 시각 {100 * dup / (len(odom_t) - 1):.1f} % · 고유 시각 {(uniq - 1) / span if span > 0 else 0:.1f} Hz · 달린 거리 {dist:.1f} m')

    # /cmd_vel
    if not cmd:
        out('cmd_vel', 'NG', '0개 — 조작 신호가 안 남음(teleop 이 돌았는지, 도메인이 같은지)')
    else:
        span = cmd[-1] - cmd[0]
        over = vmax > V_MAX + 1e-3 or wmax > W_MAX + 1e-3
        out('cmd_vel', 'NG' if over else 'OK',
            f'{len(cmd)}개 · {(len(cmd) - 1) / span if span > 0 else 0:.1f} Hz · 최대 |v| {vmax:.2f} · |ω| {wmax:.2f}'
            f' (상한 {V_MAX} · {W_MAX}) · 움직이는 명령 {100 * moving / len(cmd):.0f} %')

    # 자동 교사 라벨 · 상태
    if lab_t:
        span = lab_t[-1] - lab_t[0]
        hz = (len(lab_t) - 1) / span if span > 0 else 0.0
        bad = hz < 10.0 or lab_vmax > V_MAX + 1e-3 or lab_wmax > W_MAX + 1e-3
        out('label', 'NG' if bad else 'OK', f'/teacher/cmd_vel {len(lab_t)}개 · 헤더 시각 {hz:.1f} Hz · 최대 |v| {lab_vmax:.2f} · |ω| {lab_wmax:.2f}'
            f' (상한 {V_MAX} · {W_MAX})')
    else:
        out('label', '참고', '/teacher/cmd_vel 0개 — 사람 조작 녹화(라벨 = /cmd_vel)')
    if status:
        fol = [d for d in status if d.get('ph') == 'FOLLOW']
        ph = {}
        for d in status:
            ph[d.get('ph')] = ph.get(d.get('ph'), 0) + 1
        phs = ' · '.join(f'{k} {100 * v / len(status):.0f} %' for k, v in sorted(ph.items(), key=lambda kv: -kv[1]))
        if fol:
            nf = sum(abs(d.get('nw', 0)) > 1e-3 for d in fol) / len(fol)
            rf = sum(abs(d.get('cte', 0)) > 0.3 for d in fol) / len(fol)
            mc = max(abs(d.get('cte', 0)) for d in fol)
            cf = {c: sum(d.get('cmd') == c for d in fol) / len(fol) for c in (0, 1, 2)}
            out('teacher', '참고', f'바퀴 {max(d.get("lap", 0) for d in status)} · 단계 {phs} · 주행 중 잡음 {100 * nf:.0f} % · '
                f'이탈 0.3 m 넘음 {100 * rf:.0f} % · 최대 이탈 {mc:.2f} m · 지시(주행 중) 직진 {100 * cf[0]:.0f} · 좌 {100 * cf[1]:.0f} · 우 {100 * cf[2]:.0f} %')
        else:
            out('teacher', 'NG', f'주행(FOLLOW) 구간이 없음 — 단계 {phs}')
    rc = counts.get('/route_command', 0)
    dist_txt = ' · '.join(f'{["직진", "좌", "우"][k] if k < 3 else k} {v}' for k, v in sorted(rcmd.items()))
    out('route', '참고', f'/route_command {rc}개' + (f' ({dist_txt})' if rcmd else '') +
        (' — 수집 때는 안 내는 안(경로 파일 + 자세로 나중에 붙임)' if rc == 0 else ''))
    per_min = size / 1e9 / (dur / 60) if dur > 0 else 0.0
    out('size', '참고', f'{size / 1e9:.2f} GB · 시뮬 1분에 {per_min:.2f} GB')

    print(f'== {path}')
    print('\n'.join(lines))
    print(f'QC_RESULT {"NG" if ng else "OK"} {os.path.basename(path.rstrip("/"))} 시뮬 {dur:.1f} s · {size / 1e9:.2f} GB · {dist:.1f} m')
    return not ng


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    results = [check(p) for p in sys.argv[1:]]
    sys.exit(0 if all(results) else 1)
