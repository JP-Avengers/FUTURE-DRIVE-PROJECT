#!/usr/bin/env python3
"""sim/teacher_core.py 시험 — ROS · Isaac Sim 없이: python3 sim/test_teacher_core.py

경로(Proxy 루프 · 합성 CSV) · 회전 구간 · 투영 · 잡음 비율 · 조향 상한, 그리고 바퀴 두 개짜리 운동학 모의 주행(잡음 포함)으로
교사가 상한 안에서 루프를 도는지 본다. 끝 줄은 TEACHER_CORE_TESTS n/n.
"""
import math
import os
import random
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import looppath  # noqa: E402
import teacher_core as tc  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = []


def case(fn):
    try:
        fn()
        RESULTS.append((fn.__name__, None))
    except Exception as e:  # noqa: BLE001 — 시험 하나가 깨져도 나머지는 돈다
        RESULTS.append((fn.__name__, f"{type(e).__name__}: {e}"))
    return fn


def near(a, b, tol, what=""):
    assert abs(a - b) <= tol, f"{what} {a:.4f} != {b:.4f} (±{tol})"


def proxy_route(direction="cw"):
    r = tc.Route([(x, y) for _, x, y in looppath.load_loop()], name="proxy", step=0.25)
    return r.reversed() if direction == "ccw" else r


@case
def proxy_length():
    near(proxy_route().L, 118.965, 0.2, "L")


@case
def proxy_turns_cw():
    r = proxy_route("cw")
    w = tc.turn_windows(r)
    assert len(w) == 3, w                          # NW · NE · SW, BEND(26°)는 직진
    apex = sorted(x[4] for x in w)
    for got, want in zip(apex, (26.02, 69.96, 118.97 % r.L)):
        d = (got - want + r.L / 2) % r.L - r.L / 2
        near(d, 0.0, 0.3, "apex")
    assert all(x[2] == tc.RIGHT for x in w), w
    L = r.L
    assert tc.command_at(w, 10.0, L) == tc.STRAIGHT
    assert tc.command_at(w, 24.0, L) == tc.RIGHT    # NW 앞 2 m
    assert tc.command_at(w, 29.0, L) == tc.STRAIGHT  # NW 뒤 3 m
    assert tc.command_at(w, 94.8, L) == tc.STRAIGHT  # BEND
    assert tc.command_at(w, 1.0, L) == tc.RIGHT     # SW 회전 끝(바퀴 이음 뒤)
    assert tc.command_at(w, 116.0, L) == tc.RIGHT   # SW 앞 3 m


@case
def proxy_turns_ccw():
    r = proxy_route("ccw")
    w = tc.turn_windows(r)
    assert len(w) == 3 and all(x[2] == tc.LEFT for x in w), w
    near(r.L, proxy_route("cw").L, 1e-6, "L")


@case
def project_signed():
    r = proxy_route("cw")
    s, c = r.project(-1.0, 8.0)                    # SW→NW 변(북쪽으로 감) 근처
    x, y = r.point_at(s)
    near(math.hypot(x + 1.0, y - 8.0), abs(c), 1e-6, "|cte|")
    yaw = r.yaw_at(s)
    lx, ly = x - math.sin(yaw), y + math.cos(yaw)  # 진행 방향 왼쪽 1 m
    s2, c2 = r.project(lx, ly, hint=s)
    near(c2, 1.0, 0.02, "왼쪽 +")
    s3, c3 = r.project(x + math.sin(yaw), y - math.cos(yaw), hint=s)
    near(c3, -1.0, 0.02, "오른쪽 −")


@case
def project_hint_wraps():
    r = proxy_route("cw")
    x, y = r.point_at(0.5)
    s, c = r.project(x, y, hint=r.L - 0.3)           # 바퀴 이음을 넘어도 찾음
    near((s - 0.5 + r.L / 2) % r.L - r.L / 2, 0.0, 0.05, "s")


@case
def noise_duty():
    for duty in (0.0, 0.25, 0.4):
        n = tc.SteerNoise(duty=duty, rng=random.Random(3))
        t, dt, amax = 0.0, 0.05, 0.0
        for _ in range(int(4000 / dt)):
            amax = max(amax, abs(n.step(t, dt)))
            t += dt
        near(n.on_time / t, duty, 0.04, f"duty {duty}")
        assert amax <= 0.55 + 1e-9, amax


@case
def noise_disabled_never_starts():
    n = tc.SteerNoise(duty=0.5, rng=random.Random(1))
    t = 0.0
    for _ in range(4000):
        assert n.step(t, 0.05, enabled=False) == 0.0
        t += 0.05


@case
def pursue_caps():
    r = proxy_route("cw")
    for s in [i * 0.5 for i in range(int(r.L / 0.5))]:
        x, y = r.point_at(s)
        for dyaw in (-0.6, 0.0, 0.6):
            v, w, a = tc.pursue(r, x, y, r.yaw_at(s) + dyaw, s, 0.8, 2.0)
            assert 0.0 <= v <= 0.8 and abs(w) <= tc.W_MAX + 1e-9, (s, v, w)
    x, y = r.point_at(10.0)
    v, w, a = tc.pursue(r, x, y, r.yaw_at(10.0), 10.0, 0.8, 2.0)
    near(v, 0.8, 1e-9, "직선 v")
    near(w, 0.0, 1e-6, "직선 ω")


@case
def slew_limits():
    v = 0.0
    for _ in range(20):
        v = tc.slew(v, 0.8, 0.5, 1.0, 0.05)
    near(v, 0.5, 1e-9, "0.5 m/s² × 1 s")
    w = 0.5
    w = tc.slew(w, -0.5, 1.5, 2.0, 0.5)              # 0 까지 0.25 s(2.0) 뒤 0.25 s 동안 1.5 로 반대쪽
    near(w, -0.375, 1e-9, "부호 바뀜")


def drive(route, laps, duty, seed, mode="hold", start=(0.0, 0.0, 0.0), dt=0.05, lag=0.15, t_max=3000.0,
          amp=(0.3, 0.6), label_only=False):
    """운동학 모의 — 단륜차 + 1차 지연(lag s). TeacherCore 를 노드와 같은 순서로 부른다. start = (s, 횡 오프셋, 방향 오프셋)."""
    noise = tc.SteerNoise(duty=duty, amp=amp, rng=random.Random(seed + 1000)) if duty > 0 else None
    core = tc.TeacherCore(route, laps=laps, noise=noise, noise_mode=mode, rng=random.Random(seed), label_only=label_only)
    s0, off, dyaw = start
    x, y = route.point_at(s0)
    yaw0 = route.yaw_at(s0)
    x, y, yaw = x - off * math.sin(yaw0), y + off * math.cos(yaw0), tc.wrap(yaw0 + dyaw)
    va = wa = t = 0.0
    events, outs = [], []
    while t < t_max:
        o = core.step(t, dt, x, y, yaw, va)
        if o["event"]:
            events.append(o["event"])
        outs.append(o)
        if o["phase"] in ("DONE", "FAIL"):
            break
        va += (o["v_exe"] - va) * dt / lag
        wa += (o["w_exe"] - wa) * dt / lag
        x += va * math.cos(yaw) * dt
        y += va * math.sin(yaw) * dt
        yaw = tc.wrap(yaw + wa * dt)
        t += dt
    fol = [o for o in outs if o["phase"] == "FOLLOW"]
    st = {"t": t, "events": events, "phases": sorted({o["phase"] for o in outs}),
          "max_cte": max((abs(o["cte"]) for o in fol), default=0.0),
          "recover": sum(abs(o["cte"]) > 0.3 for o in fol) / max(1, len(fol)),
          "noise": sum(abs(o["nw"]) > 1e-9 for o in fol) / max(1, len(fol)),
          "w_lab": max(abs(o["w_lab"]) for o in outs), "w_exe": max(abs(o["w_exe"]) for o in outs),
          "v_lab": max(o["v_lab"] for o in outs), "end_gap": math.hypot(x - route.point_at(s0)[0], y - route.point_at(s0)[1])}
    return core, st


@case
def kinematic_loop_clean():
    for d in ("cw", "ccw"):
        core, st = drive(proxy_route(d), 2, 0.0, 1, start=(2.0, 0.0, 0.0))  # Proxy 시작 자세는 SW 꼭짓점 2 m 뒤
        assert st["events"].count("LAP_END") == 2 and st["events"][-1] == "DONE", st["events"]
        assert st["events"].count("DONE") == 1 and not any(e.startswith("FAIL") for e in st["events"]), st["events"]
        assert "DWELL" in st["phases"], st["phases"]
        assert st["max_cte"] < 1.0, st
        assert st["w_lab"] <= tc.W_MAX + 1e-9 and st["w_exe"] <= tc.W_MAX + 1e-9 and st["v_lab"] <= tc.V_MAX + 1e-9, st
        assert st["end_gap"] < 0.5, st                 # 출발점 근처에서 멈춤(목표 반경 1.0 m 안) — SW 회전 직후라 옆으로 0.3 m 남음. 꼭짓점에서 출발하면 0.7 m


@case
def kinematic_loop_noise_hold():
    for d in ("cw", "ccw"):
        for seed in (3, 7):
            core, st = drive(proxy_route(d), 2, 0.3, seed)
            assert st["events"].count("LAP_END") == 2, (d, seed, st["events"])
            assert st["max_cte"] < 2.0, (d, seed, st)  # 평가 개입 기준(2.0 m) 안에서 복구
            assert st["w_exe"] <= tc.W_MAX + 1e-9 and st["w_lab"] <= tc.W_MAX + 1e-9
            assert st["recover"] > 0.1, (d, seed, st)


@case
def hold_wider_than_add():
    r = proxy_route("cw")
    rec = {m: sum(drive(r, 2, 0.3, sd, mode=m)[1]["recover"] for sd in range(4)) / 4 for m in ("hold", "add")}
    assert rec["hold"] > rec["add"] * 1.3, rec


@case
def align_then_follow():
    core, st = drive(proxy_route("ccw"), 1, 0.0, 2, start=(0.0, 0.0, math.pi))  # 반대 방향을 보고 출발
    assert "ALIGN" in st["phases"] and st["events"][0] == "LAP_START" and st["events"][-1] == "DONE", st


@case
def label_only_counts_laps_without_stopping():
    core, st = drive(proxy_route("cw"), 1, 0.3, 4, label_only=True)
    assert "DWELL" not in st["phases"] and "ALIGN" not in st["phases"], st["phases"]
    assert st["noise"] == 0.0 and st["events"].count("LAP_END") == 1, st


@case
def start_off_route_fails():
    core, st = drive(proxy_route("cw"), 1, 0.0, 1, start=(10.0, 3.5, 0.0))
    assert st["events"] == ["FAIL:START_OFF_ROUTE"], st["events"]


@case
def csv_route_closes_and_smooths():
    rng = random.Random(5)
    pts = []
    for k in range(4):                              # 40 × 20 m 직사각형 1.1바퀴, 잔떨림 ±5 cm
        corners = [(0, 0), (40, 0), (40, 20), (0, 20), (0, 0)]
        (ax, ay), (bx, by) = corners[k], corners[k + 1]
        n = int(math.hypot(bx - ax, by - ay) / 0.3)
        for i in range(n):
            pts.append((ax + (bx - ax) * i / n + rng.uniform(-0.05, 0.05), ay + (by - ay) * i / n + rng.uniform(-0.05, 0.05)))
    pts += [(x, y) for x, y in pts[:20]]
    with tempfile.NamedTemporaryFile("w", suffix=".csv", delete=False) as f:
        f.write("run,x,y,z\n")
        for x, y in pts:
            f.write(f"0,{x:.3f},{y:.3f},0\n")
        for x, y in pts[:30]:
            f.write(f"1,{x:.3f},{y:.3f},0\n")
        path = f.name
    p, k, gap = tc.load_csv_route(path, 0)
    os.unlink(path)
    assert gap < 0.3, gap
    raw = tc.Route(p, step=0.25)
    sm = tc.Route(p, step=0.25, smooth=1.0)
    near(raw.L, 120.0, 1.5, "raw L")
    jit = lambda r: sum(abs(tc.wrap(r.seg_yaw[i + 1] - r.seg_yaw[i])) for i in range(r.n - 1))  # noqa: E731
    assert jit(sm) < jit(raw) * 0.6, (jit(sm), jit(raw))
    w = tc.turn_windows(sm)
    assert len(w) == 4 and all(x[2] == tc.LEFT for x in w), [(round(a[4], 1), a[2], round(a[3])) for a in w]


@case
def start_pose_from_scenes():
    pr = tc.start_pose_from_usda(os.path.join(HERE, "scenes", "parking", "parking_v0.4b_r3.usda"))
    assert pr is not None
    near(pr[0], -1.756, 1e-9, "x")              # 10.13 기준선 출발점(eval/routes/parking_r3_cw.yaml)으로 옮김
    near(pr[1], -7.621, 1e-9, "y")
    near(math.degrees(pr[2]), -96.0, 1e-9, "yaw")
    pp = tc.start_pose_from_usda(os.path.join(HERE, "scenes", "proxy", "robot.usda"))
    assert pp is not None
    near(pp[0], looppath.X0, 1e-9, "x")
    near(pp[1], looppath.Y0, 1e-9, "y")
    near(pp[2], looppath.PSI0, 1e-9, "yaw")
    assert tc.start_pose_from_usda(os.path.join(HERE, "scenes", "proxy", "proxy_parking.usda")) is None


@case
def odom_to_map_matches_looppath():
    for ox, oy, oyaw in ((0, 0, 0), (3.0, -1.0, 0.4), (-2.0, 5.0, -2.0)):
        a = tc.odom_to_map((looppath.X0, looppath.Y0, looppath.PSI0), ox, oy, oyaw)
        b = looppath.odom_to_map(ox, oy, oyaw)
        for u, v in zip(a, b):
            near(u, v, 1e-9, "odom→map")


if __name__ == "__main__":
    bad = [(n, e) for n, e in RESULTS if e]
    for n, e in RESULTS:
        print(f"{'ok ' if e is None else 'NG '} {n}" + (f" — {e}" if e else ""))
    print(f"TEACHER_CORE_TESTS {len(RESULTS) - len(bad)}/{len(RESULTS)}")
    sys.exit(1 if bad else 0)
