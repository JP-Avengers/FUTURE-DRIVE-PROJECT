"""자동 교사 계산부 — 경로 · 경로 지시 · 조향 잡음 · pure pursuit (ROS 없음, sim/teacher.py 가 쓴다).

ROS 없이 시험할 수 있도록 노드에서 떼어 냈다: python3 sim/test_teacher_core.py
좌표는 맵 좌표(Proxy = ENU, 실제 주차장 = B 맵 좌표). 경로는 닫힌 폴리라인이고 진행 방향은 점 순서다.
"""
import bisect
import csv
import math
import random
import re

V_MAX, W_MAX = 0.8, 0.7          # 규격서 상한 — 키보드 램프 · Nav2 기본 설정과 같음 (docs/interfaces/topics.md)
ROBOT_V_MAX, ROBOT_W_MAX = 1.0, 1.2  # 로봇 에셋 한도(sim/teleop_ramp.py 와 같음) — 넘는 상한은 받지 않는다


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def slew(cur, tgt, acc, dec, dt):
    """cur 를 tgt 쪽으로 가감속 한도(단위/s²)만큼만 움직인다 — sim/teleop_ramp.py 의 slew 와 같은 식(라벨 성질을 사람 조작과 맞춤)."""
    if cur * tgt < 0:
        to_zero = abs(cur) / dec
        if dt <= to_zero:
            return cur - math.copysign(dec * dt, cur)
        cur, dt = 0.0, dt - to_zero
    step = (dec if abs(tgt) < abs(cur) else acc) * dt
    return tgt if abs(tgt - cur) <= step else cur + math.copysign(step, tgt - cur)


# ── 경로 ──────────────────────────────────────────────────────────────

def _dedup(pts, eps=1e-6):
    out = []
    for p in pts:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > eps:
            out.append((float(p[0]), float(p[1])))
    if len(out) > 1 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= eps:
        out.pop()  # 닫는 점은 따로 붙인다
    return out


def _resample_closed(pts, step):
    """닫힌 폴리라인을 첫 점부터 step 간격으로 다시 찍는다(꼭짓점은 step 안에서 깎임)."""
    ring = pts + [pts[0]]
    cum = [0.0]
    for i in range(len(pts)):
        cum.append(cum[-1] + math.hypot(ring[i + 1][0] - ring[i][0], ring[i + 1][1] - ring[i][1]))
    L = cum[-1]
    n = max(3, int(round(L / step)))
    out, j = [], 0
    for k in range(n):
        s = L * k / n
        while j < len(pts) - 1 and cum[j + 1] <= s:
            j += 1
        seg = cum[j + 1] - cum[j]
        t = 0.0 if seg < 1e-12 else (s - cum[j]) / seg
        out.append((ring[j][0] + t * (ring[j + 1][0] - ring[j][0]), ring[j][1] + t * (ring[j + 1][1] - ring[j][1])))
    return out


def _smooth_closed(pts, half_m, step):
    """닫힌 점열 이동 평균(앞뒤 half_m) — 실측 경로의 잔떨림을 라벨에 넣지 않으려고. 첫 점 위치도 함께 움직인다."""
    k = int(round(half_m / step))
    if k < 1:
        return pts
    n = len(pts)
    return [(sum(pts[(i + d) % n][0] for d in range(-k, k + 1)) / (2 * k + 1),
             sum(pts[(i + d) % n][1] for d in range(-k, k + 1)) / (2 * k + 1)) for i in range(n)]


class Route:
    """닫힌 폴리라인 경로. s 는 첫 점에서 진행 방향으로 잰 거리(0 ≤ s < L)."""

    def __init__(self, pts, name="", step=0.25, smooth=0.0):
        pts = _dedup(pts)
        if len(pts) < 3:
            raise ValueError(f"경로 점이 너무 적음: {len(pts)}")
        if step:
            pts = _resample_closed(pts, step)
            if smooth > 0:
                pts = _smooth_closed(pts, smooth, step)
        self.name, self.step, self.smooth = name, step, smooth
        self.n = len(pts)
        self.pts = pts + [pts[0]]
        self.cum = [0.0]
        for i in range(self.n):
            self.cum.append(self.cum[-1] + math.hypot(self.pts[i + 1][0] - self.pts[i][0],
                                                      self.pts[i + 1][1] - self.pts[i][1]))
        self.L = self.cum[-1]
        self.seg_yaw = [math.atan2(self.pts[i + 1][1] - self.pts[i][1], self.pts[i + 1][0] - self.pts[i][0])
                        for i in range(self.n)]

    def seg_index(self, s):
        s %= self.L
        return min(self.n - 1, max(0, bisect.bisect_right(self.cum, s) - 1))

    def point_at(self, s):
        s %= self.L
        i = self.seg_index(s)
        seg = self.cum[i + 1] - self.cum[i]
        t = 0.0 if seg < 1e-12 else (s - self.cum[i]) / seg
        return (self.pts[i][0] + t * (self.pts[i + 1][0] - self.pts[i][0]),
                self.pts[i][1] + t * (self.pts[i + 1][1] - self.pts[i][1]))

    def yaw_at(self, s):
        return self.seg_yaw[self.seg_index(s)]

    def _seg_proj(self, i, x, y):
        ax, ay = self.pts[i]
        dx, dy = self.pts[i + 1][0] - ax, self.pts[i + 1][1] - ay
        l2 = dx * dx + dy * dy
        t = 0.0 if l2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / l2))
        fx, fy = ax + t * dx, ay + t * dy
        d = math.hypot(x - fx, y - fy)
        side = 1.0 if dx * (y - ay) - dy * (x - ax) >= 0 else -1.0  # 진행 방향 왼쪽이 +
        return (self.cum[i] + t * math.sqrt(l2)) % self.L, side * d

    def project(self, x, y, hint=None, back=1.5, fwd=10.0):
        """(s, cte) — cte 는 진행 방향 왼쪽이 +. hint 가 있으면 hint-back ~ hint+fwd 안에서만 찾는다(모서리를 질러도 앞
        구간을 찾도록 fwd 를 넉넉히 — 10.7 X-Mobility 판정 도구가 3 m 창 때문에 이전 변으로 오판했음)."""
        if hint is None:
            idx = range(self.n)
        else:
            i0 = self.seg_index(hint - back)
            cnt = int((back + fwd) / max(self.step or 0.25, 1e-3)) + 3
            idx = [(i0 + k) % self.n for k in range(min(cnt, self.n))]
        best = None
        for i in idx:
            s, c = self._seg_proj(i, x, y)
            if hint is not None:
                dl = (s - hint + self.L / 2) % self.L - self.L / 2
                if not -back <= dl <= fwd:
                    continue
            if best is None or abs(c) < abs(best[1]):
                best = (s, c)
        return best if best is not None else self.project(x, y)

    def reversed(self):
        """반대 방향 경로 — 출발점(첫 점)은 그대로."""
        p = self.pts[:-1]
        r = Route([p[0]] + p[1:][::-1], name=self.name + "/rev", step=0)  # 이미 고르게 찍힌 점 — 다시 찍지 않음
        r.step, r.smooth = self.step, self.smooth
        return r


def load_csv_route(path, run=0):
    """C 의 경로 CSV(run, x, y, …) 한 회차를 닫힌 루프로 — 출발점에 가장 가까이 돌아온 곳(뒤쪽 절반)에서 자른다
    (logs/parking_v04b/tools/route_pursuit.py 와 같은 규칙). (점들, 자른 곳, 닫힘 틈 m)."""
    rows = [r for r in csv.DictReader(open(path, encoding="utf-8")) if r.get("run", "0") == str(run)]
    pts = [(float(r["x"]), float(r["y"])) for r in rows]
    if len(pts) < 10:
        raise ValueError(f"{path} 회차 {run}: 점 {len(pts)}개")
    half = len(pts) // 2
    k = min(range(half, len(pts)), key=lambda i: math.hypot(pts[i][0] - pts[0][0], pts[i][1] - pts[0][1]))
    return pts[:k + 1], k, math.hypot(pts[k][0] - pts[0][0], pts[k][1] - pts[0][1])


def start_pose_from_usda(path):
    """씬 레이어의 Nova_Carter_ROS 시작 자세 (x, y, yaw_rad) — over · def 둘 다. 못 찾으면 None."""
    try:
        txt = open(path, encoding="utf-8").read()
    except OSError:
        return None
    m = re.search(r'(?:over|def)[^"\n]*"Nova_Carter_ROS"', txt)
    if not m:
        return None
    blk = txt[m.end():m.end() + 2000]
    rz = re.search(r'xformOp:rotateZ\s*=\s*([-\d.eE+]+)', blk)
    tr = re.search(r'xformOp:translate\s*=\s*\(\s*([-\d.eE+]+)\s*,\s*([-\d.eE+]+)', blk)
    if not (rz and tr):
        return None
    return float(tr.group(1)), float(tr.group(2)), math.radians(float(rz.group(1)))


def odom_to_map(pose0, ox, oy, oyaw):
    """odom(로봇 시작 = 원점) → 맵. pose0 = 시작 자세 (x, y, yaw)."""
    x0, y0, p0 = pose0
    c, s = math.cos(p0), math.sin(p0)
    return x0 + c * ox - s * oy, y0 + s * ox + c * oy, wrap(p0 + oyaw)


# ── 경로 지시 ─────────────────────────────────────────────────────────

STRAIGHT, LEFT, RIGHT = 0, 1, 2   # /route_command (topics.md 제안: 0 직진 · 1 좌회전 · 2 우회전)


def turn_windows(route, thresh_deg=40.0, half=3.0, before=5.0, after=2.0):
    """경로 기하로 회전 구간을 찾는다. 앞뒤 half m 사이 진행 방향 변화가 thresh 를 넘는 곳을 한 덩어리로 보고,
    그 가운데(꼭짓점)에서 before m 앞 ~ after m 뒤를 좌 · 우회전 구간으로 둔다. eval/routes/loop_cw.yaml 초안의
    규칙(꼭짓점 앞 5 m ~ 회전 끝, 26° BEND 는 직진)과 맞춘 값. [(시작 s, 끝 s, 지시, 꺾인 각 deg, 꼭짓점 s)]"""
    n, L = route.n, route.L
    step = L / n
    k = max(1, int(round(half / step)))
    delta = [wrap(route.seg_yaw[(i + k) % n] - route.seg_yaw[(i - k) % n]) for i in range(n)]
    th = math.radians(thresh_deg)
    mark = [abs(d) >= th for d in delta]
    if all(mark) or not any(mark):
        return []
    start = next(i for i in range(n) if not mark[i])  # 표시 안 된 곳부터 돌아야 덩어리가 0 에서 끊기지 않음
    runs, cur = [], None
    for j in range(1, n + 1):
        i = (start + j) % n
        if mark[i]:
            cur = [i] if cur is None else cur + [i]
        elif cur is not None:
            runs.append(cur)
            cur = None
    if cur is not None:
        runs.append(cur)
    out = []
    for run in runs:
        dmax = max(abs(delta[i]) for i in run)
        plateau = [p for p, i in enumerate(run) if abs(delta[i]) >= 0.95 * dmax]
        # delta[i] 는 구간 i 가운데(s = cum[i] + step/2)의 값으로 본다 — 날카로운 꼭짓점이면 평탄한 덩어리의 가운데가 꼭짓점
        if len(plateau) % 2:
            apex = (route.cum[run[plateau[len(plateau) // 2]]] + step / 2) % L
        else:  # 짝수 개 — 가운데 두 구간 사이
            a, b = run[plateau[len(plateau) // 2 - 1]], run[plateau[len(plateau) // 2]]
            apex = (route.cum[a] + step / 2 + ((route.cum[b] - route.cum[a]) % L) / 2) % L
        sgn = delta[max(run, key=lambda i: abs(delta[i]))]
        out.append(((apex - before) % L, (apex + after) % L, LEFT if sgn > 0 else RIGHT,
                    math.degrees(sgn), apex))
    return out


def command_at(windows, s, L):
    for a, b, cmd, _, _ in windows:
        if (s - a) % L <= (b - a) % L:
            return cmd
    return STRAIGHT


# ── 조향 잡음 ─────────────────────────────────────────────────────────

class SteerNoise:
    """실행 명령에만 더하는 삼각 펄스 조향 잡음(CILRS 수집 방식과 같은 꼴: 시간 일부에, 0.5~2 s, 부호 무작위).
    duty = 잡음이 켜져 있는 시간 비율 목표. 펄스 사이 빈 시간은 지수 분포라 평균 duty 가 맞춰진다."""

    def __init__(self, duty=0.25, dur=(0.5, 2.0), amp=(0.25, 0.55), rng=None):
        if not 0.0 <= duty < 1.0:
            raise ValueError("duty 는 0 이상 1 미만")
        self.duty, self.dur, self.amp = duty, dur, amp
        self.rng = rng or random.Random(0)
        mean_dur = (dur[0] + dur[1]) / 2
        self.rate = duty / (mean_dur * (1.0 - duty)) if duty > 0 else 0.0  # 빈 시간 동안 초당 시작 확률
        self.t0, self.T, self.A = None, 0.0, 0.0
        self.on_time = 0.0

    def active(self):
        return self.t0 is not None

    def step(self, t, dt, enabled=True):
        """시뮬 시각 t 에서의 잡음 ω. enabled=False 면 새 펄스를 시작하지 않는다(진행 중인 펄스는 마저 끝냄)."""
        if self.t0 is not None:
            u = (t - self.t0) / self.T
            if u >= 1.0 or u < 0.0:
                self.t0 = None
            else:
                self.on_time += dt
                return self.A * (1.0 - abs(2.0 * u - 1.0))
        if enabled and self.rate > 0 and self.rng.random() < self.rate * dt:
            self.t0, self.T = t, self.rng.uniform(*self.dur)
            self.A = self.rng.uniform(*self.amp) * self.rng.choice((-1.0, 1.0))
        return 0.0


# ── 조향 ─────────────────────────────────────────────────────────────

def pursue(route, x, y, yaw, s, v_tgt, lookahead, w_max=W_MAX, w_margin=0.9, v_min=0.15, spin_deg=75.0):
    """pure pursuit (sim/pursuit.py 와 같은 식 ω = 2 v sin α / d). 곡률이 커서 |ω| 가 상한의 w_margin 을 넘으면 속도를
    줄여 상한 안에서 돈다. 목표점이 spin_deg 넘게 옆 · 뒤면 제자리 회전. (v, ω, α)"""
    tx, ty = route.point_at(s + lookahead)
    alpha = wrap(math.atan2(ty - y, tx - x) - yaw)
    if abs(alpha) > math.radians(spin_deg):
        return 0.0, math.copysign(0.8 * w_max, alpha), alpha
    d = max(0.5, math.hypot(tx - x, ty - y))
    kappa = 2.0 * math.sin(alpha) / d
    v = v_tgt
    if abs(kappa) * v > w_margin * w_max:
        v = max(v_min, w_margin * w_max / abs(kappa))
    return v, max(-w_max, min(w_max, v * kappa)), alpha


# ── 단계 계산 (노드 · 시험 공용) ───────────────────────────────────────

class TeacherCore:
    """자동 교사 한 틱 계산 — sim/teacher.py(ROS)와 sim/test_teacher_core.py(운동학 모의)가 같은 코드를 쓴다.

    단계: INIT → (ALIGN: 진행 방향과 20° 넘게 다르면 제자리 회전) → FOLLOW → (바퀴 끝: 출발점에서 감속 정지 → DWELL) → … → DONE.
    라벨 = pure pursuit 명령(상한 · 곡률 감속 · 회전 구간 감속 · 정지 감속)을 키보드 램프와 같은 가감속 한도로 다듬은 값.
    실행 = 라벨 + 조향 잡음. noise_mode 'hold' 는 펄스 동안 교정을 멈추고(펄스 시작 때 라벨 ω 고정) 잡음을 더해 더 크게 벗어나게
    하고, 'add' 는 교사가 계속 고치는 위에 더한다. 어느 쪽이든 라벨은 그 자리에서 교사가 낼 깨끗한 명령이다.
    label_only 면 실행 명령을 내지 않는 쪽(다른 노드가 몰 때 — DAgger)이라 ALIGN · 정지 · 실패 판정을 하지 않는다."""

    def __init__(self, route, laps=1, v_range=(0.5, 0.8), v_change_m=30.0, v_turn=0.5, lookahead=2.0,
                 noise=None, noise_mode="hold", rng=None, stop_each_lap=True, dwell=2.0, a_stop=0.5,
                 label_only=False, offroad=3.0, stall_s=10.0, stall_m=0.3, v_max=V_MAX, w_max=W_MAX,
                 v_acc=0.5, v_dec=1.0, w_acc=1.5, w_dec=2.0, align_deg=20.0, noise_free_start=2.0,
                 noise_free_end=5.0, recover_m=0.3):
        if not (0 < v_range[0] <= v_range[1] <= v_max <= ROBOT_V_MAX and 0 < w_max <= ROBOT_W_MAX):
            raise ValueError(f"속도 범위 {v_range} · 상한 v {v_max} · ω {w_max} — 로봇 한도 {ROBOT_V_MAX} · {ROBOT_W_MAX} 안이어야 함")
        if noise_mode not in ("hold", "add"):
            raise ValueError(noise_mode)
        self.r, self.laps, self.v_range, self.v_change_m, self.v_turn = route, laps, v_range, v_change_m, v_turn
        self.lookahead, self.noise, self.noise_mode = lookahead, noise, noise_mode
        self.rng = rng or random.Random(0)
        self.stop_each_lap, self.dwell, self.a_stop, self.label_only = stop_each_lap, dwell, a_stop, label_only
        self.offroad, self.stall_s, self.stall_m = offroad, stall_s, stall_m
        self.v_max, self.w_max = v_max, w_max
        self.v_acc, self.v_dec, self.w_acc, self.w_dec = v_acc, v_dec, w_acc, w_dec
        self.align, self.nf0, self.nf1, self.recover_m = math.radians(align_deg), noise_free_start, noise_free_end, recover_m
        self.windows = turn_windows(route)
        self.phase, self.lap = "INIT", 0
        self.v_lab = self.w_lab = 0.0
        self.w_hold = None
        self.fail_reason = None
        self._done_sent = False
        self.pulse_hold = False
        self.s_prev = self.cum = self.lap_cum0 = 0.0
        self.v_tgt, self.v_next = v_range[1], 0.0
        self.hist = []
        self._lap_reset(0.0)

    # 바퀴 통계 — LAP_END 때 돌려줌
    def _lap_reset(self, t):
        self.lap_t0 = t
        self.st = {"n": 0, "cte_sum": 0.0, "cte_max": 0.0, "noise_s": 0.0, "recover_s": 0.0, "follow_s": 0.0,
                   "v_sum": 0.0, "turn_s": 0.0}
        self.hist = []

    def _new_v(self):
        self.v_tgt = self.rng.uniform(*self.v_range)
        self.v_next = self.cum + self.rng.uniform(0.5, 1.5) * self.v_change_m

    def _fail(self, why):
        self.phase, self.fail_reason = "FAIL", why
        return why

    def step(self, t, dt, x, y, yaw, v_meas=0.0):
        """한 틱. 돌려주는 dict: v_lab · w_lab(라벨) · v_exe · w_exe(실행) · cmd(경로 지시) · phase · lap · s · cte · he(경로
        방향 오차 rad) · nw(잡음 ω, 실행 − 라벨과 다를 수 있음 — hold) · v_tgt · event(None | LAP_START | LAP_END | DONE | FAIL:이유)
        · lap_stats(LAP_END 때)."""
        L, ev, stats = self.r.L, None, None
        v_pp = w_pp = 0.0
        if self.phase == "INIT":
            s, cte = self.r.project(x, y)
            self.s_prev, self.cum, self.lap_cum0 = s, 0.0, 0.0
            self._new_v()
            self._lap_reset(t)
            if abs(cte) > self.offroad and not self.label_only:
                ev = "FAIL:" + self._fail("START_OFF_ROUTE")
            elif abs(wrap(self.r.yaw_at(s) - yaw)) > self.align and not self.label_only:
                self.phase = "ALIGN"
            else:
                self.phase, ev = "FOLLOW", "LAP_START"
        s, cte = self.r.project(x, y, hint=self.s_prev)
        he = wrap(self.r.yaw_at(s) - yaw)
        if self.phase in ("ALIGN", "FOLLOW"):
            self.cum += (s - self.s_prev + L / 2) % L - L / 2
            self.s_prev = s
        lap_cum = self.cum - self.lap_cum0
        rem = L - lap_cum
        cmd = STRAIGHT
        if self.phase == "ALIGN":
            he_tgt = wrap(self.r.yaw_at(s) - yaw)
            if abs(he_tgt) < math.radians(3.0):
                self.phase, ev = "FOLLOW", "LAP_START"
                self.cum, self.lap_cum0, lap_cum, rem = 0.0, 0.0, 0.0, L
                self._lap_reset(t)
                self._new_v()
            else:
                w_pp = math.copysign(max(0.1, min(0.8 * self.w_max, abs(he_tgt))), he_tgt)
        elif self.phase == "FOLLOW":
            st = self.st
            if not self.label_only:
                if abs(cte) > self.offroad:
                    ev = "FAIL:" + self._fail("OFFROAD")
                elif t - self.lap_t0 > 3.0 * L / self.v_range[0] + 60.0:
                    ev = "FAIL:" + self._fail("TIMEOUT")
                else:
                    self.hist.append((t, x, y))
                    while len(self.hist) > 1 and self.hist[1][0] <= t - self.stall_s:
                        self.hist.pop(0)
                    if t - self.hist[0][0] >= self.stall_s and math.hypot(x - self.hist[0][1], y - self.hist[0][2]) < self.stall_m:
                        ev = "FAIL:" + self._fail("STALL")
            if self.phase == "FOLLOW":
                if self.cum >= self.v_next:
                    self._new_v()
                vt = self.v_tgt
                if command_at(self.windows, s + 1.0, L) != STRAIGHT:
                    vt = min(vt, self.v_turn)
                stopping = self.stop_each_lap and not self.label_only
                if stopping:
                    vt = min(vt, math.sqrt(2.0 * self.a_stop * max(0.0, rem - 0.05)))
                v_pp, w_pp, _ = pursue(self.r, x, y, yaw, s, vt, self.lookahead, self.w_max)
                cmd = command_at(self.windows, s, L)
                st["n"] += 1
                st["cte_sum"] += abs(cte)
                st["cte_max"] = max(st["cte_max"], abs(cte))
                st["follow_s"] += dt
                st["v_sum"] += v_meas
                st["recover_s"] += dt if abs(cte) > self.recover_m else 0.0
                st["turn_s"] += dt if cmd != STRAIGHT else 0.0
                lap_end = (rem <= 0.05 or (rem < 0.3 and abs(v_meas) < 0.02 and vt < 0.1)) if stopping else lap_cum >= L
                if lap_end:
                    self.lap += 1
                    stats = dict(st, lap=self.lap, t=t - self.lap_t0)
                    ev = "LAP_END"
                    if stopping:
                        self.phase, self.dwell_t0 = "DWELL", t
                        v_pp = w_pp = 0.0
                    else:
                        self.lap_cum0 += L
                        self._lap_reset(t)
                        if self.laps and self.lap >= self.laps:
                            self.phase = "DONE"
        elif self.phase == "DWELL":
            if t - self.dwell_t0 >= self.dwell:
                if self.laps and self.lap >= self.laps:
                    self.phase, ev = "DONE", "DONE"
                else:
                    self.phase, ev = "FOLLOW", "LAP_START"
                    self.lap_cum0 = self.cum
                    self._lap_reset(t)
                    self._new_v()
        if self.phase in ("DONE", "FAIL"):
            v_pp = w_pp = 0.0
            if self.phase == "DONE" and not self._done_sent and ev is None:
                ev = "DONE"
            self._done_sent = self._done_sent or ev == "DONE"
        self.v_lab = slew(self.v_lab, max(0.0, min(self.v_max, v_pp)), self.v_acc, self.v_dec, dt)
        self.w_lab = slew(self.w_lab, max(-self.w_max, min(self.w_max, w_pp)), self.w_acc, self.w_dec, dt)
        nw, w_exe = 0.0, self.w_lab
        if self.noise is not None and not self.label_only:
            if self.phase == "FOLLOW":
                was = self.noise.active()
                nw = self.noise.step(t, dt, enabled=(self.cum - self.lap_cum0) > self.nf0 and rem > self.nf1)
                if self.noise.active() and not was:
                    # 교정을 멈추는 hold 는 직선에서만 — 펄스(최대 2 s) 안에 회전 구간이 오면 모서리를 크게 지나칠 수 있어
                    # 그 펄스는 교사가 계속 고치는 add 로 넣는다
                    straight = all(command_at(self.windows, s + d, L) == STRAIGHT for d in (0.0, 1.5, 3.0))
                    self.pulse_hold = self.noise_mode == "hold" and straight
                    self.w_hold = None
                if self.noise.active() and self.pulse_hold:
                    if self.w_hold is None:
                        self.w_hold = self.w_lab
                    w_exe = self.w_hold + nw
                else:
                    self.w_hold = None
                    w_exe = self.w_lab + nw
                if self.noise.active():
                    self.st["noise_s"] += dt
            else:
                self.noise.t0, self.w_hold = None, None
        w_exe = max(-self.w_max, min(self.w_max, w_exe))
        return {"v_lab": self.v_lab, "w_lab": self.w_lab, "v_exe": self.v_lab, "w_exe": w_exe, "cmd": cmd,
                "phase": self.phase, "lap": self.lap, "s": s, "cte": cte, "he": he, "nw": w_exe - self.w_lab,
                "v_tgt": self.v_tgt, "event": ev, "lap_stats": stats}
