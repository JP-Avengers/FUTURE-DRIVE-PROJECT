"""루프 중심선 폴리라인 · odom→map 변환 공용 모듈 (sim/pursuit.py 가 쓴다)."""
import math
import os
import re

PARAMS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "scenes", "proxy", "params.yaml")
X0, Y0, PSI0 = -0.3, 2.0, math.radians(97.3)   # 로봇 시작 자세 (map) — params.yaml 의 robot.start 와 같게


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def load_loop(params=PARAMS):
    """params.yaml 의 loop 점 (이름, x, y) 리스트 — 시계 방향 순서."""
    out = []
    for line in open(params, encoding="utf-8"):
        m = re.match(r"\s*-\s*\{name:\s*(\w+),\s*x:\s*([-\d.]+),\s*y:\s*([-\d.]+)\}", line)
        if m:
            out.append((m.group(1), float(m.group(2)), float(m.group(3))))
    assert len(out) == 4, out
    return out


class Path:
    """닫힌 폴리라인. direction 'cw' 는 params 순서, 'ccw' 는 SW 부터 거꾸로."""

    def __init__(self, direction="cw", params=PARAMS):
        v = load_loop(params)
        if direction == "ccw":
            v = [v[0]] + v[1:][::-1]
        self.names = [n for n, _, _ in v]
        self.pts = [(x, y) for _, x, y in v] + [(v[0][1], v[0][2])]
        self.seg_len = [math.hypot(self.pts[i + 1][0] - self.pts[i][0], self.pts[i + 1][1] - self.pts[i][1])
                        for i in range(4)]
        self.cum = [0.0]
        for l in self.seg_len:
            self.cum.append(self.cum[-1] + l)
        self.L = self.cum[-1]
        self.seg_yaw = [math.atan2(self.pts[i + 1][1] - self.pts[i][1], self.pts[i + 1][0] - self.pts[i][0])
                        for i in range(4)]

    def point_at(self, s):
        s %= self.L
        for i in range(4):
            if s <= self.cum[i + 1] or i == 3:
                t = (s - self.cum[i]) / self.seg_len[i]
                return (self.pts[i][0] + t * (self.pts[i + 1][0] - self.pts[i][0]),
                        self.pts[i][1] + t * (self.pts[i + 1][1] - self.pts[i][1]))

    def yaw_at(self, s):
        s %= self.L
        for i in range(4):
            if s <= self.cum[i + 1] or i == 3:
                return self.seg_yaw[i]

    def project(self, x, y, hint=None, back=1.0, fwd=3.0):
        """(s, |cte|). hint 가 있으면 hint-back ~ hint+fwd 안의 후보만 쓴다(루프 반대편으로 안 튐)."""
        cands = []
        for i in range(4):
            ax, ay = self.pts[i]
            dx, dy = self.pts[i + 1][0] - ax, self.pts[i + 1][1] - ay
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy)))
            d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
            cands.append(((self.cum[i] + t * self.seg_len[i]) % self.L, d))
        if hint is None:
            return min(cands, key=lambda c: c[1])
        ok = []
        for s, d in cands:
            dl = (s - hint + self.L / 2) % self.L - self.L / 2
            if -back <= dl <= fwd:
                ok.append((s, d))
        if ok:
            return min(ok, key=lambda c: c[1])
        return min(cands, key=lambda c: abs((c[0] - hint + self.L / 2) % self.L - self.L / 2))

    def dist_to_vertex(self, x, y):
        """(가장 가까운 꼭짓점 이름, 거리)."""
        best = min(range(4), key=lambda i: math.hypot(x - self.pts[i][0], y - self.pts[i][1]))
        return self.names[best], math.hypot(x - self.pts[best][0], y - self.pts[best][1])


def odom_to_map(ox, oy, oyaw):
    c, s = math.cos(PSI0), math.sin(PSI0)
    return X0 + c * ox - s * oy, Y0 + s * ox + c * oy, wrap(PSI0 + oyaw)


def quat_rpy(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw
