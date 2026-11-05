"""X-Mobility 추론 노드의 순수 계산부 — numpy(이미지는 Pillow) 만 쓰고 rclpy · torch · X-Mobility 는 import 안 함.

학습 형식과 같게 만드는 것이 목적이라 학습 데이터를 만든 코드의 식을 그대로 옮겼다. 줄 번호는 이 레포(sim/ · e2e/)와
X-Mobility 5837aea(ros2_deployment/x_mobility_navigator/x_mobility_navigator/x_mobility_navigator.py = navigator.py) 기준.

  이미지   rgb8 → 512×320 Pillow BILINEAR → JPEG q90 → 디코드 → (3, 320, 512) float32 /255
           = sim/export_xmobility.py:234-236(학습 parquet 만들기) + X-Mobility isaac_sim_dataset.py:322-324(학습 로더)
             · e2e/eval_offline.py:98-104(오프라인 평가) · navigator.py:288-293(배포)
  경로     로봇 위치 + 기준선 앞 0.5 m 간격 19점 → 로봇 좌표계 20점 → (19, 4) 구간 벡터
           = sim/export_xmobility.py:238-246 · navigator.py:149-170 · e2e/eval_offline.py:86-95
"""
import bisect
import csv
import io
import math

import numpy as np

NUM_ROUTE_POINTS = 20                # navigator.py:41
ROUTE_VECTOR_SIZE = 4                # navigator.py:43
ROUTE_SPACING = 0.5                  # sim/export_xmobility.py:51
OUT_W, OUT_H = 512, 320              # sim/export_xmobility.py:49 = INPUT_IMAGE_SIZE (320, 512)(data_constants.py:24)
JPEG_Q = 90                          # sim/export_xmobility.py:50
ACTION_SIZE, PATH_SIZE = 6, 10       # base_train_config.gin:3-5
V_IDX, W_IDX = 0, 5                  # navigator.py:272-273
V_MIN, V_MAX, W_MAX = 0.0, 0.8, 0.7  # docs/interfaces/topics.md:61 — 0 ≤ v ≤ 0.8 m/s, |ω| ≤ 0.7 rad/s
PROJECT_BACK, PROJECT_FWD = 1.0, 10.0   # sim/export_xmobility.py:239 의 project(…, back=1.0, fwd=10.0)


# ---------------------------------------------------------------- 이미지

def image_to_hwc(data, height, width, step, encoding):
    """sensor_msgs/Image 필드 → (H, W, 3) uint8 RGB.

    줄 끝 채움(step > width·3)은 잘라냄 — sim/export_xmobility.py:234 와 같은 식(cv_bridge 안 씀).
    학습 녹화는 rgb8 뿐(export_xmobility.py:233 assert). bgr8 은 채널만 뒤집어 받아 줌.
    """
    enc = str(encoding).lower()
    if enc not in ('rgb8', 'bgr8'):
        raise ValueError(f'encoding {encoding!r} — rgb8(또는 bgr8)만 받음')
    h, w, st = int(height), int(width), int(step)
    if h <= 0 or w <= 0 or st < w * 3:
        raise ValueError(f'이미지 크기가 틀림: {w}x{h} step {st}')
    buf = np.frombuffer(bytes(data), np.uint8)   # export_xmobility.py:234 — bytes(m.data)
    if buf.size < h * st:
        raise ValueError(f'data {buf.size} B < height·step {h * st} B')
    img = buf[:h * st].reshape(h, st)[:, :w * 3].reshape(h, w, 3)
    if enc == 'bgr8':
        img = img[:, :, ::-1]
    return np.ascontiguousarray(img)


def preprocess_rgb(img_hwc, jpeg_quality=JPEG_Q):
    """(H, W, 3) uint8 RGB → (3, 320, 512) float32 [0, 1] — 학습 이미지와 같은 길.

    1. Pillow BILINEAR 로 512×320 (export_xmobility.py:236). Pillow 는 줄일 때 필터 폭을 늘려(안티에일리어싱) 평균하므로
       torch F.interpolate(bilinear, onnx_conversion.py:53-58)와 값이 다르다 — 640×480 을 모델에 그대로 넣지 않는다.
    2. jpeg_quality > 0 이면 JPEG 로 저장했다 다시 읽음(export_xmobility.py:236 quality=90 →
       isaac_sim_dataset.py:323 Image.open). 0 이면 건너뜀(학습과 달라짐).
    3. HWC → CHW, float32 로 바꾼 뒤 /255 (eval_offline.py:104 = navigator.py:292-293).
    """
    from PIL import Image as PILImage
    im = PILImage.fromarray(np.asarray(img_hwc, np.uint8)).resize((OUT_W, OUT_H), PILImage.BILINEAR)
    if jpeg_quality:
        buf = io.BytesIO()
        im.save(buf, 'JPEG', quality=int(jpeg_quality))
        im = PILImage.open(io.BytesIO(buf.getvalue()))
        if im.mode != 'RGB':                                  # eval_offline.py:102-103
            im = im.convert('RGB')
    arr = np.asarray(im)
    return np.ascontiguousarray(arr.transpose(2, 0, 1).astype(np.float32) / 255.0)


def image_to_chw(data, height, width, step, encoding, jpeg_quality=JPEG_Q):
    """sensor_msgs/Image 필드 → 모델 입력 (3, 320, 512) float32."""
    return preprocess_rgb(image_to_hwc(data, height, width, step, encoding), jpeg_quality)


# ---------------------------------------------------------------- 각도 · 좌표 변환

def wrap(a):
    """sim/export_xmobility.py:57-58."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def quat_yaw(x, y, z, w):
    """쿼터니언 → yaw — sim/export_xmobility.py:61-62 와 같은 식(인자만 풀어 씀)."""
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def to_robot(px, py, x, y, yaw):
    """지도 점 (px, py) → 로봇(x, y, yaw) 좌표계 — sim/export_xmobility.py:118-121."""
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = px - x, py - y
    return c * dx + s * dy, -s * dx + c * dy


def robot_to_fixed(points_xy, x, y, yaw):
    """to_robot 의 역 — 로봇 좌표계 점 (N, 2) → 고정 좌표계(odom 등) (N, 2). 경로 점프 판정용."""
    p = np.asarray(points_xy, np.float64).reshape(-1, 2)
    c, s = math.cos(yaw), math.sin(yaw)
    return np.stack([x + c * p[:, 0] - s * p[:, 1], y + s * p[:, 0] + c * p[:, 1]], axis=1)


def odom_to_map(ox, oy, oyaw, start):
    """odom 자세 + 시작 자세 (x0, y0, ψ0°) → 지도 자세 — sim/export_xmobility.py:211-213 = topics.md:22-23."""
    x0, y0, p0 = float(start[0]), float(start[1]), math.radians(float(start[2]))
    c, s = math.cos(p0), math.sin(p0)
    return x0 + c * ox - s * oy, y0 + s * ox + c * oy, wrap(p0 + oyaw)


def transform_points(points_xyz, translation, rotation_xyzw):
    """p' = R(q)·p + t — tf2 do_transform_pose(navigator.py:160-163)의 위치 부분과 같은 식. → (N, 3).

    translation = (x, y, z), rotation = (x, y, z, w) 쿼터니언(정규화해서 씀) — lookup_transform(target, source)
    결과를 그대로 넣으면 source 좌표의 점이 target 좌표로 바뀐다.
    """
    p = np.asarray(points_xyz, np.float64).reshape(-1, 3)
    qx, qy, qz, qw = (float(v) for v in rotation_xyzw)
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if n < 1e-12:
        raise ValueError('쿼터니언 크기가 0')
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    r = np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
    return p @ r.T + np.asarray(translation, np.float64).reshape(1, 3)


# ---------------------------------------------------------------- 경로

def select_route_points(points_xy, n=NUM_ROUTE_POINTS):
    """앞 n 점, 모자라면 끝점 반복 → (n, 2) — navigator.py:149-156."""
    p = np.asarray(points_xy).reshape(-1, 2)
    if len(p) == 0:
        raise ValueError('경로 점이 없음')
    idx = list(range(min(len(p), n)))
    idx += [idx[-1]] * (n - len(idx))
    return p[idx]


def route_vectors(route_poses):
    """경로 점(평평한 x, y … 또는 (N, 2), 로봇 좌표계) → (19, 4) float32 [x_i, y_i, x_i+1, y_i+1].

    앞 20점 · 끝점 반복(navigator.py:149-170). e2e/eval_offline.py:86-95 route_vectors 와 같은 결과,
    20점이면 학습 로더 isaac_sim_dataset.py:326-337 과도 같다.
    """
    sel = select_route_points(np.asarray(route_poses, np.float32).reshape(-1, 2))
    return np.ascontiguousarray(np.concatenate([sel[:-1], sel[1:]], axis=1), dtype=np.float32)


class Poly:
    """닫힌 폴리라인 — sim/export_xmobility.py:69-103 을 그대로 옮김(tools/xm_route.py 의 Poly 와 같다고 적혀 있음)."""

    def __init__(self, pts):
        self.pts = pts + [pts[0]]
        self.n = len(self.pts) - 1
        self.cum = [0.0]
        for i in range(self.n):
            self.cum.append(self.cum[-1] + math.hypot(self.pts[i + 1][0] - self.pts[i][0],
                                                      self.pts[i + 1][1] - self.pts[i][1]))
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


def load_route_csv(path, reverse=False):
    """기준선 CSV(run,s,x,y) 의 run 0 → [(x, y)] — sim/export_xmobility.py:106-108.

    reverse=True 면 같은 출발점에서 반대 방향(export_xmobility.py:162-163 = sim/teacher.py r.reversed()).
    eval/routes/parking_r3_ccw.csv 는 이미 반대 방향이라 reverse 없이 쓴다.
    """
    with open(path, newline='') as f:
        pts = [(float(r['x']), float(r['y'])) for r in csv.DictReader(f) if r['run'] == '0']
    if len(pts) < 2:
        raise ValueError(f'{path}: run 0 점이 2개 미만')
    if reverse:
        pts = [pts[0]] + pts[1:][::-1]
    return pts


def build_route_poses(poly, x, y, yaw, s_hint=None, goal_s=None, n=NUM_ROUTE_POINTS, spacing=ROUTE_SPACING):
    """학습 데이터의 route_poses 를 만드는 식 그대로 — sim/export_xmobility.py:239-246.

    로봇 위치 (x, y) + 기준선에서 사영한 s 앞으로 spacing 간격 n-1 점(goal_s 가 있으면 거기서 끊고 끝점 반복),
    로봇 좌표계로 바꿈. → (route (n, 2) float64, s_now, cte). 다음 호출에 s_hint=s_now 를 넘긴다(export :240).
    goal_s 는 export :172 처럼 poly.L 로 나눈 나머지를 넘긴다.
    """
    s_now, cte = poly.project(x, y, s_hint, back=PROJECT_BACK, fwd=PROJECT_FWD)
    reach = None if goal_s is None else (goal_s - s_now) % poly.L
    rp = [(x, y)]
    for k in range(1, n):
        ds = k * spacing if reach is None else min(k * spacing, reach)
        rp.append(poly.point_at(s_now + ds))
    pts = np.array([to_robot(px, py, x, y, yaw) for px, py in rp], np.float64)
    return pts, s_now, cte


def polyline_distance(points, line):
    """각 점에서 폴리라인 line (M, 2) 까지 최단 거리 → (N,). M == 1 이면 점까지 거리."""
    p = np.asarray(points, np.float64).reshape(-1, 2)
    ln = np.asarray(line, np.float64).reshape(-1, 2)
    if len(ln) == 1:
        return np.linalg.norm(p - ln[0], axis=1)
    a, b = ln[:-1], ln[1:]
    ab = b - a
    l2 = (ab * ab).sum(axis=1)
    ap = p[:, None, :] - a[None, :, :]
    t = np.where(l2 > 1e-12, (ap * ab[None]).sum(axis=2) / np.maximum(l2, 1e-12), 0.0)
    t = np.clip(t, 0.0, 1.0)
    d = np.linalg.norm(ap - t[:, :, None] * ab[None], axis=2)
    return d.min(axis=1)


def route_jump(prev_fixed, new_fixed, threshold_m):
    """새 경로가 직전 경로에서 얼마나 벗어났나 — 같은 고정 좌표계의 (N, 2) 둘.

    새 점 각각에서 직전 경로(폴리라인)까지 거리의 최댓값 d. 로봇이 앞으로 가면 경로 창이 기준선을 따라 미끄러질 뿐이라
    d 는 그동안 나아간 거리 정도, 새 목표 · 다른 경로면 크게 뛴다. → (d > threshold_m, d). 직전이 없으면 (False, 0).
    """
    if prev_fixed is None or new_fixed is None:
        return False, 0.0
    d = float(polyline_distance(new_fixed, prev_fixed).max())
    return d > threshold_m, d


# ---------------------------------------------------------------- 명령 · 지연

def clamp_cmd(v, w, v_min=V_MIN, v_max=V_MAX, w_max=W_MAX):
    """모델 (v, ω) → topics.md:61 상한 안으로. 유한값이 아니면 (0, 0, False)."""
    v, w = float(v), float(w)
    if not (math.isfinite(v) and math.isfinite(w)):
        return 0.0, 0.0, False
    return min(max(v, v_min), v_max), min(max(w, -w_max), w_max), True


class LatencyStats:
    """이름별 지연(ms) — 최근 창(보고 뒤 비움)과 전체. 백분위는 numpy 기본(linear 보간)."""

    def __init__(self):
        self.all, self.win = {}, {}

    def add(self, name, ms):
        if ms is None or not math.isfinite(ms):
            return
        self.all.setdefault(name, []).append(float(ms))
        self.win.setdefault(name, []).append(float(ms))

    def summary(self, name, window=True):
        """→ dict(n, p50, p95, max) — 값이 없으면 n 0 · 나머지 None."""
        x = (self.win if window else self.all).get(name, [])
        if not x:
            return dict(n=0, p50=None, p95=None, max=None)
        p50, p95 = np.percentile(np.asarray(x, np.float64), [50, 95])
        return dict(n=len(x), p50=float(p50), p95=float(p95), max=float(max(x)))

    def reset_window(self):
        self.win = {}


def fmt_ms(s):
    """summary dict → 'P50 12.3 · P95 15.0 ms'."""
    if not s['n']:
        return '-'
    return f"P50 {s['p50']:.1f} · P95 {s['p95']:.1f} ms"
