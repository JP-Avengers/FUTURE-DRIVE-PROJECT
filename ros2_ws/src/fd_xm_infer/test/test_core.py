"""fd_xm_infer.core 의 전처리가 학습 데이터(sim/export_xmobility.py) · 오프라인 평가(e2e/eval_offline.py)와 같은지 — README
"확인한 것" 의 대조(영상 · 경로 · 각도 · 상한)를 레포에 둔 것(E 11.5 작성 · 11.8 리뷰 재검). 노드를 고친 뒤 다시 돌린다.

  python -m pytest ros2_ws/src/fd_xm_infer/test -q     # 레포 루트에서. numpy · Pillow · pandas 만(ROS · torch 불필요)
  python ros2_ws/src/fd_xm_infer/test/test_core.py     # pytest 가 없을 때

sim/export_xmobility.py 는 import 하면 rosbag2 가 필요해 Poly · to_robot · wrap · quat_yaw 만 ast 로 떼어 실행한다(학습 데이터를
만든 식 그대로). e2e/eval_offline.py 는 파일 경로로 import(e2e/ 를 sys.path 에 넣지 않음).
"""
import ast
import bisect
import functools
import importlib.util
import io
import math
import os
import sys
import types

import numpy as np
from PIL import Image as PILImage

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)                                          # ros2_ws/src/fd_xm_infer
REPO = os.path.dirname(os.path.dirname(os.path.dirname(PKG)))
if PKG not in sys.path:
    sys.path.insert(0, PKG)
from fd_xm_infer import core  # noqa: E402

N_ROUTE, ROUTE_SPACING = 20, 0.5                                     # sim/export_xmobility.py:51
ROUTES = os.path.join(REPO, 'eval', 'routes')


@functools.lru_cache(maxsize=None)
def eval_offline():
    """e2e/eval_offline.py — decode_image(:106-112) · route_vectors(:94-103) 만 씀."""
    p = os.path.join(REPO, 'e2e', 'eval_offline.py')
    spec = importlib.util.spec_from_file_location('eval_offline', p)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@functools.lru_cache(maxsize=None)
def export_funcs():
    """sim/export_xmobility.py 의 Poly(:69-103) · to_robot(:118-121) · wrap(:57-58) · quat_yaw(:61-62) 를 ast 로 떼어 실행."""
    src = open(os.path.join(REPO, 'sim', 'export_xmobility.py'), encoding='utf-8').read()
    keep = [n for n in ast.parse(src).body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
            and n.name in ('Poly', 'to_robot', 'wrap', 'quat_yaw')]
    assert sorted(n.name for n in keep) == ['Poly', 'quat_yaw', 'to_robot', 'wrap'], [n.name for n in keep]
    ns = {'math': math, 'bisect': bisect, 'np': np}
    exec(compile(ast.Module(body=keep, type_ignores=[]), 'export_xmobility_extract', 'exec'), ns)
    return ns


def test_image_bit_identical_to_training_path():
    """export_xmobility.py:234-236(줄 끝 채움 잘라 → Pillow BILINEAR 512×320 → JPEG q90) + eval_offline.decode_image 와
    비트 단위 같음."""
    eo = eval_offline()
    rng = np.random.default_rng(0)
    h, w = 480, 640
    for pad in (0, 8, 64):                                           # sensor_msgs/Image step 의 줄 끝 채움
        step = w * 3 + pad
        data = rng.integers(0, 256, size=(h, step), dtype=np.uint8).tobytes()
        img = np.frombuffer(bytes(data), np.uint8).reshape(h, step)[:, :w * 3].reshape(h, w, 3)   # export:234
        buf = io.BytesIO()
        PILImage.fromarray(img).resize((512, 320), PILImage.BILINEAR).save(buf, 'JPEG', quality=90)   # export:236
        ref = eo.decode_image(buf.getvalue())                                            # eval_offline:106-112
        got = core.image_to_chw(data, h, w, step, 'rgb8', 90)
        assert got.shape == ref.shape == (3, 320, 512), (pad, got.shape, ref.shape)
        assert got.dtype == ref.dtype == np.float32, (pad, got.dtype, ref.dtype)
        assert np.array_equal(got, ref), f'pad {pad}: 최대 차 {np.abs(got - ref).max()}'


def test_route_vectors_same_as_eval_offline():
    """core.route_vectors = eval_offline.route_vectors(navigator.py:149-170) — 1~40점(끝점 반복 포함)."""
    eo = eval_offline()
    rng = np.random.default_rng(1)
    for n in (1, 5, 19, 20, 25, 40):
        pts = rng.normal(size=(n, 2)).astype(np.float64)
        a, b = core.route_vectors(pts), eo.route_vectors(pts.reshape(-1))
        assert a.shape == (19, 4) and a.dtype == np.float32, (n, a.shape, a.dtype)
        assert np.array_equal(a, b), n


def test_csv_route_poses_same_as_export():
    """core.build_route_poses(core.Poly) = export 239-246 식을 export 자신의 Poly · to_robot 으로 돌린 것 — cw CSV,
    목표 없음 · 40 m."""
    ex = export_funcs()
    rng = np.random.default_rng(2)
    pts = core.load_route_csv(os.path.join(ROUTES, 'parking_r3_cw.csv'))
    P_ex, P_core = ex['Poly'](list(pts)), core.Poly(list(pts))
    assert P_ex.L == P_core.L
    maxd = 0.0
    for goal_s in (None, 40.0):
        s_hint_ex = s_hint_core = None
        for _ in range(300):
            x0, y0 = P_ex.point_at(rng.uniform(0, P_ex.L))
            x, y, yaw = x0 + rng.normal(0, 0.3), y0 + rng.normal(0, 0.3), rng.uniform(-math.pi, math.pi)
            s_now, cte = P_ex.project(x, y, s_hint_ex, back=1.0, fwd=10.0)                   # export:239-240
            s_hint_ex = s_now
            reach = None if goal_s is None else (goal_s - s_now) % P_ex.L                   # export:241
            rp = [(x, y)]
            for k in range(1, N_ROUTE):                                                     # export:242-245
                ds = k * ROUTE_SPACING if reach is None else min(k * ROUTE_SPACING, reach)
                rp.append(P_ex.point_at(s_now + ds))
            ref = np.array([ex['to_robot'](px, py, x, y, yaw) for px, py in rp], np.float64)   # export:246
            got, s_hint_core, cte2 = core.build_route_poses(P_core, x, y, yaw, s_hint_core, goal_s)
            assert got.shape == (N_ROUTE, 2), got.shape
            maxd = max(maxd, float(np.abs(got - ref).max()), abs(s_hint_core - s_now), abs(cte - cte2))
    assert maxd <= 1e-9, f'최대 차 {maxd:.3e}'


def test_ccw_csv_is_cw_reversed():
    """README: parking_r3_ccw.csv = cw 를 같은 출발점에서 거꾸로(load_route_csv reverse) — 같은 s 의 점이 같은 자리."""
    ccw = core.load_route_csv(os.path.join(ROUTES, 'parking_r3_ccw.csv'))
    rev = core.load_route_csv(os.path.join(ROUTES, 'parking_r3_cw.csv'), reverse=True)
    Pc, Pr = core.Poly(ccw), core.Poly(rev)
    assert abs(Pc.L - Pr.L) < 1e-6, (Pc.L, Pr.L)
    d = max(math.hypot(*np.subtract(Pc.point_at(s), Pr.point_at(s)))
            for s in np.linspace(0, Pc.L, 2000, endpoint=False))
    assert d < 1e-6, f'같은 s 점 거리 최대 {d * 1000:.3f} mm'


def test_wrap_quat_yaw_same_as_export():
    """core.wrap · core.quat_yaw(인자만 풀어 씀) = export_xmobility.py:57-62."""
    ex = export_funcs()
    rng = np.random.default_rng(3)
    for _ in range(200):
        a = float(rng.uniform(-20, 20))
        assert core.wrap(a) == ex['wrap'](a), a
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        qo = types.SimpleNamespace(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))
        assert core.quat_yaw(*map(float, q)) == ex['quat_yaw'](qo), q


def test_clamp_cmd():
    """topics.md:61 상한 — 0 ≤ v ≤ 0.8, |ω| ≤ 0.7, 유한값 아니면 정지."""
    assert core.clamp_cmd(1.2, -0.9) == (0.8, -0.7, True)
    assert core.clamp_cmd(-0.1, 0.2) == (0.0, 0.2, True)
    assert core.clamp_cmd(0.5, 0.3) == (0.5, 0.3, True)
    assert core.clamp_cmd(float('nan'), 0.0) == (0.0, 0.0, False)
    assert core.clamp_cmd(0.3, float('inf')) == (0.0, 0.0, False)


def test_prev_action_feed_modes():
    """노드 prev_action_mode — own = 예측 그대로 · zero = 0 · clamped = v(0칸) · ω(5칸)만 상한으로(나머지 4칸 그대로).
    늘 float32 6칸 새 배열, 넘겨받은 action 은 안 바뀜."""
    for dt in (np.float32, np.float64):
        a = np.array([1.2, 0.1, -2.0, 3.0, 0.4, -0.9], dt)                 # v > 0.8 · ω < -0.7 · 다른 칸도 범위 밖
        keep = a.copy()
        want = {'own': a.astype(np.float32), 'zero': np.zeros(6, np.float32),
                'clamped': np.array([0.8, 0.1, -2.0, 3.0, 0.4, -0.7], np.float32)}
        for mode, ref in want.items():
            got = core.prev_action_feed(a, mode)
            assert got.dtype == np.float32 and got.shape == (core.ACTION_SIZE,), (dt, mode, got.dtype, got.shape)
            assert np.array_equal(got, ref), (dt, mode, got)
            assert got is not a and not np.shares_memory(got, a), (dt, mode)
            got[:] = 99.0                                                    # 돌려준 배열을 고쳐도 action 은 그대로
            assert np.array_equal(a, keep), (dt, mode, a)
    assert np.array_equal(core.prev_action_feed(a), core.prev_action_feed(a, 'own'))   # 기본 own
    lst = [-0.3, 0.0, 0.0, 0.0, 0.0, 2.5]                                    # list · v < 0 · ω > 0.7
    assert np.array_equal(core.prev_action_feed(lst, 'clamped'), np.array([0.0, 0, 0, 0, 0, 0.7], np.float32))
    assert lst == [-0.3, 0.0, 0.0, 0.0, 0.0, 2.5]
    inr = np.array([0.5, 1, 2, 3, 4, -0.3], np.float32)                      # 상한 안이면 own 과 같음
    assert np.array_equal(core.prev_action_feed(inr, 'clamped'), inr)
    got = core.prev_action_feed(np.array([0.6, 0, 0, 0, 0, -0.5], np.float32), 'clamped', 0.1, 0.4, 0.2)
    assert np.array_equal(got, np.array([0.4, 0, 0, 0, 0, -0.2], np.float32)), got   # 노드 v_min · v_max · w_max
    for bad in ('teacher', 'Own', '', None):
        try:
            core.prev_action_feed(a, bad)
        except ValueError:
            continue
        raise AssertionError(f'{bad!r} 를 받아 줌')


def test_prev_action_feed_same_as_old_tick():
    """own · clamped = 예전 node.py _tick 의 fed 식(feed_clamped_action false · true)과 비트 단위 같음 — 범위 밖 값 포함."""
    rng = np.random.default_rng(4)
    for _ in range(2000):
        action = (rng.normal(size=6) * rng.choice([0.1, 1.0, 10.0])).astype(np.float32)
        v, w, finite = core.clamp_cmd(action[core.V_IDX], action[core.W_IDX])
        assert finite
        for feed_clamped, mode in ((False, 'own'), (True, 'clamped')):
            fed = action.copy()                                              # 6799185 의 node.py:676-679
            if feed_clamped:
                fed[core.V_IDX], fed[core.W_IDX] = v, w
            ref = fed.astype(np.float32)
            got = core.prev_action_feed(action, mode)
            assert got.tobytes() == ref.tobytes(), (mode, action, got, ref)


if __name__ == '__main__':
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith('test_') and callable(v)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f'ok    {name}')
        except Exception as e:  # noqa: BLE001 — pytest 없이 돌릴 때의 요약
            failed += 1
            print(f'FAIL  {name}: {type(e).__name__}: {e}')
    print(f'{len(tests) - failed}/{len(tests)} 통과')
    sys.exit(1 if failed else 0)
