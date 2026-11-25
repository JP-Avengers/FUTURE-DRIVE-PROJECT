"""되돌아온 자리 어긋남(stray_revisit) 시험 — 방을 두 바퀴 도는 가상 촬영본에 알려진 틀어짐을 넣고 그만큼 되찾는지 본다. 서버 · 엔진은 띄우지 않음.

    cd recon
    python -m scan2sim.tests.revisit [--fixtures <폴더>] [--room <room.glb 가 든 폴더>] [--work <폴더>] [--json 결과.json]

시험 자료: samples/make_stray.py 로 만든 50 초 · 10 장/초 가상 촬영본 (방을 두 바퀴 — 한 바퀴 약 24.5 초, 500장). 실제 촬영본은 쓰지 않음.
  --fixtures 에 만들고 다음에는 다시 씀 (run.sh, 처음 한 번 약 1분). --room 에 room.glb 가 있으면 그 방을 씀 (없으면 make_room.py 로 만듦).
  안 주면 임시 폴더(TMPDIR 아래)에 만들고 지움. 틀어짐을 넣은 촬영본은 원본 파일을 링크하고 (안 되면 복사) odometry.csv 만 새로 씀
--work: 고친 촬영본 · 결과 폴더 꼴을 둘 폴더 (안 주면 임시 폴더를 만들고 끝나면 지움)
검사
  (a) 그대로 → pass · 믿을 쌍 3개 이상 · 수평 최대 3 cm · 높이 최대 1 cm 이하 (자세가 정확한 가상 촬영본 — 11/25 잰 값 0.6 · 0.1 cm)
  (b) 둘째 바퀴(25 초 ~ 끝)의 기록된 자세만 선형으로 커지는 강체 틀어짐 (끝에서 수평 (30, −20) cm · 높이 +25 cm · z 축 3°, 둘째 바퀴를
      시작한 자리 둘레) — 깊이 · 영상은 그대로 → fail. 앞 방문이 틀어짐 전(22 초 전)이고 뒤 방문이 29 초 뒤인 믿을 쌍(2개 이상)마다,
      그 쌍의 시각(t_b)에서 넣은 값과 견줌: 높이 ±2 cm · 수평 ±6 cm (1d 는 센 방향 성분, 2d 는 벡터 차) · 방향 ±0.6°.
      수평은 그 쌍이 잰 자리(at_xy = B 벽 점 가운데)에서 넣은 옮김 (방향 틀어짐이 있어 자리마다 다름)
      허용 폭 근거: 뒤 구간 B 는 경로 3 m (이 촬영본에서 약 7 초) — 그동안 틀어짐이 끝 값의 약 30% 만큼 바뀌어 잰 값은 구간의 평균에 가까움.
      11/25 같은 생성기 (끝 (40, −30, 25) cm · 4°) 에서 잰 차이 최대 높이 0.8 · 수평 4 cm 의 약 1.5~2.5 배. 수평 6 cm 는 통과 문턱
      (10 cm) 보다 작아 지표가 망가지면 걸림. 방향은 이 시험 자료에서 틀어짐이 없어도 0.43° 까지 나옴 (방위 0.5° 칸 · 1° 번짐,
      둘러보며 걸어 벽이 보이는 몫이 장마다 다름 — 11/25 잰 값) → 그보다 넉넉한 0.6° (틀어짐을 넣은 쌍의 차 최대 0.48°)
  (c) 결과 폴더 꼴 (input_map/stray_meta.json · _unzip/<촬영> · build/tsdf_meta.json 의 pose_fix sha256 · build/pose_fix.json) 에 (b) 의 틀어짐을
      프레임마다 정확히 되돌리는 보정 info (높이 → 수평 · yaw, stray_fix.apply_fix 꼴) → 원래 자세 fail · 고친 자세 pass (수평 3 · 높이 1 cm 이하) ·
      판정은 고친 자세 · pose_fix.sha256_used · input_map/stray_revisit.json 을 씀. pose_fix.json 바이트를 바꾸면 (sha256 다름) 보정을 쓰지 않고
      원래 자세만 · 까닭이 pose_fix.note 와 reasons 에. input_map/_unzip 이 없으면 --capture 를 알려 주며 멈추고, capture= 로 주면 같은 값.
      직접 준 보정 info 가 적용할 수 없는 꼴이면 ValueError 문구
  (d) 다시 지나간 곳이 없는 촬영 (처음 15 초만) → skip · CLI 종료 코드 0 · traceback 없음. CLI --json 이 (b) 결과를 씀
  (e) 단위 (시험 자료 없이): verdict() 를 요약 dict 로 — skip (짧은 촬영 · 한 방향) · 믿을 쌍 없음 warn · pass · 높이 warn · 수평 fail ·
      pass → warn 낮추기 (믿지 못한 쌍이 믿을 쌍의 가장 먼 것보다 SEG_M 넘게 멂 / 바닥을 견준 쌍 없음) 와 낮추지 않는 경계 · fail 은 그대로.
      register() 에 만든 상자 방 (4 × 4 m 벽 · 바닥) 을 옮겨 넣음: 0.5 m → 2d · 믿음 · 그 값 / 1.47 m (탐색 반지름 1.5 m 의 끝 칸) → 2d 봉우리가
      뚜렷하지만 탐색 끝 → 믿지 않음 · unreliable_why. _jumps(): 60 장/초 자세에서 10줄을 빼 한꺼번에 18 cm 움직인 곳은 튐이 아님 · 한 장에
      19 cm 튄 곳은 잡음 · jumps_total
  (f) 잘못된 촬영 (처음 15 초 촬영본을 고친 것): odometry.csv 에 머리글만 · camera_matrix.csv 없음 → measure 가 CaptureError (validate 문구) ·
      맨 위에 파일이 든 zip → source 가 zip 이름
종료 코드: 0 = 모두 통과 · 1 = 하나라도 실패 · 2 = 시험 자료를 만들지 못함
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import traceback

import numpy as np

from .. import stray, stray_revisit as rv

RECON = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SAMPLES = os.path.join(RECON, "scan2sim", "samples")
NAME = "synth_revisit"
FIXTURE = ["--seconds", "50", "--fps", "10", "--render-width", "160", "--seed", "0", "--name", NAME]   # 두 바퀴 · 500장
T1 = 25.0                                                       # 둘째 바퀴를 시작한 때 (초, 첫 줄 기준) — 여기부터 틀어짐
DRIFT = dict(d=(0.30, -0.20, 0.25), yaw_deg=3.0)                # 끝에서의 틀어짐 (맵 세계 Z 위, m · °)
TOL = dict(z_cm=2.0, h_cm=6.0, yaw_deg=0.6)                     # 되찾은 값의 허용 폭 (근거는 머리말 (b))
CLEAN = dict(h_cm=3.0, z_cm=1.0, pairs=3)
ENV = dict(os.environ, PYTHONIOENCODING="utf-8")


class Checks:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail=""):
        ok = bool(ok)
        self.rows.append(dict(name=name, ok=ok, detail=str(detail)))
        print(f"  {'통과' if ok else '실패'}  {name}" + (f" — {detail}" if detail != "" else ""), flush=True)
        return ok

    @property
    def ok(self):
        return bool(self.rows) and all(r["ok"] for r in self.rows)


def _link(src, dst):
    try:
        os.symlink(src, dst)
    except (OSError, NotImplementedError):                      # 윈도우 — 복사
        (shutil.copytree if os.path.isdir(src) else shutil.copy2)(src, dst)


# ── 틀어짐 넣기 ──────────────────────────────────────────────────────────

def _read_odo(src):
    lines = open(os.path.join(src, "odometry.csv"), encoding="utf-8").read().splitlines()
    hdr, body = lines[0], [s for s in lines[1:] if s.strip()]
    od = np.array([[float(v) for v in s.split(",")[:13]] for s in body])
    T = np.repeat(np.eye(4)[None], len(od), axis=0)
    T[:, :3, :3] = [stray._rot(q) for q in od[:, 5:9]]
    T[:, :3, 3] = od[:, 2:5]
    return hdr, body, od[:, 0] - od[0, 0], stray.Y_UP_TO_Z_UP @ T  # 자세 = 맵 세계 (Z 위)


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def drift_poses(t, T, t1=T1, d=DRIFT["d"], yaw_deg=DRIFT["yaw_deg"]):
    """t1 부터 끝까지 선형으로 커지는 강체 틀어짐 (둘째 바퀴를 시작한 자리 c 둘레 z 축 회전 + 옮김) → (틀어진 자세, f, c)"""
    f = np.clip((t - t1) / (t[-1] - t1), 0, 1)
    c = T[int(np.searchsorted(t, t1)), :3, 3].copy()
    out = T.copy()
    for i in np.nonzero(f > 0)[0]:
        R = _rz(np.radians(yaw_deg * f[i]))
        out[i, :3, :3] = R @ T[i, :3, :3]
        out[i, :3, 3] = R @ (T[i, :3, 3] - c) + c + f[i] * np.asarray(d)
    return out, f, c


def make_capture(src, dst, rows=None, poses=None):
    """src 촬영 폴더를 링크해 dst 에 — odometry.csv 만 새로 씀 (rows = 남길 줄 수, poses = 바꿀 자세 (Z 위))"""
    os.makedirs(dst)
    for n in sorted(os.listdir(src)):
        if n == "odometry.csv":
            continue
        if rows and n in ("depth", "confidence"):                # 줄을 줄이면 깊이 · 신뢰도도 그만큼만 (stray.validate 가 장 수를 봄)
            os.makedirs(os.path.join(dst, n))
            for f in sorted(os.listdir(os.path.join(src, n)))[:rows]:
                _link(os.path.join(src, n, f), os.path.join(dst, n, f))
            continue
        _link(os.path.join(src, n), os.path.join(dst, n))
    hdr, body, _, _ = _read_odo(src)
    body = body[:rows] if rows else body
    if poses is not None:
        Ty = stray.Y_UP_TO_Z_UP.T @ poses                       # Z 위 → 아이폰 세계 (Y 위). 돌림 행렬이라 역 = 전치
        new = []
        for s, P in zip(body, Ty):
            c = s.split(",")
            c[2:5] = [repr(float(v)) for v in P[:3, 3]]
            c[5:9] = [repr(float(v)) for v in stray._quat(P[:3, :3])]
            new.append(",".join(c))
        body = new
    with open(os.path.join(dst, "odometry.csv"), "w", encoding="utf-8") as fh:
        fh.write("\n".join([hdr] + body) + "\n")
    return dst


def undo_info(tabs, T, Td, f):
    """틀어진 자세 Td 를 T 로 정확히 되돌리는 보정 info (pose_fix.json 꼴: 높이 → 수평 + yaw, 프레임마다) — apply_fix 는
    높이 z −= dz, 수평 R ← Rz(−ψ)·R · x −= dx · y −= dy 를 차례로 함"""
    t = [float(x) for x in tabs]
    dz = (Td[:, 2, 3] - T[:, 2, 3]).tolist()
    dxy = Td[:, :2, 3] - T[:, :2, 3]
    yaw = (DRIFT["yaw_deg"] * f).tolist()
    steps = [dict(method="floor_cells", kind="floor", applied=True, status="applied", t=t, dz=dz, tilt=None, reasons=[], flags=[]),
             dict(method="wall_cells", kind="xy", applied=True, status="applied", t=t, dx=dxy[:, 0].tolist(), dy=dxy[:, 1].tolist(),
                  dyaw_deg=yaw, reasons=[], flags=[])]
    return dict(method="chain", fix="floor+xy", applied=True, status="applied", reasons=[], flags=[], keyframes=len(t), steps=steps)


def make_job(root, cap, info):
    """결과 폴더 꼴: input_map/stray_meta.json (source) · input_map/_unzip/<촬영> · build/pose_fix.json · build/tsdf_meta.json (sha256)"""
    name = os.path.basename(cap)
    mp, bd = os.path.join(root, "input_map"), os.path.join(root, "build")
    os.makedirs(os.path.join(mp, "_unzip"))
    os.makedirs(bd)
    _link(cap, os.path.join(mp, "_unzip", name))
    json.dump(dict(source=name, format="stray_scanner"), open(os.path.join(mp, "stray_meta.json"), "w", encoding="utf-8"))
    raw = json.dumps(info).encode("utf-8")
    open(os.path.join(bd, "pose_fix.json"), "wb").write(raw)
    pf = dict(name="pose_fix", file="pose_fix.json", sha256=hashlib.sha256(raw).hexdigest(), method="chain", fix="floor+xy", applied=True, status="applied")
    json.dump(dict(pose_fix=pf), open(os.path.join(bd, "tsdf_meta.json"), "w", encoding="utf-8"))
    return root


# ── 검사 ─────────────────────────────────────────────────────────────────

def _sm(r):
    s = r["summary"]
    return f"쌍 {s.get('pairs')} · 믿을 쌍 {s.get('reliable')} · 수평 최대 {s.get('h_max_cm')} · 높이 최대 {s.get('z_max_cm')} cm · 방향 {s.get('yaw_max_deg')}° · {r['verdict']}"


def check_clean(C, src):
    r = rv.measure(src, write=False)["raw"]
    s = r["summary"]
    C.add(f"(a) 틀어짐 없음 → pass · 믿을 쌍 {CLEAN['pairs']}개 이상 · 수평 ≤ {CLEAN['h_cm']:g} · 높이 ≤ {CLEAN['z_cm']:g} cm",
          r["verdict"] == "pass" and s.get("reliable", 0) >= CLEAN["pairs"] and s.get("h_max_cm", 99) <= CLEAN["h_cm"]
          and (s.get("z_max_cm") or 0) <= CLEAN["z_cm"], _sm(r))


def check_drift(C, cap, f_of, c):
    res = rv.measure(cap, write=False)
    r = res["raw"]
    s = r["summary"]
    C.add("(b) 둘째 바퀴에 틀어짐 → fail (높이 > 15 cm 또는 수평 > 30 cm)", r["verdict"] == "fail" and r["reasons"], _sm(r))
    d, yaw = np.asarray(DRIFT["d"]), DRIFT["yaw_deg"]
    n, worst = 0, dict(z=0.0, h=0.0, yaw=0.0)
    for p in r["pairs"]:
        if not (p.get("ok") and p.get("reliable") and p["t_a"] < T1 - 3 and p["t_b"] > T1 + 4 and "dz_cm" in p):
            continue
        f = f_of(p["t_b"])
        at = np.asarray(p["at_xy"])
        e = 100 * ((_rz(np.radians(yaw * f))[:2, :2] @ (at - c[:2]) + c[:2] + f * d[:2]) - at)
        got = np.array([p["dx_cm"], p["dy_cm"]])
        if p["h_axis"] == "1d":
            u = np.radians(p["strong_dir_deg"])
            u = np.array([np.cos(u), np.sin(u)])
            dh = abs(float(got @ u - e @ u))
        else:
            dh = float(np.linalg.norm(got - e))
        dz, dy = abs(p["dz_cm"] - 100 * f * d[2]), abs(p["dyaw_deg"] - yaw * f)
        n += 1
        worst = dict(z=max(worst["z"], dz), h=max(worst["h"], dh), yaw=max(worst["yaw"], dy))
        C.add(f"(b) 쌍 {p['t_a']:g} → {p['t_b']:g} 초 ({p['h_axis']}): 넣은 값을 되찾음 (높이 ±{TOL['z_cm']:g} · 수평 ±{TOL['h_cm']:g} cm · "
              f"방향 ±{TOL['yaw_deg']:g}°)", dz <= TOL["z_cm"] and dh <= TOL["h_cm"] and dy <= TOL["yaw_deg"],
              f"높이 {p['dz_cm']:+.1f} (넣은 {100 * f * d[2]:+.1f}) · 수평 차 {dh:.1f} cm · 방향 {p['dyaw_deg']:+.2f} (넣은 {yaw * f:+.2f})°")
    C.add("(b) 시험이 헛돌지 않음 — 틀어짐 전 ↔ 뒤를 잇는 믿을 쌍이 2개 이상", n >= 2,
          f"{n}개 · 가장 큰 차 높이 {worst['z']:.1f} · 수평 {worst['h']:.1f} cm · 방향 {worst['yaw']:.2f}°")
    return res


def check_job(C, job, cap):
    res = rv.measure(job, write=True)
    out = os.path.join(job, "input_map", "stray_revisit.json")
    fx = res.get("fixed")
    C.add("(c) 결과 폴더 꼴: 맵과 같은 보정(sha256)을 읽어 원래 자세 fail · 고친 자세 pass (수평 ≤ 3 · 높이 ≤ 1 cm) · 판정은 고친 자세",
          res["raw"]["verdict"] == "fail" and fx is not None and fx["verdict"] == "pass" and res["verdict"] == "pass"
          and fx["summary"].get("h_max_cm", 99) <= CLEAN["h_cm"] and (fx["summary"].get("z_max_cm") or 0) <= CLEAN["z_cm"]
          and (res.get("pose_fix") or {}).get("applied") is True,
          "원래 " + _sm(res["raw"]) + (" / 고친 " + _sm(fx) if fx else " / 고친 자세 없음"))
    pf = res.get("pose_fix") or {}
    C.add("(c) 결과 폴더 꼴: pose_fix 에 맞춰 쓴 sha256 (sha256_used = build/tsdf_meta.json 의 값)",
          pf.get("sha256_used") and pf.get("sha256_used") == pf.get("sha256_expected"), f"{str(pf.get('sha256_used'))[:12]}…")
    with open(out, encoding="utf-8") as fh:
        ok = os.path.isfile(out) and json.load(fh).get("verdict") == res["verdict"] and not os.path.exists(out + ".writing")
    C.add("(c) 결과 폴더 꼴: input_map/stray_revisit.json 을 씀", ok, out)
    with open(os.path.join(job, "build", "pose_fix.json"), "ab") as fh:
        fh.write(b" ")                                          # JSON 으로는 같게 읽히지만 바이트가 다름
    res2 = rv.measure(job, write=False)
    pf2 = res2.get("pose_fix") or {}
    C.add("(c) pose_fix.json 바이트가 맵과 다름 (sha256) → 보정을 쓰지 않고 원래 자세만 · 판정 fail · 까닭(note)이 pose_fix · reasons 에 남음",
          "fixed" not in res2 and res2["verdict"] == "fail" and pf2.get("applied") is False and pf2.get("sha256_used") is None
          and pf2.get("sha256_expected") and any(m.startswith("자세 보정:") and "sha256" in m for m in res2["reasons"]),
          f"{res2['verdict']} · {[m[:60] for m in res2['reasons'] if m.startswith('자세 보정:')]}")
    link = os.path.join(job, "input_map", "_unzip", os.path.basename(cap))
    if os.path.islink(link):
        os.unlink(link)
    else:
        shutil.rmtree(link, ignore_errors=True)
    try:
        rv.measure(job, write=False)
        C.add("(c) input_map/_unzip 이 없음 (서버가 지운 뒤) → --capture 를 알려 주며 멈춤", False, "멈추지 않음")
    except SystemExit as e:
        C.add("(c) input_map/_unzip 이 없음 (서버가 지운 뒤) → --capture 를 알려 주며 멈춤", "--capture" in str(e), str(e)[:120])
    res3 = rv.measure(job, write=False, capture=cap)
    C.add("(c) input_map/_unzip 이 없어도 capture= (폴더) 로 잼 · 같은 원래 자세 값", res3["raw"]["summary"] == res["raw"]["summary"], _sm(res3["raw"]))
    try:
        rv.measure(cap, fix_info=dict(applied=True), write=False)
        C.add("(c) 직접 준 보정 info 가 적용할 수 없는 꼴 → ValueError 문구", False, "오류가 안 남")
    except ValueError as e:
        C.add("(c) 직접 준 보정 info 가 적용할 수 없는 꼴 → ValueError 문구", "적용할 수 없음" in str(e), str(e)[:120])


def _box(shift=(0.0, 0.0, 0.0), size=4.0):
    """4 × 4 m 상자 방의 벽 (안쪽 법선, 높이 0.3~2.0 m) · 바닥 점 → (점, 법선, 넓이, 바닥) — register 단위 검사용"""
    u = np.arange(0.0, size, 0.03)
    hz = np.arange(0.3, 2.0, 0.1)
    U, Z = [a.ravel() for a in np.meshgrid(u, hz)]
    walls = [(np.c_[0 * U, U, Z], (1, 0, 0)), (np.c_[0 * U + size, U, Z], (-1, 0, 0)),
             (np.c_[U, 0 * U, Z], (0, 1, 0)), (np.c_[U, 0 * U + size, Z], (0, -1, 0))]
    g = np.arange(0.1, size, 0.1)
    X, Y = [a.ravel() for a in np.meshgrid(g, g)]
    P = np.concatenate([w for w, _ in walls] + [np.c_[X, Y, 0 * X]]) + np.asarray(shift)
    N = np.concatenate([np.tile(n, (len(w), 1)) for w, n in walls] + [np.tile((0, 0, 1), (len(X), 1))]).astype(float)
    fl = np.r_[np.zeros(sum(len(w) for w, _ in walls), bool), np.ones(len(X), bool)]
    return P, N, np.ones(len(P)), fl


def check_units(C):
    V = rv.verdict
    w = dict(t_a=1.0, t_b=40.0, path_between_m=20.0)
    ok_sm = dict(pairs=4, reliable=4, h_max_cm=5.0, z_max_cm=1.0, path_between_max_m=20.0, h_worst=w, z_worst=w)
    cases = [
        ("짧은 촬영 · 쌍 없음 → skip (촬영이 짧다는 문구)", dict(pairs=0, reliable=0, duration_s=13.9), "skip", "13.9 초뿐"),
        ("긴 촬영 · 쌍 없음 → skip (한 방향 문구)", dict(pairs=0, reliable=0, duration_s=60.0), "skip", "한 방향"),
        ("쌍은 있는데 믿을 쌍 없음 → warn", dict(pairs=3, reliable=0), "warn", "맞추지 못함"),
        ("작은 어긋남 → pass", dict(ok_sm), "pass", None),
        ("높이 5 cm → warn", dict(ok_sm, z_max_cm=5.0), "warn", "높이 어긋남 최대 5 cm"),
        ("수평 40 cm → fail", dict(ok_sm, h_max_cm=40.0), "fail", "수평 어긋남 최대 40 cm"),
        (f"pass 인데 믿지 못한 쌍이 {rv.SEG_M:g} m 넘게 더 멂 (24 > 20 + {rv.SEG_M:g}) → warn", dict(ok_sm, unreliable_path_max_m=24.0), "warn", "더 멀리"),
        (f"믿지 못한 쌍이 {rv.SEG_M:g} m 안쪽으로만 더 멂 (23 = 20 + {rv.SEG_M:g}) → pass 그대로", dict(ok_sm, unreliable_path_max_m=23.0), "pass", None),
        ("pass 인데 바닥을 견준 믿을 쌍 없음 (z_max_cm 없음) → warn", {k: v for k, v in ok_sm.items() if k not in ("z_max_cm", "z_worst")}, "warn", "바닥 높이를"),
        ("fail 은 낮추기 규칙과 상관없이 fail", dict(ok_sm, h_max_cm=40.0, unreliable_path_max_m=99.0), "fail", None),
    ]
    for name, sm, want, must in cases:
        v, why = V(sm)
        C.add(f"(e) verdict: {name}", v == want and (must is None or any(must in m for m in why)), f"{v} · {[m[:70] for m in why]}")

    A = _box()
    r = rv.register(A, _box((0.5, 0.0, 0.03)))
    C.add("(e) register: 상자 방을 (0.5, 0, 0.03) m 옮김 → 2d · 믿음 · 수평 50 · 높이 3 cm (±1)",
          r.get("ok") and r["reliable"] and r["h_axis"] == "2d" and abs(r["dx"] - 0.5) < 0.01 and abs(r["dy"]) < 0.01 and abs(r.get("dz", 9) - 0.03) < 0.01,
          {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items() if k in ("h_axis", "dx", "dy", "dz", "fit", "peak2", "reliable")})
    r = rv.register(A, _box((rv.SEARCH_M - 0.03, 0.0, 0.0)))
    C.add(f"(e) register: {rv.SEARCH_M - 0.03:g} m 옮김 (탐색 끝 칸) → 2d 봉우리는 뚜렷하지만 믿지 않음 · unreliable_why 에 탐색 끝",
          r.get("ok") and r["h_axis"] == "2d" and r["edge"] and r["peak2"] < rv.MAX_PEAK2 and not r["reliable"]
          and any("탐색 끝" in m for m in r.get("unreliable_why", [])),
          {k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items() if k in ("h_axis", "dx", "edge", "peak2", "reliable", "unreliable_why")})

    n, fps = 300, 60.0
    t = np.arange(n) / fps
    x = 1.0 * t                                                 # 1 m/s 로 걷기 → 장마다 1.7 cm
    x[200:] += 0.19                                             # 200번째 장에서 19 cm 튐
    keep = np.r_[0:100, 110:n]                                  # 100~109 줄이 빠짐 → 그 사이 한꺼번에 18 cm
    t, x = t[keep], x[keep]
    T = np.repeat(np.eye(4)[None], len(t), axis=0)
    T[:, 0, 3] = x
    step = np.r_[0, np.abs(np.diff(x))]
    jumps, total = rv._jumps(dict(t=t, T=T, step=step))
    C.add("(e) _jumps: 줄이 빠져 한꺼번에 18 cm 움직인 곳은 튐이 아님 · 한 장에 19 cm 튄 곳만 잡음",
          total == 1 and len(jumps) == 1 and abs(jumps[0]["t_s"] - 200 / fps) < 0.01 and jumps[0]["frames"] == 1,
          f"{total}곳 · {jumps}")


def check_inputs(C, short, work):
    for name, fix, must in (("odometry.csv 에 머리글만", "odo", "자세 줄이 없음"), ("camera_matrix.csv 없음", "cam", "camera_matrix.csv")):
        d = os.path.join(work, f"bad_{fix}")
        make_capture(short, d, rows=150)
        if fix == "odo":
            with open(os.path.join(d, "odometry.csv"), encoding="utf-8") as fh:
                hdr = fh.readline()
            with open(os.path.join(d, "odometry.csv"), "w", encoding="utf-8") as fh:
                fh.write(hdr)
        else:
            os.remove(os.path.join(d, "camera_matrix.csv"))
        try:
            rv.measure(d, write=False)
            C.add(f"(f) 잘못된 촬영: {name} → CaptureError", False, "오류가 안 남")
        except stray.CaptureError as e:
            C.add(f"(f) 잘못된 촬영: {name} → CaptureError (validate 문구)", must in str(e), str(e)[:120])
    import zipfile
    zp = os.path.join(work, "top_files.zip")
    with zipfile.ZipFile(zp, "w", zipfile.ZIP_STORED) as z:     # 맨 위에 파일 (폴더 없이)
        for root, _, files in os.walk(short, followlinks=True):
            for f in sorted(files):
                q = os.path.join(root, f)
                z.write(q, os.path.relpath(q, short))
    r = rv.measure(zp, write=False)
    C.add("(f) 맨 위에 파일이 든 zip → source = zip 이름 · 임시 폴더 이름이 아님", r["source"] == "top_files" and r["verdict"] == "skip",
          f"source {r['source']} · {r['verdict']}")


def _cli(args):
    p = subprocess.run([sys.executable, "-m", "scan2sim.stray_revisit"] + args, cwd=RECON, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", env=ENV, timeout=300)
    return p.returncode, p.stdout + p.stderr


def check_norevisit(C, short, drifted, work):
    r = rv.measure(short, write=False)
    C.add("(d) 다시 지나간 곳이 없는 촬영 (처음 15 초) → skip · 이유 문구", r["verdict"] == "skip" and any("다시 지나간 곳이 없음" in m for m in r["reasons"]),
          f"{r['verdict']} · {r['reasons'][:1]}")
    rc, txt = _cli([short, "--no-write"])
    C.add("(d) CLI: 다시 지나간 곳 없음 → 종료 코드 0 · traceback 없음", rc == 0 and "Traceback" not in txt and "skip" in txt,
          f"종료 코드 {rc} · {(txt.strip().splitlines() or [''])[-1][:140]}")
    jp = os.path.join(work, "cli_drift.json")
    rc, txt = _cli([drifted, "--no-write", "--json", jp])
    ok = rc == 0 and "Traceback" not in txt and os.path.isfile(jp) and json.load(open(jp, encoding="utf-8")).get("verdict") == "fail"
    C.add("(d) CLI --json: 틀어진 촬영 → 종료 코드 0 · 결과 파일 판정 fail", ok, f"종료 코드 {rc}")


# ── 차례 ────────────────────────────────────────────────────────────────

def make_fixture(d, room=None):
    """두 바퀴 가상 촬영본을 d/<NAME> 에 만듦 (있으면 그대로) → 촬영 폴더 · 못 만들면 None"""
    cap = os.path.join(d, NAME)
    if os.path.isfile(os.path.join(cap, "odometry.csv")):
        return cap
    os.makedirs(d, exist_ok=True)
    t = time.time()
    room = room if room and os.path.isfile(os.path.join(room, "room.glb")) else os.path.join(d, "room")
    cmds = [] if os.path.isfile(os.path.join(room, "room.glb")) else [[sys.executable, os.path.join(SAMPLES, "make_room.py"), "--out", room, "--seed", "0"]]
    cmds.append([sys.executable, os.path.join(SAMPLES, "make_stray.py"), "--room", room, "--out", d, "--keep-folder"] + FIXTURE)
    for cmd in cmds:
        p = subprocess.run(cmd, cwd=RECON, capture_output=True, text=True, encoding="utf-8", errors="replace", env=ENV)
        if p.returncode:
            print(f"  시험 자료를 만들지 못함 — {os.path.basename(cmd[1])} 종료 코드 {p.returncode}\n" + (p.stdout + p.stderr)[-1500:], flush=True)
            return None
    zp = os.path.join(d, NAME + ".zip")
    if os.path.isfile(zp):
        os.remove(zp)                                           # 풀린 폴더만 씀
    print(f"  시험 자료: {' '.join(FIXTURE)} 가상 촬영본을 만듦 ({time.time() - t:.0f}초) → {cap}", flush=True)
    return cap if os.path.isfile(os.path.join(cap, "odometry.csv")) else None


def section(C, name, fn, *args):
    try:
        return fn(*args)
    except Exception as e:
        traceback.print_exc()
        C.add(f"{name}: 끝까지 돎", False, f"{type(e).__name__}: {str(e)[:200]}")
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fixtures", help="가상 촬영본을 둘 폴더 (있으면 다시 만들지 않음). 안 주면 임시 폴더에 만들고 지움")
    ap.add_argument("--room", help="room.glb 가 든 폴더 (있으면 방을 다시 만들지 않음)")
    ap.add_argument("--work", help="고친 촬영본 · 결과 폴더 꼴을 둘 폴더 (안 주면 임시 폴더를 만들고 끝나면 지움)")
    ap.add_argument("--json", help="결과를 이 파일에")
    a = ap.parse_args()
    t0 = time.time()
    C, rc = Checks(), None
    tmp = tempfile.mkdtemp(prefix="scan2sim_revisit_")
    work = os.path.join(os.path.abspath(a.work) if a.work else tmp, "t")
    shutil.rmtree(work, ignore_errors=True)
    os.makedirs(work)
    src = None
    try:
        section(C, "(e)", check_units, C)                         # 시험 자료 없이 — 자료를 못 만들어도 돎
        src = make_fixture(os.path.abspath(a.fixtures) if a.fixtures else os.path.join(tmp, "fixture"), a.room and os.path.abspath(a.room))
        print(f"revisit: {src}", flush=True)
        if not src:
            rc = 2
        else:
            _, _, t, T = _read_odo(src)
            if t[-1] < 2 * T1 - 2:
                print(f"  중단 — 시험 자료가 짧음 ({t[-1]:.1f} 초, 두 바퀴가 아님)", flush=True)
                rc = 2
            else:
                Td, f, c = drift_poses(t, T)
                drifted = make_capture(src, os.path.join(work, NAME + "_drift"), poses=Td)
                short = make_capture(src, os.path.join(work, NAME + "_short"), rows=int(np.searchsorted(t, 15.0)))
                f_of = lambda tb: float(np.clip((tb - T1) / (t[-1] - T1), 0, 1))   # noqa: E731
                tabs = t + np.genfromtxt(os.path.join(src, "odometry.csv"), delimiter=",", skip_header=1, usecols=0, max_rows=1)
                section(C, "(a)", check_clean, C, src)
                section(C, "(b)", check_drift, C, drifted, f_of, c)
                section(C, "(c)", check_job, C, make_job(os.path.join(work, "job"), drifted, undo_info(tabs, T, Td, f)), drifted)
                section(C, "(d)", check_norevisit, C, short, drifted, work)
                section(C, "(f)", check_inputs, C, short, work)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
        ok = C.ok and rc is None
        secs = round(time.time() - t0, 1)
        if a.json:
            json.dump(dict(fixture=src, drift=DRIFT, t1_s=T1, tol=TOL, ok=ok, checks=C.rows, total_s=secs),
                      open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"revisit {'통과' if ok else '실패'} — 검사 {sum(r['ok'] for r in C.rows)}/{len(C.rows)} · {secs}초", flush=True)
    sys.exit(rc if rc is not None else (0 if ok else 1))


if __name__ == "__main__":
    main()
