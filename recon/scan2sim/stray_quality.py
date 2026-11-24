"""아이폰 LiDAR 원본(Stray Scanner)으로 만든 맵의 복원 정확도 — 맵을 만들 때 안 쓴 프레임의 깊이와 맵을 비교.

    cd recon
    python -m scan2sim.stray_quality <scan2sim 결과 폴더> [--n 30] [--json <경로>] [--capture <촬영 폴더 또는 zip>] [--drift]
                                     [--min-conf 0|1|2] [--min-gap-s 0.1] [--k-source odometry|camera_matrix]

무엇을 재나 (measure)
    stray.py 는 촬영 프레임 중 5 cm · 5° 넘게 움직인 장만 키프레임으로 골라 맵(TSDF)을 만든다. 고르지 않은 프레임은 맵에 안 들어갔으므로
    그 프레임의 자세에서 맵을 렌더한 깊이와 그 프레임이 실제로 잰 LiDAR 깊이(256×192 원본)를 비교하면 맵이 얼마나 맞는지 알 수 있다.
    - 촬영 확인: 찾은 촬영 폴더 이름 = stray_meta.json 의 source, 맵 키프레임 시각이 모두 odometry.csv 의 줄과 맞아야 함. 아니면 멈춤
      (예전에 푼 input_map/_unzip 이 남았거나 --capture 가 다른 촬영 — 다른 자세로 재면 숫자가 그럴듯하게 틀림)
    - 안 쓴 프레임 n 장: 처음 skip_s(1 초)와 너무 빠른 장 · 깊이 없는 장 · 키프레임은 빼고, 촬영 시간(처음 skip_s 뒤 ~ 끝)을 n 구간으로 나눠
      구간마다 한 장 — 목표 시각(seed 0 = 구간 가운데, 다른 seed = 구간 안 다른 자리)에 가장 가까운, 가장 가까운 키프레임에서
      min_gap_s(0.1 초) 넘게 떨어진 장. 구간 안에 그런 장이 없으면 그 구간만 기준을 낮춰 키프레임에서 가장 멀리 떨어진 장
      (결과 held_out.relaxed_cells · notes 에 남김). 키프레임이 촘촘하든 성기든 같은 구간 · 거의 같은 시각을 잼.
      11/24 전에는 0.1 초 넘는 후보 전체를 n 묶음으로 나눠 무작위로 골랐음. 기준 촬영본(118 초)에서 키프레임 1133장 맵은 그런 후보가
      175장뿐(681장 맵은 721장)이라 두 맵이 고른 장이 통째로 달랐고 (k번째 장끼리 시각 차 중앙 4.5~6.6 초) 중앙 차이가 맵끼리 0.3~0.9 cm 달랐음.
      구간마다 고르면 시각 차 중앙 0.15~0.5 초 · 맵끼리 0.2~0.5 cm (n=60 에서 0.22~0.26 cm). 같은 장으로 재면 두 맵은 0.04 cm 안으로 같음 —
      남은 차이는 맵이 아니라 장마다 다른 오차 (1133장 맵은 n=60 중 38구간에서 기준을 낮춤)
      실제로 떨어진 시간 · 장 수 · 거리 · 목표 시각과의 차이 분포는 결과 held_out 에
    - 자세 보정: 맵을 stray_mesh 가 고친 자세로 만들었으면 (보정 info 의 applied · build/tsdf_meta.json 의 pose_fix.applied 둘 다 참)
      안 쓴 프레임 자세에도 같은 보정을 시각으로 이어 적용 (stray_fix.apply_fix — 높이 → 수평, applied 인 단계만).
      info 파일 = build/pose_fix.json (tsdf_meta.json 의 pose_fix.file · 없으면 input_map/ 의 같은 이름) — 파일 sha256 이 tsdf_meta.json 의
      pose_fix.sha256 과 같을 때만 씀 (다르면 이 메쉬와 다른 보정 — 적용하지 않고 note). sha256 이 없는 예전 빌드는 input_map/pose_fix.json →
      floor_fix.json (11/24 af65f1b 판). 결과 pose_fix 에 적용 여부 · 파일 · 단계(steps) · 높이 dz_p5_p95_cm · 수평 xy_shift_cm_p50_p95 ·
      적용 못 한 이유(note)
    - 짝짓기: 깊이 · 신뢰도 파일은 stray.frame_ids 로 (odometry.csv 의 frame 번호 — stray.py 와 같음). 신뢰도 크기가 깊이와 다른 장은 재지 않음
    - 깊이: 맵을 만들 때와 같은 규칙 (stray_meta.json 의 depth_rule → stray.depth_keep). hf 면 바닥 높이 조건은 stray.py 와 똑같이 —
      보정 전 자세(stray.py 는 stray_fix 보다 먼저 돎) · camera_matrix.csv 한 값을 깊이 크기로 줄인 내부 파라미터 (아래 '내부 파라미터' 와 상관없이).
      기준 촬영본 키프레임 120장에 적용하면 stray.py 가 남긴 화소와 100% 같음. no_confidence 맵(신뢰도 그림 없음)은 near_conf1_m(3 m)까지.
      depth_rule 이 없는 예전 맵이나 min_conf 를 주면 신뢰도 min_conf 이상(주지 않으면 stray_meta.json 의 min_confidence).
      쓴 규칙은 결과 depth_rule. measure · drift 모두 같음
      깊이 한계 = min(맵 TSDF 의 depth_trunc, stray.py 가 키프레임 깊이를 자른 select.max_depth_m) 이하. depth_trunc 는 build/tsdf_meta.json —
      없으면 stray_mesh 기본 5 m (depth_rule 이 없는 예전 맵은 3 m — 그때는 3 m 로 합침)
    - 내부 파라미터: 기본은 odometry.csv 의 프레임별 값(그 장의 실제 값 — 맵이 한 값을 써서 생긴 오차도 맵 오차로 잡으려고).
      맵(stray.py)은 camera_matrix.csv 한 값으로 만들었음 — --k-source camera_matrix 로 맵과 같은 값을 쓸 수 있음.
      stray1(프레임마다 초점거리가 최대 1.5% 다름)에서 중앙 차이 1.23 → 1.21 cm 로 거의 같음. 색 영상 크기 → 깊이 크기로 줄임 (화소 중심 기준)
    - 자세: odometry.csv (카메라 광학 → 세계, y 위) 에 stray.py 와 같은 Y_UP_TO_Z_UP 을 곱함 = 맵 폴더 세계 = mesh_raw.ply 좌표.
      visual.ply(최종 보이는 메쉬)는 씬 좌표라 build/transform.json 의 T_input_to_scene 을 한 번 더 곱해 따로 잼
    - 결과: 차이(잰 깊이 − 맵 깊이, 광축 방향 z) 절댓값의 중앙 · 90% · 2 cm 안 · 5 cm 안, 잰 깊이 중 맵에 닿은 비율, 프레임별 중앙값
    - 판정: 중앙 3 cm 이하 통과 · 3 cm 넘으면 주의 · 10 cm 넘으면 실패(자세 · 좌표가 어긋난 것). 맵에 닿은 비율 70% 미만도 주의.
      쓸 수 있는 깊이가 있는 안 쓴 프레임이 하나도 없으면 실패 (잴 수 없음)
    - <결과 폴더>/input_map/stray_quality.json 에 씀 (다른 키 — drift 등 — 는 그대로 두고 합침)

한계
    - 키프레임을 5 cm · 5° 마다 고르므로 어느 프레임이든 가까운 키프레임과 몇 cm · 몇 도 안이고, 시간으로도 멀 수 없음.
      stray1(17 초 · 839장 · 키프레임 136장): 키프레임 사이 중앙 0.1 초 · 최대 0.58 초. 안 쓴 후보 665장 중 가장 가까운 키프레임에서
      0.05 초 이상 289장 · 0.1 초 이상 59장 · 0.15 초 이상 21장 · 0.25 초 이상 6장 · 0.3 초 이상 0장.
      그래서 min_gap_s 는 n=30 을 기준 그대로 채우는 가장 큰 값 근처인 0.1 초(60 장/초에서 6장). 고른 장은 키프레임에서 중앙 0.12 초 · 위치 1.7 cm.
      0.1 초를 넘는 장은 천천히 움직인 구간에 몰림 (stray1 에서 17 초 중 10 개 초에만) — 그래서 구간마다 고르고, 그런 장이 없는 구간은 기준을 낮춤
      (위 숫자는 11/23 예전 고르기로 잰 것)
    - 그래도 자세는 같은 VIO 가 키프레임과 0.1~0.3 초 사이에 낸 것이라 사실상 독립이 아님 — 자세 흔들림 · 깊이 잡음 · 복셀(2 cm) 크기를 재는 것이고,
      긴 구간에 걸쳐 쌓이는 틀어짐(드리프트)은 안 보임. stray1 중앙 차이: 키프레임 자체 1.10 cm · 기준 없이(키프레임 바로 옆 장) 1.08 cm ·
      0.1 초 기준 1.23 cm (seed 0~3 에서 1.23~1.34, 기준 없이는 1.08~1.16) — 0.1~0.2 cm 커질 뿐. 드리프트는 drift() 가 따로 (되돌아온 촬영만)
    - 움직이는 물체(사람)는 차이로 잡힘. 유리 · 거울은 LiDAR 깊이 자체가 틀릴 수 있음

drift (기본으로 돌지 않음 — --drift 또는 drift() 를 따로 부를 때만)
    촬영이 출발한 자리 근처로 돌아왔을 때(처음과 끝 위치 1 m 안 · 바라보는 방향 30° 안)만 잼. 처음 구간(최대 30 초, 촬영 길이의 1/3 이하)의
    원본 깊이로만 TSDF 를 따로 만들고, 마지막 구간 프레임의 깊이를 (1) 그 메쉬와 비교하고 (2) 그 점들을 ICP 로 메쉬에 맞춰 옮겨야 하는 양을 잼.
    - 옮긴 양 = 마지막 구간 점들이 ICP 변환 M 으로 실제 움직인 거리 ‖M p − p‖ 의 중앙 · 90% (점은 복셀 크기로 솎은 것).
      M 의 이동 성분(transform_translation_cm)은 세계 원점 기준이라 회전이 있으면 점의 움직임과 다름 — 참고로만 남김
    - 처음과 끝 사이에 쌓인 틀어짐의 추정. 두 구간이 같은 곳을 충분히 겹쳐 봤을 때만 뜻이 있음 (ICP 맞은 비율 fitness 로 확인)
    - 중간 구간의 틀어짐 분포 · 되돌아오지 않는 촬영의 틀어짐은 모름. max_gap_m · max_turn_deg 를 크게 주면 억지로 잴 수 있음 (점검용)
    - 맵을 고친 자세로 만들었으면 (measure 와 같은 조건) 모든 프레임 자세에 같은 보정을 적용하고 잼 — 보정 뒤에 남은 틀어짐

오류
    촬영을 못 찾음 · 다른 촬영 · 맵 메쉬 없음은 SystemExit(한국어 문장)으로 멈춤 — stray.CaptureError 도 이 문장으로 바꿈 (CLI 에 traceback 없음)
"""
import argparse
import json
import os
import time

import numpy as np

from . import stray, stray_fix
from .stray import _rot

PASS_CM, FAIL_CM, MIN_HIT_PCT = 3.0, 10.0, 70.0                  # 판정 기준 (중앙 차이 cm · 맵에 닿은 비율 %)
MIN_GAP_S = 0.1                                                 # 안 쓴 프레임 ↔ 가장 가까운 키프레임 최소 시간 (근거는 위 한계)
Y_UP_TO_Z_UP = stray.Y_UP_TO_Z_UP                               # 아이폰 세계(y 위) → 맵 폴더 세계(z 위). stray.py 와 같은 값
K_SOURCES = ("odometry", "camera_matrix")
_STALE = "맵을 만든 그 촬영을 --capture 로 줄 것 (input_map/_unzip 에 예전 촬영이 남아 있으면 지울 것)"


# ── 읽기 ───────────────────────────────────────────────────────────────

def _meta(mp):
    p = os.path.join(mp, "stray_meta.json")
    if not os.path.exists(p):
        raise SystemExit(f"stray_meta.json 이 없음 ({p}) — 아이폰(Stray Scanner) 원본으로 만든 결과 폴더가 아님")
    return json.load(open(p, encoding="utf-8"))


def _capture(out_dir, capture=None, source=None):
    """촬영 폴더 찾기: input_map/_unzip/<source> → input_map/_unzip (zip 으로 받은 경우) → capture 인자 (폴더 또는 zip).
    source(stray_meta.json 의 촬영 폴더 이름)가 있으면 이름이 같은 것만 받음. 못 찾으면 SystemExit (stray.CaptureError 도 문장으로 바꿈)"""
    work = os.path.join(out_dir, "input_map", "_unzip")
    cap_p = os.path.abspath(os.path.expanduser(capture)) if capture else None
    tries = ([os.path.join(work, source)] if source else []) + [work] + ([cap_p] if cap_p else [])
    errs, other = [], []
    for p in tries:
        if not os.path.exists(p):
            continue
        try:
            src = stray.find_capture(p, work if p == cap_p else None)
        except stray.CaptureError as e:
            errs.append(f"{p}: {e}")
            continue
        if not src:
            continue
        name = os.path.basename(os.path.normpath(src))
        if source and name != source:
            other.append(f"'{name}' ({src})")
            continue
        return src
    if other:
        raise SystemExit(f"촬영 원본이 이 맵과 다름 — 맵은 '{source}' 촬영으로 만들었는데 찾은 것은 {', '.join(other)}. {_STALE}")
    raise SystemExit("촬영 원본을 찾지 못함 — input_map/_unzip 이 없으면 --capture <촬영 폴더 또는 zip> 을 줄 것"
                     + ("".join(f" / {e}" for e in errs)))


def _load(src, meta, k_source="odometry"):
    """odometry.csv → 시각 · 맵 폴더 세계(Z 위) 자세 · 프레임별 내부 파라미터(색 영상 크기) · 속력.
    k_source=camera_matrix 면 내부 파라미터를 camera_matrix.csv 한 값으로 (맵을 만들 때와 같음)"""
    if k_source not in K_SOURCES:
        raise ValueError(f"k_source 는 {K_SOURCES} 중 하나: {k_source!r}")
    od = np.atleast_2d(np.genfromtxt(os.path.join(src, "odometry.csv"), delimiter=",", skip_header=1, usecols=range(13)))
    t, pos = od[:, 0], od[:, 2:5]
    T = np.repeat(np.eye(4)[None], len(od), axis=0)
    T[:, :3, :3] = [_rot(q) for q in od[:, 5:9]]
    T[:, :3, 3] = pos
    T = Y_UP_TO_Z_UP @ T                                        # stray.py 와 같은 세계 (모듈 값을 부를 때 읽음 — 점검에서 바꿔 볼 수 있게)
    step = np.r_[0, np.linalg.norm(np.diff(pos, axis=0), axis=1)]
    dt = np.r_[1, np.maximum(np.diff(t), 1e-3)]
    speed = np.convolve(step / dt, np.ones(5) / 5, mode="same")   # stray.py 와 같은 5장 평균 속력
    K = od[:, 9:13]
    Kc = np.loadtxt(os.path.join(src, "camera_matrix.csv"), delimiter=",")
    K_map = (Kc[0, 0], Kc[1, 1], Kc[0, 2], Kc[1, 2])            # stray.py 가 쓴 한 값 — 깊이 규칙(바닥 높이)은 늘 이것으로
    if k_source == "camera_matrix":
        K = np.tile(K_map, (len(od), 1))
    cs = meta.get("color_size") or [1920, 1440]
    fid, _ = stray.frame_ids(src, od[:, 1])                     # 줄 번호 → 깊이 · 신뢰도 파일 번호 (stray.py 와 같은 짝짓기)
    return dict(t=t, T=T, T_raw=T, K=K, K_map=K_map, speed=speed, color_w=cs[0], color_size=cs,   # T_raw = 보정 전 자세 (apply_fix 는 새 배열)
                fid=fid)


def _dname(cap, i):
    """줄 i 의 깊이 · 신뢰도 파일 이름 (frame 번호 — stray.frame_ids)"""
    return f"{int(cap['fid'][i]):06d}.png"


def _keyframe_rows(mp, t, src=None, frames_total=None):
    """trajectory_camera_optical.txt 의 시각 → odometry.csv 의 줄 번호 (키프레임으로 쓴 장).
    키프레임이 모두 서로 다른 줄과 맞지 않거나 줄 수가 stray_meta.json 과 다르면 다른 촬영 → SystemExit"""
    tk = np.atleast_1d(np.loadtxt(os.path.join(mp, "trajectory_camera_optical.txt"), comments="#", usecols=0))
    j = np.clip(np.searchsorted(t, tk), 1, len(t) - 1)
    j = np.where(np.abs(t[j - 1] - tk) <= np.abs(t[j] - tk), j - 1, j)
    hit = np.abs(t[j] - tk) < 1e-3
    rows = set(j[hit].tolist())
    if len(rows) != len(tk):
        raise SystemExit(f"촬영 원본이 이 맵과 맞지 않음 — 맵 키프레임 {len(tk)}장 중 {len(rows)}장만 odometry.csv 의 시각과 맞음 ({src}). {_STALE}")
    if frames_total and int(frames_total) != len(t):
        raise SystemExit(f"촬영 원본이 이 맵과 맞지 않음 — odometry.csv 줄 수 {len(t)} ≠ 맵을 만들 때 {frames_total} ({src}). {_STALE}")
    return rows


DEPTH_TRUNC_DEFAULT, DEPTH_TRUNC_OLD = 5.0, 3.0                 # tsdf_meta.json 이 없을 때: stray_mesh.build 기본 · depth_rule 이 없는 예전 맵


def _depth_trunc(bd, default=DEPTH_TRUNC_DEFAULT):
    """맵을 만들 때 쓴 깊이 한계 (stray_mesh · tsdf.py 가 남긴 설정). 없으면 default"""
    p = os.path.join(bd, "tsdf_meta.json")
    return float(json.load(open(p, encoding="utf-8"))["tsdf"]["depth_trunc"]) if os.path.exists(p) else float(default)


def _limits(meta, bd, min_conf, rule=None):
    """→ (TSDF depth_trunc, 쓸 깊이 한계 = min(depth_trunc, select.max_depth_m), 신뢰도 기준 — None 이면 맵을 만들 때와 같게).
    rule 이 no_confidence 면 한계도 near_conf1_m (그 맵은 그 안의 깊이로만 만듦)"""
    trunc = _depth_trunc(bd, DEPTH_TRUNC_DEFAULT if meta.get("depth_rule") else DEPTH_TRUNC_OLD)
    mx = (meta.get("select") or {}).get("max_depth_m")
    lim = min(trunc, float(mx)) if mx else trunc
    if rule is not None and rule.get("name") == "no_confidence" and rule.get("near_conf1_m"):
        lim = min(lim, float(rule["near_conf1_m"]))
    if min_conf is None:
        min_conf = int(meta.get("min_confidence", 1))
    return trunc, lim, int(min_conf)


FIX_FILES = ("pose_fix.json", "floor_fix.json")                 # input_map 의 자세 보정 info — 예전 판(단계 묶음) · 11/24 af65f1b 판(높이 하나)


def _sha256(path):
    import hashlib
    with open(path, "rb") as f:
        return hashlib.sha256(f.read()).hexdigest()


def _fix_step_ok(s):
    """applied 인 단계 info 에 적용할 값이 다 있나 → 빠진 것 설명 또는 None"""
    kind = stray_fix.step_kind(s)
    t = s.get("t")
    if kind is None:
        return f"어떤 보정인지 모르는 단계 (method {s.get('method')!r})"
    need = ("dz",) if kind == "floor" else ("dx", "dy")
    if not isinstance(t, list) or len(t) < 1 or any(not isinstance(s.get(k), list) or len(s[k]) != len(t) for k in need):
        return f"{kind} 단계에 키프레임 시각 · 보정값({' · '.join(('t',) + need)})이 없음"
    for k in ("tilt", "dyaw_deg"):
        if s.get(k) is not None and (not isinstance(s[k], list) or len(s[k]) != len(t)):
            return f"{kind} 단계의 {k} 길이가 키프레임 시각과 다름"
    return None


def _pose_fix(mp, bd):
    """맵을 고친 자세로 만들었으면 그 보정 info → (info 또는 None, 결과에 남길 요약).
    새 빌드 (build/tsdf_meta.json 의 pose_fix 에 sha256 이 있음 — 11/25 stray_mesh): pose_fix.applied 가 거짓이면 아무것도 읽지 않음.
      참이면 build/<pose_fix.file> → input_map/<같은 이름> 차례로 찾아 파일 sha256 이 같은 것만 씀. 같은 것이 없으면 (다른 보정으로 다시 만들다
      멈춘 빌드 · 손으로 고친 파일 · 지운 파일) 적용하지 않고 note 에 남김 — 이 메쉬와 다른 보정을 조용히 적용하지 않게.
    예전 빌드 (sha256 없음): input_map/<pose_fix.file> — file 이 없으면 input_map/pose_fix.json → floor_fix.json (11/24 af65f1b 판) 차례.
      info 가 applied 라도 build/tsdf_meta.json 이 보정 없이 만든 맵이면 (pipeline/tsdf.py · --no-fix · 예전 파일) 적용하지 않음.
    단계 묶음이면 applied 인 단계만 차례대로 (stray_fix.apply_fix)"""
    pf, tp = None, os.path.join(bd, "tsdf_meta.json")
    if os.path.exists(tp):
        try:
            pf = json.load(open(tp, encoding="utf-8")).get("pose_fix")
        except (OSError, ValueError, AttributeError):
            pf = None
    pf = pf if isinstance(pf, dict) else None
    named = os.path.basename(str(pf.get("file"))) if pf and pf.get("file") else None
    want = str(pf["sha256"]) if pf and pf.get("sha256") else None
    if want:                                                    # 새 빌드: 메쉬와 같이 쓴 info 만 (sha256)
        name = named or FIX_FILES[0]
        if not pf.get("applied"):
            return None, dict(applied=False, status=pf.get("status"), file=f"build/{name}",
                              steps=[dict(kind=s.get("kind"), applied=bool(s.get("applied")), status=s.get("status")) for s in pf.get("steps") or []
                                     if isinstance(s, dict)])
        tries = [(os.path.join(bd, name), f"build/{name}"), (os.path.join(mp, name), f"input_map/{name}")]
        found = [(p, rel) for p, rel in tries if os.path.exists(p)]
        hit = next(((p, rel) for p, rel in found if _sha256(p) == want), None)
        if hit is None:
            return None, dict(applied=False, status=None, file=found[0][1] if found else None, note=(
                f"맵은 고친 자세로 만들었는데 {' · '.join(rel for _, rel in found)} 이 그 보정과 다름 (sha256 이 build/tsdf_meta.json 과 다름 — "
                "다른 보정으로 다시 만들다 멈춘 빌드 · 고친 파일일 수 있음) — 안 쓴 프레임 자세를 그대로 씀" if found else
                f"맵은 고친 자세로 만들었는데 보정 info({' · '.join(rel for _, rel in tries)})가 없음 — 안 쓴 프레임 자세를 그대로 씀"))
        path, rel = hit
    else:                                                       # 예전 빌드
        names = [named] if named else [f for f in FIX_FILES if os.path.exists(os.path.join(mp, f))][:1]
        if not names or not os.path.exists(os.path.join(mp, names[0])):
            st = dict(applied=False, status=None)
            if pf and pf.get("applied"):
                st["note"] = f"맵은 고친 자세로 만들었는데 보정 info(input_map/{names[0] if names else FIX_FILES[0]})가 없음 — 안 쓴 프레임 자세를 그대로 씀"
            return None, st
        path, rel = os.path.join(mp, names[0]), f"input_map/{names[0]}"
    try:
        info = json.load(open(path, encoding="utf-8"))
    except (OSError, ValueError) as e:
        return None, dict(applied=False, status=None, note=f"{rel} 을 읽지 못함 — 자세 그대로 잼 ({e})")
    if not isinstance(info, dict):
        return None, dict(applied=False, status=None, note=f"{rel} 이 dict 가 아님 — 자세 그대로 잼")
    steps = info["steps"] if isinstance(info.get("steps"), list) else [info]
    st = dict(applied=False, status=info.get("status"), file=rel,
              steps=[dict(kind=stray_fix.step_kind(s), applied=bool(s.get("applied")), status=s.get("status")) for s in steps if isinstance(s, dict)])
    if not info.get("applied"):
        return None, st
    if os.path.exists(tp) and not (pf and pf.get("applied")):
        st["note"] = f"{rel} 은 고쳤다고 하지만 맵(build/tsdf_meta.json)은 자세를 고치지 않고 만들어짐 — 안 쓴 프레임 자세를 그대로 씀"
        return None, st
    bad = [why for s in steps if isinstance(s, dict) and s.get("applied") for why in [_fix_step_ok(s)] if why]
    if bad or not all(isinstance(s, dict) for s in steps):
        st["note"] = f"{rel}: {'; '.join(bad) or '단계 info 가 dict 가 아님'} — 안 쓴 프레임 자세를 그대로 씀"
        return None, st
    st["applied"] = True
    for s in steps:
        k = stray_fix.step_kind(s)
        if not s.get("applied"):
            continue
        if k == "floor":
            dz = np.asarray(s["dz"], dtype=float)
            st.update(dz_p5_p95_cm=s.get("dz_p5_p95_cm"), dz_max_abs_cm=round(100 * float(np.abs(dz).max()), 1))
        elif k == "xy":
            sh = np.hypot(np.asarray(s["dx"], dtype=float), np.asarray(s["dy"], dtype=float))
            st.update(xy_axes=s.get("axes_applied"), xy_shift_cm_p50_p95=s.get("shift_cm_p50_p95"), xy_shift_max_cm=round(100 * float(sh.max()), 1))
    return info, st


def _fix_text(pf):
    """결과 pose_fix 요약 → 한 줄 설명 (CLI)"""
    parts = []
    if pf.get("dz_p5_p95_cm") is not None:
        parts.append(f"높이 5~95% {pf['dz_p5_p95_cm']} cm")
    if pf.get("xy_shift_cm_p50_p95") is not None:
        parts.append(f"수평({'·'.join(pf.get('xy_axes') or [])}) 중앙 · 95% {pf['xy_shift_cm_p50_p95']} cm")
    return f"{pf.get('file')}: " + " · ".join(parts)


def _rule(meta, min_conf):
    """안 쓴 프레임 깊이에 쓸 규칙: min_conf 를 주지 않았으면 맵을 만들 때의 depth_rule (없는 예전 맵 · "min" 규칙은 None = 신뢰도 기준만)"""
    r = meta.get("depth_rule")
    return r if min_conf is None and isinstance(r, dict) and r.get("name") != "min" else None


def _depth(src, i, trunc, min_conf, rule=None, cap=None):
    """원본 깊이(m) · 쓸 수 있는 화소. 없거나 깨졌으면 (신뢰도 크기가 다름 등) None. rule 이 있으면 stray.py 와 같은 규칙
    (높이 조건은 cap 의 보정 전 자세 · stray.py 와 같은 내부 파라미터 _intr_map). 파일은 frame 번호로 (cap 이 없으면 줄 번호)"""
    import cv2
    name = _dname(cap, i) if cap is not None and "fid" in cap else f"{i:06d}.png"
    z = cv2.imread(os.path.join(src, "depth", name), cv2.IMREAD_UNCHANGED)
    if z is None or z.ndim != 2:
        return None, None
    cp = os.path.join(src, "confidence", name)
    c = cv2.imread(cp, cv2.IMREAD_UNCHANGED) if os.path.exists(cp) else None
    if c is not None and c.shape != z.shape:                    # 깨졌거나 섞인 파일 — 이 장은 재지 않음 (stray.py 는 키프레임이면 멈춤)
        return None, None
    ok = (z > 0) & (z <= trunc * 1000)
    if rule is not None:
        ok &= stray.depth_keep(z, c, rule, cap["T_raw"][i], _intr_map(cap, z.shape[1], z.shape[0]))
    elif c is not None:
        ok &= c >= min_conf
    return z.astype(np.float32) / 1000.0, ok


def _intr(cap, i, w):
    """프레임 i 의 내부 파라미터를 색 영상 크기 → 깊이 크기 w 로. 화소 중심 기준: c' = (c + 0.5)·s − 0.5"""
    s = w / cap["color_w"]
    fx, fy, cx, cy = cap["K"][i]
    return fx * s, fy * s, (cx + 0.5) * s - 0.5, (cy + 0.5) * s - 0.5


def _intr_map(cap, w, h):
    """stray.py 가 키프레임 깊이를 고를 때 쓴 내부 파라미터: camera_matrix.csv 한 값을 깊이 크기 w×h 로 (가로 · 세로 따로, 화소 중심 기준)"""
    W0, H0 = cap["color_size"]
    sx, sy = w / W0, h / H0
    fx, fy, cx, cy = cap["K_map"]
    return fx * sx, fy * sy, (cx + 0.5) * sx - 0.5, (cy + 0.5) * sy - 0.5


def _rays(cap, i, z, ok):
    """쓸 수 있는 화소마다 광선 (원점 · 단위 방향, 세계) 과 z/거리 비, 잰 깊이."""
    v, u = np.nonzero(ok)
    fx, fy, cx, cy = _intr(cap, i, z.shape[1])
    d = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones(len(u))], 1)
    nr = np.linalg.norm(d, axis=1)
    T = cap["T"][i]
    return np.c_[np.broadcast_to(T[:3, 3], d.shape), (d / nr[:, None]) @ T[:3, :3].T], nr, z[v, u]


def _scene(mesh):
    import open3d as o3d
    sc = o3d.t.geometry.RaycastingScene()
    sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return sc


def _cast(sc, rays, nr):
    """광선 → 맵 깊이 (광축 방향 z). 안 닿으면 inf."""
    import open3d as o3d
    th = sc.cast_rays(o3d.core.Tensor(rays.astype(np.float32)))["t_hit"].numpy()
    return th / nr


def _stats(zm, zr):
    hit = np.isfinite(zr)
    d = (zm - zr)[hit]
    a = np.abs(d)
    if not len(a):
        return dict(rays=int(len(zm)), hit_pct=0.0)
    return dict(rays=int(len(zm)), hit_pct=round(100 * float(hit.mean()), 1),
                median_cm=round(100 * float(np.median(a)), 2), p90_cm=round(100 * float(np.percentile(a, 90)), 2),
                within_2cm_pct=round(100 * float((a < 0.02).mean()), 1), within_5cm_pct=round(100 * float((a < 0.05).mean()), 1),
                signed_median_cm=round(100 * float(np.median(d)), 2))   # + = 잰 깊이가 맵보다 멂


def _time_gap(t, kf, idx):
    """프레임 idx 마다 시간으로 가장 가까운 키프레임까지 → (초, 장 수). kf 는 정렬된 줄 번호"""
    kf, idx = np.asarray(kf), np.asarray(idx, dtype=int)
    j = np.searchsorted(kf, idx)
    lo, hi = kf[np.clip(j - 1, 0, len(kf) - 1)], kf[np.clip(j, 0, len(kf) - 1)]
    return np.minimum(np.abs(t[idx] - t[lo]), np.abs(t[idx] - t[hi])), np.minimum(np.abs(idx - lo), np.abs(idx - hi))


def _grid_picks(t, cand, gap, min_gap_s, t_a, t_b, n, seed=0, eps=1e-3):
    """촬영 시간 [t_a, t_b] 를 n 구간으로 나눠 구간마다 한 장: 목표 시각(seed 0 = 구간 가운데, 다른 seed = 구간 안 다른 자리 — 모든 구간에 같은 값)에
    가장 가까운, 키프레임에서 min_gap_s 넘게 떨어진 후보 (같은 거리면 앞 장). 구간에 그런 장이 없으면 그 구간만 기준을 낮춰
    키프레임에서 가장 멀리 떨어진 장 (같으면 목표에 가까운 장). 후보가 없는 구간은 건너뜀.
    cand = 후보 줄 번호 (시각 순), gap = 후보마다 가장 가까운 키프레임까지 초. → (고른 줄 번호, 목표 시각과의 차이 초, 기준을 낮춘 구간 수)"""
    cand, gap = np.asarray(cand, dtype=int), np.asarray(gap, dtype=float)
    if n <= 0 or not len(cand):
        return [], np.zeros(0), 0
    phase = 0.5 if not seed else float(np.random.default_rng(seed).uniform())
    w = (t_b - t_a) / n
    tc = t[cand]
    cell = np.clip(np.floor((tc - t_a) / w).astype(int) if w > 0 else np.zeros(len(cand), int), 0, n - 1)
    picks, off, relaxed = [], [], 0
    for j in range(n):
        s = np.nonzero(cell == j)[0]
        if not len(s):
            continue
        tg = t_a + (j + phase) * w
        d = np.abs(tc[s] - tg)
        ok = gap[s] >= min_gap_s - eps
        if ok.any():
            k = s[ok][np.argmin(d[ok])]                         # argmin 은 같은 값이면 앞 장
        else:
            relaxed += 1
            k = s[np.lexsort((d, -gap[s]))[0]]
        picks.append(int(cand[k]))
        off.append(abs(float(tc[k] - tg)))
    return picks, np.asarray(off), relaxed


def _dist(x):
    x = np.asarray(x, dtype=float)
    return dict(min=round(float(x.min()), 3), median=round(float(np.median(x)), 3), p90=round(float(np.percentile(x, 90)), 3),
                max=round(float(x.max()), 3)) if len(x) else {}


def _write(mp, res, key=None):
    """input_map/stray_quality.json 에 합쳐 씀 — key 가 있으면 그 키만, 없으면 res 의 키만 바꾸고 다른 키(drift 등)는 그대로"""
    p = os.path.join(mp, "stray_quality.json")
    try:
        q = json.load(open(p, encoding="utf-8")) if os.path.exists(p) else {}
    except (OSError, ValueError):
        q = {}
    if not isinstance(q, dict):
        q = {}
    if key:
        q[key] = res
    else:
        q.update(res)
    json.dump(q, open(p, "w", encoding="utf-8"), ensure_ascii=False, indent=1)


# ── 복원 정확도 ─────────────────────────────────────────────────────────

def verdict(st):
    """→ (pass|warn|fail, 이유 목록)"""
    if "median_cm" not in st:
        return "fail", ["안 쓴 프레임의 깊이가 맵에 하나도 닿지 않음 — 자세 · 좌표가 어긋났거나 맵이 비었음"]
    why, v = [], "pass"
    if st["median_cm"] > FAIL_CM:
        v = "fail"
        why.append(f"깊이 차이 중앙 {st['median_cm']} cm — {FAIL_CM:g} cm 를 넘음 (자세 · 좌표가 어긋난 것)")
    elif st["median_cm"] > PASS_CM:
        v = "warn"
        why.append(f"깊이 차이 중앙 {st['median_cm']} cm — {PASS_CM:g} cm 를 넘음 (자세가 흔들렸거나 움직인 물체가 많음)")
    if st["hit_pct"] < MIN_HIT_PCT:
        v = "fail" if v == "fail" else "warn"
        why.append(f"잰 깊이 중 맵에 닿은 것 {st['hit_pct']}% — {MIN_HIT_PCT:g}% 미만 (맵에 빈 곳이 많음)")
    return v, why


_MAIN_KEYS = ("hit_pct", "median_cm", "p90_cm", "within_2cm_pct", "within_5cm_pct", "signed_median_cm")


def measure(out_dir, n=30, seed=0, capture=None, min_conf=None, write=True, min_gap_s=MIN_GAP_S, k_source="odometry"):
    """scan2sim 결과 폴더(input_map/ · build/ 가 있는 곳) → 복원 정확도 dict. write 면 input_map/stray_quality.json 에도 합쳐 씀.
    min_conf=None → 맵을 만들 때와 같은 신뢰도 기준. min_gap_s = 안 쓴 프레임이 가장 가까운 키프레임에서 떨어져야 할 시간(초).
    k_source = 내부 파라미터 (odometry: 프레임별 · camera_matrix: 맵을 만들 때와 같은 한 값)"""
    import open3d as o3d
    t0 = time.time()
    out_dir = os.path.abspath(os.path.expanduser(out_dir))
    mp, bd = os.path.join(out_dir, "input_map"), os.path.join(out_dir, "build")
    meta = _meta(mp)
    sel = meta.get("select") or {}
    src = _capture(out_dir, capture, meta.get("source"))
    cap = _load(src, meta, k_source)
    t, N = cap["t"], len(cap["t"])
    kf = _keyframe_rows(mp, t, src, meta.get("frames_total"))
    kfs = np.array(sorted(kf))
    fix, fix_st = _pose_fix(mp, bd)
    if fix is not None:                                         # 맵을 고친 자세로 만들었음 → 안 쓴 프레임 자세에도 같은 보정
        cap["T"] = stray_fix.apply_fix(t, cap["T"], fix)
    rule = _rule(meta, min_conf)                                # _limits 가 min_conf 를 채우기 전에 (주지 않았을 때만 맵의 규칙)
    trunc, lim, min_conf = _limits(meta, bd, min_conf, rule)
    start = int(np.searchsorted(t, t[0] + sel.get("skip_s", 1.0)))
    cand = np.array([i for i in range(start, N) if i not in kf and cap["speed"][i] <= sel.get("max_speed_mps", 1.5)
                     and os.path.exists(os.path.join(src, "depth", _dname(cap, i)))], dtype=int)

    # 안 쓴 프레임: 촬영 시간을 n 구간으로 나눠 구간마다 목표 시각에 가장 가까운, 키프레임에서 min_gap_s 넘게 떨어진 후보
    # (그런 장이 없는 구간만 기준을 낮춤) — 키프레임 설정이 바뀌어도 같은 구간 · 거의 같은 시각을 잼
    notes = [fix_st["note"]] if fix_st.get("note") else []
    gs, gf = _time_gap(t, kfs, cand) if len(cand) else (np.zeros(0), np.zeros(0, int))
    eps = 1e-3                                                  # 1 ms — 타임스탬프 흔들림 (60 장/초 → 한 장 16.7 ms)
    pool = cand[gs >= min_gap_s - eps]
    picks, target_off, n_relaxed = _grid_picks(t, cand, gs, float(min_gap_s), t[min(start, N - 1)], t[-1], n, seed, eps)
    pick_gap = _time_gap(t, kfs, picks)[0] if picks else np.zeros(0)
    used_gap = float(min(min_gap_s, pick_gap.min())) if len(pick_gap) else float(min_gap_s)
    if n_relaxed:
        notes.append(f"{n}구간 중 {n_relaxed}구간은 키프레임에서 {min_gap_s:g} 초 넘게 떨어진 안 쓴 프레임이 없어 그 구간에서 가장 멀리 떨어진 장을 씀 "
                     f"(가장 가까운 것 {used_gap:.3f} 초) — 키프레임이 촘촘한 촬영")

    raw_p, vis_p = os.path.join(bd, "mesh_raw.ply"), os.path.join(bd, "visual.ply")
    meshes = []                                                 # (이름, 파일, 입력 → 메쉬 좌표)
    if os.path.exists(raw_p):
        meshes.append(("mesh_raw", raw_p, np.eye(4)))
    if os.path.exists(vis_p):
        tp = os.path.join(bd, "transform.json")
        Ts = np.array(json.load(open(tp, encoding="utf-8"))["T_input_to_scene"]) if os.path.exists(tp) else np.eye(4)
        meshes.append(("visual", vis_p, Ts))
    if not meshes:
        raise SystemExit(f"맵 메쉬가 없음 ({raw_p} · {vis_p})")

    R, NR, ZM, FR = [], [], [], []
    for i in picks:
        z, ok = _depth(src, i, lim, min_conf, rule, cap)
        if z is None or not ok.any():
            continue
        r, nr, zm = _rays(cap, i, z, ok)
        R.append(r), NR.append(nr), ZM.append(zm), FR.append((i, len(zm), float(ok.mean())))

    pk = np.array([f[0] for f in FR], dtype=int)
    pg_s, pg_f = _time_gap(t, kfs, pk) if len(pk) else (np.zeros(0), np.zeros(0, int))
    P, PK = cap["T"][:, :3, 3], cap["T"][kfs, :3, 3]
    near_cm = [100 * float(np.linalg.norm(PK - P[i], axis=1).min()) for i in pk]   # 공간으로 가장 가까운 키프레임 (카메라 위치)
    res = dict(frames=len(FR), frames_picked=len(picks), frames_candidates=int(len(pool)), frames_candidates_all=int(len(cand)),
               keyframes=len(kf), frames_total=N, depth_limit_m=lim, depth_trunc_m=trunc, max_depth_m=sel.get("max_depth_m"),
               min_conf=min_conf, min_conf_build=meta.get("min_confidence"), depth_rule=rule["name"] if rule else f"min_conf {min_conf}",
               intrinsics=k_source, intrinsics_build="camera_matrix",   # 맵(stray.py)은 camera_matrix.csv 한 값으로 만듦
               seed=seed, source=os.path.basename(os.path.normpath(src)),
               held_out=dict(method="time_grid", min_gap_s=float(min_gap_s), min_gap_s_used=round(used_gap, 3), relaxed=bool(n_relaxed),
                             relaxed_cells=int(n_relaxed),
                             candidates_gap_s=_dist(gs),                  # 모든 안 쓴 후보 (기준 전)
                             gap_s=_dist(pg_s), gap_frames=_dist(pg_f),   # 실제로 잰 장
                             target_offset_s=_dist(target_off),           # 고른 장 ↔ 목표 시각 (고른 장 모두 — 깊이가 없어 빠진 장 포함)
                             nearest_keyframe_cm=_dist(near_cm)),
               pose_fix=fix_st,
               measured_valid_pct=round(100 * float(np.mean([f[2] for f in FR])), 1) if FR else 0.0, meshes={}, notes=notes)
    res["thresholds"] = dict(pass_median_cm=PASS_CM, fail_median_cm=FAIL_CM, min_hit_pct=MIN_HIT_PCT)

    if not FR:                                                  # 잴 깊이가 없음 → 실패로 돌려줌 (멈추지 않음)
        res.update({k: None for k in _MAIN_KEYS}, mesh=None, verdict="fail")
        what = f"깊이 규칙 {rule['name']} 로 고른" if rule else f"신뢰도 {min_conf} 이상인"
        res["reasons"] = ([f"쓸 수 있는 깊이가 있는 안 쓴 프레임이 없음 — 고른 {len(picks)}장 모두 깊이 파일이 없거나 "
                           f"{what} {lim:g} m 이하 화소가 없음 (잴 수 없음)"] if picks else
                          ["비교할 안 쓴 프레임이 없음 — 키프레임 말고 쓸 수 있는 프레임(처음 1 초 · 너무 빠른 장 · 깊이 없는 장 제외)이 없음 (잴 수 없음)"])
        res["summary"] = "복원 정확도: 잴 수 없음 — " + res["reasons"][0]
        res["seconds"] = round(time.time() - t0, 1)
        if write:
            _write(mp, res)
        return res

    R, NR, ZM = np.concatenate(R), np.concatenate(NR), np.concatenate(ZM)
    cut = np.cumsum([f[1] for f in FR])[:-1]
    gap_of = dict(zip(pk.tolist(), pg_s.tolist()))
    for name, path, Tm in meshes:
        m = o3d.io.read_triangle_mesh(path)
        if not len(m.triangles):
            continue
        Rm = Tm[:3, :3]
        rays = np.c_[R[:, :3] @ Rm.T + Tm[:3, 3], R[:, 3:] @ Rm.T]   # 입력 → 메쉬 좌표 (강체)
        zr = _cast(_scene(m), rays, NR)
        st = _stats(ZM, zr)
        st["file"] = os.path.relpath(path, out_dir)
        st["per_frame"] = [dict(frame=i, t_s=round(float(t[i] - t[0]), 2), gap_s=round(gap_of[i], 3), rays=k,
                                **{x: s.get(x) for x in ("hit_pct", "median_cm", "signed_median_cm")})
                           for (i, k, _), zm_, zr_ in zip(FR, np.split(ZM, cut), np.split(zr, cut)) for s in [_stats(zm_, zr_)]]
        res["meshes"][name] = st
    main_ = res["meshes"].get("mesh_raw") or res["meshes"].get("visual") or {}
    res["mesh"] = "mesh_raw" if "mesh_raw" in res["meshes"] else "visual"
    res.update({k: main_.get(k) for k in _MAIN_KEYS})
    res["verdict"], res["reasons"] = verdict(main_)
    res["summary"] = (f"복원 정확도: 안 쓴 프레임 {len(FR)}장의 깊이와 맵 차이 중앙 {res['median_cm']} cm · 2 cm 안 {res['within_2cm_pct']}% · "
                      f"맵에 닿음 {res['hit_pct']}%" if res.get("median_cm") is not None else "복원 정확도: 안 쓴 프레임의 깊이가 맵에 닿지 않음")
    res["seconds"] = round(time.time() - t0, 1)
    if write:
        _write(mp, res)
    return res


# ── 되돌아온 촬영의 틀어짐 (따로 부를 때만) ─────────────────────────────

def drift(out_dir, capture=None, window_s=30.0, max_gap_m=1.0, max_turn_deg=30.0, voxel=0.02, min_conf=None, write=True, k_source="odometry"):
    """처음 구간의 깊이로 만든 TSDF 에 마지막 구간의 깊이를 맞춰 쌓인 틀어짐을 잼. 출발 자리로 돌아온 촬영만. → dict (applicable 이 False 면 reason)
    min_conf=None → 맵을 만들 때와 같은 신뢰도 기준"""
    import open3d as o3d
    t0 = time.time()
    out_dir = os.path.abspath(os.path.expanduser(out_dir))
    mp, bd = os.path.join(out_dir, "input_map"), os.path.join(out_dir, "build")
    meta = _meta(mp)
    sel = meta.get("select") or {}
    src = _capture(out_dir, capture, meta.get("source"))
    cap = _load(src, meta, k_source)
    _keyframe_rows(mp, cap["t"], src, meta.get("frames_total"))  # 같은 촬영인지 확인만
    fix, fix_st = _pose_fix(mp, bd)
    if fix is not None:                                         # 맵을 고친 자세로 만들었음 → 모든 프레임 자세에 같은 보정 (measure 와 같음)
        cap["T"] = stray_fix.apply_fix(cap["t"], cap["T"], fix)
    t, T, N = cap["t"], cap["T"], len(cap["t"])
    rule = _rule(meta, min_conf)
    _, trunc, min_conf = _limits(meta, bd, min_conf, rule)
    a = int(np.searchsorted(t, t[0] + sel.get("skip_s", 1.0)))
    b = N - 1
    gap = float(np.linalg.norm(T[b, :3, 3] - T[a, :3, 3]))
    f0, f1 = T[a, :2, 2], T[b, :2, 2]                           # 바라보는 방향(광축)의 수평 성분
    turn = float(np.degrees(np.arccos(np.clip(f0 @ f1 / max(np.linalg.norm(f0) * np.linalg.norm(f1), 1e-9), -1, 1))))
    res = dict(start_end_gap_m=round(gap, 2), start_end_turn_deg=round(turn, 1), depth_limit_m=trunc, min_conf=min_conf,
               depth_rule=rule["name"] if rule else f"min_conf {min_conf}", intrinsics=k_source,
               pose_fix=fix_st)
    if gap > max_gap_m or turn > max_turn_deg:
        res.update(applicable=False, reason=f"출발 자리로 돌아오지 않은 촬영 (처음과 끝 거리 {gap:.2f} m · 방향 차 {turn:.0f}° — 기준 {max_gap_m:g} m · {max_turn_deg:g}° 안)")
    else:
        W = min(window_s, (t[b] - t[a]) / 3)
        ok_speed = cap["speed"] <= sel.get("max_speed_mps", 1.5)
        first = [i for i in range(a, N) if t[i] <= t[a] + W and ok_speed[i]]
        last = [i for i in range(a, N) if t[i] >= t[b] - W and ok_speed[i]]
        first = first[::max(1, len(first) // 300)]               # 최대 약 300장
        vol = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=voxel, sdf_trunc=4 * voxel, color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)
        used = 0
        for i in first:
            z, ok = _depth(src, i, trunc, min_conf, rule, cap)
            if z is None:
                continue
            h, w = z.shape
            fx, fy, cx, cy = _intr(cap, i, w)
            zz = np.where(ok, z * 1000, 0).astype(np.uint16)
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.zeros((h, w, 3), np.uint8)), o3d.geometry.Image(zz),
                                                                      depth_scale=1000.0, depth_trunc=trunc, convert_rgb_to_intensity=False)
            vol.integrate(rgbd, o3d.camera.PinholeCameraIntrinsic(w, h, fx, fy, cx, cy), np.linalg.inv(T[i]))
            used += 1
        mesh = vol.extract_triangle_mesh()
        mesh.compute_vertex_normals()
        lp = last[::max(1, len(last) // 30)]
        R_, NR, ZM, P = [], [], [], []
        for i in lp:
            z, ok = _depth(src, i, trunc, min_conf, rule, cap)
            if z is None or not ok.any():
                continue
            r, nr, zm = _rays(cap, i, z, ok)
            R_.append(r), NR.append(nr), ZM.append(zm), P.append(r[:, :3] + r[:, 3:] * (zm * nr)[:, None])
        if not len(mesh.triangles) or not R_:
            res.update(applicable=False, reason="처음 구간 메쉬 또는 마지막 구간 깊이가 비었음")
        else:
            R_, NR, ZM, P = np.concatenate(R_), np.concatenate(NR), np.concatenate(ZM), np.concatenate(P)
            st = _stats(ZM, _cast(_scene(mesh), R_, NR))
            tgt = o3d.geometry.PointCloud(mesh.vertices)
            tgt.normals = mesh.vertex_normals
            srcp = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P)).voxel_down_sample(voxel)
            reg = o3d.pipelines.registration.registration_icp(srcp, tgt, 0.15, np.eye(4), o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                                                              o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
            M = np.asarray(reg.transformation)
            Ps = np.asarray(srcp.points)
            disp = np.linalg.norm(Ps @ M[:3, :3].T + M[:3, 3] - Ps, axis=1)   # 점마다 실제로 옮겨진 거리 ‖M p − p‖
            dmed, d90 = float(np.median(disp)), float(np.percentile(disp, 90))
            rot = float(np.degrees(np.arccos(np.clip((np.trace(M[:3, :3]) - 1) / 2, -1, 1))))
            res.update(applicable=True, window_s=round(float(W), 1), frames_first=used, frames_last=len(lp), depth_vs_first=st,
                       icp=dict(displacement_median_cm=round(100 * dmed, 1), displacement_p90_cm=round(100 * d90, 1), points=int(len(Ps)),
                                rotation_deg=round(rot, 2), fitness=round(float(reg.fitness), 3), rmse_cm=round(100 * float(reg.inlier_rmse), 2),
                                transform_translation_cm=round(100 * float(np.linalg.norm(M[:3, 3])), 1),       # 세계 원점 기준 — 회전이 있으면 점의 움직임과 다름
                                transform_translation_xyz_cm=np.round(100 * M[:3, 3], 1).tolist()))
            reliable = reg.fitness >= 0.3
            res["summary"] = (f"되돌아온 자리 틀어짐: 처음 {W:.0f} 초로 만든 맵에 마지막 {W:.0f} 초 깊이를 맞추면 점들이 중앙 {100 * dmed:.1f} cm "
                              f"(90% {100 * d90:.1f} cm) · {rot:.1f}° 옮겨짐 "
                              f"(깊이 차이 중앙 {st.get('median_cm')} cm" + ("" if reliable else f", 겹친 비율 {reg.fitness:.0%} 로 낮아 믿기 어려움") + ")")
    res["seconds"] = round(time.time() - t0, 1)
    if write:
        _write(mp, res, "drift")
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("out", help="scan2sim 결과 폴더 (input_map/ · build/ 가 있는 곳)")
    ap.add_argument("--n", type=int, default=30, help="비교할 안 쓴 프레임 수")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--min-conf", type=int, default=None, choices=[0, 1, 2], help="깊이 신뢰도 기준 (주지 않으면 맵을 만들 때와 같게)")
    ap.add_argument("--min-gap-s", type=float, default=MIN_GAP_S, help="안 쓴 프레임이 가장 가까운 키프레임에서 떨어져야 할 시간 (초)")
    ap.add_argument("--k-source", default="odometry", choices=K_SOURCES, help="내부 파라미터: odometry(프레임별, 기본) · camera_matrix(맵과 같은 한 값)")
    ap.add_argument("--capture", help="촬영 원본 (input_map/_unzip 이 없을 때 — 폴더로 받은 경우)")
    ap.add_argument("--json", help="결과를 이 경로에도 씀")
    ap.add_argument("--drift", action="store_true", help="되돌아온 촬영의 틀어짐도 잼 (출발 자리로 돌아온 촬영만)")
    a = ap.parse_args()
    r = measure(a.out, a.n, a.seed, a.capture, a.min_conf, min_gap_s=a.min_gap_s, k_source=a.k_source)
    ho = r["held_out"]
    print(f"[품질] {r['summary']} | 90% {r['p90_cm']} cm · 5 cm 안 {r['within_5cm_pct']}% · 치우침 {r['signed_median_cm']} cm | 판정 {r['verdict']} | {r['seconds']} s")
    if ho.get("gap_s"):
        print(f"       안 쓴 프레임 ↔ 가장 가까운 키프레임: 중앙 {ho['gap_s']['median']} 초 (최소 {ho['gap_s']['min']} · 최대 {ho['gap_s']['max']}) · "
              f"위치 중앙 {ho['nearest_keyframe_cm']['median']} cm | 후보 {r['frames_candidates']}/{r['frames_candidates_all']}장 | "
              f"목표 시각과 차이 중앙 {(ho.get('target_offset_s') or {}).get('median')} 초")
    if (r.get("pose_fix") or {}).get("applied"):
        print(f"       안 쓴 프레임 자세에 맵과 같은 자세 보정을 적용함 ({_fix_text(r['pose_fix'])})")
    for k, s in r["meshes"].items():
        if k != r["mesh"]:
            print(f"       {k}: 중앙 {s.get('median_cm')} cm · 2 cm 안 {s.get('within_2cm_pct')}% · 맵에 닿음 {s.get('hit_pct')}%")
    for m in r["reasons"]:
        print("       - " + m)
    for m in r["notes"]:
        print("       · " + m)
    if a.drift:
        d = drift(a.out, a.capture, min_conf=a.min_conf, k_source=a.k_source)
        r["drift"] = d
        print("[품질] " + (d["summary"] if d.get("applicable") else "틀어짐: 재지 않음 — " + d["reason"]) + f" | {d['seconds']} s")
    if a.json:
        json.dump(r, open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
