"""되돌아온 자리 어긋남 — 같은 곳을 다른 때 다시 본 깊이끼리 맞춰, 그 사이에 쌓인 자세 틀어짐(수평 · 높이 · 방향)을 직접 잼. B 의 단계 (C4).

    cd recon
    python -m scan2sim.stray_revisit <촬영 폴더 · zip · scan2sim 결과 폴더> [--capture <촬영 폴더 또는 zip>] [--fix-json <pose_fix.json>]
                                     [--json <경로>] [--pairs] [--no-write]

왜
    아이폰 자세(VIO)는 되돌아온 자리를 맞추지 않아 틀어짐이 쌓인다. 복도를 갔다 되돌아오면 같은 바닥 · 벽이 두 자리에 찍혀 맵을 못 쓴다
    (922acd4755: 걸어간 길 36% 가 장애물). 안 쓴 프레임 정확도(stray_quality.measure)는 키프레임 0.1~0.3 초 옆 장과 비교하므로 이것을
    못 본다 (그 촬영본 중앙 6.13 cm — 주의에 그침). stray_quality.drift 는 처음 · 끝 30 초가 1 m · 30° 안에서 겹칠 때 한 쌍만 잰다.
    자세끼리의 거리로는 틀어짐을 모름 (틀어짐이 자세 안에 있음) — 그래서 두 방문의 깊이(벽 · 바닥)를 맞춰 봄

무엇을 재나 (measure)
    1. 프레임: 0.1 초마다 (처음 1 초 · 1.5 m/s 넘는 장 뺌). 깊이는 신뢰도 2 · 4 m 안만 (틀어짐을 재려는 것 — 맵의 깊이 규칙과 다름)
    2. 점: stray_fix.frame_points (256×192 에서 2 화소마다, 법선 ±2 칸) → 벽 (수직 면 |법선 z| < 0.15) · 바닥 (위를 향한 면 법선 z > 0.9 ·
       카메라보다 0.6~2.2 m 아래). 장마다 5 cm 칸으로 솎음. 카메라 좌표로 두고 자세 묶음(원래 · 고친)마다 세계로 옮김
    3. 다시 본 곳: 촬영을 경로 3 m 마다 구간 B 로 나눔. B 보다 20 초 넘게 앞선 프레임 중 (그 장이 본 벽 칸(1 m)의 30% 이상을 B 도 봄) 또는
       (카메라가 B 의 카메라에서 수평 1.5 m 안) → 시간으로 이어진 묶음(2 초 넘게 끊기면 나눔) 중 가장 이른 것 · 겹친 칸이 가장 많은 것 = 앞 방문 A
       (가장 이른 방문이 쌓인 틀어짐이 가장 큼). A · B 모두 5장 이상일 때만. 카메라 거리만으로 찾으면 방에서 반대쪽을 본 장끼리 짝지어졌고,
       벽 칸만으로 찾으면 방 촬영본(6bc1ae021f)의 처음 · 끝 쌍을 놓쳤음. A 의 점은 그 묶음의 처음 ~ 끝 장 사이 모든 장 (묶음 안 2 초 미만으로
       빠진 장 포함 — 같은 방문) — 쌍의 frames 는 실제로 쓴 장 수
    4. B 를 A 에 맞춤 (B 의 점만 옮김, 반복 없는 전수 탐색 — ICP 처럼 처음 값에 기대지 않아 1 m 넘게 어긋나도 잡음)
       방향 dψ: 벽 법선 방위 분포(넓이 가중, 0.5° 칸)를 원형 상관 · ±5° → B 를 그 벽 가운데 둘레로 돌림
       수평: 벽 점을 법선 방위 8 갈래로 나눠 5 cm 격자 그림 → 갈래마다 FFT 상관을 더한 그림 (이동 반지름 1.5 m 안)
         B 벽 법선 2×2 행렬(넓이 가중)의 고윳값 비 ≥ 0.1 이고 그림의 봉우리가 뚜렷하면 (둘째 봉우리 < 0.8) 그 2차원 이동 (h_axis "2d").
         아니면 (긴 복도 — 길 방향으로 능선이 되어 봉우리가 없음) 센 방향(벽에 수직 = 옆)으로 눌러 담은 단면의 봉우리 (h_axis "1d" —
         dx · dy 도 그 성분만). 비 0.2 안팎의 복도 구간(문 · 기둥)은 2차원 봉우리가 여럿이라 (둘째 0.87~0.98) 길 방향 값이 틀렸음.
         2차원 봉우리가 뚜렷한데 탐색 끝에 닿았으면 (1.5 m 넘게 어긋났을 수 있음) 1d 로 넘기지 않고 믿지 않음
       높이 dz: 옮긴 B 바닥 점과 A 바닥 점을 25 cm 칸으로 — 둘 다 3점 넘는 칸의 높이 차 중앙 (칸 5개 이상)
       맞은 비율 fit: 상관이 가장 큰 이동에서 B 벽 칸 중 A 벽 칸(같은 갈래, 이웃 칸 포함)과 겹친 비율
    5. 결과는 뒤 − 앞 (뒤 방문이 앞 방문보다 얼마나 옮겨져 찍혔나, 맵 세계 Z 위, cm · °). 수평은 B 벽 점 가운데(at_xy)에서의 값 —
       방향 어긋남이 있으면 자리마다 다르므로 (1 m 당 dψ × 1.7 cm/°) 카메라 자리(cam_b)의 값과 다를 수 있음.
       좌표 (dx · dy · at_xy · cam_b · floor_track) 는 맵 폴더 세계 (input_map — odometry.csv 를 Y_UP_TO_Z_UP 으로 돌린 것, mesh_raw.ply 와 같음)
       — 씬 좌표가 아님 (씬은 build/transform.json 의 T_input_to_scene 을 더 곱함)
       믿을 쌍 = fit ≥ 0.3 · 둘째 봉우리 < 0.8 · 탐색 끝(수평 1.5 m · 방향 ±5°)이 아님. 아니면 unreliable_why 에 이유
    결과 폴더면 맵을 만든 보정(stray_quality._pose_fix — build/pose_fix.json 의 sha256 이 맞을 때만)을 읽어 원래 자세(raw)와 고친 자세(fixed)를
    둘 다 잼 (판정은 fixed). 결과 pose_fix = 그 요약 + sha256_expected (build/tsdf_meta.json 이 적은 값) · sha256_used (맞아서 쓴 값, 못 쓰면
    None) — 못 쓴 이유(note)는 reasons 에도. 촬영 원본은 input_map/_unzip 에서 찾고, 없으면 (서버가 지운 뒤 · 폴더로 받은 입력) --capture 로 줌.
    --fix-json 으로 보정 info 를 직접 줄 수도 있음 (적용할 수 없는 꼴이면 문구로 멈춤). 결과 폴더면 input_map/stray_revisit.json 에 씀
    (임시 이름으로 쓰고 바꿔 놓음, --no-write 로 끔). 촬영 자료가 잘못됐으면 stray.validate 의 문구로 멈춤 (CaptureError)
    덧붙임 (판정에 안 씀): floor_track = 프레임마다 카메라 아래 2.5 m 안 바닥 높이의 10 초 중앙 (한 층 바닥이면 높이 틀어짐이 그대로 보임) ·
    jumps = 자세가 한 장 사이에 10 cm · 5° 넘게 튄 곳 (처음 20곳 — 줄이 빠져 시간이 벌어진 곳은 벌어진 장 수로 나눠 봄, 모두 몇 곳은
    jumps_total) · *_rate_cm_per_10m = 10 m 넘게 떨어진 쌍의 어긋남 / 그 사이 걸음
    (틀어짐이 걸음에 비례하지 않아 참고만 — 방 촬영본은 끝의 튐 때문에 복도보다 크게 나옴)

판정 (verdict — 믿을 쌍 중 가장 큰 값)
    높이 |dz| 3 cm 이하 통과 · 15 cm 넘으면 실패 · 그 사이 주의. 3 cm = 진단 floor_two_levels 의 아래 끝, 15 cm = 걸어간 길 진단이 봐주는 폭
    수평 h 10 cm 이하 통과 · 30 cm 넘으면 실패 · 그 사이 주의 (30 cm = 로봇이 지나갈 폭이 눈에 띄게 줄어드는 정도 — 두 촬영본으로 잡은 잠정값)
    다시 본 곳이 없으면 skip (20 초 넘게 떨어져 같은 곳을 다시 본 구간이 없음 · 촬영이 짧음 · 벽 · 바닥 점이 있는 장이 거의 없음 — 이유를
    나눠 적음). 믿을 쌍이 없으면 warn (다시 지나갔는데 맞추지 못함).
    pass 여도 warn 으로 낮춤: 믿지 못한 쌍 중 믿을 쌍의 가장 먼 것보다 SEG_M(3 m) 넘게 더 떨어진 쌍이 있음 (가장 큰 틀어짐을 못 잼) ·
    바닥 높이를 견준 믿을 쌍이 없음 (높이 어긋남을 모름)

측정 (11/25 C4 — 원래 자세 → 기본 보정 xy → floor+xy, 믿을 쌍 중 최대)
    922acd4755 복도 왕복 172 m: 수평 123 → 125 → 125 cm · 높이 44 → 44 → 1.2 cm → 모두 fail. 쌍 31 중 믿을 쌍 24.
      어긋남은 다시 본 사이 걸음과 함께 고르게 커짐 (튄 곳 없음): 17 m 5 cm · 1.7 cm / 50 m 13 · 13 / 100 m 62 · 23 / 165 m 85~121 · 43.
      xy 보정은 옆(u) 축이 47 cm > 40 cm 관문에 걸려 길 방향(v)만 고침 — 관문을 1.5 m 로 풀어도 수평 91 cm (벽 칸이 30 cm 넘게 벌어진
      두 방문을 잇지 못함). 바닥 높이 흐름(floor_track)과 복도 벽 자리로 따로 잰 값이 같음 (끝 −41 cm · 옆 −0.6~−0.9 m).
      엔진 (같은 맵 폴더, stray_mesh 의 fix 만 바꿈): 막힌 길 끔 32.7% · xy 35.8% · floor+xy 0.0% (판정 쓸 수 없음 → 주의,
      안 쓴 프레임 중앙 7.12 · 6.13 · 3.49 cm) — 길을 막은 것은 높이 어긋남 (두 방문의 바닥이 40 cm 차이). 옆 어긋남은 남음 (벽 두 겹)
    6bc1ae021f 방 한 바퀴 44 m: 수평 24 → 10 cm · 높이 11 → 11 cm → warn. 처음 · 끝 쌍(36 m)은 수평 15 → 2.5 cm · 높이 −7.4 → −7.6 cm,
      가장 큰 높이 차는 끝의 자세 튐(114.6 초 19 cm) 뒤 구간
    알려진 어긋남을 넣어 봄 (뒤 방문 자세에 옮김 · 회전): 922 에 (−40, 25, −15) cm → 쌍 20개 옆 성분 차 ±1.4 cm · 높이 ±0.7 cm · 2d 쌍 3개
      ±0.2 cm. 6bc 에 (30, −20, 10) cm · 2° → 옆 30.2 (넣은 30.9) · 높이 9.9 (10) cm · 2.0°.
      가상의 방 두 바퀴 (samples/make_stray.py 50 초): 그대로 수평 0.6 · 높이 0.1 cm → pass. 둘째 바퀴에 끝 (40, −30, 25) cm · 4° 로
      커지는 틀어짐 → 벽 가운데 기준 수평 차 0.2~4 cm · 높이 ±0.4 cm · 방향 ±0.16° 로 되찾고 fail — 그때 엔진은 막힌 길 1.4% · 주의뿐
      (TSDF 가 겹친 바닥 일부를 지움). (50, −40, 45) cm · 4° 면 엔진도 막힌 길 58.5% · 쓸 수 없음.
      걸린 시간 2~17 초 (922 · 1772장, 메모리 0.9 GB)

한계
    - 수평은 벽으로만 잼 — 벽이 한 방향뿐인 긴 복도는 길 방향 어긋남을 모름 (h_axis "1d"). 높이는 평평한 바닥 기준 (계단 · 경사로에서
      칸이 어긋나면 틀림)
    - 앞 방문과 뒤 방문 사이의 상대 틀어짐만 잼 — 한 번만 지나간 곳의 틀어짐(한 방향 촬영의 휨 · 기울어짐)은 모름
    - 사람처럼 움직인 물체 · 반대쪽에서 본 얇은 물체는 맞은 비율을 낮춤. 탐색 반지름 1.5 m · ±5° 를 넘는 어긋남은 못 잼 (믿지 않음으로 나옴)
"""
import argparse
import json
import os
import time

import numpy as np

from . import stray, stray_fix

EVERY_S, SKIP_S, MAX_SPEED = 0.1, 1.0, 1.5                       # 프레임 간격 · 처음 건너뛸 초 · 이보다 빠른 장 뺌 (stray.py 와 같은 값)
MAX_DEPTH, MIN_CONF = 4.0, 2                                      # 깊이: 신뢰도 2 · 4 m 안
WALL_NZ, FLOOR_NZ, FLOOR_BAND = 0.15, 0.9, (0.6, 2.2)            # 벽 · 바닥 법선, 바닥 = 카메라보다 이만큼 아래 (stray.depth_rule 의 띠)
THIN_M = 0.05                                                     # 장마다 점 솎는 칸
SEG_M, MIN_GAP_S, MIN_FRAMES, RADIUS_M = 3.0, 20.0, 5, 1.5        # 구간 길이 · 다시 본 것으로 칠 시간 차 · 구간 최소 장 수 · 카메라 거리
FOOT_M, FOOT_SHARE = 1.0, 0.3                                     # 앞 방문 찾기: 본 벽 칸 크기 · 그 장이 본 칸 중 B 가 본 칸의 몫
GRID_M, SEARCH_M, YAW_DEG, YAW_BIN = 0.05, 1.5, 5.0, 0.5          # 벽 격자 · 수평 탐색 · 방향 탐색 · 방위 칸
FLOOR_CELL, MIN_FLOOR_CELLS = 0.25, 5
MIN_FIT, MAX_PEAK2, WEAK_AXIS = 0.3, 0.8, 0.1                     # 믿을 쌍 · 잴 수 있는 방향
PASS_Z_CM, FAIL_Z_CM, PASS_H_CM, FAIL_H_CM = 3.0, 15.0, 10.0, 30.0
JUMP_M, JUMP_DEG = 0.10, 5.0


# ── 읽기 ───────────────────────────────────────────────────────────────

def _load(src):
    """odometry.csv → 시각 · 맵 세계(Z 위) 자세 · 속력 · 깊이 파일 번호 · 깊이 크기 내부 파라미터 (stray.py 의 Kd 와 같음)"""
    import cv2
    od = np.atleast_2d(np.genfromtxt(os.path.join(src, "odometry.csv"), delimiter=",", skip_header=1, usecols=range(13)))
    t = od[:, 0]
    T = np.repeat(np.eye(4)[None], len(od), axis=0)
    T[:, :3, :3] = [stray._rot(q) for q in od[:, 5:9]]
    T[:, :3, 3] = od[:, 2:5]
    T = stray.Y_UP_TO_Z_UP @ T
    step = np.r_[0, np.linalg.norm(np.diff(od[:, 2:5], axis=0), axis=1)]
    speed = np.convolve(step / np.r_[1, np.maximum(np.diff(t), 1e-3)], np.ones(5) / 5, mode="same")
    fid, _ = stray.frame_ids(src, od[:, 1])
    z0 = cv2.imread(os.path.join(src, "depth", f"{int(fid[0]):06d}.png"), cv2.IMREAD_UNCHANGED)
    if z0 is None:
        raise stray.CaptureError(f"depth/{int(fid[0]):06d}.png 를 읽지 못함")
    Kc = np.loadtxt(os.path.join(src, "camera_matrix.csv"), delimiter=",")
    cap = cv2.VideoCapture(os.path.join(src, "rgb.mp4"))
    W0, H0 = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1920, int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 1440
    cap.release()
    sx, sy = z0.shape[1] / W0, z0.shape[0] / H0
    K = (Kc[0, 0] * sx, Kc[1, 1] * sy, (Kc[0, 2] + 0.5) * sx - 0.5, (Kc[1, 2] + 0.5) * sy - 0.5)
    return dict(t=t, T=T, speed=speed, step=step, fid=fid, K=K)


def _frames(cap, every_s=EVERY_S):
    t = cap["t"]
    k = max(1, int(round(every_s / max(float(np.median(np.diff(t))), 1e-3))))
    a = int(np.searchsorted(t, t[0] + SKIP_S))
    return np.array([i for i in range(a, len(t), k) if cap["speed"][i] <= MAX_SPEED], dtype=int)


def _points(src, cap, idx):
    """프레임마다 카메라 좌표의 벽 · 바닥 점 → 목록 [(줄 번호, 점 (n,3), 법선 (n,3), 넓이, 바닥이면 True)]. 바닥 · 벽 가르기는 원래 자세의 방향으로"""
    import cv2
    out = []
    has_conf = os.path.isdir(os.path.join(src, "confidence"))
    for i in idx:
        name = f"{int(cap['fid'][i]):06d}.png"
        z = cv2.imread(os.path.join(src, "depth", name), cv2.IMREAD_UNCHANGED)
        if z is None or z.ndim != 2:
            continue
        c = cv2.imread(os.path.join(src, "confidence", name), cv2.IMREAD_UNCHANGED) if has_conf else None
        ok = (z > 0) & (z <= 1000 * (MAX_DEPTH if c is not None else 3.0))
        if c is not None:
            if c.shape != z.shape:
                continue
            ok &= c >= MIN_CONF
        zz = np.where(ok, z, 0).astype(np.float32) / 1000.0
        Pc, nc, a = stray_fix.frame_points(zz, cap["K"], np.eye(4), max(1, int(round(z.shape[1] / 128))), MAX_DEPTH, nb=2)
        R, tr = cap["T"][i][:3, :3], cap["T"][i][:3, 3]
        nz = nc @ R[2]
        below = tr[2] - (Pc @ R[2] + tr[2])
        fl = (nz > FLOOR_NZ) & (below > FLOOR_BAND[0]) & (below < FLOOR_BAND[1])
        keep = fl | (np.abs(nz) < WALL_NZ)
        if keep.sum() < 50:
            continue
        q = np.floor(Pc[keep] / THIN_M).astype(np.int64)
        _, first = np.unique((q[:, 0] * 100003 + q[:, 1]) * 100003 + q[:, 2], return_index=True)
        k = np.nonzero(keep)[0][np.sort(first)]
        out.append((int(i), Pc[k].astype(np.float32), nc[k].astype(np.float32), a[k].astype(np.float32), fl[k]))
    return out


def _world(fr, T):
    """카메라 좌표 점 목록 → 세계 (자세 T). → (줄 번호, 점, 법선, 넓이, 바닥) 이어 붙인 배열"""
    F, P, N, A, L = [], [], [], [], []
    for i, Pc, nc, a, fl in fr:
        R, tr = T[i][:3, :3], T[i][:3, 3]
        F.append(np.full(len(Pc), i)), P.append(Pc @ R.T + tr), N.append(nc @ R.T), A.append(a), L.append(fl)
    return np.concatenate(F), np.concatenate(P), np.concatenate(N), np.concatenate(A), np.concatenate(L)


# ── 맞추기 ─────────────────────────────────────────────────────────────

def _azimuth_hist(n, a):
    nb = int(round(360 / YAW_BIN))
    h = np.bincount((np.floor((np.degrees(np.arctan2(n[:, 1], n[:, 0])) % 360) / YAW_BIN).astype(int)) % nb, a, nb)
    k = np.exp(-0.5 * (np.arange(-6, 7) * YAW_BIN / 1.0) ** 2)    # 1° 로 번지게 (원형)
    return sum(w * np.roll(h, j) for j, w in zip(range(-6, 7), k))


def _yaw(nA, aA, nB, aB):
    """B 를 A 에 맞추는 z 축 회전 (°) — 벽 법선 방위 분포의 원형 상관, ±YAW_DEG"""
    hA, hB = _azimuth_hist(nA, aA), _azimuth_hist(nB, aB)
    js = np.arange(-int(YAW_DEG / YAW_BIN), int(YAW_DEG / YAW_BIN) + 1)
    sc = np.array([hA @ np.roll(hB, j) for j in js])
    k = int(np.argmax(sc))
    off = 0.0
    if 0 < k < len(sc) - 1:
        d = sc[k - 1] - 2 * sc[k] + sc[k + 1]
        off = 0.5 * (sc[k - 1] - sc[k + 1]) / d if d < 0 else 0.0
    return float((js[k] + off) * YAW_BIN)


def _bins(n):
    return (np.floor(((np.degrees(np.arctan2(n[:, 1], n[:, 0])) + 22.5) % 360) / 45).astype(int)) % 8


def _profile(win, e, g, Rc):
    """상관 그림 win (이동 ±Rc 칸) 을 방향 e 로 눌러 담은 것 (같은 e 성분마다 최댓값) → (그 방향 이동 m, 둘째 봉우리 비, 끝에 닿음)"""
    yy, xx = np.mgrid[-Rc:Rc + 1, -Rc:Rc + 1]
    disc = xx ** 2 + yy ** 2 <= Rc * Rc
    b = np.rint(xx * e[0] + yy * e[1]).astype(int)[disc] + Rc
    pr = np.full(2 * Rc + 1, -np.inf)
    np.maximum.at(pr, b, win[disc])
    k = int(np.argmax(pr))
    off = 0.0
    if 0 < k < 2 * Rc and np.all(np.isfinite(pr[k - 1:k + 2])):
        d = pr[k - 1] - 2 * pr[k] + pr[k + 1]
        off = 0.5 * (pr[k - 1] - pr[k + 1]) / d if d < 0 else 0.0
    far = np.abs(np.arange(2 * Rc + 1) - k) > 0.3 / g
    p2 = float(pr[far & np.isfinite(pr)].max() / pr[k]) if pr[k] > 0 else 1.0
    return (k - Rc + off) * g, p2, bool(k <= 1 or k >= 2 * Rc - 1)


def _shift(PA, nA, PB, nB, e_strong):
    """B 벽 점을 A 에 맞추는 수평 이동 — 법선 8 갈래 격자 그림(5 cm)의 FFT 상관 (반지름 SEARCH_M 안).
    → dict: d2 · peak2_2d · edge_2d (2차원 봉우리 m · 둘째 봉우리 비 · 끝에 닿음), strong · peak2 · edge (센 방향 e_strong 으로 눌러 담은
    단면의 봉우리 — 긴 복도는 길 방향으로 상관이 능선이 되어 2차원 봉우리가 없음), fit (2차원 봉우리에서 맞은 비율)"""
    g, Rc = GRID_M, int(round(SEARCH_M / GRID_M))
    lo = np.minimum(PA[:, :2].min(0), PB[:, :2].min(0)) - (Rc + 2) * g
    hi = np.maximum(PA[:, :2].max(0), PB[:, :2].max(0)) + (Rc + 2) * g
    W, H = (np.ceil((hi - lo) / g).astype(int) + 1)
    iA, iB = np.floor((PA[:, :2] - lo) / g).astype(int), np.floor((PB[:, :2] - lo) / g).astype(int)
    bA, bB = _bins(nA), _bins(nB)
    imA, imB = np.zeros((8, H, W), np.float32), np.zeros((8, H, W), np.float32)
    imA[bA, iA[:, 1], iA[:, 0]] = 1
    imB[bB, iB[:, 1], iB[:, 0]] = 1
    F = (np.fft.rfft2(imA) * np.conj(np.fft.rfft2(imB))).sum(0)
    ky, kx = np.fft.fftfreq(H)[:, None], np.fft.rfftfreq(W)[None, :]
    F *= np.exp(-4 * (np.pi * 1.0) ** 2 * (kx ** 2 + ky ** 2))  # 두 그림 모두 1 칸 가우스로 번진 것과 같음
    c = np.fft.irfft2(F, s=(H, W))
    win = np.roll(np.roll(c, Rc, 0), Rc, 1)[:2 * Rc + 1, :2 * Rc + 1]   # win[Rc + dy, Rc + dx] = B 를 (dx, dy) 칸 옮겼을 때
    yy, xx = np.mgrid[-Rc:Rc + 1, -Rc:Rc + 1]
    win = np.where(xx ** 2 + yy ** 2 <= Rc * Rc, win, -np.inf)  # 탐색은 반지름 SEARCH_M 원 안
    ky0, kx0 = np.unravel_index(int(np.argmax(win)), win.shape)
    best = float(win[ky0, kx0])

    def sub(v):
        d = v[0] - 2 * v[1] + v[2]
        return 0.5 * (v[0] - v[2]) / d if np.all(np.isfinite(v)) and d < 0 else 0.0
    fy = sub(win[ky0 - 1:ky0 + 2, kx0]) if 0 < ky0 < 2 * Rc else 0.0
    fx = sub(win[ky0, kx0 - 1:kx0 + 2]) if 0 < kx0 < 2 * Rc else 0.0
    far = (yy - ky0 + Rc) ** 2 + (xx - kx0 + Rc) ** 2 > (0.3 / g) ** 2
    p2 = float(win[far].max() / best) if best > 0 else 1.0
    dy, dx = ky0 - Rc, kx0 - Rc
    ls, p2s, edge_s = _profile(win, e_strong, g, Rc)
    dil = imA.copy()                                            # 맞은 비율: 옮긴 B 칸이 A 칸(이웃 포함)과 겹침
    for oy in (-1, 0, 1):
        for ox in (-1, 0, 1):
            dil = np.maximum(dil, np.roll(np.roll(imA, oy, 1), ox, 2))
    bb, yb, xb = np.nonzero(imB)
    yb, xb = yb + dy, xb + dx
    ins = (yb >= 0) & (yb < H) & (xb >= 0) & (xb < W)
    fit = float(dil[bb[ins], yb[ins], xb[ins]].sum() / max(len(bb), 1))
    return dict(d2=np.array([dx + fx, dy + fy]) * g, peak2_2d=p2, edge_2d=bool(dx * dx + dy * dy >= (Rc - 1) ** 2),
                strong=ls, peak2=p2s, edge=edge_s, fit=fit)


def _cell_median(xy, z, cell):
    key = np.floor(xy / cell).astype(np.int64)
    key = key[:, 0] * 1000003 + key[:, 1]
    o = np.lexsort((z, key))
    key, z = key[o], z[o]
    st = np.r_[0, np.nonzero(np.diff(key))[0] + 1]
    cnt = np.diff(np.r_[st, len(key)])
    return key[st], z[st + cnt // 2], cnt


def register(A, B):
    """A · B = (점, 법선, 넓이, 바닥) 세계 좌표 (앞 · 뒤 방문). → dict: 뒤 − 앞 어긋남 dx · dy · dz (m) · dyaw (°) · 품질"""
    PA, nA, aA, fA = A
    PB, nB, aB, fB = B
    wA, wB = ~fA, ~fB
    lo, hi = PB[wB, :2].min(0) - SEARCH_M - 0.5, PB[wB, :2].max(0) + SEARCH_M + 0.5
    inA = np.all((PA[:, :2] >= lo) & (PA[:, :2] <= hi), axis=1)
    wA, fA = wA & inA, fA & inA
    if wA.sum() < 200 or wB.sum() < 200:
        return dict(ok=False, reason="벽 점이 적음")
    psi = _yaw(nA[wA], aA[wA], nB[wB], aB[wB])
    c = PB[wB, :2].mean(0)
    r = np.radians(psi)
    Rz = np.array([[np.cos(r), -np.sin(r)], [np.sin(r), np.cos(r)]])
    PBr = PB.copy()
    PBr[:, :2] = (PB[:, :2] - c) @ Rz.T + c
    nBr = nB.copy()
    nBr[:, :2] = nB[:, :2] @ Rz.T
    nb2 = nBr[wB, :2]                                           # 센 방향 = B 벽 법선 2×2 행렬의 큰 고유 방향 (복도면 벽에 수직 = 옆 방향)
    ev, evec = np.linalg.eigh((nb2 * aB[wB, None]).T @ nb2)
    ratio = float(ev[0] / max(ev[1], 1e-12))
    e_s = evec[:, 1]
    sh = _shift(PA[wA], nA[wA], PBr[wB], nBr[wB], e_s)
    # 2차원 봉우리가 뚜렷하면 그것 (방 · 복도 끝 · 로비). 아니면 (긴 복도 — 길 방향으로 능선) 센 방향(옆) 성분만.
    # 비 0.2 안팎의 복도 구간(문 · 기둥)은 2차원 봉우리가 여럿이라 (둘째 0.87~0.98) 길 방향 값이 틀렸음 (922acd4755).
    # 2차원 봉우리가 뚜렷한데 탐색 끝이면 1d 로 넘기지 않음 — 아래에서 믿지 않음 (1.5 m 넘는 어긋남을 옆 성분만으로 작게 보이지 않게)
    two = ratio >= WEAK_AXIS and sh["peak2_2d"] < MAX_PEAK2
    if two:
        d, peak2, edge = sh["d2"], sh["peak2_2d"], sh["edge_2d"]
        h = float(np.linalg.norm(d))
    else:
        d, peak2, edge = sh["strong"] * e_s, sh["peak2"], sh["edge"]
        h = abs(sh["strong"])
    fit = sh["fit"]
    res = dict(ok=True, dx=-float(d[0]), dy=-float(d[1]), dyaw=-psi, h=h, h_axis="2d" if two else "1d", axis_ratio=round(ratio, 3),
               strong_dir_deg=round(float(np.degrees(np.arctan2(e_s[1], e_s[0]))) % 180, 1),
               peak2=round(peak2, 3), edge=edge, fit=round(fit, 3), walls=[int(wA.sum()), int(wB.sum())],
               at_xy=np.round(c, 2).tolist())       # 어긋남을 잰 자리 = B 벽 점 가운데 (방향 어긋남은 이 둘레로 뺐음)
    yaw_edge = abs(psi) >= YAW_DEG - YAW_BIN                   # 탐색 끝에 닿음 — 더 큰 어긋남일 수 있어 믿지 않음
    # 높이: 옮긴 B 바닥과 A 바닥, 같은 칸
    if fA.sum() >= 30 and fB.sum() >= 30:
        xyB = PBr[fB, :2] + d
        kA, zA, cA = _cell_median(PA[fA, :2], PA[fA, 2], FLOOR_CELL)
        kB, zB, cB = _cell_median(xyB, PBr[fB, 2], FLOOR_CELL)
        _, ia, ib = np.intersect1d(kA, kB, return_indices=True)
        good = (cA[ia] >= 3) & (cB[ib] >= 3)
        if good.sum() >= MIN_FLOOR_CELLS:
            dz = zB[ib][good] - zA[ia][good]
            res.update(dz=float(np.median(dz)), dz_mad_cm=round(100 * float(np.median(np.abs(dz - np.median(dz)))), 1), floor_cells=int(good.sum()))
    why = ([f"맞은 비율 {fit:.2f} < {MIN_FIT:g}"] if fit < MIN_FIT else []) + \
          ([f"봉우리가 뚜렷하지 않음 (둘째 {peak2:.2f} ≥ {MAX_PEAK2:g})"] if peak2 >= MAX_PEAK2 else []) + \
          ([f"수평 탐색 끝({SEARCH_M:g} m)에 닿음"] if edge else []) + ([f"방향 탐색 끝(±{YAW_DEG:g}°)에 닿음"] if yaw_edge else [])
    res["reliable"] = not why
    if why:
        res["unreliable_why"] = why
    return res


# ── 다시 본 곳 ─────────────────────────────────────────────────────────

def _pairs(t, s, pos, F, P, L, idx):
    """구간 B (경로 SEG_M) 마다 앞 방문 A 프레임 → [(A 줄 번호들, B 줄 번호들)]. 앞 방문 = MIN_GAP_S 넘게 앞선 프레임 중 본 벽 칸(FOOT_M)의
    FOOT_SHARE 이상이 B 가 본 칸이거나 카메라가 B 의 카메라에서 수평 RADIUS_M 안인 것 → 시간으로 이어진 묶음 (2 초 넘게 끊기면 나눔) 중
    가장 이른 것과 겹친 칸 수 합이 가장 큰 것 (가장 이른 방문이 쌓인 틀어짐이 가장 큼 — 겹침만 보면 방 촬영본에서 바로 앞 바퀴만 골라
    처음 · 끝 차이를 못 봄). 맞춰 보고 겹침이 적으면 (fit) 믿지 않으므로 후보는 넉넉히"""
    from scipy.spatial import cKDTree
    tree = cKDTree(pos[idx, :2])
    W = ~L
    c = np.floor(P[W, :2] / FOOT_M).astype(np.int64)
    fk = np.unique(np.c_[F[W], c[:, 0] * 1000003 + c[:, 1]], axis=0)   # (프레임, 칸) 한 번씩
    fr_of = np.searchsorted(idx, fk[:, 0])
    n_cells = np.bincount(fr_of, minlength=len(idx))
    seg = np.floor(s[idx] / SEG_M).astype(int)
    out = []
    for g in np.unique(seg):
        B = idx[seg == g]
        if len(B) < MIN_FRAMES:
            continue
        cells = np.unique(fk[np.isin(fk[:, 0], B), 1])
        early = t[fk[:, 0]] <= t[B[0]] - MIN_GAP_S
        hit = np.bincount(fr_of[early & np.isin(fk[:, 1], cells)], minlength=len(idx))
        ok = (hit >= 3) & (hit >= FOOT_SHARE * np.maximum(n_cells, 1))
        near = np.zeros(len(idx), bool)
        near[[k for nb in tree.query_ball_point(pos[B, :2], RADIUS_M) for k in nb]] = True
        ok |= near & (t[idx] <= t[B[0]] - MIN_GAP_S)
        if ok.sum() < MIN_FRAMES:
            continue
        h = idx[ok]
        runs = [r for r in np.split(np.nonzero(ok)[0], np.nonzero(np.diff(t[h]) > 2.0)[0] + 1) if len(r) >= MIN_FRAMES]
        if not runs:
            continue
        big = max(runs, key=lambda r: hit[r].sum())
        for run in [runs[0]] + ([big] if big[0] != runs[0][0] else []):
            out.append((idx[run], B))
    return out


def _floor_track(F, P, L, T, t):
    """프레임마다 카메라 아래 (수평 2.5 m 안) 바닥 높이 → 10 초 중앙 [(초, m)]"""
    rows = []
    o = np.argsort(F[L], kind="stable")
    Fi, Pz, Pxy = F[L][o], P[L][o, 2], P[L][o, :2]
    st = np.r_[0, np.nonzero(np.diff(Fi))[0] + 1, len(Fi)]
    for a, b in zip(st[:-1], st[1:]):
        i = Fi[a]
        near = np.linalg.norm(Pxy[a:b] - T[i][:2, 3], axis=1) < 2.5
        if near.sum() >= 30:
            rows.append((t[i] - t[0], float(np.median(Pz[a:b][near]))))
    if not rows:
        return []
    r = np.array(rows)
    return [[int(a), round(float(np.median(r[(r[:, 0] >= a) & (r[:, 0] < a + 10), 1])), 3)]
            for a in np.arange(0, r[:, 0].max() + 10, 10) if ((r[:, 0] >= a) & (r[:, 0] < a + 10)).any()]


def evaluate(fr, cap, T, idx):
    """한 자세 묶음 T 로 다시 본 곳을 모두 맞춤 → dict (pairs · summary · verdict)"""
    t = cap["t"]
    if len(fr) < 2 * MIN_FRAMES:                                # 벽 · 바닥 점이 있는 장이 거의 없음
        return dict(summary=dict(pairs=0, reliable=0), verdict="skip", pairs=[], floor_track=[],
                    reasons=[f"벽 · 바닥 점이 있는 프레임이 {len(fr)}장뿐 — 되돌아온 자리 어긋남을 잴 수 없음"])
    pos = T[:, :3, 3]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pos, axis=0), axis=1))]
    F, P, N, A, L = _world(fr, T)
    order = np.argsort(F, kind="stable")
    F, P, N, A, L = F[order], P[order], N[order], A[order], L[order]
    have = np.array(sorted(set(F.tolist())))
    pairs = []
    for fa, fb in _pairs(t, s, pos, F, P, L, have):              # A · B 는 시간으로 이어진 프레임 — F 가 정렬돼 있어 구간으로 자름
        ma = slice(np.searchsorted(F, fa[0]), np.searchsorted(F, fa[-1], "right"))
        mb = slice(np.searchsorted(F, fb[0]), np.searchsorted(F, fb[-1], "right"))
        r = register((P[ma], N[ma], A[ma], L[ma]), (P[mb], N[mb], A[mb], L[mb]))
        ca, cb = int(np.median(fa)), int(np.median(fb))
        r.update(t_a=round(float(t[ca] - t[0]), 1), t_b=round(float(t[cb] - t[0]), 1), path_between_m=round(float(s[cb] - s[ca]), 1),
                 cam_b=np.round(pos[cb, :2], 2).tolist(), frames=[len(np.unique(F[ma])), len(np.unique(F[mb]))])   # 실제로 쓴 장 수
        pairs.append(r)
    good = [p for p in pairs if p.get("ok") and p["reliable"]]
    sm = dict(pairs=len(pairs), reliable=len(good), duration_s=round(float(t[have[-1]] - t[have[0]]), 1))
    bad = [p["path_between_m"] for p in pairs if not (p.get("ok") and p["reliable"])]
    if bad:
        sm["unreliable_path_max_m"] = round(float(max(bad)), 1)
    if good:
        h = np.array([p["h"] for p in good])
        z = np.array([abs(p["dz"]) for p in good if "dz" in p])
        y = np.array([abs(p["dyaw"]) for p in good])
        L_ = np.array([p["path_between_m"] for p in good])
        k = int(np.argmax(h))
        sm.update(h_max_cm=round(100 * float(h.max()), 1), h_p50_cm=round(100 * float(np.median(h)), 1),
                  h_worst=dict(t_a=good[k]["t_a"], t_b=good[k]["t_b"], path_between_m=good[k]["path_between_m"], h_axis=good[k]["h_axis"]),
                  yaw_max_deg=round(float(y.max()), 2), path_between_max_m=round(float(L_.max()), 1))
        far = L_ >= 10
        if far.any():
            sm["h_rate_cm_per_10m"] = round(float(np.median(1000 * h[far] / L_[far])), 2)
        if len(z):
            zz = np.array([p["dz"] for p in good if "dz" in p])
            Lz = np.array([p["path_between_m"] for p in good if "dz" in p])
            kz = int(np.argmax(np.abs(zz)))
            sm.update(z_max_cm=round(100 * float(np.abs(zz).max()), 1), z_p50_cm=round(100 * float(np.median(np.abs(zz))), 1),
                      z_worst=dict(dz_cm=round(100 * float(zz[kz]), 1), t_a=[p for p in good if "dz" in p][kz]["t_a"],
                                   t_b=[p for p in good if "dz" in p][kz]["t_b"], path_between_m=float(Lz[kz])))
            if (Lz >= 10).any():
                sm["z_rate_cm_per_10m"] = round(float(np.median(1000 * np.abs(zz[Lz >= 10]) / Lz[Lz >= 10])), 2)
    v, why = verdict(sm)
    for p in pairs:                                             # 숫자를 cm · 반올림으로
        for k in ("dx", "dy", "dz", "h"):
            if k in p:
                p[k + "_cm"] = round(100 * p.pop(k), 1)
        if "dyaw" in p:
            p["dyaw_deg"] = round(p.pop("dyaw"), 2)
    return dict(summary=sm, verdict=v, reasons=why, pairs=pairs, floor_track=_floor_track(F, P, L, T, t))


def verdict(sm):
    """요약 → (pass | warn | fail | skip, 이유 목록)"""
    if not sm.get("pairs"):
        dur = sm.get("duration_s")
        if dur is not None and dur <= MIN_GAP_S:
            return "skip", [f"다시 지나간 곳이 없음 — 촬영이 {dur:g} 초뿐이라 {MIN_GAP_S:g} 초 넘게 떨어져 같은 곳을 다시 본 장이 없음. "
                            "되돌아온 자리 어긋남을 잴 수 없음"]
        return "skip", [f"다시 지나간 곳이 없음 — {MIN_GAP_S:g} 초 넘게 떨어져 같은 곳을 다시 본 구간이 없음 (한 방향 촬영). "
                        "되돌아온 자리 어긋남을 잴 수 없음"]
    if not sm.get("reliable"):
        return "warn", [f"다시 지나간 곳 {sm['pairs']}군데를 맞추지 못함 (겹친 벽이 적거나 움직인 물체가 많음)"]
    v, why = "pass", []
    z, h = sm.get("z_max_cm"), sm["h_max_cm"]
    for val, ps, fl, name, w in ((z, PASS_Z_CM, FAIL_Z_CM, "높이", sm.get("z_worst")), (h, PASS_H_CM, FAIL_H_CM, "수평", sm.get("h_worst"))):
        if val is None or val <= ps:
            continue
        bad = val > fl
        v = "fail" if bad or v == "fail" else "warn"
        why.append(f"다시 지나간 곳의 {name} 어긋남 최대 {val:g} cm — {fl if bad else ps:g} cm 를 넘음 "
                   f"(촬영 {w['t_a']:g} 초에 본 곳을 {w['t_b']:g} 초에 다시 봄, 그 사이 {w['path_between_m']:g} m 걸음)")
    if v == "pass":                                             # 통과라도 잴 수 있었던 것이 모자라면 주의로
        far, near = sm.get("unreliable_path_max_m"), sm.get("path_between_max_m")
        if far is not None and near is not None and far > near + SEG_M:
            v = "warn"
            why.append(f"더 멀리 떨어져 다시 본 곳(그 사이 {far:g} m)을 맞추지 못함 — 믿을 쌍은 {near:g} m 까지라 그 사이 쌓인 틀어짐은 모름")
        if z is None:
            v = "warn"
            why.append("바닥 높이를 견준 믿을 쌍이 없음 (같이 본 바닥 칸이 모자람) — 높이 어긋남은 모름")
    return v, why


def _jumps(cap):
    """자세가 한 장 사이에 JUMP_M · JUMP_DEG 넘게 튄 곳 → (처음 20곳, 모두 몇 곳). 줄이 빠져 시간이 벌어진 곳은 그 사이 장 수(시간 차 /
    보통 간격)로 나눠 봄 — 빠진 줄 때문에 한꺼번에 움직인 것을 튐으로 세지 않게"""
    t, T = cap["t"], cap["T"]
    dt = np.diff(t)
    g = np.maximum(1.0, np.rint(dt / max(float(np.median(dt)), 1e-6)))
    ang = np.degrees(np.arccos(np.clip((np.einsum("kij,kij->k", T[1:, :3, :3], T[:-1, :3, :3]) - 1) / 2, -1, 1)))
    k = np.nonzero((cap["step"][1:] / g > JUMP_M) | (ang / g > JUMP_DEG))[0] + 1
    return [dict(t_s=round(float(t[i] - t[0]), 2), step_cm=round(100 * float(cap["step"][i]), 1), turn_deg=round(float(ang[i - 1]), 1),
                 frames=int(g[i - 1])) for i in k[:20]], int(len(k))


def _build_sha(path):
    """결과 폴더의 build/tsdf_meta.json 이 적은 보정 파일 sha256 (없으면 None)"""
    try:
        with open(os.path.join(path, "build", "tsdf_meta.json"), encoding="utf-8") as f:
            pf = json.load(f).get("pose_fix")
        return pf.get("sha256") if isinstance(pf, dict) else None
    except (OSError, ValueError, AttributeError):
        return None


def check_fix(info):
    """직접 받은 보정 info 가 stray_fix.apply_fix 로 적용할 수 있는 꼴인지 — 아니면 ValueError (문구). applied 가 아니면 볼 것 없음"""
    from . import stray_quality
    if not isinstance(info, dict):
        raise ValueError("자세 보정 info 가 JSON 객체(dict)가 아님")
    if not info.get("applied"):
        return
    bad = []
    for st in info["steps"] if isinstance(info.get("steps"), list) else [info]:
        if not isinstance(st, dict):
            bad.append("단계 info 가 dict 가 아님")
        elif st.get("applied"):
            w = stray_quality._fix_step_ok(st)
            if w:
                bad.append(w)
    if bad:
        raise ValueError("자세 보정 info 를 적용할 수 없음: " + "; ".join(bad))


def measure(path, fix_info=None, write=True, every_s=EVERY_S, capture=None):
    """촬영 폴더 · zip · scan2sim 결과 폴더 → dict {raw, fixed (보정이 있으면), verdict (고친 자세가 있으면 그것), pose_fix, …}.
    결과 폴더면 맵과 같은 보정(stray_quality._pose_fix — sha256 이 맞을 때만)을 읽고, write 면 input_map/stray_revisit.json 에 씀.
    capture = 결과 폴더의 촬영 원본 (input_map/_unzip 이 없을 때 — stray_quality.measure 와 같음). fix_info = 보정 info (pose_fix.json 내용) 를
    직접 줄 때 (적용할 수 없는 꼴이면 ValueError). zip 은 임시 폴더에 풀고 지움. 촬영 자료가 잘못됐으면 CaptureError (stray.validate 의 문구)"""
    import shutil
    import tempfile
    from . import stray_quality
    t0 = time.time()
    path = os.path.abspath(os.path.expanduser(path))
    mp = os.path.join(path, "input_map")
    fix_st, tmp, given = None, None, fix_info is not None
    if given:
        check_fix(fix_info)
    try:
        if os.path.exists(os.path.join(mp, "stray_meta.json")):
            meta = stray_quality._meta(mp)
            src = stray_quality._capture(path, capture, meta.get("source"))
            name = os.path.basename(os.path.normpath(src))
            if not given:
                fix_info, fix_st = stray_quality._pose_fix(mp, os.path.join(path, "build"))
                want = _build_sha(path)
                fix_st = dict(fix_st, sha256_expected=want, sha256_used=want if fix_info is not None and want else None)
        else:
            write = False
            if os.path.isfile(path) and path.lower().endswith(".zip"):
                tmp = tempfile.mkdtemp(prefix="revisit_")
            src = stray.find_capture(path, tmp)                 # 깨진 zip 이면 CaptureError — 임시 폴더는 finally 가 지움
            if not src:
                raise SystemExit(f"촬영 폴더 · zip · scan2sim 결과 폴더가 아님: {path}")
            name = os.path.basename(os.path.normpath(src))
            if tmp and os.path.normpath(src) == os.path.normpath(tmp):
                name = os.path.splitext(os.path.basename(path))[0]   # zip 맨 위에 파일이 있음 — 임시 폴더 이름 대신 zip 이름
        errs, _, _ = stray.validate(src)
        if errs:
            raise stray.CaptureError("촬영 자료에 문제가 있어 재지 않음: " + " / ".join(errs))
        cap = _load(src)
        idx = _frames(cap, every_s)
        fr = _points(src, cap, idx)
    finally:
        if tmp:
            shutil.rmtree(tmp, ignore_errors=True)
    jumps, n_jumps = _jumps(cap)
    res = dict(source=name, frames=len(fr), every_s=every_s,
               params=dict(seg_m=SEG_M, min_gap_s=MIN_GAP_S, foot_m=FOOT_M, foot_share=FOOT_SHARE, radius_m=RADIUS_M, search_m=SEARCH_M,
                           max_depth_m=MAX_DEPTH, min_conf=MIN_CONF),
               jumps=jumps, jumps_total=n_jumps, raw=evaluate(fr, cap, cap["T"], idx))
    if fix_st is not None or given:                             # 보정을 쓰지 못했어도 그 까닭이 남게
        res["pose_fix"] = fix_st if fix_st is not None else dict(applied=bool(fix_info.get("applied")), status=fix_info.get("status"),
                                                                 file="(fix_info 로 받음)")
    main = res["raw"]
    if fix_info is not None and fix_info.get("applied"):
        res["fixed"] = evaluate(fr, cap, stray_fix.apply_fix(cap["t"], cap["T"], fix_info), idx)
        main = res["fixed"]
    reasons = list(main["reasons"])
    note = (res.get("pose_fix") or {}).get("note")
    if note:
        reasons.append(f"자세 보정: {note}")
    res.update(verdict=main["verdict"], reasons=reasons, seconds=round(time.time() - t0, 1))
    if write:
        stray._write_text(os.path.join(mp, "stray_revisit.json"), json.dumps(res, ensure_ascii=False, indent=1))   # 임시 이름으로 쓰고 바꿔 놓음
    return res


def _line(name, r):
    s = r["summary"]
    if not s.get("reliable"):
        return f"{name}: 쌍 {s['pairs']} · 믿을 쌍 0 — {r['verdict']}"
    return (f"{name}: 쌍 {s['pairs']} (믿을 쌍 {s['reliable']}) | 수평 최대 {s['h_max_cm']} cm (중앙 {s['h_p50_cm']}) · "
            f"높이 최대 {s.get('z_max_cm')} cm (중앙 {s.get('z_p50_cm')}) · 방향 최대 {s['yaw_max_deg']}° | "
            f"10 m 당 수평 {s.get('h_rate_cm_per_10m')} · 높이 {s.get('z_rate_cm_per_10m')} cm | 판정 {r['verdict']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="촬영 폴더 · zip · scan2sim 결과 폴더 (input_map/ · build/)")
    ap.add_argument("--capture", help="결과 폴더의 촬영 원본 (input_map/_unzip 이 없을 때 — 폴더 또는 zip)")
    ap.add_argument("--fix-json", help="자세 보정 info (pose_fix.json) — 주면 그 보정을 적용한 자세도 잼")
    ap.add_argument("--every-s", type=float, default=EVERY_S, help="프레임 간격 초 (기본 0.1)")
    ap.add_argument("--json", help="결과를 이 경로에도 씀")
    ap.add_argument("--pairs", action="store_true", help="쌍마다 출력")
    ap.add_argument("--no-write", action="store_true", help="결과 폴더의 input_map/stray_revisit.json 에 쓰지 않음")
    a = ap.parse_args()
    try:
        fix = None
        if a.fix_json:
            with open(a.fix_json, encoding="utf-8") as f:
                fix = json.load(f)
        r = measure(a.path, fix, write=not a.no_write, every_s=a.every_s, capture=a.capture)
    except (stray.CaptureError, ValueError, OSError) as e:      # JSON 이 깨짐 (ValueError) · 파일 없음 (OSError) 포함
        raise SystemExit(f"[되돌아온 자리] 실패: {e}")
    print(f"[되돌아온 자리] {r['source']} · 프레임 {r['frames']} · {r['seconds']} s")
    for k in ("raw", "fixed"):
        if k in r:
            print("  " + _line("원래 자세" if k == "raw" else "고친 자세", r[k]))
            if a.pairs:
                for p in r[k]["pairs"]:
                    print("    " + json.dumps({q: p.get(q) for q in ("t_a", "t_b", "path_between_m", "dx_cm", "dy_cm", "h_cm", "h_axis", "axis_ratio", "dz_cm",
                                                                  "dyaw_deg", "fit", "peak2", "floor_cells", "reliable", "reason", "unreliable_why")},
                                                  ensure_ascii=False))
    if r["jumps"]:
        print(f"  자세가 한 장 사이에 튄 곳 {r['jumps_total']}곳: " + ", ".join(f"{j['t_s']} s ({j['step_cm']} cm · {j['turn_deg']}°)" for j in r["jumps"]))
    for m in r["reasons"]:
        print("  - " + m)
    if a.json:
        with open(a.json, "w", encoding="utf-8") as f:
            json.dump(r, f, ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
