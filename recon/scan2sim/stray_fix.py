"""아이폰(Stray) 키프레임 자세의 틀어짐을 바닥(높이) · 벽(수평)으로 바로잡음 — 깊이를 합치기(stray_mesh) 전에. B 의 단계.

    cd recon
    python -m scan2sim.stray_fix <맵 폴더> [--fix xy|floor+xy|floor] [--tilt | --no-tilt] [--yaw] [--json <경로>]
        # 재기만 함 (맵 폴더 · 메쉬는 그대로). 고쳐서 합치기는 stray_mesh 가 함. 기본 xy (DEFAULT_FIX) — --tilt 만 주면 floor+xy

왜
    아이폰 자세(VIO)에는 되돌아온 자리를 맞추는 보정이 없어 긴 촬영에서 카메라 자리가 조금씩 틀어진다. 그대로 합치면 같은 바닥이
    두 높이로, 같은 벽이 두 자리에 찍혀 진단에 '바닥 두 겹' · '낮은 턱' 으로 나온다 (기준 촬영본 118 초: 키프레임 높이 오차 5~95% 약 ±5 cm,
    최대 약 20 cm · 수평 5~95% 약 −13 ~ +5 cm).

차례 (fix_poses)
    높이(floor_fix) → 수평(xy_fix). 수평 보정은 높이를 고친 자세를 받음. 단계마다 관문이 따로 — 한 단계가 거부 · 오류여도 다른 단계는 돎.
    기본(DEFAULT_FIX)은 수평만 "xy" — 높이 단계는 hf 깊이 규칙 · 5 m 에서 천장을 나쁘게 해서 뺌 (아래 "측정")
    info 는 하나로 묶음: {method "chain", steps: [높이 info, 수평 info], applied, status, …} (아래 "info")

높이 보정 floor_fix (키프레임 자세의 높이만 고침 — 수평 위치 · 방향은 그대로)
    1. 키프레임마다 위를 향한 면(법선 z > 0.9 · 카메라보다 0.3~2.6 m 아래 · 깊이 3 m 안 — 바닥 · 턱 · 단 · 가구 윗면)을 20 cm 칸으로 나눠
       칸마다 넓이 가중 중앙 높이 y_kc 를 잼. 깊이는 키프레임 깊이 그림을 원래 크기(아이폰 256×192)로 모든 화소를 씀
       (960 처럼 키운 그림이면 256 근처가 되게 건너뜀 — step)
    2. y_kc = e_k + f_c 를 풂. e_k = 키프레임 k 자세의 높이 오차, f_c = 칸 c 의 실제 높이 (칸마다 따로 — 단 · 계단 · 턱을 평평하게 누르지 않음).
       같은 칸을 다른 때 본 키프레임끼리 높이가 다르면 그 차이만 e 로 감. 한 키프레임만 본 칸은 아무것도 정하지 않음.
       희소 가중 최소제곱 + Huber (2 cm, 6회, 3회째부터 10 cm 넘게 어긋난 관측은 버림 — 움직인 물체 · 가구 가장자리).
       e 는 시간으로 이어지게 (무작위 걸음 사전 q = 0.02 m/√s — 바닥을 못 본 키프레임은 앞뒤를 이어 메움), 바닥 근거가 있는 키프레임의
       e 중앙값 = 0 (가장 적게 옮김)
    3. 고친 자세 = 카메라 높이 − e_k. tilt 면 키프레임마다 바닥 기울기 오차 (gx, gy) 도 같이 풀어 (사전 σ 1° · 시간 이음 0.5°/√s —
       11/25 전에는 √s 없이 키프레임 사이마다 0.5° 라 키프레임이 촘촘하면 사전이 약했음) 카메라 중심 둘레로 돌림
       (높이 단계를 쓸 때 기본 켬 — 아래 "예전 측정")
    관문 — 걸리면 자세를 그대로 둠 (info.applied = False)
      refused  바닥을 다른 키프레임과 10칸 넘게 같이 본 키프레임이 30% 미만 · 보정이 40 cm 넘음(틀어짐이 아니라 추적이 깨진 것) ·
               고친 뒤에도 바닥 높이 잔차 RMS 가 3 cm 넘음 (이 모델로 못 고침)
      skipped  보정 폭(2~98%)이 1.5 cm 미만 — 고칠 것 없음 (가상의 방처럼 자세가 정확한 촬영)
    표시 (info.flags): 키프레임 5% 넘게가 주 바닥과 8 cm 넘게 다른 층에 서 있었음 (단 · 계단 — 층은 칸 높이로 따로 두므로 누르지 않음) ·
      바닥을 못 본 구간이 10 초 넘음 (그 사이는 앞뒤를 이은 값) · 한 장만 튄 키프레임 (시간으로 9장 중앙에서 10 cm 넘게 — 목록만, 빼지 않음) ·
      tilt 일 때 기울기 보정이 5° 넘는 키프레임 (추적이 깨진 자리를 기울기가 흡수하면 튄 장 표시가 사라지므로 — 목록만)

수평 보정 xy_fix (키프레임 자세의 수평 위치만 고침 — 높이 · 방향은 그대로)
    1. 방의 벽 방향: 키프레임마다 수직 면(|법선 z| < 0.15 · 깊이 3 m 안 · 256×192 에서 2 화소마다, 법선은 ±4 화소 차이)의 법선 각을 90° 로 접은
       넓이 가중 막대그림 → 가장 많은 방향 = 서로 직각인 두 축 u · v. 확신: r4 = |Σ a·e^{4iθ}| / Σ a (맨해튼이면 1 근처, 둥근 · 육각형 방은 0 근처) ·
       aligned = 축 ±6° 안 넓이 비율 · peak2 = 축 ±10° 밖 가장 큰 봉우리 / 축 봉우리 · 축마다 넓이 몫 share_u · share_v
    2. 벽 칸: 법선이 ±u (또는 ±v) 에서 16° 안인 점을 (축 · 방향 · 벽을 따라 20 cm · 높이 20 cm · 벽 앞뒤 25 cm) 로 묶어 키프레임마다 벽 자리
       (넓이 가중 중앙). 같은 (축 · 방향 · 벽을 따라 · 높이) 값들을 자리로 정렬해 30 cm 넘게 벌어진 곳에서 끊은 것 = 한 벽 조각
    3. p_kc = x_k + g_c 를 u · v 따로 (한 풀이로) — 높이와 같은 풀이 (Huber 2 cm × 6회 · 3회째부터 10 cm 넘는 관측 버림 · q = 0.02 m/√s ·
       근거 있는 키프레임 중앙 = 0). yaw=True 면 z 축 방향 오차 ψ 도 풂 (기본 끔 — 기준 촬영본에서 이득 없음)
    4. 고친 자세 = 카메라 위치 − (x_u u + x_v v) (yaw 면 회전 Rz(−ψ)·R, 카메라 중심 둘레)
    관문
      refused (전부)  벽이 서로 직각인 두 방향으로 뚜렷하게 모이지 않음 (r4 < 0.5 · aligned < 0.5 · peak2 > 0.5 중 하나 — 둥근 · 육각형 · 어수선한 방)
      그 축만 그대로  그 축 벽 넓이가 수직 면의 8% 미만 · 벽을 다른 키프레임과 10칸 넘게 같이 본 키프레임 20% 미만 · 보정 40 cm 넘음 ·
                      고친 뒤 벽 자리 잔차 RMS 3 cm 넘음 (두 축 모두 걸리면 refused)
      skipped         쓴 축 모두 보정 폭(2~98%) 2.5 cm 미만 — 고칠 것 없음
    표시: 벽을 못 본 구간이 10 초 넘음 · 앞뒤와 10 cm 넘게 튀거나 1 초 안에 10 cm 넘게 건너뛴 키프레임 (추적이 깨졌다 이어진 것 — 목록만)
    같은 입력이면 같은 결과 (난수 없음 · 스레드는 장마다 따로 계산하고 순서대로 모음) — 두 단계 모두

info (stray_mesh 가 <빌드 폴더>/pose_fix.json 으로 씀 (되면 맵 폴더에도 같은 바이트) · tsdf_meta.json 의 pose_fix 에 단계별 요약과 그 파일의 sha256)
    묶음: method "chain" · version · fix ("floor+xy" 등) · order · applied (한 단계라도 고침) · status (applied | skipped | refused | error | off) ·
          reasons · flags (단계 이름 "높이: " · "수평: " 을 앞에 붙여 모음) · keyframes · seconds{floor, xy, total} · steps
    높이 단계 (kind "floor", method "floor_cells"): params · keyframes · applied · status · reasons · flags · t (키프레임 시각) ·
          dz (키프레임마다 카메라 높이에서 뺀 값 m — 고치지 않았으면 0) · dz_estimate (고치지 않았을 때만) · tilt (키프레임마다 (gx, gy),
          끄거나 고치지 않았으면 None) · tilt_deg_p50_p95 · tilt_max_deg · tilt_big_frames ·
          frames_supported(_pct) · cells · observations(_dropped) · resid_rms_cm · resid_med_cm · dz_range_cm (2~98%) · dz_p5_p95_cm ·
          dz_max_abs_cm · max_unsupported_s · jump_frames · jump_times · levels · seconds{observe, solve, total}
    수평 단계 (kind "xy", method "wall_cells"): params · keyframes · applied · status · reasons · flags · t · axes_applied ·
          manhattan{yaw_deg, r4, aligned, peak2, share_u, share_v, vertical_area_m2, frame_yaw_deg_p5_p95, frame_yaw_noise_deg, frames_with_walls_pct} ·
          axes{u, v: applied · share · supported_pct · cells · observations(_dropped) · spread_before_cm · resid_rms_cm · resid_med_cm · range_cm ·
          max_abs_cm · p5_p95_cm · max_unsupported_s · jump_frames · reasons} · dx · dy (세계 m, 카메라 위치에서 뺀 값 — 고치지 않았으면 0) ·
          du · dv (축 성분) · du_estimate · dv_estimate (고치지 않았을 때만) · dyaw_deg (yaw 를 쓸 때만, 아니면 None) · yaw_fit · shift_cm_p50_p95 ·
          seconds{observe, solve, total}
    apply_fix(시각, 자세, info) = 같은 보정을 키프레임이 아닌 프레임에도 (시각으로 이어 붙임, steps 차례대로) — stray_quality 가 안 쓴 프레임에 씀.
    예전 floor_fix.json (높이 info 하나) 도 같은 함수로 적용됨

측정 (11/24 밤 · 11/25 B — 지금 기본값의 근거. 깊이 규칙 hf · TSDF 2 cm · 5 m (stray.py · stray_mesh 기본), 키프레임 깊이 256×192.
      바닥 = 메쉬에서 바닥 근처 위를 향한 면 · C 띠 = 바닥 10~25 cm 아래 · 5~10 cm 위 넓이, 두 겹 = 10 cm 칸에 4 cm 넘게 떨어진 두 층.
      벽 두 겹 = 맨해튼 벽 10 cm 칸. 천장 = 두 보정 모두 안 쓴 면 — 메쉬 두 겹 칸 · 쌍 = 10 초 넘게 떨어져 같은 천장 칸(20 cm)을 본 키프레임 쌍의
      높이 차가 5 cm 넘는 비율 (키프레임 깊이 5 m 까지). 안 쓴 프레임 = stray_quality n=60 중앙 · 90% (같은 60장, 그 변형의 보정을 그 장에도
      적용) — seed 0 / seed 2. 엔진 = scan2sim 이 그 메쉬로 낸 floor_two_levels · visible_floor_missing (%, R10 과 같은 길))
    기준 촬영본 118 초 · 키프레임 1133장
                                바닥 두 겹 · >5 cm · C 띠 | 벽 두 겹 | 천장 두 겹 · 쌍 | 안 쓴 프레임 중앙 · 90%       | 엔진     | 보정 초
      끔                         4.3 · 3.8 · 2.6          | 29.1     |  4.3 · 21.3     | 2.10 · 11.9 / 2.07 · 12.4    | 3.2 · 6.9 | 0
      xy 만 (기본)               4.0 · 3.4 · 2.3          | 17.3     |  4.0 · 21.7     | 1.64 · 8.0  / 1.66 · 8.6     | 2.7 · 6.2 | 2.5
      floor+xy (tilt 끔)         5.0 · 3.2 · 1.6          | 17.3     | 10.3 · 40.3     | 1.57 · 7.0  / 1.58 · 7.2     | 2.7 · 7.1 | 7.2
      floor+xy (tilt, 예전 사전) 6.0 · 3.5 · 1.8          | 17.1     | 14.6 · 55.4     | 1.33 · 6.9  / 1.36 · 7.0     | 2.1 · 6.7 | 9.2
      floor+xy (tilt, √s 사전)   4.8 · 3.0 · 1.5          | 17.3     | 12.9 · 46.7     | 1.31 · 6.9  / 1.34 · 6.9     | 0.6 · 6.9 | 7.8
      장마다 (seed 0, 같은 장, 중앙 차이가 0.05 cm 넘게 좋아진 장 : 나빠진 장, xy 대비): 끔 8:35 · floor+xy (tilt 끔) 31:18 · (tilt, 예전) 38:17 ·
      (tilt, √s) 43:12. √s 사전: 기울기 중앙 · 95% 0.65 · 1.52° · 최대 2.7° (예전 사전 0.74 · 1.96° · 4.7°), 높이 5~95% −6.2~+5.1 cm · 잔차 1.27 cm.
      결론 → 기본 xy: 수평 보정은 벽 두 겹 29 → 17% · 안 쓴 프레임 2.10 → 1.64 cm 로 좋게 하고 천장을 바꾸지 않음. 높이 단계는 hf 규칙이 먼 바닥의
      신뢰도 1 깊이를 이미 걸러 바닥 두 겹이 4% 근처라 이득이 작고 (>3 cm 10.9 → 6.2% · 안 쓴 프레임 −0.3 cm), 천장을 나쁘게 함 (메쉬 두 겹
      4.0 → 10~15% · 쌍 21.7 → 40~55%, 까닭은 아래 예전 측정의 천장 문단). 천장이 없거나 바닥 턱이 중요한 촬영은 --fix floor+xy (tilt 는 높이
      단계를 쓸 때 기본 켬 — √s 사전에서 tilt 끔보다 바닥 · 안 쓴 프레임 · 엔진 두 높이가 낫고 천장은 조금 더 나쁨)
예전 측정 (11/24 저녁 B — 예전 깊이 규칙 (신뢰도 1 이상 · 3 m) · TSDF 3 m · tilt 예전 사전. 이것으로 floor+xy (tilt) 를 기본으로 정했었음. 정의는 위와 같고
      천장 >5 cm = 메쉬 천장이 주 높이에서 5 cm 넘게 벗어난 넓이)
    기준 촬영본 118 초 · 키프레임 1133장
                           바닥 두 겹 · >5 cm · C 띠 | 벽 두 겹 | 천장 두 겹 · >5 cm | 안 쓴 프레임 중앙 · 90%         | 보정 초
      끔                    19.8 · 17.8 · 14.6      | 22.7     | 5.2 · 50.7         | 2.03 · 13.2 / 1.99 · 13.9      | 0
      xy 만                 19.4 · 17.4 · 14.1      | 16.2     | 5.6 · 50.8         | 1.59 · 9.1  / 1.62 · 9.7       | 2.6
      floor (tilt 끔)       12.9 · 11.2 ·  7.8      | 21.9     | 10.3 · 37.4        | 1.89 · 12.2 / 1.85 · 12.2      | 5.2
      floor+xy (tilt 끔)    12.4 · 11.2 ·  7.8      | 15.9     | 10.3 · 37.6        | 1.48 · 8.3  / 1.52 · 8.2       | 8.0
      floor (tilt)           5.0 ·  5.0 ·  3.0      | 21.7     | 10.1 · 31.7        | 1.78 · 12.7 / 1.74 · 12.8      | 6.3
      floor+xy (tilt) 그때 기본 4.8 · 4.8 · 3.0     | 15.5     |  9.4 · 32.2        | 1.38 · 8.2  / 1.42 · 8.1       | 8.4
      장마다 (같은 장, 중앙 차이가 0.05 cm 넘게 좋아진 장 : 나빠진 장): floor → floor+xy 34:6 · 35:7 / tilt 끔 → 켬 (floor+xy) 30:19 · 29:21 /
      끔 → 그때 기본 48:9. 수평 보정은 천장을 바꾸지 않고 안 쓴 프레임을 늘 좋게 함 → 켬. tilt 는 안 쓴 프레임 · 천장 두 겹 · >5 cm 가 조금씩 좋아짐 → 켬
      수평 단계: 벽 방향 89.6° · r4 0.87 · aligned 0.84 · peak2 0.03 · 벽 몫 u 27% · v 57% → 근거 u 32% · v 52%, 보정 5~95% u −10.7~+3.9 ·
      v −12.5~+5.4 cm, 잔차 1.85 · 1.51 cm, 2.2 초 (깊이 읽기 포함 — 읽기는 0.3~0.6 초뿐이라 두 단계가 따로 읽음). 표시: u 벽을 못 본 구간 최대 16 초 ·
      51 초 근처 튀거나 건너뛴 키프레임. 높이 단계 (tilt): 근거 89.7% · 5~95% −10.1~+6.0 cm · 잔차 1.53 cm · 기울기 중앙 0.93° · 95% 3.07° ·
      최대 8.9° (5° 넘는 9장 = tilt 를 끄면 튄 장으로 나오는 7.8 · 114 초 근처 — 표시), 6.2 초. 합치기까지 14 초 (보정 8.2 · 합치기 3.8 ·
      메쉬 꺼내기 1.3), 최대 메모리 1.82 GB (끔 1.65)
      천장 (안 쓴 면) — 높이 보정은 tilt 와 상관없이 천장을 나쁘게 함 (메쉬 두 겹 5.2 → 9.4~10.3%). 메쉬를 거치지 않고 다시 잼: 10 초 넘게 떨어져
      같은 천장 칸(20 cm)을 본 키프레임 쌍 (천장을 본 112장)의 높이 차가 5 cm 넘는 쌍 끔 10.6% · xy 만 10.8% → floor 24.5% · floor+xy 24.4% ·
      그때 기본 25.1% (중앙 1.65 → 1.95 · 2.29 cm, 90% 5.2 → 10.8 · 9.7 cm). 그 쌍에서 높이 보정량은 고치기 전 천장 높이 차와 상관이 없음 (+0.08) —
      천장을 보려고 위로 든 장에서는 바닥으로 잰 보정이 카메라 높이 오차가 아님 (원인 모름: 그동안 바닥을 못 봐 앞뒤를 이은 값이거나, 자세 방향에 따라
      달라지는 깊이 오차를 바닥 보정이 받아 낸 것). 그때 결론은 "바닥 · 안 쓴 프레임 이득이 커서 높이 보정은 켬" — hf · 5 m 에서는 바닥 이득이
      작아져 위 "측정" 대로 기본에서 뺌
    가상의 방 (자세가 정확, 193장): 높이 폭 0.3 cm (tilt 0.8 cm) · 수평 폭 u 0.8 · v 1.7 cm → 모두 건너뜀, mesh_raw.ply 가 바이트까지 같음
        (끔 · floor · floor+xy · tilt). 턱 4.38 · 판 1.40 cm 그대로.
        높이 (8 cm 사인 + 12 초에 6 cm 계단) + 수평 (x 8 cm 사인 + 12 초에 6 cm 계단 · y −6 cm 기울기 + 4 cm 사인) 을 넣으면: 턱 2.65 → 4.39 (tilt 4.40) ·
        판 −0.95 → 1.41 (1.43) cm · 바닥 두 겹 2.0 → 0.07%. 자세 오차 (상수 이동 뺀 뒤) 높이 중앙 · 95% 2.93 · 6.47 → 0.06 · 0.14 cm (tilt 0.12 · 0.39 ·
        방향 0.15°) · 수평 6.21 · 7.68 → 2.66 · 4.76 cm (남은 수평은 같은 벽을 본 장들이 같이 틀어진 성분 — 벽으로는 알 수 없음).
        육각형 · 둥근 방: 수평 거부 (r4 0.15 · 0.21) — 수평 틀어짐을 넣어도 그대로. 한 축 벽만 있는 방 흉내: u 만 고치고 v 는 0
"""
import argparse
import json
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor

import numpy as np

METHOD, VERSION = "floor_cells", 1                              # 높이 보정
XY_METHOD, XY_VERSION = "wall_cells", 1                         # 수평 보정
CHAIN_METHOD, CHAIN_VERSION = "chain", 1                        # 여러 단계를 묶은 info (pose_fix.json)
NATIVE_W = 256                                                  # 아이폰 깊이 원래 가로 크기 — 키운 깊이 그림은 이 근처가 되게 건너뜀
WORKERS = 2                                                     # 깊이를 읽고 칸으로 나누는 스레드 (cv2 · numpy 가 GIL 을 놓음)
DEFAULTS = dict(step=None, max_depth=3.0, cell=0.20, nz_min=0.9, band=(0.3, 2.6), min_views=2,
                q=0.02, sig0=0.01, sig_r=0.005, huber=0.02, iters=6,
                tilt=True, tilt_sigma_deg=1.0, tilt_q_deg=0.5)  # tilt: 높이 단계를 쓸 때 기본 켬 (근거: 머리말 "측정" · "예전 측정"). tilt_q_deg 는 °/√s
GATES = dict(min_frames_frac=0.3, min_shared=10, max_abs_m=0.40, max_resid_cm=3.0, noop_range_m=0.015,
             level_step_m=0.08, level_flag_pct=5.0, gap_flag_s=10.0, jump_m=0.10, tilt_flag_deg=5.0)
XY_DEFAULTS = dict(step=None, nb=2, max_depth=3.0, vert_nz=0.15, axis_cos=0.96, cell=0.20, hcell=0.20, pbin=0.25, gap=0.30, min_views=2,
                   q=0.02, sig0=0.01, sig_r=0.005, huber=0.02, iters=6,
                   yaw=False, yaw_sigma_deg=2.0, yaw_q_deg=0.2)  # step None = 256×192 에서 2 화소 (키운 그림은 그만큼 크게)
XY_GATES = dict(min_r4=0.5, min_aligned=0.5, max_peak2=0.5, min_axis_share=0.08, min_frames_frac=0.2, min_shared=10,
                noop_range_m=0.025, max_abs_m=0.40, max_resid_cm=3.0, max_yaw_deg=3.0, gap_flag_s=10.0, jump_m=0.10)
STEPS = ("floor", "xy")                                         # 단계와 차례 (높이 → 수평)
STEP_NAMES = dict(floor="높이", xy="수평")
FIXES = {"floor+xy": ("floor", "xy"), "floor": ("floor",), "xy": ("xy",)}   # stray_mesh.build(fix=…) 이름 → 단계
DEFAULT_FIX = "xy"                                              # 11/24 밤 hf 깊이 규칙 · 5 m 로 다시 잼 — 높이 단계는 천장을 나쁘게 하고 바닥 이득이 작아 끔 (머리말 "측정")
SUMMARY_KEYS = ("method", "applied", "status", "reasons", "flags", "keyframes", "frames_supported_pct", "dz_p5_p95_cm", "dz_max_abs_cm",
                "resid_rms_cm", "max_unsupported_s", "stand_other_pct", "jump_frames", "tilt_deg_p50_p95", "tilt_max_deg", "tilt_big_frames", "seconds")
XY_SUMMARY_KEYS = ("method", "applied", "status", "reasons", "flags", "keyframes", "axes_applied", "shift_cm_p50_p95", "yaw_fit", "seconds")
XY_AXIS_SUMMARY_KEYS = ("applied", "share", "supported_pct", "p5_p95_cm", "max_abs_cm", "resid_rms_cm", "max_unsupported_s")
AX = ("u", "v")


# ── 읽기 ─────────────────────────────────────────────────────────────────

def read_depth(path):
    """키프레임 깊이 그림(16비트 mm) → m (float32). 없거나 못 읽으면 None"""
    import cv2
    if not path:
        return None
    z = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    return None if z is None else z.astype(np.float32) / 1000.0


def _per_frame(depth, n, one, workers):
    """키프레임마다 one(k) → 목록 (None 은 뺌). depth 가 함수면 한 스레드 (받은 함수는 스레드에 안전한지 모름), 아니면 workers 스레드 —
    map 은 순서를 지킴 (결과가 실행마다 같음)"""
    if callable(depth) or workers <= 1:
        out = [one(k) for k in range(n)]
    else:
        with ThreadPoolExecutor(workers) as ex:
            out = list(ex.map(one, range(n)))
    return [o for o in out if o is not None]


# ── 키프레임 하나: 점 · 법선 ─────────────────────────────────────────────

def frame_points(z, K, T, step=1, max_depth=3.0, nb=1, rel_tol=0.08):
    """깊이(m) → 세계 점 · 세계 법선(카메라 쪽) · 면 넓이(m²), step 화소마다. K = (fx, fy, cx, cy) — 그림 크기 기준, 화소 중심 좌표.
    법선은 ±nb 칸 (= ±nb·step 화소) 차이로. 이웃과 깊이가 rel_tol·z + 2 cm 넘게 다르면 물체 가장자리로 보고 뺌"""
    fx, fy, cx, cy = K
    z = z[::step, ::step]
    h, w = z.shape
    X = (np.arange(w)[None, :] * step - cx) / fx * z
    Y = (np.arange(h)[:, None] * step - cy) / fy * z
    P = np.stack([X, Y, z], -1)
    ok = (z > 0) & (z <= max_depth)
    s = nb
    c = z[s:-s, s:-s]
    m = ok[s:-s, s:-s].copy()
    for q in (z[s:-s, 2 * s:], z[s:-s, :-2 * s], z[2 * s:, s:-s], z[:-2 * s, s:-s]):
        m &= (q > 0) & (np.abs(q - c) < rel_tol * c + 0.02)    # 물체 가장자리 건너뛰기
    n = -np.cross(P[s:-s, 2 * s:] - P[s:-s, :-2 * s], P[2 * s:, s:-s] - P[:-2 * s, s:-s])
    nn = np.linalg.norm(n, axis=-1)
    m &= nn > 0
    Pc, n, zc = P[s:-s, s:-s][m], n[m] / nn[m][:, None], c[m]
    inc = np.clip(np.abs((n * Pc).sum(1)) / np.linalg.norm(Pc, axis=1), 0.25, 1.0)
    area = zc ** 2 * step * step / (fx * fy) / inc
    R, t = T[:3, :3], T[:3, 3]
    return Pc @ R.T + t, n @ R.T, area


# ── 높이: 키프레임 하나의 위를 향한 면 → 칸별 높이 ──────────────────────

def frame_cells(Pw, nw, area, cam, cell=0.2, nz_min=0.9, band=(0.3, 2.6)):
    """위를 향한 면(카메라보다 band 만큼 아래)을 칸으로 → 열 (ix, iy, 넓이 가중 중앙 높이, 넓이, 카메라→칸 dx, dy). 면이 적으면 None"""
    h = cam[2] - Pw[:, 2]
    s = (nw[:, 2] > nz_min) & (h > band[0]) & (h < band[1])
    if s.sum() < 30:
        return None
    P, a = Pw[s], area[s]
    ij = np.floor(P[:, :2] / cell).astype(np.int64)
    key = ij[:, 0] * 2000003 + ij[:, 1]
    o = np.lexsort((P[:, 2], key))
    key, P, a, ij = key[o], P[o], a[o], ij[o]
    st = np.r_[0, np.nonzero(np.diff(key))[0] + 1]
    asum = np.add.reduceat(a, st)
    cw = np.cumsum(a)
    base = np.r_[0, cw[st[1:] - 1]]
    med = P[np.minimum(np.searchsorted(cw, base + 0.5 * asum), len(P) - 1), 2]
    dx = np.add.reduceat(a * (P[:, 0] - cam[0]), st) / asum
    dy = np.add.reduceat(a * (P[:, 1] - cam[1]), st) / asum
    keep = asum > 0.1 * cell * cell
    return np.c_[ij[st, 0], ij[st, 1], med, asum, dx, dy][keep]


def observe(depth, K, poses, step=None, max_depth=3.0, cell=0.2, nz_min=0.9, band=(0.3, 2.6), workers=WORKERS, **_):
    """키프레임마다 칸별 높이 → 한 표 (열: k, ix, iy, y, area, dx, dy).
    depth = 깊이 그림 경로 목록(None 이면 그 장 건너뜀) 또는 k → 깊이(m) 함수. K = (fx, fy, cx, cy) 하나 또는 장마다 (N×4)"""
    Kf = np.asarray(K, dtype=float)
    per = Kf.ndim == 2

    def one(k):
        z = depth(k) if callable(depth) else read_depth(depth[k])
        if z is None:
            return None
        s = int(step) if step else max(1, int(round(z.shape[1] / NATIVE_W)))
        Pw, nw, ar = frame_points(z, Kf[k] if per else Kf, poses[k], s, max_depth)
        c = frame_cells(Pw, nw, ar, poses[k][:3, 3], cell, nz_min, band)
        return None if c is None else np.c_[np.full(len(c), k), c]

    out = _per_frame(depth, len(poses), one, workers)
    return np.concatenate(out) if out else np.zeros((0, 7))


# ── 높이: 풀이 ───────────────────────────────────────────────────────────

def solve(t, n_frames, obs, cell=0.2, min_views=2, q=0.02, sig0=0.01, sig_r=0.005, huber=0.02, iters=6,
          tilt=False, tilt_sigma_deg=1.0, tilt_q_deg=0.5, **_):
    """→ dict(e (n_frames,), g (n_frames, 2) 또는 None, cells (ix, iy, f, views), res · used · w0 (관측마다), shared (키프레임마다 쓴 칸 수)).
    두 키프레임 이상이 본 칸이 없으면 None"""
    from scipy import sparse
    from scipy.sparse.linalg import spsolve
    k = obs[:, 0].astype(int)
    key = obs[:, 1].astype(np.int64) * 2000003 + obs[:, 2].astype(np.int64)
    _, ci, cnt = np.unique(key, return_inverse=True, return_counts=True)
    good = cnt[ci] >= min_views                                 # 두 키프레임 이상이 본 칸만
    if not good.any():
        return None
    k, ci, o = k[good], ci[good], obs[good]
    _, ci = np.unique(ci, return_inverse=True)
    C, N = int(ci.max()) + 1, n_frames
    nt = 2 * N if tilt else 0
    nx = N + nt + C
    d = o[:, 5:7]
    order = np.argsort(ci, kind="stable")
    f0 = np.array([np.median(v) for v in np.split(o[order, 3], np.cumsum(np.bincount(ci, minlength=C))[:-1])])
    y = o[:, 3] - f0[ci]                                        # 칸마다 중앙값을 빼고 풂 — 0 사전이 절대 높이에 끌리지 않게
    r_h = np.hypot(d[:, 0], d[:, 1])
    w0 = np.minimum(1.0, o[:, 4] / (0.5 * cell * cell)) / (sig0 ** 2 + (sig_r * r_h) ** 2)
    m = len(y)
    rows, cols, vals = [np.arange(m), np.arange(m)], [k, N + nt + ci], [np.ones(m), np.ones(m)]
    if tilt:
        rows += [np.arange(m), np.arange(m)]
        cols += [N + 2 * k, N + 2 * k + 1]
        vals += [d[:, 0], d[:, 1]]
    A_d = sparse.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(m, nx))
    # 사전: e 시간 이음 (무작위 걸음) · e 0 근처 (σ 1 m — 게이지만 잡음) · 기울기 0 근처 · 시간 이음
    dt = np.maximum(np.diff(t), 1e-3)
    sw = 1.0 / (q * np.sqrt(dt) + 0.002)
    i = np.arange(N - 1)
    P_rows = [sparse.csr_matrix((np.r_[-sw, sw], (np.r_[i, i], np.r_[i, i + 1])), shape=(N - 1, nx)),
              sparse.csr_matrix((np.full(N, 1.0), (np.arange(N), np.arange(N))), shape=(N, nx))]
    if tilt:
        sg = np.radians(tilt_sigma_deg)
        sq = np.radians(tilt_q_deg) * np.sqrt(dt) + np.radians(0.01)   # 무작위 걸음 tilt_q_deg °/√s (높이 · yaw 사전과 같은 꼴). 11/25 전에는 √dt 없이 장마다 tilt_q_deg
        for a in (0, 1):
            P_rows.append(sparse.csr_matrix((np.full(N, 1 / sg), (np.arange(N), N + 2 * np.arange(N) + a)), shape=(N, nx)))
            P_rows.append(sparse.csr_matrix((np.r_[-1 / sq, 1 / sq], (np.r_[i, i], N + 2 * np.r_[i, i + 1] + a)), shape=(N - 1, nx)))
    A_p = sparse.vstack(P_rows).tocsr()
    ApA = (A_p.T @ A_p + sparse.eye(nx) * 1e-4).tocsc()        # 아주 약한 0 사전 — 관측이 모두 버려진 칸도 풀리게
    w = w0.copy()
    for it in range(iters):
        AtW = A_d.T.multiply(w)
        x = spsolve((AtW @ A_d).tocsc() + ApA, AtW @ y)
        res = y - A_d @ x
        s = np.abs(res)
        w = w0 * np.where(s < huber, 1.0, huber / np.maximum(s, 1e-9))
        if it >= 2:
            w[s > 5 * huber] = 0                                # 3회째부터 10 cm 넘게 어긋난 관측을 버림 (움직인 물체 · 가구 가장자리)
    used = w > 0
    cells = np.c_[o[np.unique(ci, return_index=True)[1], 1:3], x[N + nt:] + f0, np.bincount(ci[used], minlength=C)]
    return dict(e=x[:N], g=x[N:N + nt].reshape(N, 2) if tilt else None, cells=cells, res=res, used=used, w0=w0,
                shared=np.bincount(k[used], minlength=N))


# ── 높이: 층 · 고친 자세 ─────────────────────────────────────────────────

def _levels(cells, cam, cell, step_m, near=0.8, hand=(0.9, 2.2)):
    """키프레임마다 '서 있던 바닥' = 카메라 near m 안 · 카메라보다 hand 만큼 아래인 칸 높이의 중앙 (책상 · 소파 윗면은 hand 밖이라 빠짐).
    그 높이가 주 층에서 step_m 넘게 다른 키프레임이 많으면 단 · 계단 위를 걸은 촬영. cam = 고친 카메라 위치 (n, 3)"""
    from scipy.spatial import cKDTree
    c = cells[cells[:, 3] >= 3]
    if not len(c):
        return dict(stand_frames=0)
    tr = cKDTree((c[:, :2] + 0.5) * cell)
    stand = np.full(len(cam), np.nan)
    for k, nb in enumerate(tr.query_ball_point(cam[:, :2], near)):
        if len(nb) >= 3:
            f = c[nb, 2]
            f = f[(cam[k, 2] - f > hand[0]) & (cam[k, 2] - f < hand[1])]
            if len(f) >= 3:
                stand[k] = np.median(f)
    s = stand[np.isfinite(stand)]
    if len(s) < 10:
        return dict(stand_frames=int(len(s)))
    h, e = np.histogram(s, bins=np.arange(s.min() - 0.02, s.max() + 0.03, 0.02))
    main = e[np.argmax(h)] + 0.01
    off = s - main
    hi, lo = off[off > step_m], off[off < -step_m]
    return dict(stand_frames=int(len(s)), stand_main_m=round(float(main), 3),
                stand_other_pct=round(100 * float((np.abs(off) > step_m).mean()), 1),
                stand_above_m=round(float(np.median(hi)), 3) if len(hi) else None, stand_below_m=round(float(np.median(lo)), 3) if len(lo) else None,
                cells_other_pct=round(100 * float((np.abs(c[:, 2] - main) > step_m).mean()), 1))   # 참고: 가구 윗면 · 턱 포함


def _tilt_rot(gx, gy):
    """보인 바닥 기울기 (gx, gy) 를 없애는 회전 (기울어진 법선 → 수직, 로드리게스)"""
    n = np.array([-gx, -gy, 1.0])
    n /= np.linalg.norm(n)
    ax = np.cross(n, [0, 0, 1.0])
    s = np.linalg.norm(ax)
    if s < 1e-12:
        return np.eye(3)
    a = ax / s
    th = np.arcsin(min(s, 1.0))
    Kx = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * Kx + (1 - np.cos(th)) * Kx @ Kx


def _rz(a):
    """z 축(위) 둘레 회전 a rad"""
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _longest_gap(times, sup):
    """근거가 없는 (sup 거짓) 구간 중 가장 긴 것 — 앞뒤 근거 있는 장 사이 시간 (초)"""
    gap, i, N = 0.0, 0, len(sup)
    while i < N:
        if sup[i]:
            i += 1
            continue
        j = i
        while j + 1 < N and not sup[j + 1]:
            j += 1
        gap = max(gap, float(times[min(j + 1, N - 1)] - times[max(i - 1, 0)]))
        i = j + 1
    return gap


# ── info: 적용 · 요약 · 설정 ─────────────────────────────────────────────

def step_kind(info):
    """단계 info → "floor" · "xy" · None (모름). kind 가 없는 예전 info 는 method · 값 이름으로 (floor_fix.json 은 "floor")"""
    if not isinstance(info, dict):
        return None
    if info.get("kind") in STEPS:
        return info["kind"]
    m = info.get("method")
    if m == METHOD:
        return "floor"
    if m == XY_METHOD:
        return "xy"
    if "dz" in info:
        return "floor"
    if "dx" in info and "dy" in info:
        return "xy"
    return None


def apply_fix(times, poses, info):
    """info 의 키프레임 보정을 시각으로 이어 아무 프레임(키프레임 · 안 쓴 프레임)에 적용 → 새 자세 (n, 4, 4).
    묶음 info(steps 목록 — pose_fix.json)면 단계 차례대로: 높이 (tilt 면 R ← 기울기 회전 · R, z −= dz) → 수평 (yaw 면 R ← Rz(−ψ)·R,
    x −= dx · y −= dy). 단계 하나의 info (예전 floor_fix.json) 도 받음. applied 가 아닌 단계는 그대로 (복사본). 키프레임 시각 밖은 처음 · 끝 값.
    applied 인데 어떤 보정인지 모르는 info 면 ValueError (조용히 넘기지 않음)"""
    T = np.array(poses, dtype=float, copy=True)
    if not isinstance(info, dict):
        return T
    if isinstance(info.get("steps"), list):
        for s in info["steps"]:
            T = apply_fix(times, T, s)
        return T
    if not info.get("applied"):
        return T
    kind = step_kind(info)
    tk = np.asarray(info["t"], dtype=float)
    times = np.atleast_1d(np.asarray(times, dtype=float))
    if kind == "floor":
        if info.get("tilt") is not None:
            g = np.asarray(info["tilt"], dtype=float)
            gx, gy = np.interp(times, tk, g[:, 0]), np.interp(times, tk, g[:, 1])
            for i in range(len(T)):
                T[i, :3, :3] = _tilt_rot(gx[i], gy[i]) @ T[i, :3, :3]
        T[:, 2, 3] -= np.interp(times, tk, np.asarray(info["dz"], dtype=float))
    elif kind == "xy":
        if info.get("dyaw_deg") is not None:
            ps = np.radians(np.interp(times, tk, np.asarray(info["dyaw_deg"], dtype=float)))
            for i in range(len(T)):
                T[i, :3, :3] = _rz(-ps[i]) @ T[i, :3, :3]
        T[:, 0, 3] -= np.interp(times, tk, np.asarray(info["dx"], dtype=float))
        T[:, 1, 3] -= np.interp(times, tk, np.asarray(info["dy"], dtype=float))
    else:
        raise ValueError(f"어떤 보정인지 모르는 info (method {info.get('method')!r} · kind {info.get('kind')!r}) — 적용할 수 없음")
    return T


def not_applied(status, reason, kind="floor", **extra):
    """고치지 않은 단계 info (보정 중 오류 등) — pose_fix.json 에 같은 모양으로 남기려고"""
    m, v = (XY_METHOD, XY_VERSION) if kind == "xy" else (METHOD, VERSION)
    return dict(method=m, version=v, kind=kind, applied=False, status=status, reasons=[reason], flags=[], **extra)


def chain_info(steps, fix=None, seconds=None, keyframes=None, status=None, reason=None):
    """단계 info 목록 → 묶음 info (pose_fix.json). status 를 주지 않으면 한 단계라도 고쳤으면 applied, 아니면 error > refused > skipped > off.
    reasons · flags = 단계 이름("높이: " · "수평: ")을 앞에 붙여 모은 것 (화면 · 보고에 그대로 보이게)"""
    applied = any(bool(s.get("applied")) for s in steps)
    if status is None:
        sts = [s.get("status") for s in steps]
        status = "applied" if applied else next((x for x in ("error", "refused", "skipped", "off") if x in sts), "off")
    name = lambda s: STEP_NAMES.get(s.get("kind"), s.get("kind") or "보정")
    reasons = ([reason] if reason else []) + [f"{name(s)}: {m}" for s in steps for m in s.get("reasons") or []]
    flags = [f"{name(s)}: {m}" for s in steps for m in s.get("flags") or []]
    return dict(method=CHAIN_METHOD, version=CHAIN_VERSION, fix=fix, order=[s.get("kind") for s in steps], applied=applied, status=status,
                reasons=reasons, flags=flags, keyframes=keyframes, seconds=seconds or {}, steps=list(steps))


def summary(info):
    """tsdf_meta.json 의 pose_fix 에 넣을 짧은 요약 (t · dz · dx 같은 키프레임별 목록은 뺌). 묶음 info 면 steps 에 단계마다"""
    info = info if isinstance(info, dict) else {}
    if isinstance(info.get("steps"), list):
        s = {k: info.get(k) for k in ("method", "fix", "order", "applied", "status", "reasons", "flags", "keyframes", "seconds") if k in info}
        s["steps"] = [summary(x) for x in info["steps"]]
    elif step_kind(info) == "xy":
        s = {k: info.get(k) for k in XY_SUMMARY_KEYS if k in info}
        s["kind"] = "xy"
        M = info.get("manhattan") or {}
        s["manhattan"] = {k: M[k] for k in ("yaw_deg", "r4", "aligned", "peak2", "share_u", "share_v") if k in M}
        s["axes"] = {a: dict({k: x.get(k) for k in XY_AXIS_SUMMARY_KEYS if k in x}, jumps=len(x.get("jump_frames") or []))
                     for a, x in (info.get("axes") or {}).items()}
    else:
        s = {k: info.get(k) for k in SUMMARY_KEYS if k in info}
        if step_kind(info):
            s["kind"] = step_kind(info)
        lv = info.get("levels") or {}
        if "stand_other_pct" in lv:
            s["stand_other_pct"] = lv["stand_other_pct"]
    s["applied"] = bool(info.get("applied"))
    s.setdefault("status", "applied" if s["applied"] else "unknown")
    return s


def check_opts(kw, kind="floor"):
    """보정 단계(floor · xy)에 줄 설정 이름 확인 — 모르는 이름이면 ValueError"""
    if kind not in STEPS:
        raise ValueError(f"보정 단계 이름이 틀림: {kind!r} — {list(STEPS)} 중 하나")
    d, g, fn = (DEFAULTS, GATES, "floor_fix") if kind == "floor" else (XY_DEFAULTS, XY_GATES, "xy_fix")
    bad = sorted(set(kw) - set(d) - set(g) - {"workers"})
    if bad:
        raise ValueError(f"{fn} 에 없는 설정: {bad} (설정: {sorted(d)} · 관문: {sorted(g)})")


def with_floor(fix):
    """보정 이름 → 높이(floor) 단계를 더한 이름 ("xy" → "floor+xy"). 이미 있으면 그대로 · FIXES 에 없는 이름이면 None"""
    if fix not in FIXES:
        return None
    steps = tuple(s for s in STEPS if s == "floor" or s in FIXES[fix])
    return next((k for k, v in FIXES.items() if v == steps), None)


def tilt_fix(fix, tilt, explicit=False, log=print, who="pose_fix"):
    """CLI 의 --tilt / --no-tilt (높이 단계의 설정) → 쓸 보정 이름. tilt 를 주지 않았거나 fix 에 높이 단계가 있으면 그대로.
    --fix 를 주지 않았으면 (explicit 거짓) 높이 단계를 더하고 한 줄 남김 (기본 xy → floor+xy). --fix 로 높이 단계가 없는 보정을 줬거나
    보정을 껐으면 (fix None) ValueError — 예전에는 split_opts 의 "보정 설정에 이 보정(xy)에 없는 단계 'floor' 의 설정이 있음" 으로 멈췄음"""
    if tilt is None or (fix and "floor" in FIXES.get(fix, ())):
        return fix
    flag = "--tilt" if tilt else "--no-tilt"
    if not fix:
        raise ValueError(f"{flag} 는 높이 보정의 설정 — 자세 보정을 끄면(--no-fix · --fix none) 쓸 데가 없음")
    wf = with_floor(fix)
    if explicit or wf is None:
        raise ValueError(f"{flag} 는 높이(floor) 단계의 설정인데 --fix {fix} 에는 높이 단계가 없음 — --fix {wf or 'floor+xy'} 와 같이 줄 것")
    log(f"[{who}] {flag} → 높이 단계를 더해 {wf} 로 돎 (기본 {fix} 에는 높이 단계가 없음)")
    return wf


def split_opts(steps, opts=None):
    """보정 설정 → {단계: dict}. {"floor": {...}, "xy": {...}} 꼴, 또는 예전 꼴(높이 설정만 평평하게 — 예: dict(tilt=True)).
    이 보정에 없는 단계의 설정 · 모르는 설정 이름이면 ValueError (조용히 넘기지 않음)"""
    steps = tuple(steps)
    opts = dict(opts or {})
    if opts and all(k in STEPS for k in opts):
        per = {}
        for k, v in opts.items():
            if not isinstance(v, dict):
                raise ValueError(f"보정 설정 {k!r} 는 dict 여야 함: {v!r}")
            per[k] = dict(v)
    else:
        per = {"floor": opts} if opts else {}
    for k, v in per.items():
        if k not in steps:
            raise ValueError(f"보정 설정에 이 보정({'+'.join(steps) or '끔'})에 없는 단계 '{k}' 의 설정이 있음")
        check_opts(v, k)
    return per


# ── 높이 보정 ────────────────────────────────────────────────────────────

def floor_fix(times, poses, depth, K, *, tilt=None, cell=0.2, q=0.02, log=print, **kw):
    """키프레임 시각 · 자세(카메라 광학 → 세계 Z 위, N×4×4) · 깊이(경로 목록 또는 k → 깊이 m 함수) · 내부 파라미터((fx, fy, cx, cy) 또는 N×4)
    → (고친 자세, info). 관문에 걸리면 자세를 그대로(복사본) 돌려주고 info.applied = False · status · reasons.
    tilt = 바닥 기울기 오차도 풂 (None = DEFAULTS 값, 기본 켬). kw: DEFAULTS · GATES 의 값을 바꿈 (예: noop_range_m=0 — 시험용으로 관문 끄기),
    workers = 스레드 수"""
    check_opts(kw, "floor")
    workers = kw.pop("workers", WORKERS)
    p = dict(DEFAULTS, cell=float(cell), q=float(q))
    if tilt is not None:
        p["tilt"] = bool(tilt)
    g = dict(GATES)
    for k, v in kw.items():
        (p if k in DEFAULTS else g)[k] = v
    t0 = time.time()
    times = np.asarray(times, dtype=float)
    poses = np.asarray(poses, dtype=float)
    N = len(times)
    info = dict(method=METHOD, version=VERSION, kind="floor", params={k: (list(v) if isinstance(v, tuple) else v) for k, v in {**p, **g}.items()},
                keyframes=N, applied=False, status="refused", reasons=[], flags=[], t=times.tolist())

    def done(out, why=None):
        if why:
            info["reasons"].append(why)
        info.setdefault("dz", [0.0] * N)
        info.setdefault("seconds", dict(total=round(time.time() - t0, 2)))
        log(f"[floor_fix] {dict(applied='고침', skipped='고칠 것 없음', refused='안 고침').get(info['status'], info['status'])} | 키프레임 {N}장"
            + (f" 중 바닥 근거 {info['frames_supported_pct']}% | 보정 5~95% {info['dz_p5_p95_cm']} cm · 최대 {info['dz_max_abs_cm']} cm | "
               f"잔차 {info['resid_rms_cm']} cm" if "dz_p5_p95_cm" in info else "") + f" | {info['seconds']['total']} s"
            + "".join(f"\n  - {m}" for m in info["reasons"] + info["flags"]))
        return out, info

    if poses.shape != (N, 4, 4):
        raise ValueError(f"자세 모양이 {poses.shape} — 시각 {N}개에 맞는 ({N}, 4, 4) 여야 함")
    if N < 2:
        return done(poses.copy(), f"키프레임이 {N}장 — 2장 이상이어야 고칠 수 있음")
    if np.any(np.diff(times) < 0):
        return done(poses.copy(), "키프레임 시각이 차례대로가 아님 — 고치지 않음")
    obs = observe(depth, K, poses, workers=workers, **p)
    t1 = time.time()
    S = solve(times, N, obs, **p) if len(obs) else None
    t2 = time.time()
    if S is None:
        return done(poses.copy(), "위를 향한 면(바닥)을 두 키프레임 이상이 같이 본 곳이 없음 — 고칠 근거가 없음")
    e, gk, shared = S["e"], S["g"], S["shared"]
    sup = shared >= g["min_shared"]
    e = e - np.median(e[sup] if sup.any() else e)
    from scipy.ndimage import median_filter                    # 시간으로 매끈한 e 와 크게 다른 키프레임 = 자세가 한 장만 튐 (목록만)
    jump = sup & (np.abs(e - median_filter(e, size=9, mode="nearest")) > g["jump_m"])
    u = S["used"]
    r = S["res"][u]
    resid_cm = 100 * float(np.sqrt(np.average(r ** 2, weights=S["w0"][u]))) if u.any() else float("nan")
    gap = _longest_gap(times, sup)                              # 바닥을 못 본 (공유 칸이 모자란) 구간 중 가장 긴 것
    rng = float(np.percentile(e, 98) - np.percentile(e, 2))
    lv = _levels(S["cells"], np.c_[poses[:, :2, 3], poses[:, 2, 3] - e], p["cell"], g["level_step_m"])
    info.update(frames_supported=int(sup.sum()), frames_supported_pct=round(100 * float(sup.mean()), 1),
                cells=int(len(S["cells"])), observations=int(u.sum()), observations_dropped=int((~u).sum()),
                resid_rms_cm=round(resid_cm, 2) if np.isfinite(resid_cm) else None, resid_med_cm=round(100 * float(np.median(np.abs(r))), 2) if len(r) else None,
                dz_range_cm=round(100 * rng, 1), dz_max_abs_cm=round(100 * float(np.max(np.abs(e))), 1),
                dz_p5_p95_cm=[round(100 * float(np.percentile(e, 5)), 1), round(100 * float(np.percentile(e, 95)), 1)],
                max_unsupported_s=round(gap, 1), jump_frames=np.nonzero(jump)[0].tolist(),
                jump_times=[round(float(times[j]), 6) for j in np.nonzero(jump)[0]], levels=lv)
    if gk is not None:
        tl = np.degrees(np.arctan(np.hypot(gk[:, 0], gk[:, 1])))
        big = np.nonzero(tl > g["tilt_flag_deg"])[0]
        info.update(tilt_deg_p50_p95=[round(float(np.median(tl)), 2), round(float(np.percentile(tl, 95)), 2)],
                    tilt_max_deg=round(float(tl.max()), 2), tilt_big_frames=big.tolist())
        if len(big):                                            # 기준 촬영본: 튄 키프레임(추적이 깨진 자리)에서 5~9° — 기울기로 흡수되면 jump 표시가 사라짐
            info["flags"].append(f"바닥 기울기 보정이 {g['tilt_flag_deg']:g}° 넘는 키프레임 {len(big)}장 (최대 {tl.max():.1f}°, 첫 장 "
                                 f"{float(times[big[0]] - times[0]):.1f} 초, tilt_big_frames) — 추적이 깨졌던 자리일 수 있음, 고친 메쉬를 눈으로 확인")
    # 관문
    refuse = []
    if sup.mean() < g["min_frames_frac"]:
        refuse.append(f"바닥을 다른 키프레임과 {g['min_shared']}칸 넘게 같이 본 키프레임이 {sup.mean():.0%} 뿐 (< {g['min_frames_frac']:.0%}) — 고칠 근거가 모자람")
    if info["dz_max_abs_cm"] > 100 * g["max_abs_m"]:
        refuse.append(f"높이 보정이 {info['dz_max_abs_cm']} cm (> {100 * g['max_abs_m']:.0f} cm) — 틀어짐이 아니라 추적이 깨진 것으로 봄, 다시 촬영 권함")
    if not resid_cm <= g["max_resid_cm"]:
        refuse.append(f"고친 뒤에도 바닥 높이가 키프레임끼리 {resid_cm:.1f} cm (RMS) 어긋남 (> {g['max_resid_cm']:g} cm) — 수평 틀어짐 · 움직인 바닥 등 이 모델로 못 고침")
    noop = rng < g["noop_range_m"]
    if lv.get("stand_other_pct", 0) > g["level_flag_pct"]:
        info["flags"].append(f"키프레임 {lv['stand_other_pct']}% 가 주 바닥과 {100 * g['level_step_m']:.0f} cm 넘게 다른 층에 서 있었음 "
                             f"(위 {lv.get('stand_above_m')} m · 아래 {lv.get('stand_below_m')} m) — 단 · 계단 위를 걸은 촬영일 수 있음. "
                             "층은 칸 높이로 따로 두므로 평평하게 누르지 않음 — 결과를 눈으로 확인")
    if gap > g["gap_flag_s"]:
        info["flags"].append(f"바닥을 못 본 구간이 최대 {gap:.0f} 초 — 그 사이 보정은 앞뒤를 이은 값")
    if jump.any():
        info["flags"].append(f"자세가 앞뒤와 {100 * g['jump_m']:.0f} cm 넘게 튄 키프레임 {int(jump.sum())}장 (jump_frames) — 고치지 않고 목록만 남김")
    info["reasons"] += refuse
    if noop:
        info["reasons"].append(f"고칠 것 없음: 높이 보정 폭(2~98%) {100 * rng:.1f} cm < {100 * g['noop_range_m']:.1f} cm")
    info["status"] = "refused" if refuse else "skipped" if noop else "applied"
    info["applied"] = info["status"] == "applied"
    if info["applied"]:
        info["dz"] = np.round(e, 5).tolist()                    # 0.01 mm 로 반올림한 값을 그대로 적용 — pose_fix.json 으로 다시 적용해도 같은 자세
        info["tilt"] = np.round(gk, 6).tolist() if gk is not None else None
    else:
        info["dz"], info["tilt"] = [0.0] * N, None
        info["dz_estimate"] = np.round(e, 5).tolist()
    out = apply_fix(times, poses, info)
    info["seconds"] = dict(observe=round(t1 - t0, 2), solve=round(t2 - t1, 2), total=round(time.time() - t0, 2))
    return done(out)


# ── 수평: 수직 면 → 방의 직각 축 · 벽 칸 ─────────────────────────────────

def vertical_points(depth, K, poses, step=None, nb=2, max_depth=3.0, vert_nz=0.15, workers=WORKERS, **_):
    """키프레임마다 수직 면(|법선 z| < vert_nz) 점 → [(k, 세계 점 (n, 3), 법선 수평 성분 (n, 2), 넓이 (n,))] — float32 (메모리를 줄이려고).
    step None = 256×192 에서 2 화소 (키운 그림은 그만큼 크게). 법선은 ±nb 칸 차이 (기본 ±4 화소)"""
    Kf = np.asarray(K, dtype=float)
    per = Kf.ndim == 2

    def one(k):
        z = depth(k) if callable(depth) else read_depth(depth[k])
        if z is None:
            return None
        r = max(1, int(round(z.shape[1] / NATIVE_W)))
        s = int(step) if step else 2 * r
        P, n, a = frame_points(z, Kf[k] if per else Kf, poses[k], s, max_depth, nb=nb, rel_tol=0.03 * nb * s / r)
        v = np.abs(n[:, 2]) < vert_nz
        return k, P[v].astype(np.float32), n[v, :2].astype(np.float32), a[v].astype(np.float32)

    return _per_frame(depth, len(poses), one, workers)


def manhattan(frames, n_frames, bin_deg=0.5, win_deg=6.0, frame_win_deg=10.0):
    """수직 면 법선 → 방의 직각 축 yaw(도, 0~90) · 확신 지표 · frame_yaw (키프레임마다 그 장의 벽 법선만으로 잰 축과의 차이, 도 — 참고)"""
    n = np.concatenate([f[2] for f in frames]).astype(float) if frames else np.zeros((0, 2))
    a = np.concatenate([f[3] for f in frames]).astype(float) if frames else np.zeros(0)
    if a.sum() <= 0:
        return dict(yaw_deg=0.0, r4=0.0, aligned=0.0, peak2=1.0, share_u=0.0, share_v=0.0, vertical_area_m2=0.0,
                    frame_yaw=np.full(n_frames, np.nan))
    th = np.degrees(np.arctan2(n[:, 1], n[:, 0]))
    th90 = th % 90
    nbin = int(round(90 / bin_deg))
    h, _ = np.histogram(th90, bins=nbin, range=(0, 90), weights=a)
    k = int(round(2.0 / bin_deg))
    hs = np.convolve(np.r_[h[-k:], h, h[:k]], np.ones(2 * k + 1), "valid")
    y0 = (np.argmax(hs) + 0.5) * bin_deg
    for _ in range(2):                                          # 창 안 가중 평균으로 다듬기
        d = (th90 - y0 + 45) % 90 - 45
        s = np.abs(d) < win_deg
        if a[s].sum() > 0:
            y0 = (y0 + np.average(d[s], weights=a[s])) % 90
    d = (th90 - y0 + 45) % 90 - 45
    s = np.abs(d) < win_deg
    r4 = float(np.abs(np.sum(a * np.exp(4j * np.radians(th)))) / a.sum())
    phi = (th - y0) % 180                                       # 0 · 180 근처 = u 벽, 90 근처 = v 벽
    isu = (phi < 45) | (phi > 135)
    off = np.abs(((np.arange(nbin) + 0.5) * bin_deg - y0 + 45) % 90 - 45) > 10   # 둘째 봉우리: 축 ±10° 밖 가장 큰 것 / 축 봉우리
    peak2 = float(hs[off].max() / hs.max()) if off.any() and hs.max() > 0 else 0.0
    fy = np.full(n_frames, np.nan)
    for kk, _, nk, ak in frames:
        if not len(ak):
            continue
        dk = (np.degrees(np.arctan2(nk[:, 1].astype(float), nk[:, 0].astype(float))) - y0 + 45) % 90 - 45
        sk = np.abs(dk) < frame_win_deg
        if ak[sk].sum() > 0.05:
            fy[kk] = np.average(dk[sk], weights=ak[sk].astype(float))
    return dict(yaw_deg=round(float(y0), 3), r4=round(r4, 3), aligned=round(float(a[s].sum() / a.sum()), 3), peak2=round(peak2, 3),
                share_u=round(float(a[s & isu].sum() / a.sum()), 3), share_v=round(float(a[s & ~isu].sum() / a.sum()), 3),
                vertical_area_m2=round(float(a.sum()), 1), frame_yaw=fy)


def _wmed(key, val, w):
    """key 마다 넓이 가중 중앙 → (정렬 순서, 묶음 시작, 넓이 합, 중앙값)"""
    o = np.lexsort((val, key))
    key, val, w = key[o], val[o], w[o]
    st = np.r_[0, np.nonzero(np.diff(key))[0] + 1]
    asum = np.add.reduceat(w, st)
    cw = np.cumsum(w)
    base = np.r_[0, cw[st[1:] - 1]]
    med = val[np.minimum(np.searchsorted(cw, base + 0.5 * asum), len(val) - 1)]
    return o, st, asum, med


def wall_obs(frames, poses, yaw_deg, axis_cos=0.96, cell=0.2, hcell=0.2, pbin=0.25, gap=0.30, **_):
    """벽 칸 관측 → 표 (열: k, 축 0/1, 벽 조각 번호, 벽 자리 p (m, 축 방향), 넓이, 카메라→칸 dx, dy).
    벽 조각 = (축 · 방향 · 벽을 따라 cell · 높이 hcell) 이 같은 값들을 p 로 정렬해 gap 넘게 벌어진 곳에서 끊은 것"""
    y = np.radians(yaw_deg)
    U, V = np.array([np.cos(y), np.sin(y)]), np.array([-np.sin(y), np.cos(y)])
    rows = []
    for k, P, n, a in frames:
        if not len(a):
            continue
        cam = poses[k][:2, 3]
        for ai, (ax, side) in enumerate(((U, V), (V, U))):
            cs = n @ ax
            for sg in (1, -1):
                s = cs * sg > axis_cos
                if s.sum() < 20:
                    continue
                q, w = P[s].astype(float), a[s].astype(float)
                p = q[:, :2] @ ax
                ia = np.floor(q[:, :2] @ side / cell).astype(np.int64)
                ih = np.floor(q[:, 2] / hcell).astype(np.int64)
                ip = np.floor(p / pbin).astype(np.int64)
                key = ((ia + 50000) * 2000 + (ih + 1000)) * 200000 + (ip + 100000)
                o, st, asum, med = _wmed(key, p, w)
                qq, ww = q[o], w[o]
                dx = np.add.reduceat(ww * (qq[:, 0] - cam[0]), st) / asum
                dy = np.add.reduceat(ww * (qq[:, 1] - cam[1]), st) / asum
                keep = asum > 0.1 * cell * hcell
                grp = (ai * 2 + (sg > 0)) * 10 ** 12 + (ia[o][st] + 50000) * 2000 + (ih[o][st] + 1000)   # 벽 조각 묶음 (자리 p 는 아래에서 나눔)
                rows.append(np.c_[np.full(keep.sum(), k), np.full(keep.sum(), ai), grp[keep], med[keep], asum[keep], dx[keep], dy[keep]])
    if not rows:
        return np.zeros((0, 7))
    ob = np.concatenate(rows)
    o = np.lexsort((ob[:, 3], ob[:, 2]))                        # 같은 묶음 안에서 p 로 정렬해 gap 넘게 벌어진 곳에서 끊음 → 조각 번호
    ob = ob[o]
    cid = np.cumsum(np.r_[True, (np.diff(ob[:, 2]) != 0) | (np.diff(ob[:, 3]) > gap)]) - 1
    kk = ob[:, 0].astype(np.int64)
    key = cid.astype(np.int64) * (int(kk.max()) + 1) + kk      # 같은 키프레임 · 같은 조각이 둘 이상이면 (벽 앞뒤 칸 경계) 넓이 가중 평균으로 합침
    u, inv = np.unique(key, return_inverse=True)
    wsum = np.bincount(inv, ob[:, 4])
    mean = lambda col: np.bincount(inv, ob[:, 4] * ob[:, col]) / wsum
    first = np.unique(inv, return_index=True)[1]
    return np.c_[ob[first, 0], ob[first, 1], (u // (int(kk.max()) + 1)).astype(float), mean(3), wsum, mean(5), mean(6)]


def solve_xy(t, n_frames, ob, yaw_deg, yaw=False, cell=0.2, hcell=0.2, min_views=2, q=0.02, sig0=0.01, sig_r=0.005, huber=0.02, iters=6,
             yaw_sigma_deg=2.0, yaw_q_deg=0.2, **_):
    """p_kc = x_k(축) + g_c 를 u · v 한 번에 (yaw 면 − ψ_k (d·v) / + ψ_k (d·u) 도) → dict(x (2, N), psi (N,) 또는 None, 관측마다 res · used ·
    w0 · ax · k · before (칸 중앙에서 벗어난 정도, 고치기 전), shared (2, N) 축별 키프레임마다 쓴 관측 수, cells (축별 조각 수)).
    두 키프레임 이상이 본 조각이 없으면 None"""
    from scipy import sparse
    from scipy.sparse.linalg import spsolve
    N = n_frames
    _, ci, cnt = np.unique(ob[:, 2], return_inverse=True, return_counts=True)
    good = cnt[ci] >= min_views
    if not good.any():
        return None
    ob, ci = ob[good], ci[good]
    k, ax = ob[:, 0].astype(int), ob[:, 1].astype(int)
    _, ci = np.unique(ci, return_inverse=True)
    C = int(ci.max()) + 1
    ny = N if yaw else 0
    nx = 2 * N + ny + C
    order = np.argsort(ci, kind="stable")
    f0 = np.array([np.median(v) for v in np.split(ob[order, 3], np.cumsum(np.bincount(ci, minlength=C))[:-1])])
    y = ob[:, 3] - f0[ci]                                       # 조각마다 중앙값을 빼고 풂
    d = ob[:, 5:7]
    r_h = np.hypot(d[:, 0], d[:, 1])
    w0 = np.minimum(1.0, ob[:, 4] / (0.5 * cell * hcell)) / (sig0 ** 2 + (sig_r * r_h) ** 2)
    m = len(y)
    rr, cc, vv = [np.arange(m), np.arange(m)], [ax * N + k, 2 * N + ny + ci], [np.ones(m), np.ones(m)]
    if yaw:
        yr = np.radians(yaw_deg)
        U, V = np.array([np.cos(yr), np.sin(yr)]), np.array([-np.sin(yr), np.cos(yr)])
        rr.append(np.arange(m))
        cc.append(2 * N + k)
        vv.append(np.where(ax == 0, -(d @ V), d @ U))
    A_d = sparse.csr_matrix((np.concatenate(vv), (np.concatenate(rr), np.concatenate(cc))), shape=(m, nx))
    dt = np.maximum(np.diff(t), 1e-3)
    i = np.arange(N - 1)
    sw = 1.0 / (q * np.sqrt(dt) + 0.002)
    P = []
    for a in (0, 1):                                            # 시간 이음 (무작위 걸음) · 0 근처 (σ 1 m — 게이지만 잡음)
        P.append(sparse.csr_matrix((np.r_[-sw, sw], (np.r_[i, i], a * N + np.r_[i, i + 1])), shape=(N - 1, nx)))
        P.append(sparse.csr_matrix((np.full(N, 1.0), (np.arange(N), a * N + np.arange(N))), shape=(N, nx)))
    if yaw:
        sq = np.radians(yaw_q_deg) * np.sqrt(dt) + np.radians(0.01)
        P.append(sparse.csr_matrix((np.r_[-1 / sq, 1 / sq], (np.r_[i, i], 2 * N + np.r_[i, i + 1])), shape=(N - 1, nx)))
        P.append(sparse.csr_matrix((np.full(N, 1 / np.radians(yaw_sigma_deg)), (np.arange(N), 2 * N + np.arange(N))), shape=(N, nx)))
    A_p = sparse.vstack(P).tocsr()
    ApA = (A_p.T @ A_p + sparse.eye(nx) * 1e-4).tocsc()
    w = w0.copy()
    x = np.zeros(nx)
    for it in range(iters):
        AtW = A_d.T.multiply(w)
        x = spsolve((AtW @ A_d).tocsc() + ApA, AtW @ y)
        s = np.abs(y - A_d @ x)
        w = w0 * np.where(s < huber, 1.0, huber / np.maximum(s, 1e-9))
        if it >= 2:
            w[s > 5 * huber] = 0                                # 3회째부터 10 cm 넘게 어긋난 관측을 버림
    res = y - A_d @ x
    used = w > 0
    return dict(x=x[:2 * N].reshape(2, N), psi=x[2 * N:3 * N] if yaw else None, res=res, used=used, w0=w0, ax=ax, k=k, before=y,
                shared=np.stack([np.bincount(k[used & (ax == a)], minlength=N) for a in (0, 1)]),
                cells=np.bincount(ax[np.unique(ci, return_index=True)[1]], minlength=2))


# ── 수평 보정 ────────────────────────────────────────────────────────────

def _axis_name(a, M):
    return f"{AX[a]} 축(벽 법선 {M['yaw_deg'] + 90 * a:.0f}°)"


def xy_fix(times, poses, depth, K, *, yaw=None, log=print, **kw):
    """키프레임 시각 · 자세(카메라 광학 → 세계 Z 위, N×4×4) · 깊이(경로 목록 또는 k → 깊이 m 함수) · 내부 파라미터((fx, fy, cx, cy) 또는 N×4)
    → (고친 자세, info). 관문에 걸린 축은 그대로, 두 축 모두 걸리면 자세를 그대로(복사본) 돌려주고 info.applied = False · status · reasons.
    yaw = z 축 방향 오차도 풂 (None = XY_DEFAULTS 값, 기본 끔). kw: XY_DEFAULTS · XY_GATES 의 값을 바꿈 (예: noop_range_m=0 — 시험용으로
    관문 끄기), workers = 스레드 수"""
    from scipy.ndimage import median_filter
    check_opts(kw, "xy")
    workers = kw.pop("workers", WORKERS)
    p = dict(XY_DEFAULTS)
    if yaw is not None:
        p["yaw"] = bool(yaw)
    g = dict(XY_GATES)
    for k, v in kw.items():
        (p if k in XY_DEFAULTS else g)[k] = v
    t0 = time.time()
    times = np.asarray(times, dtype=float)
    poses = np.asarray(poses, dtype=float)
    N = len(times)
    info = dict(method=XY_METHOD, version=XY_VERSION, kind="xy", params={k: (list(v) if isinstance(v, tuple) else v) for k, v in {**p, **g}.items()},
                keyframes=N, applied=False, status="refused", reasons=[], flags=[], axes_applied=[], t=times.tolist(), manhattan={}, axes={})
    clock = {}

    def done(out, why=None):
        if why:
            info["reasons"].append(why)
        for key in ("dx", "dy", "du", "dv"):
            info.setdefault(key, [0.0] * N)
        info.setdefault("dyaw_deg", None)
        t1 = clock.get("observed", time.time())
        info["seconds"] = dict(observe=round(t1 - t0, 2), solve=round(time.time() - t1, 2), total=round(time.time() - t0, 2))
        M = info["manhattan"]
        st = dict(applied="고침 " + "·".join(info["axes_applied"]), skipped="고칠 것 없음", refused="안 고침").get(info["status"], info["status"])
        log(f"[xy_fix] {st} | 키프레임 {N}장"
            + (f" | 벽 방향 {M['yaw_deg']}° · 직각 정도 r4 {M['r4']} · 두 축 안 {M['aligned']} (u {M['share_u']} · v {M['share_v']})" if "r4" in M else "")
            + "".join(f" | {a}: 근거 {s['supported_pct']}% · 보정 5~95% {s['p5_p95_cm']} cm · 잔차 {s['resid_rms_cm']} cm"
                      for a, s in info["axes"].items() if "supported_pct" in s)
            + f" | {info['seconds']['total']} s" + "".join(f"\n  - {m}" for m in info["reasons"] + info["flags"]))
        return out, info

    if poses.shape != (N, 4, 4):
        raise ValueError(f"자세 모양이 {poses.shape} — 시각 {N}개에 맞는 ({N}, 4, 4) 여야 함")
    if N < 2:
        return done(poses.copy(), f"키프레임이 {N}장 — 2장 이상이어야 고칠 수 있음")
    if np.any(np.diff(times) < 0):
        return done(poses.copy(), "키프레임 시각이 차례대로가 아님 — 고치지 않음")
    frames = vertical_points(depth, K, poses, workers=workers, **p)
    clock["observed"] = time.time()
    M = manhattan(frames, N)
    fy = M.pop("frame_yaw")
    ok_fy = np.isfinite(fy)
    if ok_fy.sum() > 10:                                        # 장마다 벽 법선 방향 (참고 — 자세 방향 틀어짐의 직접 측정)
        sm = median_filter(np.where(ok_fy, fy, np.nanmedian(fy)), size=15, mode="nearest")
        M["frame_yaw_deg_p5_p95"] = [round(float(np.percentile(sm[ok_fy], 5)), 2), round(float(np.percentile(sm[ok_fy], 95)), 2)]
        M["frame_yaw_noise_deg"] = round(float(np.median(np.abs(fy[ok_fy] - sm[ok_fy]))), 2)
    M["frames_with_walls_pct"] = round(100 * float(ok_fy.mean()), 1) if N else 0.0
    info["manhattan"] = M
    out = poses.copy()
    # 관문 1: 맨해튼 구조
    if M["r4"] < g["min_r4"] or M["aligned"] < g["min_aligned"] or M["peak2"] > g["max_peak2"]:
        return done(out, f"방의 벽이 서로 직각인 두 방향으로 뚜렷하게 모이지 않음 (직각 정도 r4 {M['r4']} · 기준 {g['min_r4']:g} 이상, "
                         f"두 축 ±6° 안 벽 넓이 {100 * M['aligned']:.0f}% · 기준 {100 * g['min_aligned']:.0f}% 이상, "
                         f"다른 방향 벽 봉우리 {100 * M['peak2']:.0f}% · 기준 {100 * g['max_peak2']:.0f}% 이하) — "
                         "둥글거나 비스듬한 벽 · 어수선한 방은 벽으로 수평을 고치지 않음")
    ob = wall_obs(frames, poses, M["yaw_deg"], **p)
    del frames
    use_axes = [a for a in (0, 1) if M[f"share_{AX[a]}"] >= g["min_axis_share"]]
    for a in (0, 1):
        if a not in use_axes:
            info["axes"][AX[a]] = dict(applied=False, share=M[f"share_{AX[a]}"])
            (info["flags"] if use_axes else info["reasons"]).append(
                f"{_axis_name(a, M)} 벽 넓이가 수직 면의 {100 * M[f'share_{AX[a]}']:.0f}% 뿐 (< {100 * g['min_axis_share']:.0f}%) — 이 축은 고치지 않음")
    use_yaw = bool(p["yaw"]) and len(use_axes) == 2
    X, S, axes_ok, aflags = np.zeros((2, N)), None, [], []
    obu = ob[np.isin(ob[:, 1].astype(int), use_axes)] if len(ob) else ob
    if use_axes and not len(obu):
        info["reasons"].append("벽 칸 관측이 없음 — 벽이 가까이(깊이 한계 안) 충분히 보이지 않음")
    for _attempt in range(2 if len(obu) else 0):
        S = solve_xy(times, N, obu, M["yaw_deg"], **dict(p, yaw=use_yaw))
        if S is None:
            info["reasons"].append("같은 벽을 두 키프레임 이상이 같이 본 곳이 없음 — 고칠 근거가 없음")
            break
        X = S["x"].copy()
        axes_ok, aflags = [], []
        for a in use_axes:
            sup = S["shared"][a] >= g["min_shared"]
            x = X[a] - (np.median(X[a][sup]) if sup.any() else 0.0)
            X[a] = x
            sel = S["used"] & (S["ax"] == a)
            r, wr = S["res"][sel], S["w0"][sel]
            resid = 100 * float(np.sqrt(np.average(r ** 2, weights=wr))) if sel.any() else float("nan")
            b = S["before"][(S["ax"] == a) & (np.abs(S["before"]) < 5 * p["huber"])]
            rng = float(np.percentile(x, 98) - np.percentile(x, 2))
            jump = sup & (np.abs(x - median_filter(x, size=9, mode="nearest")) > g["jump_m"])   # 한 장만 튄 것
            stepped = np.zeros(N)                               # 1 초 안에 jump_m 넘게 건너뜀 (추적이 깨졌다 이어짐)
            for j in range(1, min(11, N)):
                near = times[j:] - times[:-j] <= 1.0
                stepped[:-j] = np.maximum(stepped[:-j], np.where(near, np.abs(x[j:] - x[:-j]), 0))
            jump |= stepped > g["jump_m"]
            gap = _longest_gap(times, sup)
            st = dict(applied=False, share=M[f"share_{AX[a]}"], supported_pct=round(100 * float(sup.mean()), 1), cells=int(S["cells"][a]),
                      observations=int(sel.sum()), observations_dropped=int(((S["ax"] == a) & ~S["used"]).sum()),
                      spread_before_cm=round(100 * float(np.sqrt(np.mean(b ** 2))), 2) if len(b) else None,
                      resid_rms_cm=round(resid, 2) if np.isfinite(resid) else None, resid_med_cm=round(100 * float(np.median(np.abs(r))), 2) if len(r) else None,
                      range_cm=round(100 * rng, 1), max_abs_cm=round(100 * float(np.max(np.abs(x))), 1),
                      p5_p95_cm=[round(100 * float(np.percentile(x, 5)), 1), round(100 * float(np.percentile(x, 95)), 1)],
                      max_unsupported_s=round(gap, 1), jump_frames=np.nonzero(jump)[0].tolist(), reasons=[])
            nm = _axis_name(a, M)
            if sup.mean() < g["min_frames_frac"]:
                st["reasons"].append(f"{nm}: 벽을 다른 키프레임과 {g['min_shared']}칸 넘게 같이 본 키프레임이 {sup.mean():.0%} 뿐 "
                                     f"(< {g['min_frames_frac']:.0%}) — 고칠 근거가 모자람")
            if st["max_abs_cm"] > 100 * g["max_abs_m"]:
                st["reasons"].append(f"{nm}: 수평 보정이 {st['max_abs_cm']} cm (> {100 * g['max_abs_m']:.0f} cm) — 틀어짐이 아니라 추적이 깨진 것으로 봄, 다시 촬영 권함")
            if not np.isfinite(resid):
                st["reasons"].append(f"{nm}: 남은 벽 관측이 없음 — 고칠 근거가 없음")
            elif resid > g["max_resid_cm"]:
                st["reasons"].append(f"{nm}: 고친 뒤에도 벽 자리가 키프레임끼리 {resid:.1f} cm (RMS) 어긋남 (> {g['max_resid_cm']:g} cm) — 이 모델로 못 고침")
            if gap > g["gap_flag_s"]:
                aflags.append(f"{nm}: 벽을 못 본 구간이 최대 {gap:.0f} 초 — 그 사이 보정은 앞뒤를 이은 값")
            if jump.any():
                aflags.append(f"{nm}: 자세가 앞뒤와 {100 * g['jump_m']:.0f} cm 넘게 튀거나 1 초 안에 그만큼 건너뛴 키프레임 {int(jump.sum())}장 "
                              f"(첫 장 {float(times[np.argmax(jump)] - times[0]):.1f} 초, jump_frames) — 추적이 깨졌다 이어진 것으로 보임, 고친 메쉬를 눈으로 확인")
            info["axes"][AX[a]] = st
            if not st["reasons"]:
                axes_ok.append(a)
        if use_yaw:
            psi_deg = np.degrees(S["psi"] - np.median(S["psi"]))
            info["yaw_fit"] = dict(p5_p95_deg=[round(float(np.percentile(psi_deg, 5)), 2), round(float(np.percentile(psi_deg, 95)), 2)],
                                   max_abs_deg=round(float(np.abs(psi_deg).max()), 2), range_deg=round(float(np.ptp(psi_deg)), 2))
            if len(axes_ok) < 2 or info["yaw_fit"]["max_abs_deg"] > g["max_yaw_deg"]:
                info["flags"].append(f"방향(yaw) 보정은 두 축이 모두 통과하고 {g['max_yaw_deg']:g}° 안일 때만 — 빼고 다시 풂")
                use_yaw = False
                continue
        break
    info["flags"].extend(aflags)
    X_est = X.copy()
    for a in (0, 1):
        if a not in axes_ok:
            X[a] = 0.0
        if a in use_axes and a not in axes_ok and AX[a] in info["axes"]:
            (info["flags"] if axes_ok else info["reasons"]).extend(info["axes"][AX[a]].get("reasons", []))
    yaw_used = bool(use_yaw and S is not None and S["psi"] is not None)
    if axes_ok and all(info["axes"][AX[a]]["range_cm"] < 100 * g["noop_range_m"] for a in axes_ok) and \
            not (yaw_used and info["yaw_fit"]["range_deg"] > 0.3):
        info["reasons"].append(f"고칠 것 없음: 수평 보정 폭(2~98%)이 축마다 {100 * g['noop_range_m']:.1f} cm 미만 ("
                               + ", ".join(f"{AX[a]} {info['axes'][AX[a]]['range_cm']}" for a in axes_ok) + " cm)")
        axes_ok = []
        info["status"] = "skipped"
    info["applied"] = bool(axes_ok)
    if info["applied"]:
        info["status"] = "applied"
    info["axes_applied"] = [AX[a] for a in axes_ok]
    for a, st in info["axes"].items():
        st["applied"] = AX.index(a) in axes_ok
    yr = np.radians(M["yaw_deg"])
    U, V = np.array([np.cos(yr), np.sin(yr)]), np.array([-np.sin(yr), np.cos(yr)])
    if info["applied"]:
        Xr = np.round(X, 5)                                     # 0.01 mm 로 반올림한 값을 그대로 적용 — pose_fix.json 으로 다시 적용해도 같은 자세
        dxy = np.round(Xr[0][:, None] * U + Xr[1][:, None] * V, 5)
        info.update(du=Xr[0].tolist(), dv=Xr[1].tolist(), dx=dxy[:, 0].tolist(), dy=dxy[:, 1].tolist(),
                    dyaw_deg=np.round(np.degrees(S["psi"]), 5).tolist() if yaw_used else None)   # yaw 는 가운데 맞추지 않음 — x 와 같이 푼 값 그대로
        sh = np.linalg.norm(dxy, axis=1)
    else:
        info.update(dx=[0.0] * N, dy=[0.0] * N, du=[0.0] * N, dv=[0.0] * N, dyaw_deg=None)
        if S is not None:
            info.update(du_estimate=np.round(X_est[0], 5).tolist(), dv_estimate=np.round(X_est[1], 5).tolist())
        sh = np.zeros(N)
    info["shift_cm_p50_p95"] = [round(100 * float(np.median(sh)), 1), round(100 * float(np.percentile(sh, 95)), 1)]
    return done(apply_fix(times, poses, info))


# ── 차례대로 (높이 → 수평) ───────────────────────────────────────────────

def fix_poses(times, poses, depth, K, steps=FIXES[DEFAULT_FIX], opts=None, log=print, fix=None):
    """보정 단계를 차례로 (높이 → 수평 — 수평 단계는 높이를 고친 자세를 받음) → (고친 자세, 묶음 info — pose_fix.json).
    단계마다 관문이 따로이고, 한 단계가 거부 · 오류여도 다음 단계는 그 앞 자세로 돎 (오류 = status "error", 자세는 그 단계 전 그대로).
    opts = {단계: 설정 dict} (split_opts 꼴), fix = info 에 남길 이름 (없으면 단계를 + 로 이음)"""
    steps = tuple(steps)
    if any(s not in STEPS for s in steps) or list(steps) != [s for s in STEPS if s in steps]:
        raise ValueError(f"보정 단계 {list(steps)} — {list(STEPS)} 중에서 이 차례로 (높이 → 수평)")
    opts = opts or {}
    for kind in opts:
        if kind not in steps:
            raise ValueError(f"보정 설정에 이 보정({'+'.join(steps)})에 없는 단계 '{kind}' 의 설정이 있음")
        check_opts(opts[kind], kind)
    t0 = time.time()
    out = np.array(poses, dtype=float, copy=True)
    infos, sec = [], {}
    for kind in steps:
        ts = time.time()
        try:
            new, info = (floor_fix if kind == "floor" else xy_fix)(times, out, depth, K, log=log, **opts.get(kind, {}))
            out = np.asarray(new, dtype=float)
        except Exception as e:                                  # 한 단계가 깨져도 다른 단계 · 메쉬는 계속
            why = f"{STEP_NAMES[kind]} 보정 중 오류 — 이 단계는 자세를 고치지 않음 ({type(e).__name__}: {e})"
            log(f"[pose_fix] {why}")
            info = not_applied("error", why, kind=kind)
        infos.append(dict(info, kind=kind))
        sec[kind] = round(time.time() - ts, 2)
    sec["total"] = round(time.time() - t0, 2)
    return out, chain_info(infos, fix=fix or "+".join(steps), seconds=sec, keyframes=len(times))


def make_pose_fix(map_root, log=print, fix="floor", **kw):
    """맵 폴더 → (times, poses) → (poses, info) 함수 — stray_mesh.build(pose_fix=…) 에 넣는 꼴.
    fix = "floor" (kw = 높이 설정, 예전과 같음 — info 는 높이 info 하나) · "floor+xy" (kw = split_opts 꼴 — info 는 묶음).
    부를 때 맵 폴더의 키프레임 깊이 · 내부 파라미터를 직접 읽고, 받은 시각으로 키프레임을 짝지음 (짝이 없는 장은 깊이 없이 — 시간 이음으로만)"""
    if fix not in FIXES:
        raise ValueError(f"fix 는 {sorted(FIXES)} 중 하나: {fix!r}")
    if fix == "floor":
        check_opts(kw, "floor")
    else:
        kw = split_opts(FIXES[fix], kw)
    map_root = os.path.abspath(map_root)

    def pose_fix(times, poses):
        from .stray_mesh import read_map                       # 그림이 있는 줄만 · 내부 파라미터 한 종류 — stray_mesh 와 같은 읽기
        fr = read_map(map_root)
        tk, times = fr["times"], np.asarray(times, dtype=float)
        if len(tk) > 1:                                         # 시각이 가장 가까운 키프레임
            j = np.clip(np.searchsorted(tk, times), 1, len(tk) - 1)
            j = np.where(np.abs(tk[j - 1] - times) <= np.abs(tk[j] - times), j - 1, j)
        else:
            j = np.zeros(len(times), dtype=int)
        hit = np.abs(tk[j] - times) < 1e-4
        paths = [fr["depth"][jj] if h else None for jj, h in zip(j, hit)]
        if fix == "floor":
            return floor_fix(times, poses, paths, fr["intrinsic"][2:], log=log, **kw)
        return fix_poses(times, poses, paths, fr["intrinsic"][2:], FIXES[fix], kw, log=log, fix=fix)

    pose_fix.__name__ = "floor_fix" if fix == "floor" else "pose_fix"
    return pose_fix


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("map", help="맵 폴더 (stray.py 출력: keyframes/ · trajectory_camera_optical.txt)")
    ap.add_argument("--fix", choices=sorted(FIXES), default=None,
                    help=f"보정 단계 (기본 {DEFAULT_FIX}). xy = 수평(벽)만 · floor = 높이(바닥)만 · floor+xy = 높이 → 수평")
    ap.add_argument("--tilt", action=argparse.BooleanOptionalAction, default=None,
                    help=f"높이 보정에서 바닥 기울기 오차도 풂 (높이 단계의 기본 {'켬' if DEFAULTS['tilt'] else '끔'}). "
                         f"--fix 없이 주면 높이 단계를 더함 ({DEFAULT_FIX} → {with_floor(DEFAULT_FIX)})")
    ap.add_argument("--yaw", action=argparse.BooleanOptionalAction, default=None,
                    help=f"수평 보정에서 방향(yaw) 오차도 풂 (기본 {'켬' if XY_DEFAULTS['yaw'] else '끔'})")
    ap.add_argument("--json", help="info 를 이 경로에 씀 (맵 · 빌드 폴더의 pose_fix.json 은 건드리지 않음)")
    a = ap.parse_args()
    from . import stray
    from .stray_mesh import read_map
    try:
        fix = tilt_fix(a.fix or DEFAULT_FIX, a.tilt, explicit=a.fix is not None)
    except ValueError as e:
        ap.error(str(e))
    opts = {}
    if a.tilt is not None:
        opts["floor"] = dict(tilt=a.tilt)
    if a.yaw is not None:
        opts["xy"] = dict(yaw=a.yaw)
    try:
        per = split_opts(FIXES[fix], opts)
        fr = read_map(a.map)
    except (ValueError, stray.CaptureError) as e:
        sys.exit(f"[pose_fix] 실패: {e}")
    _, info = fix_poses(fr["times"], fr["poses"], fr["depth"], fr["intrinsic"][2:], FIXES[fix], per, fix=fix)
    print(f"[pose_fix] {fix}: {info['status']} | " + " · ".join(f"{STEP_NAMES[s['kind']]} {s['status']}" for s in info["steps"])
          + f" | {info['seconds'].get('total')} s")
    if a.json:
        json.dump(info, open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)


if __name__ == "__main__":
    main()
