#!/usr/bin/env python3
"""X-Mobility 체크포인트 오프라인(개루프) 평가 — 학습 형식(xm_v1 등) split 의 녹화를 배포 때와 같은 방식으로 한 프레임씩 넣고
예측 명령(v, ω) · 경로를 교사 값과 비교한다.

개루프 ≠ 폐루프: 입력(카메라 · 속도 · 경로)은 교사가 실제로 지나간 상태에서 나온 것이라 모델 오차가 다음 입력에 쌓이지 않는다.
여기 숫자가 좋아도 폐루프 주행(경로 이탈 · 충돌 · 완주)은 따로 시뮬레이터에서 봐야 한다. 모델 비교 · 퇴행 확인 · 약한 구간
(시나리오 · 경로 지시 · 장애물 근처) 찾기에만 쓴다.

배포를 따라 한 것 (X-Mobility 5837aea, 줄 번호는 그 커밋 기준):
  상태      run 파일(= 끊김 없는 한 구간) 시작마다 history · sample 0, 이전 action 0
            (ros2_deployment/.../x_mobility_navigator.py:101-104 · 135-136, model/eval/prediction_evaluator.py:84-88).
            프레임마다 inference 가 돌려준 history · sample 을 다음 프레임에 넣음(navigator.py:267-268).
  이전 명령 기본 = 직전 프레임의 예측 action 6칸 그대로(navigator.py:242 · 265).
            --teacher-forcing = 직전 프레임의 교사 driving_command(prediction_evaluator.py:120, 학습 rssm.py:152-154 와 같은 뜻).
  입력      image (1,1,3,320,512) RGB float32 /255(navigator.py:288-293), 크기가 다를 때만 bilinear(align_corners=False)
            로 INPUT_IMAGE_SIZE 에 맞춤(onnx_conversion.py:53-58) — xm_v1 은 이미 512×320 이라 그대로.
            route (1,1,19,4) — 키 이름은 'route'('route_vectors' 아님: x_mobility.py:150, action_policy.py:210-211).
            앞 20점(모자라면 끝점 반복)의 이웃 두 점 [x0,y0,x1,y1](navigator.py:149-170 = isaac_sim_dataset.py:326-337).
            speed (1,1,1) = ego_speed, action (1,6), history (1,hidden), sample (1,state)(onnx_conversion.py:83-88).
            확산 정책이면 policy_noise (1,16) 를 한 번 뽑아 고정(onnx_conversion.py:41-47 버퍼와 같음, --seed).
  출력      x_mobility.py:175 (action, path, history, sample, …) — action[0] = v, action[5] = ω(navigator.py:272-273),
            path 10 = 5점 × (x, y) 로봇 좌표계(학습 목표 = 데이터 path 열, losses.py:392-396).
  5 Hz      배포 타이머 0.2 s(navigator.py:94) = 데이터 5 Hz → 행 하나에 추론 한 번.
배포와 다른 점: 배포는 goal 이 바뀔 때 history · sample 만 0 으로 하고 action 은 그대로 둠(navigator.py:132-136) — 여기서는
run 시작마다 action 도 0(학습 · prediction_evaluator 와 같음). 배포는 이미지 · odom · route 를 동기화 없이 최신 값으로 씀(navigator.py:217).
가중치는 e2e/finetune.py 와 같은 방식(gin 으로 모델 생성 → state_dict 만 넣고 missing · unexpected 대조, missing 있으면 멈춤).
gin 은 모르는 이름이면 멈추되, 평가에서 import 하지 않는 X-Mobility train.py · evaluate.py 의 train · evaluate_* 와 우리
finetune.py 의 finetune 만 건너뜀(base_train_config.gin:16-26 에 있음). gin 파싱 동안만 X-Mobility 레포로 cd(parse_gin 참고).

지표 (프레임 단위, 평균은 유한값만):
  v MAE · ω MAE                     |예측 − 교사|
  ω 부호 일치                        |ω 교사| > 0.1 rad/s 인 프레임에서 sign(예측) == sign(교사) 비율 (예측 0 은 불일치)
  경로 ADE · FDE (m)                5점(t + k·0.84 s, k = 0..4) 점별 거리의 평균 · 마지막 점(k = 4) 거리. k = 0 포함
묶음: 전체 · 시나리오별 · 경로 지시별(route_command 0 직진 · 1 좌 · 2 우) · 장애물 근처(obstacle_dist < 1.5 m).
분석 열(acqtime · route_command · obstacle_dist · cte · teacher_noise)이 없거나 NaN 이면 그 묶음에서만 빠짐.

출력(-o 폴더): frames.csv(프레임별) · summary.json(묶음별 지표) · run_meta.json(이 레포 · X-Mobility 커밋, ckpt sha256 12자리,
gin, 명령, 키 대조, 프레임/초) · gin_operative.txt. summary.json 이 이미 있으면 멈춤(--overwrite).
frames.csv 끝의 위치 열(route_s · map_x · map_y · map_yaw(map_pose 3칸) · lap)은 지표에 안 쓰고 기록만 함 — 기준선 구간별 약한
곳 찾기(e2e/weak_segments.py)용. 데이터에 없거나 map_pose 길이가 3 이 아니면 NaN.

사용(server207 등 X-Mobility 의존성이 깔린 python, GPU):
  python e2e/eval_offline.py --xm-repo ~/xmobility_test/X-Mobility \\
      -c e2e/configs/finetune_parking_r3.gin -d data/teacher/xm_v1 \\
      -p data/checkpoints/xm_ft_<버전>/checkpoints/last.ckpt -o logs/xm_eval_<버전>
  제로샷 비교: 같은 명령에 -p <x_mobility-nav2-semantic_action_path.ckpt> (semantic_decoder 가 unexpected 로 나오는 것은 정상).
  시험: --max-files 2 --device cpu. 교사 명령을 이전 명령으로: --teacher-forcing.
지표 함수 · 파일 순회는 numpy · pandas 만 씀(X-Mobility · torch import 는 main() 안) — 모델 없이 시험 가능.
"""
import argparse
import datetime
import hashlib
import io
import json
import os
# transformers 가 TensorFlow 를 안 불러오게(USE_TF=0). Kaggle 이미지의 TF 2.21 은 protobuf ≥6.31 인데
# wandb==0.19.4 가 protobuf 를 5.29 로 내려서, TF 를 불러오면 diffusers→transformers.clip import 에서 VersionError.
os.environ.setdefault("USE_TF", "0")
import subprocess
import sys
import time
import unicodedata

import numpy as np
import pandas as pd

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
SPLITS = ("train", "val", "test")
REQUIRED = ("driving_command", "ego_speed", "path", "camera_image", "route_poses")   # isaac_sim_dataset.py:47-49
ANALYSIS = ("acqtime", "route_command", "obstacle_dist", "cte", "teacher_noise")      # sim/export_xmobility.py:248-262
ACTION_SIZE, PATH_SIZE = 6, 10
PATH_SCALE = 1.0                # 데이터 path 배수(scale_path_xm.py) — main 에서 정함
V_IDX, W_IDX = 0, 5               # linear x · angular z (x_mobility_metrics.py:86-87, navigator.py:272-273)
NUM_ROUTE_POINTS = 20             # navigator.py:41
ROUTE_POSE_SIZE = 2               # isaac_sim_dataset.py:45
EXPECTED_ROUTE_LEN = NUM_ROUTE_POINTS * ROUTE_POSE_SIZE
W_SIGN_THR = 0.1                  # rad/s — 이보다 작은 교사 ω 는 부호 일치에서 뺌
NEAR_OBS_M = 1.5
ROUTE_CMD = {0: "straight", 1: "left", 2: "right"}
ROUTE_CMD_KO = {0: "직진", 1: "좌", 2: "우"}
# 사전학습 ckpt 에는 있지만 ENABLE_SEMANTIC=False 설정에는 없는 모듈 — unexpected 로 나와도 정상(e2e/finetune.py 와 같음)
EXPECTED_UNEXPECTED = ("model.semantic_decoder.",)
# gin 에서 건너뛸 configurable — 평가에서 import 하지 않는 train.py:31 · evaluate.py:29,52 와 e2e/finetune.py 의 finetune
GIN_SKIP = ("train", "evaluate_observation", "evaluate_prediction", "finetune")
LOCATION = ("route_s", "lap")                                                          # sim/export_xmobility.py:256-261
COLUMNS = ["split", "scenario", "file", "idx", "acqtime", "route_command", "obstacle_dist", "cte", "teacher_noise",
           "ego_speed", "v_teacher", "w_teacher", "v_pred", "w_pred", "ade_m", "fde_m",
           "route_s", "map_x", "map_y", "map_yaw", "lap"]


# ---------------------------------------------------------------- 입력 만들기 (numpy)

def route_vectors(route_poses):
    """route_poses(평평한 x, y …) → (19, 4) float32 [x_i, y_i, x_i+1, y_i+1].
    앞 20점, 모자라면 끝점 반복(navigator.py:149-170). 20점(40값)이면 학습 로더(isaac_sim_dataset.py:326-337)와 같다."""
    p = np.asarray(route_poses, np.float32).reshape(-1, ROUTE_POSE_SIZE)
    if len(p) == 0:
        raise ValueError("route_poses 가 비어 있음")
    idx = list(range(min(len(p), NUM_ROUTE_POINTS)))
    idx += [idx[-1]] * (NUM_ROUTE_POINTS - len(idx))
    sel = p[idx]
    return np.ascontiguousarray(np.concatenate([sel[:-1], sel[1:]], axis=1), dtype=np.float32)


def decode_image(jpeg):
    """JPEG bytes → (3, H, W) float32 [0, 1] — navigator.py:288-293 과 같은 연산(HWC→CHW, float32 로 바꾼 뒤 /255)."""
    from PIL import Image
    im = Image.open(io.BytesIO(jpeg))
    if im.mode != "RGB":
        im = im.convert("RGB")
    return np.ascontiguousarray(np.asarray(im).transpose(2, 0, 1).astype(np.float32) / 255.0)


# ---------------------------------------------------------------- 지표 (numpy)

def path_ade_fde(pred, gt):
    """(N, 10) 평평한 5점 경로 둘 → (ADE (N,), FDE (N,)) m. 점별 유클리드 거리의 평균 · 마지막 점. NaN 이 낀 행은 NaN."""
    pred, gt = np.asarray(pred, np.float64), np.asarray(gt, np.float64)
    if pred.shape != gt.shape or pred.ndim != 2 or pred.shape[1] % 2:
        raise ValueError(f"경로 모양이 다름/틀림: {pred.shape} vs {gt.shape}")
    if len(gt) == 0:
        return np.zeros(0), np.zeros(0)
    d = np.linalg.norm(pred.reshape(len(gt), -1, 2) - gt.reshape(len(gt), -1, 2), axis=-1)
    return d.mean(axis=1), d[:, -1]


def finite_mean(x):
    x = np.asarray(x, np.float64)
    x = x[np.isfinite(x)]
    return float(x.mean()) if x.size else None


def metrics_block(v_t, w_t, v_p, w_p, ade, fde, w_thr=W_SIGN_THR):
    """한 묶음의 지표 dict. 값이 하나도 없으면 None(JSON null)."""
    v_t, w_t, v_p, w_p = (np.asarray(x, np.float64) for x in (v_t, w_t, v_p, w_p))
    m = np.isfinite(w_t) & np.isfinite(w_p) & (np.abs(w_t) > w_thr)
    agree = np.sign(w_p[m]) == np.sign(w_t[m])
    return dict(n=int(len(v_t)), v_mae=finite_mean(np.abs(v_p - v_t)), w_mae=finite_mean(np.abs(w_p - w_t)),
                w_sign_agree=float(agree.mean()) if m.any() else None, n_w_sign=int(m.sum()),
                ade_m=finite_mean(ade), fde_m=finite_mean(fde))


def summarize(frames, near_m=NEAR_OBS_M, w_thr=W_SIGN_THR):
    """프레임 표(COLUMNS) → 전체 · 시나리오별 · 경로 지시별 · 장애물 근처 지표."""
    def blk(df):
        return metrics_block(df["v_teacher"], df["w_teacher"], df["v_pred"], df["w_pred"], df["ade_m"], df["fde_m"], w_thr)

    rc = pd.to_numeric(frames["route_command"], errors="coerce").astype("float64")
    od = pd.to_numeric(frames["obstacle_dist"], errors="coerce").astype("float64")
    by_rc = {}
    for c in sorted({int(x) for x in rc.dropna()}):
        by_rc[f"{c}_{ROUTE_CMD.get(c, 'unknown')}"] = blk(frames[rc == c])
    near = blk(frames[od < near_m])
    return dict(overall=blk(frames),
                by_scenario={s: blk(g) for s, g in frames.groupby("scenario", sort=True)},
                by_route_command=by_rc, route_command_missing=int(rc.isna().sum()),
                near_obstacle=dict(threshold_m=near_m, n_obstacle_dist_known=int(od.notna().sum()), **near))


def _w(s):
    return sum(2 if unicodedata.east_asian_width(ch) in "WF" else 1 for ch in s)


def _pad(s, width):
    return s + " " * max(0, width - _w(s))


def format_table(summary):
    def f(x, nd=3):
        return "-" if x is None else f"{x:.{nd}f}"

    rows = [("전체", summary["overall"])]
    rows += [(f"시나리오 {k}", v) for k, v in summary["by_scenario"].items()]
    for k, v in summary["by_route_command"].items():
        c = int(k.split("_")[0])
        rows.append((f"지시 {c} {ROUTE_CMD_KO.get(c, '?')}", v))
    no = summary["near_obstacle"]
    rows.append((f"장애물 < {no['threshold_m']:g} m", no))
    lw = max(_w(r[0]) for r in rows) + 2
    lines = [_pad("구분", lw) + f"{'N':>7s} {'v MAE':>7s} {'ω MAE':>7s} {'ω 부호':>7s} {'(n)':>7s} {'ADE m':>7s} {'FDE m':>7s}"]
    for name, m in rows:
        lines.append(_pad(name, lw) + f"{m['n']:7d} {f(m['v_mae']):>7s} {f(m['w_mae']):>7s} "
                     f"{f(m['w_sign_agree'], 2):>7s} {m['n_w_sign']:7d} {f(m['ade_m']):>7s} {f(m['fde_m']):>7s}")
    if summary.get("route_command_missing"):
        lines.append(f"(route_command 없는 프레임 {summary['route_command_missing']} — 지시별 묶음에서 빠짐)")
    return "\n".join(lines)


# ---------------------------------------------------------------- 파일 순회 (pandas)

def list_run_files(root, split, max_files=None):
    """<root>/<split>/<scenario>/*.pqt → [(scenario, 경로)] — 시나리오 · 파일 이름 순. max_files 는 전체 개수 상한."""
    sp = os.path.join(root, split)
    if not os.path.isdir(sp):
        raise FileNotFoundError(f"{split}/ 없음: {sp}")
    out = []
    for scen in sorted(os.listdir(sp)):
        p = os.path.join(sp, scen)
        if os.path.isdir(p):
            out += [(scen, os.path.join(p, f)) for f in sorted(os.listdir(p)) if f.endswith("pqt")]
    return out if max_files is None else out[:max_files]


def load_run(path):
    """run 파일 하나 → dict(n, images, action (n,6), path (n,10), route [n], speed (n,), 분석 열 (n,) — 없으면 NaN)."""
    df = pd.read_parquet(path, engine="pyarrow")
    miss = [c for c in REQUIRED if c not in df.columns]
    if miss:
        raise ValueError(f"{path}: 필수 열 없음 {miss}")
    n = len(df)

    def stack(col, k):
        rows = [np.asarray(x, np.float64).reshape(-1) for x in df[col]]
        bad = [i for i, r in enumerate(rows) if r.size != k]
        if bad:
            raise ValueError(f"{path}: {col} 길이 {k} 아닌 행 {len(bad)}개 (첫 행 {bad[0]})")
        return np.stack(rows) if rows else np.zeros((0, k))

    run = dict(n=n, images=df["camera_image"].tolist(), action=stack("driving_command", ACTION_SIZE),
               path=stack("path", PATH_SIZE), route=[np.asarray(x, np.float64).reshape(-1) for x in df["route_poses"]],
               speed=pd.to_numeric(df["ego_speed"], errors="coerce").to_numpy(np.float64))
    # 필수 수치 열의 NaN 은 이어 받는 상태(history · 이전 명령)를 그 run 끝까지 오염시킴 → 조용히 넘기지 않고 멈춤
    bad = [c for c, ok in (("driving_command", np.isfinite(run["action"]).all()), ("path", np.isfinite(run["path"]).all()),
                           ("ego_speed", np.isfinite(run["speed"]).all()),
                           ("route_poses", all(np.isfinite(r).all() for r in run["route"]))) if not ok]
    if bad:
        raise ValueError(f"{path}: {bad} 에 NaN/Inf — python3 e2e/check_xm_dataset.py 로 먼저 확인")
    for c in ANALYSIS + LOCATION:
        run[c] = (pd.to_numeric(df[c], errors="coerce").to_numpy(np.float64) if c in df.columns
                  else np.full(n, np.nan))
    pose = np.full((n, 3), np.nan)                       # map_pose = (x, y, yaw) — 기록용이라 틀린 행은 멈추지 않고 NaN
    if "map_pose" in df.columns:
        for i, x in enumerate(df["map_pose"]):
            r = np.asarray(x if x is not None else [], np.float64).reshape(-1)
            if r.size == 3:
                pose[i] = r
    run["map_pose"] = pose
    return run


def run_eval(files, predictor, split, teacher_forcing=False, log=print):
    """files 를 차례로 predictor 에 넣음. predictor: reset() · step(image, speed, route_vec, prev_action) → (action6, path10).
    run 파일마다 reset · 이전 명령 0. 이전 명령 = 직전 예측(기본) 또는 직전 교사 명령(teacher_forcing).
    → (프레임 표 DataFrame[COLUMNS], 통계 dict)"""
    parts = []
    stats = dict(files=0, frames=0, route_len_mismatch=0)
    for k, (scenario, path) in enumerate(files):
        run = load_run(path)
        n = run["n"]
        predictor.reset()
        prev = np.zeros(ACTION_SIZE, np.float32)
        pred_a = np.full((n, ACTION_SIZE), np.nan)
        pred_p = np.full((n, PATH_SIZE), np.nan)
        for i in range(n):
            rp = run["route"][i]
            if rp.size != EXPECTED_ROUTE_LEN:
                stats["route_len_mismatch"] += 1
            a, p = predictor.step(decode_image(run["images"][i]), float(run["speed"][i]), route_vectors(rp), prev)
            pred_a[i] = np.asarray(a, np.float64).reshape(-1)
            pred_p[i] = np.asarray(p, np.float64).reshape(-1)
            prev = (run["action"][i] if teacher_forcing else pred_a[i]).astype(np.float32)
        ade, fde = path_ade_fde(pred_p, run["path"])
        ade, fde = ade / PATH_SCALE, fde / PATH_SCALE          # 축척 데이터면 m 로 되돌림
        rc, lap = run["route_command"], run["lap"]
        part = pd.DataFrame(dict(
            split=split, scenario=scenario, file=os.path.basename(path), idx=np.arange(n),
            acqtime=run["acqtime"],
            route_command=pd.array(np.where(np.isfinite(rc), np.round(rc), np.nan), dtype="Int64"),
            obstacle_dist=run["obstacle_dist"], cte=run["cte"], teacher_noise=run["teacher_noise"], ego_speed=run["speed"],
            v_teacher=run["action"][:, V_IDX], w_teacher=run["action"][:, W_IDX],
            v_pred=pred_a[:, V_IDX], w_pred=pred_a[:, W_IDX], ade_m=ade, fde_m=fde,
            route_s=run["route_s"], map_x=run["map_pose"][:, 0], map_y=run["map_pose"][:, 1], map_yaw=run["map_pose"][:, 2],
            lap=pd.array(np.where(np.isfinite(lap), np.round(lap), np.nan), dtype="Int64")), columns=COLUMNS)
        parts.append(part)
        stats["files"] += 1
        stats["frames"] += n
        m = metrics_block(part["v_teacher"], part["w_teacher"], part["v_pred"], part["w_pred"], ade, fde)
        fm = lambda x: "-" if x is None else f"{x:.3f}"  # noqa: E731
        log(f"[{k + 1}/{len(files)}] {scenario}/{os.path.basename(path)} 행 {n} · v MAE {fm(m['v_mae'])} · "
            f"ω MAE {fm(m['w_mae'])} · ADE {fm(m['ade_m'])}")
    frames = pd.concat(parts, ignore_index=True) if parts else pd.DataFrame(columns=COLUMNS)
    return frames, stats


def write_outputs(out, frames, summary, meta, gin_operative=None):
    os.makedirs(out, exist_ok=True)
    frames.to_csv(os.path.join(out, "frames.csv"), index=False)
    with open(os.path.join(out, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=1)
    with open(os.path.join(out, "run_meta.json"), "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=1, default=str)
    if gin_operative is not None:
        with open(os.path.join(out, "gin_operative.txt"), "w", encoding="utf-8") as f:
            f.write(gin_operative)


# ---------------------------------------------------------------- 모델 (torch · X-Mobility — main 에서만 씀)

class XMobilityStepper:
    """X-Mobility 한 프레임 추론 = onnx_conversion.py:49-69 (XMobilityInference.forward) + navigator.py:225-268 상태 이어 받기."""

    def __init__(self, xm_model, device, image_size, policy_noise=None):
        import torch
        import torch.nn.functional as F
        self.torch, self.F = torch, F
        self.model, self.device = xm_model, device
        self.size = tuple(int(s) for s in image_size)
        self.hidden_dim = int(xm_model.rssm.hidden_state_dim)   # 1024 (base_train_config.gin:100)
        self.state_dim = int(xm_model.rssm.state_dim)           # 512 (base_train_config.gin:101)
        self.policy_noise = policy_noise
        self.resized = 0
        self.reset()

    def reset(self):
        t = self.torch
        self.history = t.zeros((1, self.hidden_dim), dtype=t.float32, device=self.device)
        self.sample = t.zeros((1, self.state_dim), dtype=t.float32, device=self.device)

    def step(self, image, speed, route_vec, prev_action):
        t, dev = self.torch, self.device
        img = t.from_numpy(image).to(dev).unsqueeze(0)                       # (1, 3, H, W)
        if tuple(img.shape[-2:]) != self.size:
            img = self.F.interpolate(img, size=self.size, mode="bilinear", align_corners=False)
            self.resized += 1
        batch = {
            "image": img.unsqueeze(0),                                                 # (1, 1, 3, 320, 512)
            "route": t.tensor(route_vec, dtype=t.float32, device=dev).view(1, 1, *route_vec.shape),  # (1, 1, 19, 4)
            "speed": t.tensor([[[speed]]], dtype=t.float32, device=dev),               # (1, 1, 1)
            "action": t.tensor(np.asarray(prev_action, np.float32).reshape(1, -1), device=dev),      # (1, 6)
            "history": self.history,                                                   # (1, 1024)
            "sample": self.sample,                                                     # (1, 512)
        }
        if self.policy_noise is not None:
            batch["policy_noise"] = self.policy_noise
        with t.inference_mode():
            out = self.model.inference(batch, enable_semantic_inference=False, enable_rgb_inference=False,
                                       enable_depth=False)
        action, path, history, sample = out[:4]
        if action is None or path is None:
            raise RuntimeError("inference 가 action 을 안 돌려줌 — IS_GWM_PRETRAIN 이거나 'route' 키 누락(x_mobility.py:150)")
        self.history, self.sample = history, sample
        return action.reshape(-1).float().cpu().numpy(), path.reshape(-1).float().cpu().numpy()


def parse_gin(gin, xm, cfgs, bindings):
    """include 'configs/base_train_config.gin' 을 X-Mobility 레포 기준으로 찾게 파싱하는 동안만 그 레포로 cd.
    gin 0.5.0(X-Mobility requirements.txt:3)은 include 를 (1) 현재 폴더 (2) 'configs' 를 파이썬 패키지로 찾기 (3) 검색 경로 순으로
    찾는데, X-Mobility 레포 · e2e/ 가 sys.path 에 있으면 (2)에서 configs 가 namespace 패키지(origin None)로 잡혀
    gin/resource_reader.py:89 가 TypeError 로 죽는다 — (1)에서 찾게 하면 (2)까지 안 감."""
    cfgs = [os.path.abspath(c) for c in cfgs]
    gin.add_config_file_search_path(xm)
    cwd = os.getcwd()
    os.chdir(xm)
    try:
        gin.parse_config_files_and_bindings(cfgs, bindings, skip_unknown=list(GIN_SKIP))
    finally:
        os.chdir(cwd)


def load_weights(torch, module, ckpt, allow_missing):
    """e2e/finetune.py 와 같은 대조. missing 이 있으면(무작위 가중치로 평가하게 되므로) 멈춤."""
    sd = torch.load(ckpt, map_location="cpu", weights_only=False)
    hp = sd.get("hyper_parameters", {}) if isinstance(sd, dict) else {}
    sd = sd.get("state_dict", sd)
    res = module.load_state_dict(sd, strict=False)   # 모양이 다른 키는 여기서 RuntimeError
    unexp_other = [k for k in res.unexpected_keys if not k.startswith(EXPECTED_UNEXPECTED)]
    keys = dict(loaded=len(sd) - len(res.unexpected_keys), missing=list(res.missing_keys),
                unexpected_expected=len(res.unexpected_keys) - len(unexp_other), unexpected_other=unexp_other,
                ckpt_hparams={k: hp[k] for k in ("lr", "weight_decay", "scheduler_pct_start") if k in hp})
    print(f"[ckpt] {ckpt}\n  넣은 키 {keys['loaded']} · missing {len(res.missing_keys)} · "
          f"unexpected {len(res.unexpected_keys)}(그중 의미 분할 등 예상분 {keys['unexpected_expected']})")
    for k in res.missing_keys[:20]:
        print(f"  missing: {k}")
    for k in unexp_other[:20]:
        print(f"  unexpected(예상 밖): {k}")
    if res.missing_keys and not allow_missing:
        sys.exit("ckpt 에 없는 가중치가 있음(무작위로 남음) — gin 설정이 ckpt 학습 때와 다른지 확인. 알고도 진행하려면 --allow-missing")
    return keys


def git_rev(path):
    try:
        rev = subprocess.check_output(["git", "-C", path, "rev-parse", "--short=12", "HEAD"], text=True).strip()
        dirty = subprocess.call(["git", "-C", path, "diff", "--quiet", "HEAD"]) != 0
        return rev + ("-dirty" if dirty else "")
    except Exception:  # noqa: BLE001 — git 이 없거나 레포가 아니면 기록만 비움
        return None


def sha12(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:12]


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="X-Mobility 오프라인(개루프) 평가 (E)")
    ap.add_argument("--xm-repo", default=os.environ.get("XM_REPO"), help="X-Mobility 레포 경로(또는 XM_REPO)")
    ap.add_argument("-c", "--config-files", nargs="+", required=True)
    ap.add_argument("-d", "--dataset-path", required=True)
    ap.add_argument("--split", choices=SPLITS, default="test")
    ap.add_argument("-p", "--checkpoint-path", required=True)
    ap.add_argument("-o", "--output-dir", required=True)
    ap.add_argument("--device", default=None, help="cuda · cuda:1 · cpu (기본: cuda 가 되면 cuda)")
    ap.add_argument("--max-files", type=int, default=None, help="시나리오 · 파일 이름 순으로 앞 N 개 run 파일만")
    ap.add_argument("--teacher-forcing", action="store_true", help="이전 명령 입력으로 직전 교사 명령을 넣음(기본: 직전 예측)")
    ap.add_argument("--gin", action="append", default=[], help="gin 바인딩 덧붙이기(여러 번)")
    ap.add_argument("--allow-missing", action="store_true", help="ckpt 에 없는 가중치가 있어도 진행")
    ap.add_argument("--seed", type=int, default=0, help="확산 정책일 때 고정 policy_noise 시드(MLP 정책이면 무관)")
    ap.add_argument("--path-scale", type=float, default=None,
                    help="데이터의 path 에 곱해진 배수(scale_path_xm.py 의 PATH_SCALE.json). ADE · FDE 를 이 값으로 나눠 m 로. "
                         "기본: 데이터 폴더에 PATH_SCALE.json 이 있으면 그 scale, 없으면 1")
    ap.add_argument("--overwrite", action="store_true", help="출력 폴더에 summary.json 이 있어도 덮어씀")
    return ap.parse_args(argv)


def main():
    a = parse_args()
    global PATH_SCALE
    _ps = os.path.join(a.dataset_path, "PATH_SCALE.json")
    PATH_SCALE = a.path_scale if a.path_scale is not None else (
        float(json.load(open(_ps, encoding="utf-8"))["scale"]) if os.path.isfile(_ps) else 1.0)
    if PATH_SCALE != 1.0:
        print(f"[경로 축척] path × {PATH_SCALE:.4f} 데이터 — ADE · FDE 는 /{PATH_SCALE:.4f} 해서 m 로 보고")
    if not a.xm_repo or not os.path.isfile(os.path.join(os.path.expanduser(a.xm_repo), "train.py")):
        sys.exit("--xm-repo(또는 XM_REPO)에 X-Mobility 레포 경로를 주세요 (train.py 가 있는 폴더)")
    xm = os.path.abspath(os.path.expanduser(a.xm_repo))
    cfgs = [os.path.abspath(c) for c in a.config_files]
    data = os.path.abspath(a.dataset_path)
    out = os.path.abspath(a.output_dir)
    ckpt = os.path.abspath(a.checkpoint_path)
    if not os.path.isfile(ckpt):
        sys.exit(f"ckpt 없음: {ckpt}")
    try:
        files = list_run_files(data, a.split, a.max_files)
    except FileNotFoundError as e:
        sys.exit(f"{e} — 먼저 python3 e2e/check_xm_dataset.py {data}")
    if not files:
        sys.exit(f"{a.split}/ 에 .pqt 없음: {data}")
    if os.path.exists(os.path.join(out, "summary.json")) and not a.overwrite:
        sys.exit(f"이미 결과가 있음: {out}/summary.json — 덮어쓰려면 --overwrite")

    sys.path.insert(0, xm)
    import gin
    import torch
    # gin 이 바인딩을 알아보도록 X-Mobility 모듈을 먼저 import (train.py:26-28 과 같은 순서)
    from model.dataset.isaac_sim_dataset import XMobilityIsaacSimDataModule  # noqa: F401
    from model.dataset.data_constants import INPUT_IMAGE_SIZE
    from model.trainer import XMobilityTrainer

    parse_gin(gin, xm, cfgs, a.gin)
    device = a.device or ("cuda" if torch.cuda.is_available() else "cpu")
    torch.manual_seed(a.seed)
    trainer = XMobilityTrainer()               # gin 값으로 생성(ImageDINOEncoder 가 HF facebook/dinov2-small 을 받음)
    keys = load_weights(torch, trainer, ckpt, a.allow_missing)
    model = trainer.model.to(device).eval()
    if model.is_gwm_pretrain:
        sys.exit("IS_GWM_PRETRAIN=True — action_policy 가 없어 명령을 못 냄(x_mobility.py:67-68)")
    noise = None
    diffusion = bool(model.action_policy.enable_policy_diffusion)
    if diffusion:                              # onnx_conversion.py:41-47 — 한 번 뽑아 고정
        g = torch.Generator().manual_seed(a.seed)
        noise = torch.randn((1, model.action_policy.policy_diffuser.num_input_channels()), generator=g,
                            dtype=torch.float32).to(device)
    stepper = XMobilityStepper(model, device, INPUT_IMAGE_SIZE, noise)
    print(f"[평가] {a.split} run {len(files)}개 · device {device} · 이전 명령 "
          f"{'교사(teacher forcing)' if a.teacher_forcing else '직전 예측'} · 정책 {'확산' if diffusion else 'MLP'}")

    meta = dict(started=datetime.datetime.now().isoformat(timespec="seconds"), argv=sys.argv,
                repo_rev=git_rev(REPO), xm_repo=xm, xm_rev=git_rev(xm), configs=cfgs, gin_extra=a.gin,
                gin_skip=list(GIN_SKIP), dataset=data, split=a.split, max_files=a.max_files,
                checkpoint=ckpt, checkpoint_sha12=sha12(ckpt), keys=keys, device=device,
                cuda_device=torch.cuda.get_device_name(device) if str(device).startswith("cuda") else None,
                teacher_forcing=a.teacher_forcing, policy_diffusion=diffusion, seed=a.seed,
                input_image_size=list(INPUT_IMAGE_SIZE), torch=torch.__version__)
    t0 = time.perf_counter()
    frames, stats = run_eval(files, stepper, a.split, a.teacher_forcing)
    elapsed = time.perf_counter() - t0
    summary = summarize(frames)
    summary["setup"] = dict(split=a.split, files=stats["files"], frames=stats["frames"], teacher_forcing=a.teacher_forcing,
                            checkpoint_sha12=meta["checkpoint_sha12"], w_sign_threshold=W_SIGN_THR,
                            path="ADE = 5점(k=0..4, t+k·0.84 s) 거리 평균, FDE = k=4 거리, m")
    meta.update(finished=datetime.datetime.now().isoformat(timespec="seconds"), elapsed_s=round(elapsed, 2),
                frames_per_s=round(stats["frames"] / elapsed, 2) if elapsed > 0 else None,
                images_resized=stepper.resized, **stats)
    write_outputs(out, frames, summary, meta, gin.operative_config_str())
    print(format_table(summary))
    if stats["route_len_mismatch"]:
        print(f"경고: route_poses 길이 {EXPECTED_ROUTE_LEN} 아닌 프레임 {stats['route_len_mismatch']}개(앞 20점 · 끝점 반복으로 맞춤)")
    if stepper.resized:
        print(f"경고: 512×320 이 아닌 이미지 {stepper.resized}장을 다시 줄임")
    print(f"[끝] 프레임 {stats['frames']} · {meta['frames_per_s']} 프레임/초 · 기록 {out}/(frames.csv · summary.json · run_meta.json)")


if __name__ == "__main__":
    main()
