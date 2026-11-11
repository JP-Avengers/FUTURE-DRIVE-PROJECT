#!/usr/bin/env python3
"""r3 무작위 장애물 배치 시나리오 생성 — 장애물 교사 보강(11.05, D).

11.04 폐루프에서 파인튜닝 X-Mobility 가 경로 한가운데 상자(south_box cw)에 그대로 부딪힘. 학습 데이터의 장애물 장면이
Nav2 교사 16회(시나리오마다 1회, 장애물 1~2개)뿐이라, 자리 · 옆 오프셋 · 종류 · 크기 · 색을 무작위로 바꾼 배치를 많이 만들어
Nav2 교사로 녹화함. 한 바퀴에 장애물 4개(직선 구간마다 하나씩 넣되 구간은 무작위로 고름).

  python3 eval/scenarios/gen_rand_obstacles.py --prefix tr --n 20 --seed 1000   # 학습용(녹화)
  python3 eval/scenarios/gen_rand_obstacles.py --prefix ev --n 6  --seed 9000   # 평가용(녹화 안 함)
  → eval/scenarios/rand/parking_r3_rand_<prefix>_NN.yaml

자리 규칙(10.14 시나리오 결과 바탕):
- s 창 = 직선 구간에서 모서리 앞뒤를 뺀 곳: 동 4.5~8.5 · 남 20~37 · 서 사선 53~58.5 · 북 67~82(시계 방향 s, 한 바퀴 90.0 m).
  모서리 직후(corner_exit s 17.5)는 Nav2 가 2.0 m 한도 근처라 뺌.
- 장애물 사이 9 m 이상(앞 장애물을 피하고 기준선에 돌아올 거리).
- 옆 오프셋: 절반은 가운데(|lateral| ≤ 0.15, 좌우를 골라야 하는 경우 — 11.04 충돌 유형), 나머지는 ±0.3~0.7.
- 종류: 상자 50 % · 원기둥(사람 대역) 25 % · 라바콘 25 %, 크기 · 색 무작위.
"""
import argparse, os, random

WINDOWS = [("east", 4.5, 8.5), ("south", 20.0, 37.0), ("west", 53.0, 58.5), ("north", 67.0, 82.0)]
MIN_GAP = 9.0

HEAD = """# 실제 주차장 r3 — 무작위 장애물 배치 {name} (11.05, D, eval/scenarios/gen_rand_obstacles.py --seed {seed}).
# 장애물 자리 = 기준선 길이 s(시계 방향) · lateral(시계 방향 진행의 왼쪽 + — 화단 반대쪽, − 는 화단 쪽).
# 용도: {use}
name: {name}
version: v1
scene: sim/scenes/parking/parking_v0.8_r3.usda
route: eval/routes/parking_r3_cw.csv
directions: [cw, ccw]
start: {{x: -1.756, y: -7.621, yaw_deg: -96.0}}
goal: {{lap: 1, radius_m: 1.0}}
obstacles:
"""
TAIL = """deviation_limit_m: 2.0
stall_limit_s: 10
timeout_s: 300
"""


def jitter(rng, base, d=0.08):
    return [round(min(1.0, max(0.0, c + rng.uniform(-d, d))), 2) for c in base]


def obstacle(rng, i, s):
    lat = rng.uniform(-0.15, 0.15) if rng.random() < 0.5 else rng.choice([-1, 1]) * rng.uniform(0.3, 0.7)
    r = rng.random()
    if r < 0.5:
        w, d, h = rng.uniform(0.4, 0.8), rng.uniform(0.4, 0.8), rng.uniform(0.5, 1.0)
        base = rng.choice([[0.55, 0.38, 0.22], [0.35, 0.35, 0.38], [0.80, 0.75, 0.65], [0.20, 0.35, 0.25]])
        return (f"  - {{name: box_{i}, type: box, s: {s:.2f}, lateral: {lat:.2f}, size: [{w:.2f}, {d:.2f}, {h:.2f}], "
                f"yaw_deg: {rng.uniform(0, 90):.1f}, color: {jitter(rng, base)}}}\n")
    if r < 0.75:
        base = rng.choice([[0.20, 0.30, 0.55], [0.55, 0.15, 0.15], [0.15, 0.15, 0.15], [0.85, 0.85, 0.80]])
        return (f"  - {{name: ped_{i}, type: cylinder, s: {s:.2f}, lateral: {lat:.2f}, "
                f"size: [{rng.uniform(0.4, 0.6):.2f}, {rng.uniform(1.5, 1.85):.2f}], color: {jitter(rng, base)}}}\n")
    return (f"  - {{name: cone_{i}, type: cone, s: {s:.2f}, lateral: {lat:.2f}, "
            f"size: [{rng.uniform(0.35, 0.45):.2f}, {rng.uniform(0.6, 0.8):.2f}], color: {jitter(rng, [0.95, 0.40, 0.05], 0.05)}}}\n")


def layout(rng, k):
    for _ in range(1000):
        ss = sorted(rng.uniform(a, b) for _, a, b in rng.sample(WINDOWS, k))
        gaps = [b - a for a, b in zip(ss, ss[1:])] + [ss[0] + 90.0 - ss[-1]]
        if min(gaps) >= MIN_GAP:
            return ss
    raise RuntimeError("배치 실패")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True)
    ap.add_argument("--n", type=int, required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--k", type=int, default=4, help="한 바퀴 장애물 수(구간 수 4 이하)")
    ap.add_argument("--outdir", default="eval/scenarios/rand")
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    use = "학습(Nav2 교사 녹화)" if a.prefix == "tr" else "평가(폐루프, 학습에 안 씀)"
    for j in range(a.n):
        seed = a.seed + j
        rng = random.Random(seed)
        name = f"parking_r3_rand_{a.prefix}_{j:02d}"
        body = "".join(obstacle(rng, i, s) for i, s in enumerate(layout(rng, a.k)))
        path = os.path.join(a.outdir, name + ".yaml")
        with open(path, "w") as f:
            f.write(HEAD.format(name=name, seed=seed, use=use) + body + TAIL)
        print(path)


if __name__ == "__main__":
    main()
