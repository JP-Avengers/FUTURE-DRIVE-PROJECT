#!/usr/bin/env python3
"""무작위 장애물 배치 시나리오 생성 — 장애물 교사 보강(11.05, D). 장소 값은 eval/sites/<장소>.yaml 에서 읽음(11.17, 기본 parking_r3).

11.04 폐루프에서 파인튜닝 X-Mobility 가 경로 한가운데 상자(south_box cw)에 그대로 부딪힘. 학습 데이터의 장애물 장면이
Nav2 교사 16회(시나리오마다 1회, 장애물 1~2개)뿐이라, 자리 · 옆 오프셋 · 종류 · 크기 · 색을 무작위로 바꾼 배치를 많이 만들어
Nav2 교사로 녹화함. 한 바퀴에 장애물 4개(직선 구간마다 하나씩 넣되 구간은 무작위로 고름).

  python3 eval/scenarios/gen_rand_obstacles.py --prefix tr --n 20 --seed 1000   # 학습용(녹화)
  python3 eval/scenarios/gen_rand_obstacles.py --prefix ev --n 6  --seed 9000   # 평가용(녹화 안 함)
  python3 eval/scenarios/gen_rand_obstacles.py --prefix trside --n 10 --seed 2000 --min-lateral 0.4   # 11.12 E: 중앙 없이 '같은 쪽' 녹화용
  → eval/scenarios/rand/<장소 name>_rand_<prefix>_NN.yaml   (기본 장소면 parking_r3_rand_<prefix>_NN.yaml)
  다른 장소: --site eval/sites/<이름>.yaml — 씬 · 기준선 · 출발 자세 · 장애물을 놓는 구간을 그 파일에서 읽음(형식은 eval/sites/parking_r3.yaml).

자리 규칙(10.14 시나리오 결과 바탕, 주차장 r3 의 값 — 장소 파일 rand_obstacles 칸):
- s 창 = 직선 구간에서 모서리 앞뒤를 뺀 곳: 동 4.5~8.5 · 남 20~37 · 서 사선 53~58.5 · 북 67~82(시계 방향 s, 한 바퀴 90.0 m).
  모서리 직후(corner_exit s 17.5)는 Nav2 가 2.0 m 한도 근처라 뺌.
- 장애물 사이 9 m 이상(앞 장애물을 피하고 기준선에 돌아올 거리).
- 옆 오프셋: 절반은 가운데(|lateral| ≤ 0.15, 좌우를 골라야 하는 경우 — 11.04 충돌 유형), 나머지는 ±0.3~0.7.
- 종류: 상자 50 % · 원기둥(사람 대역) 25 % · 라바콘 25 %, 크기 · 색 무작위.
같은 장소 파일 · 같은 --seed 면 같은 배치가 나옴(11.17 에 장소 값을 파일로 옮긴 뒤에도 주차장 r3 은 그 전 파일과 바이트 단위로 같음).
"""
import argparse, datetime, os, random

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
DEFAULT_SITE = "eval/sites/parking_r3.yaml"

HEAD = """# {title} — 무작위 장애물 배치 {name} ({stamp}, eval/scenarios/gen_rand_obstacles.py --seed {seed}).
# 장애물 자리 = 기준선 길이 s(시계 방향) · lateral({lateral_note}).
# 용도: {use}
name: {name}
version: v1
scene: {scene}
route: {route}
directions: [{directions}]
start: {{x: {x}, y: {y}, yaw_deg: {yaw}}}
goal: {{lap: 1, radius_m: 1.0}}
obstacles:
"""
TAIL = """deviation_limit_m: 2.0
stall_limit_s: 10
timeout_s: 300
"""


def load_site(path):
    """장소 파일 → (이름, 머리글에 넣을 값, 구간 목록, 최소 간격, 한 바퀴 길이 또는 None)."""
    import yaml
    p = path if os.path.isabs(path) or os.path.exists(path) else os.path.join(REPO, path)
    with open(p, encoding="utf-8") as f:
        site = yaml.safe_load(f)
    ro = site.get("rand_obstacles") or {}
    windows = [(str(w[0]), float(w[1]), float(w[2])) for w in ro.get("windows") or []]
    if not windows:
        raise SystemExit(f"{path}: rand_obstacles.windows(장애물을 놓는 직선 구간 [이름, 시작 s, 끝 s])가 필요")
    st = site["start"]
    head = {"title": site.get("title", site["name"]), "stamp": ro.get("stamp", f"{datetime.date.today():%m.%d}"),
            "lateral_note": ro.get("lateral_note", "진행 방향의 왼쪽 +"), "scene": site["scene"], "route": site["route"],
            "directions": ", ".join(site.get("directions") or ["cw"]), "x": st["x"], "y": st["y"], "yaw": st["yaw_deg"]}
    lap = ro.get("lap_m")
    return site["name"], head, windows, float(ro.get("min_gap_m", 9.0)), (float(lap) if lap is not None else None)


def jitter(rng, base, d=0.08):
    return [round(min(1.0, max(0.0, c + rng.uniform(-d, d))), 2) for c in base]


def obstacle(rng, i, s, min_lat=None):
    if min_lat is None:   # 기본(11.05 규칙, 같은 seed 면 같은 배치)
        lat = rng.uniform(-0.15, 0.15) if rng.random() < 0.5 else rng.choice([-1, 1]) * rng.uniform(0.3, 0.7)
    else:                 # --min-lateral: 모두 ±min_lat~0.7 — Nav2 가 오프셋 반대쪽으로 지나는 비율 0.4 m 이상 77/77(side_vs_geometry_11_11)
        lat = rng.choice([-1, 1]) * rng.uniform(min_lat, 0.7)
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


def layout(rng, k, windows, min_gap, lap_m):
    """구간 k 곳을 골라 한 곳에 하나씩. lap_m 이 있으면(닫힌 경로) 마지막 장애물에서 바퀴 이음 너머 첫 장애물까지도 간격으로 봄."""
    for _ in range(1000):
        ss = sorted(rng.uniform(a, b) for _, a, b in rng.sample(windows, k))
        gaps = [b - a for a, b in zip(ss, ss[1:])] + ([ss[0] + lap_m - ss[-1]] if lap_m is not None else [])
        if not gaps or min(gaps) >= min_gap:
            return ss
    raise RuntimeError("배치 실패")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--prefix", required=True)
    ap.add_argument("--n", type=int, required=True)
    ap.add_argument("--seed", type=int, required=True)
    ap.add_argument("--k", type=int, default=4, help="한 바퀴 장애물 수(장소 파일의 구간 수 이하 — 주차장 r3 은 4)")
    ap.add_argument("--outdir", default="eval/scenarios/rand")
    ap.add_argument("--min-lateral", type=float, help="모든 장애물을 |lateral| ≥ 이 값(~0.7 m)에 — 중앙 장애물 없음(예 0.4)")
    ap.add_argument("--site", default=DEFAULT_SITE, help="장소 파일(eval/sites/<이름>.yaml) — 씬 · 기준선 · 출발 자세 · 장애물을 놓는 구간")
    a = ap.parse_args()
    if a.min_lateral is not None and not 0.0 < a.min_lateral < 0.7:
        ap.error("--min-lateral 은 0 과 0.7 사이")
    site_name, head, windows, min_gap, lap_m = load_site(a.site)
    if not 1 <= a.k <= len(windows):
        ap.error(f"--k 는 1 과 장소 파일의 구간 수({len(windows)}) 사이")
    os.makedirs(a.outdir, exist_ok=True)
    use = "학습(Nav2 교사 녹화)" if a.prefix.startswith("tr") else "평가(폐루프, 학습에 안 씀)"
    if a.min_lateral is not None:
        use += f" · 중앙 없음(|lateral| ≥ {a.min_lateral:g})"
    for j in range(a.n):
        seed = a.seed + j
        rng = random.Random(seed)
        name = f"{site_name}_rand_{a.prefix}_{j:02d}"
        body = "".join(obstacle(rng, i, s, a.min_lateral) for i, s in enumerate(layout(rng, a.k, windows, min_gap, lap_m)))
        path = os.path.join(a.outdir, name + ".yaml")
        with open(path, "w") as f:
            f.write(HEAD.format(name=name, seed=seed, use=use, **head) + body + TAIL)
        print(path)


if __name__ == "__main__":
    main()
