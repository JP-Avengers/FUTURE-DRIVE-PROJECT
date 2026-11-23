"""시험용 가짜 실내 방 GLB — 결함을 정해 둔 자리에 일부러 넣어, scan2sim 진단이 그 자리를 짚는지 확인하는 데 씀.

    cd recon
    python scan2sim/samples/make_room.py --out ~/webapp_runs/samples [--seed 0] [--no-defects]
    python -m scan2sim ~/webapp_runs/samples/room.glb --out ~/webapp_runs/samples/run_glb

방 (씬 좌표 = Z 위, m. 방 가운데가 원점, 바닥 z 0)
    안쪽 8 × 6 m · 천장 2.8 m · 벽 두께 0.1 m · 북쪽 벽(y = +3)에 문 구멍 (x 2.6~3.5, 높이 2.0)
    바닥 = 0.5 m 타일 (정점 색, 타일마다 색이 조금씩 다름) · 가구 상자 4개 (탁자 · 소파 · 수납장 · 화분)

넣는 결함 (--no-defects 면 셋 다 뺌)
    double_floor      x −2.5~−1.5 · y 1.1~2.1 에 바닥 위 1.0~1.5 cm 로 뜬 얇은 판 (스캔이 바닥을 두 번 찍어 어긋난 층을 흉내).
                      glb_to_usd 는 바닥 ±5 cm 의 위 향한 면(판 윗면)만 충돌에서 빼므로, 아래를 향한 판 밑면(1.0 cm)이
                      평평한 충돌 바닥(0 cm) 위 1 cm 에 남아 '충돌 바닥 두 겹'이 됨
    no_visible_floor  x 1.5~2.5 · y −2.1~−1.1 의 보이는 바닥 삼각형을 지움 (1 m²). 충돌 바닥은 받침 바닥으로 메워짐
    low_bump          x 0.0~1.6 · y 1.55~1.85 에 높이 4.5 cm 띠 (문턱 · 전선 덮개). 윗면이 바닥 ±5 cm 안이라 충돌에서는 빠지고
                      보이는 바닥만 4.5 cm 솟음

좌표
    GLB 는 glTF 규약(Y 위, m). 씬 좌표 (x, y, z) ↔ glTF (x, z, −y). 바닥은 glTF y = 0.
    room_truth.json 에 결함마다 center_input (glTF) 과 center_scene_expected (씬) 을 적음.
    center_scene_expected 는 glb_to_usd.py 가 하는 변환(Y 위 → Z 위, 바닥 맞춤, origin: center 로 x · y 범위 가운데를 원점)을
    이 메쉬의 정점 범위로 직접 계산한 값 — 바닥 평면 맞춤(RANSAC)이 정확히 z 0 · 기울기 0 을 준다고 가정함.
    가장 정확한 비교는 실행 결과의 build/transform.json (T_input_to_scene) 으로 center_input 을 옮겨 보는 것.
    두 값 모두 결함 가운데의 '바닥 높이 점' (scan2sim 의 center_scene · center_input 과 같은 뜻). 결함 면의 높이는 height_m.

출력: <out>/room.glb · <out>/room_truth.json. 같은 --seed 면 같은 파일.
"""
import argparse
import json
import os

import numpy as np

ROOM = dict(x=8.0, y=6.0, h=2.8, wall=0.1)                      # 안쪽 크기 (m)
DOOR = dict(x0=2.6, x1=3.5, h=2.0)                              # 북쪽 벽 (y = +3) 의 문 구멍
FURNITURE = [                                                   # (이름, x0, x1, y0, y1, 높이, 색) — 윗면은 모두 0.6 m 넘게 (바닥 평평함 판정에 안 섞이게)
    ("table", -3.5, -2.3, -2.6, -1.8, 0.75, (0.55, 0.38, 0.22)),
    ("sofa", -1.0, 1.0, -2.95, -2.1, 0.70, (0.25, 0.35, 0.55)),
    ("cabinet", 3.45, 3.95, -0.3, 1.3, 1.00, (0.82, 0.80, 0.74)),
    ("plant", -0.7, -0.3, -0.2, 0.2, 0.90, (0.30, 0.50, 0.30)),
]
DEFECTS = [                                                     # 씬 좌표 (x0, x1, y0, y1) · 높이
    dict(type="double_floor", rect=(-2.5, -1.5, 1.1, 2.1), z0=0.010, z1=0.015,
         note="바닥 위 1.0~1.5 cm 에 뜬 얇은 판. 윗면은 충돌에서 빠지고 밑면(1.0 cm)이 충돌 바닥 위에 남아 두 겹이 됨"),
    dict(type="no_visible_floor", rect=(1.5, 2.5, -2.1, -1.1), z0=0.0, z1=0.0,
         note="보이는 바닥 삼각형을 지운 1 m² 구멍 (충돌 바닥은 받침 바닥으로 메워짐)"),
    dict(type="low_bump", rect=(0.0, 1.6, 1.55, 1.85), z0=0.0, z1=0.045,
         note="높이 4.5 cm · 길이 1.6 m 띠. 윗면이 바닥 ±5 cm 안이라 충돌에서는 빠지고 보이는 바닥만 솟음"),
]
Y_UP_TO_Z_UP = np.array([[1, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]], float)   # glTF (x, y, z) → 씬 (x, −z, y)


class Builder:
    """삼각형 · 정점 색을 모으는 곳 (씬 좌표, Z 위)."""

    def __init__(self, rng):
        self.V, self.F, self.C, self.n, self.rng = [], [], [], 0, rng

    def face(self, p0, du, dv, step, color, skip=None, jitter=0.012):
        """p0 에서 du · dv 로 펼친 사각형을 step 간격 격자로 (법선 = du × dv 방향). color = (3,) 또는 정점 xyz → (N,3) 함수.
        skip(quad 가운데 xyz (M,3)) → bool (M,) 이면 그 칸은 뺌."""
        p0, du, dv = (np.asarray(a, float) for a in (p0, du, dv))
        nu, nv = max(1, int(np.ceil(np.linalg.norm(du) / step - 1e-9))), max(1, int(np.ceil(np.linalg.norm(dv) / step - 1e-9)))
        a, b = np.meshgrid(np.linspace(0, 1, nu + 1), np.linspace(0, 1, nv + 1), indexing="ij")
        P = p0 + a.reshape(-1, 1) * du + b.reshape(-1, 1) * dv
        idx = np.arange((nu + 1) * (nv + 1)).reshape(nu + 1, nv + 1)
        q = np.stack([idx[:-1, :-1], idx[1:, :-1], idx[1:, 1:], idx[:-1, 1:]], -1).reshape(-1, 4)
        if skip is not None:
            q = q[~skip(P[q].mean(1))]
        if len(q) == 0:
            return
        col = np.asarray(color(P) if callable(color) else np.broadcast_to(np.asarray(color, float), P.shape), float)
        col = np.clip(col + self.rng.normal(0, jitter, P.shape), 0, 1)
        tri = np.vstack([q[:, [0, 1, 2]], q[:, [0, 2, 3]]])
        self.V.append(P)
        self.C.append(col)
        self.F.append(tri + self.n)
        self.n += len(P)

    def box(self, x0, x1, y0, y1, z0, z1, step, color, bottom=False):
        """축에 맞춘 상자, 법선은 바깥. 스캔처럼 밑면은 기본으로 뺌."""
        dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
        self.face((x0, y0, z1), (dx, 0, 0), (0, dy, 0), step, color)          # 위 (+z)
        if bottom:
            self.face((x0, y0, z0), (0, dy, 0), (dx, 0, 0), step, color)      # 아래 (−z)
        self.face((x1, y0, z0), (0, dy, 0), (0, 0, dz), step, color)          # +x
        self.face((x0, y0, z0), (0, 0, dz), (0, dy, 0), step, color)          # −x
        self.face((x0, y1, z0), (0, 0, dz), (dx, 0, 0), step, color)          # +y
        self.face((x0, y0, z0), (dx, 0, 0), (0, 0, dz), step, color)          # −y

    def arrays(self):
        return np.vstack(self.V), np.vstack(self.F), np.vstack(self.C)


def inside(rect, p):
    x0, x1, y0, y1 = rect
    return (p[:, 0] > x0) & (p[:, 0] < x1) & (p[:, 1] > y0) & (p[:, 1] < y1)


def build(seed=0, defects=True, bump_h=None):
    """→ (정점 (N,3) 씬 좌표, 삼각형 (M,3), 정점 색 (N,3) 0~1, 넣은 결함 목록). bump_h = 턱 높이 (m, 없으면 4.5 cm)"""
    rng = np.random.default_rng(seed)
    b = Builder(rng)
    X, Y, H, T = ROOM["x"] / 2, ROOM["y"] / 2, ROOM["h"], ROOM["wall"]
    D = [dict(d) for d in DEFECTS] if defects else []
    for d in D:
        if d["type"] == "low_bump" and bump_h is not None and abs(bump_h - d["z1"]) > 1e-9:
            d["z1"] = bump_h
            d["note"] = (f"높이 {100 * bump_h:.1f} cm · 길이 1.6 m 띠. " + ("윗면이 바닥 ±5 cm 안이라 충돌에서는 빠지고 보이는 바닥만 솟음" if bump_h < 0.05 else
                         "윗면이 바닥 위 5 cm 를 넘어 충돌에도 남음 (충돌 바닥 위 계단). 아이폰 원본 경로의 바닥 높이 지도(띠 5 cm)에는 안 섞임"))
    hole = next((d["rect"] for d in D if d["type"] == "no_visible_floor"), None)

    tile = 0.5                                                  # 바닥 타일 — 타일마다 정점을 따로 둬서 색 경계가 또렷함
    base = [np.array([0.72, 0.66, 0.56]), np.array([0.64, 0.58, 0.49])]
    for i in range(int(round(ROOM["x"] / tile))):
        for j in range(int(round(ROOM["y"] / tile))):
            c = base[(i + j) % 2] + rng.normal(0, 0.025, 3)
            b.face((-X + i * tile, -Y + j * tile, 0.0), (tile, 0, 0), (0, tile, 0), 0.1, c, jitter=0.01,
                   skip=(lambda p: inside(hole, p)) if hole else None)
    b.face((-X, -Y, H), (0, 2 * Y, 0), (2 * X, 0, 0), 0.25, (0.93, 0.93, 0.92), jitter=0.006)   # 천장 (아래를 봄)

    def wall_col(p):                                            # 벽: 아래 0.9 m 는 회색 띠, 위는 미색
        return np.where(p[:, 2:3] < 0.9, [0.62, 0.65, 0.68], [0.88, 0.86, 0.80])
    b.box(-X - T, -X, -Y - T, Y + T, 0, H, 0.2, wall_col)                  # 서 (x = −4)
    b.box(X, X + T, -Y - T, Y + T, 0, H, 0.2, wall_col)                    # 동
    b.box(-X, X, -Y - T, -Y, 0, H, 0.2, wall_col)                          # 남
    b.box(-X, DOOR["x0"], Y, Y + T, 0, H, 0.2, wall_col)                   # 북 (문 왼쪽)
    b.box(DOOR["x1"], X, Y, Y + T, 0, H, 0.2, wall_col)                    # 북 (문 오른쪽)
    b.box(DOOR["x0"], DOOR["x1"], Y, Y + T, DOOR["h"], H, 0.2, wall_col, bottom=True)   # 문 위 인방
    for _, x0, x1, y0, y1, h, col in FURNITURE:
        b.box(x0, x1, y0, y1, 0, h, 0.1, col)

    truth = []
    for d in D:
        x0, x1, y0, y1 = d["rect"]
        if d["type"] == "double_floor":                         # 뜬 얇은 판 — 밑면까지 닫힌 상자
            b.box(x0, x1, y0, y1, d["z0"], d["z1"], 0.05, (0.62, 0.57, 0.48), bottom=True)
        elif d["type"] == "low_bump":
            b.box(x0, x1, y0, y1, d["z0"], d["z1"], 0.05, (0.30, 0.30, 0.32))
        truth.append(d)
    V, F, C = b.arrays()
    return V, F, C, truth


def expected_transform(V):
    """glb_to_usd.py 가 이 메쉬에 줄 T_input_to_scene 의 예상값: Y 위 → Z 위, (바닥 맞춤 = 단위 행렬로 가정), origin: center."""
    M = np.eye(4)
    M[:2, 3] = -0.5 * (V[:, :2].min(0) + V[:, :2].max(0))
    return M @ Y_UP_TO_Z_UP


def write_glb(path, V, F, C):
    import trimesh
    Vg = V @ np.linalg.inv(Y_UP_TO_Z_UP)[:3, :3].T               # 씬 → glTF (x, z, −y)
    rgba = np.c_[np.round(C * 255), np.full(len(C), 255)].astype(np.uint8)
    m = trimesh.Trimesh(vertices=Vg, faces=F, vertex_colors=rgba, process=False)
    sc = trimesh.Scene()
    sc.add_geometry(m, node_name="room", geom_name="room")
    open(path, "wb").write(trimesh.exchange.gltf.export_glb(sc))
    return Vg


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="room.glb · room_truth.json 을 쓸 폴더 (레포 밖)")
    ap.add_argument("--seed", type=int, default=0, help="색 흔들림 난수 (결함 자리는 고정)")
    ap.add_argument("--no-defects", action="store_true", help="결함 없는 깨끗한 방")
    ap.add_argument("--bump-cm", type=float, default=4.5, help="낮은 턱 높이 (cm, 3~12). 5 cm 를 넘기면 GLB 충돌에도 남음")
    a = ap.parse_args()
    if not 3 < a.bump_cm < 12:
        ap.error("--bump-cm 은 3~12 cm 사이 (진단의 '낮은 턱' 범위)")
    out = os.path.abspath(os.path.expanduser(a.out))
    os.makedirs(out, exist_ok=True)
    V, F, C, D = build(a.seed, not a.no_defects, a.bump_cm / 100)
    Vg = write_glb(os.path.join(out, "room.glb"), V, F, C)
    T = expected_transform(V)                                   # V (만든 좌표) = Y_UP_TO_Z_UP @ glTF 정점 이므로 범위 가운데는 V 로 잼
    Ti = np.linalg.inv(T)
    defects = []
    for d in D:
        x0, x1, y0, y1 = d["rect"]
        cs = np.array([(x0 + x1) / 2, (y0 + y1) / 2, 0.0])     # 만든 좌표의 결함 가운데 바닥 점
        cs_exp = cs + T[:3, 3]                                  # 씬 = 범위 가운데를 원점으로 옮긴 것 (이 방은 대칭이라 거의 0)
        ci = (Ti @ np.r_[cs_exp, 1])[:3]
        defects.append(dict(type=d["type"], center_input=np.round(ci, 4).tolist(), center_scene_expected=np.round(cs_exp, 4).tolist(),
                            size_m=[round(x1 - x0, 3), round(y1 - y0, 3)], height_m=[d["z0"], d["z1"]], note=d["note"]))
    truth = dict(
        file="room.glb", seed=a.seed, defects_injected=not a.no_defects,
        room=dict(inner_m=[ROOM["x"], ROOM["y"]], ceiling_m=ROOM["h"], wall_m=ROOM["wall"], door=DOOR,
                  furniture=[dict(name=n, rect_scene=[x0, x1, y0, y1], height_m=h) for n, x0, x1, y0, y1, h, _ in FURNITURE]),
        coords=("center_input = glTF 좌표 (Y 위, m) 의 결함 가운데 바닥 점. center_scene_expected = 씬 좌표 (Z 위, 바닥 z 0) 예상값 — "
                "glb_to_usd 의 변환을 정점 범위로 계산하고 바닥 맞춤은 정확하다고 가정함. 실행 결과와 견줄 때는 build/transform.json 의 "
                "T_input_to_scene 으로 center_input 을 옮기는 것이 가장 정확함"),
        T_input_to_scene_expected=np.round(T, 6).tolist(),
        mesh=dict(vertices=int(len(V)), triangles=int(len(F)), extent_input_m=np.round(np.ptp(Vg, axis=0), 3).tolist()),
        defects=defects)
    json.dump(truth, open(os.path.join(out, "room_truth.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[make_room] 삼각형 {len(F):,} · 결함 {len(defects)}개 ({', '.join(d['type'] for d in defects) or '없음'}) → {os.path.join(out, 'room.glb')}")


if __name__ == "__main__":
    main()
