"""주행 통로에서 보이는 구멍 점검 — 가상 카메라에서 광선을 쏴서 처음 맞는 면을 분류한다.

    python recon/pipeline/check_visibility.py --config recon/configs/lounge.yaml

입력 (<out>/): visual.ply, transform.json (build_usd.py 출력) + config 의 input (카메라 경로)
출력 (<out>/visibility/): summary.json, holes_list.csv (다시 찍을 곳 후보), holes_seen.ply, hole_map.png

광선 분류
  ok        면의 앞쪽에 맞음 (정상)
  backface  면의 뒤쪽에 맞음 → 구멍 너머로 물체 안쪽이 보임 (물체 구멍)
  down      아래로 빠져나감 → 바닥 구멍 (바퀴가 빠짐)
  side      옆으로 빠져나감 → 벽 구멍 / 실외면 먼 배경 없음
  up        위로 빠져나감 → 실내면 천장 구멍, 실외면 하늘 (정상)
"""
import argparse, csv, json, os, time

import numpy as np
import open3d as o3d
import yaml

import io_rgbd
from mesh_utils import Footprint, Ground

NAMES = ["ok", "backface", "down", "side", "up"]


def Rz(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])


def virtual_cameras(poses, scene, c):
    """경로 위 지점마다 좌우로 옮기고(offsets_m) 고개를 돌린(yaws_deg) 카메라. 면에 붙었거나 발밑에 바닥이 없으면 버림."""
    path = np.array([p[:3, 3] for p in poses])
    step = c["path_step"]
    cams = []
    for i in range(step, len(poses) - step, step):
        move = path[i + step] - path[i - step]
        move[2] = 0
        side = np.zeros(3) if np.linalg.norm(move) < 0.05 else np.cross([0, 0, 1], move / np.linalg.norm(move))
        for off in (c["offsets_m"] if side.any() else [0.0]):
            for yaw in c["yaws_deg"]:
                P = np.eye(4)
                P[:3, :3] = Rz(yaw) @ poses[i][:3, :3]
                P[:3, 3] = path[i] + off * side
                cams.append((off, P))
    centers = np.array([P[:3, 3] for _, P in cams], dtype=np.float32)
    clear = scene.compute_distance(o3d.core.Tensor(centers)).numpy()
    down = np.c_[centers, np.tile([0, 0, -1], (len(cams), 1))].astype(np.float32)
    floor = np.isfinite(scene.cast_rays(o3d.core.Tensor(down))["t_hit"].numpy())
    keep = (clear > c["min_clearance_m"]) & floor
    return [cam for cam, k in zip(cams, keep) if k], len(cams), path


def classify(scene, K, P, W, H, horizon):
    """픽셀마다 광선 하나 → (분류, 광선 시작점, 방향, t_hit)."""
    rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
        o3d.core.Tensor(K), o3d.core.Tensor(np.linalg.inv(P)), W, H).numpy().reshape(-1, 6)
    ans = scene.cast_rays(o3d.core.Tensor(rays))
    t, nrm = ans["t_hit"].numpy(), ans["primitive_normals"].numpy()
    o, d = rays[:, :3], rays[:, 3:]
    du = d / np.linalg.norm(d, axis=1, keepdims=True)
    hit = np.isfinite(t)
    cls = np.zeros(len(t), int)
    cls[hit & (np.sum(du * nrm, axis=1) > 0)] = 1           # 광선과 법선이 같은 쪽 = 면 뒤쪽
    cls[~hit & (du[:, 2] < -horizon)] = 2
    cls[~hit & (np.abs(du[:, 2]) <= horizon)] = 3
    cls[~hit & (du[:, 2] > horizon)] = 4
    return cls, o, d, t


def hole_map(path_xy, pts, fp, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    lo, hi = np.minimum(path_xy.min(0), pts[:, :2].min(0) if len(pts) else path_xy.min(0)) - 0.5, \
        np.maximum(path_xy.max(0), pts[:, :2].max(0) if len(pts) else path_xy.max(0)) + 0.5
    bins = [np.arange(lo[0], hi[0], 0.1), np.arange(lo[1], hi[1], 0.1)]
    fig, ax = plt.subplots(1, 2, figsize=(14, 6))
    for a, kind, title in ((ax[0], 2, "floor holes (down)"), (ax[1], 1, "object holes (backface)")):
        q = pts[pts[:, 3] == kind]
        if len(q):
            h = a.hist2d(q[:, 0], q[:, 1], bins=bins, cmin=1, norm=matplotlib.colors.LogNorm(), cmap="inferno_r")
            fig.colorbar(h[3], ax=a, label="rays")
        gx = fp.lo[0] + (np.arange(fp.grid.shape[0]) + 0.5) * fp.cell
        gy = fp.lo[1] + (np.arange(fp.grid.shape[1]) + 0.5) * fp.cell
        a.contour(gx, gy, fp.grid.T.astype(float), levels=[0.5], colors="green", linewidths=1)
        a.plot([], [], c="green", lw=1, label="mesh footprint")
        a.plot(path_xy[:, 0], path_xy[:, 1], c="tab:blue", lw=0.6, label="path")
        a.set_aspect("equal"); a.set_title(title); a.set_xlabel("x (m)"); a.set_ylabel("y (m)"); a.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_png, dpi=90)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", help="build_usd.py 출력 폴더 (기본: config 의 out)")
    args = ap.parse_args()
    cfg = yaml.safe_load(open(args.config, encoding="utf-8"))
    out = os.path.expanduser(args.out or cfg["out"])
    c = cfg["visibility"]
    vdir = os.path.join(out, "visibility")
    os.makedirs(vdir, exist_ok=True)

    mesh = o3d.io.read_triangle_mesh(os.path.join(out, "visual.ply"))
    T = np.array(json.load(open(os.path.join(out, "transform.json"), encoding="utf-8"))["T_input_to_scene"])
    fr = io_rgbd.load(cfg["input"])
    poses = [T @ p for p in fr.poses]                         # 카메라 경로를 메쉬와 같은 씬 좌표로
    gpath = os.path.join(out, "ground.npz")                   # 실외: 바닥 높이 지도 (없으면 z=0)
    ground = Ground.load(gpath) if os.path.exists(gpath) else Ground()
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    W = c["ray_width"]
    H = int(round(fr.intrinsic.height * W / fr.intrinsic.width))
    K = fr.intrinsic.intrinsic_matrix.copy()
    K[:2] *= W / fr.intrinsic.width                          # 해상도 줄인 만큼 초점거리·중심 축소
    horizon = np.sin(np.radians(c["horizon_deg"]))

    cams, n_all, path = virtual_cameras(poses, scene, c)
    print(f"[visibility] 가상 카메라 {n_all:,}개 중 유효 {len(cams):,}개 | 광선 {W}x{H}")
    t0 = time.time()
    counts = np.zeros(5, int)
    per_off = {o: np.zeros(5, int) for o in c["offsets_m"]}
    per_off.setdefault(0.0, np.zeros(5, int))
    hole_pts = []
    for k, (off, P) in enumerate(cams):
        cls, o, d, t = classify(scene, K, P, W, H, horizon)
        cnt = np.bincount(cls, minlength=5)
        counts += cnt
        per_off[off] += cnt
        m1, m2 = cls == 1, cls == 2
        p1 = o[m1] + d[m1] * t[m1, None]                     # 물체 구멍: 뒤쪽 면에 맞은 지점
        p2 = o[m2] - d[m2] * ((o[m2, 2:3] - ground.height(o[m2])[:, None]) / d[m2, 2:3])   # 바닥 구멍: 바닥이 있어야 할 지점
        if len(p2):                                          # 바닥 높이가 곳곳 다르면 닿는 자리 높이로 한 번 더
            p2 = o[m2] - d[m2] * ((o[m2, 2:3] - ground.height(p2)[:, None]) / d[m2, 2:3])
        hole_pts += [np.c_[p1, np.full(len(p1), 1), np.full(len(p1), k)],
                     np.c_[p2, np.full(len(p2), 2), np.full(len(p2), k)]]
    pct = 100 * counts / counts.sum()
    hp = np.vstack(hole_pts) if hole_pts else np.zeros((0, 5))
    fp = Footprint(mesh)
    is_floor = hp[:, 3] == 2
    floor_in = is_floor & fp.inside(hp[:, :2])
    # 바닥 구멍(down) 중 메쉬 범위 안 = 지도 안의 진짜 구멍, 밖 = 지도 가장자리 너머 (촬영 범위 밖)
    down_in_pct = float(pct[2]) * floor_in.sum() / max(is_floor.sum(), 1)
    hp = hp[~is_floor | floor_in]                                 # 목록·지도에는 지도 안의 구멍만

    # 다시 찍을 곳 후보: 칸(list_cell_m)마다 몇 개 카메라에서 보였나
    cell = c["list_cell_m"]
    rows = []
    if len(hp):
        key = np.c_[np.floor(hp[:, :3] / cell).astype(int), hp[:, 3].astype(int)]
        uniq, inv = np.unique(key, axis=0, return_inverse=True)
        inv = inv.ravel()
        pairs = np.unique(np.c_[inv, hp[:, 4].astype(int)], axis=0)          # (칸, 카메라) 중복 제거
        cams_seen = np.bincount(pairs[:, 0], minlength=len(uniq))
        rays = np.bincount(inv, minlength=len(uniq))
        for j in np.argsort(-cams_seen, kind="stable")[:c["list_top"]]:
            x, y, z, kind = uniq[j]
            rows.append(dict(kind="floor" if kind == 2 else "object", x=round((x + .5) * cell, 2), y=round((y + .5) * cell, 2),
                             z=round(float(ground.height(np.array([[(x + .5) * cell, (y + .5) * cell]]))[0]), 2) if kind == 2
                             else round((z + .5) * cell, 2), cameras=int(cams_seen[j]), rays=int(rays[j])))
    with open(os.path.join(vdir, "holes_list.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["kind", "x", "y", "z", "cameras", "rays"])
        w.writeheader()
        w.writerows(rows)

    pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(hp[:, :3]))
    pc.colors = o3d.utility.Vector3dVector(np.where(hp[:, 3:4] == 2, [1.0, 0.5, 0.0], [1.0, 0.0, 0.0]))
    o3d.io.write_point_cloud(os.path.join(vdir, "holes_seen.ply"), pc.voxel_down_sample(0.01))
    hole_map(path[:, :2], hp, fp, os.path.join(vdir, "hole_map.png"))

    summary = dict(cameras=len(cams), cameras_total=n_all, rays=int(counts.sum()), seconds=round(time.time() - t0, 1),
                   params=c, pct={n: round(float(p), 3) for n, p in zip(NAMES, pct)},
                   holes_visible_pct=round(float(pct[1:4].sum()), 2),
                   down_inside_map_pct=round(down_in_pct, 2), down_beyond_edge_pct=round(float(pct[2]) - down_in_pct, 2),
                   holes_visible_pct_by_offset={f"{o:+.1f}": round(100 * float(v[1:4].sum()) / max(v.sum(), 1), 2)
                                                for o, v in sorted(per_off.items()) if v.sum()},
                   top_holes=rows[:10])
    json.dump(summary, open(os.path.join(vdir, "summary.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)

    print("  " + " | ".join(f"{n} {p:.2f}%" for n, p in zip(NAMES, pct)))
    print(f"  통로에서 보이는 구멍 (backface + down + side): {summary['holes_visible_pct']}% | {summary['seconds']}s")
    print(f"  └ 바닥 구멍 {pct[2]:.2f}% = 지도 안 {summary['down_inside_map_pct']}% + 지도 가장자리 너머 {summary['down_beyond_edge_pct']}%")
    for r in rows[:5]:
        print(f"  {r['kind']:6s} x={r['x']:6.2f} y={r['y']:6.2f} z={r['z']:5.2f} — 카메라 {r['cameras']}개에서 보임")
    print(f"[visibility] → {vdir}")


if __name__ == "__main__":
    main()
