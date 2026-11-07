"""받은 데이터 점검 — 빌드(수십 분) 전에 형식 · 경로 · 자세 · 깊이 단위가 맞는지 몇 초~1분 안에 확인한다.

    python recon/pipeline/check_input.py --config recon/configs/<장소>.yaml [--out $OUT]

출력: 화면 표 (OK / 경고 / 오류) + <out>/check_input.json + <out>/check_input_path.png (위에서 본 카메라 경로 · 메쉬 범위)
오류가 하나라도 있으면 종료 코드 1 (run.py 가 빌드를 시작하지 않음).
"""
import argparse, json, os

import numpy as np
import open3d as o3d

import config
import io_rgbd

FILE_KEYS = {"mesh", "far_mesh", "ground_map", "objects", "zones", "normal", "roughness", "color"}


class Report:
    def __init__(self):
        self.rows = []

    def add(self, level, what, detail):
        self.rows.append(dict(level=level, what=what, detail=detail))
        print(f"  [{ {'ok': 'OK', 'warn': '경고', 'err': '오류'}[level]:>2}] {what}: {detail}")

    @property
    def errors(self):
        return sum(r["level"] == "err" for r in self.rows)


def files_in(cfg, root, path=""):
    """설정 안의 파일 경로 (FILE_KEYS 이름의 문자열 값). ~ · 절대 경로는 그대로, 아니면 input.root 기준."""
    out = []
    if isinstance(cfg, dict):
        for k, v in cfg.items():
            if k == "input":
                continue
            if isinstance(v, str) and k in FILE_KEYS and "." in os.path.basename(v):
                p = os.path.expanduser(v)
                out.append((f"{path}{k}", p if os.path.isabs(p) else os.path.join(root, p)))
            else:
                out += files_in(v, root, f"{path}{k}.")
    elif isinstance(cfg, list):
        for i, v in enumerate(cfg):
            out += files_in(v, root, f"{path}{i}.")
    return out


def check_frames(rep, name, inp, sample=30):
    root = os.path.expanduser(inp.get("root", ""))
    if inp.get("root") and not os.path.isdir(root):
        rep.add("err", f"{name} 폴더", f"없음: {root}")
        return None
    if root and not root.isascii():
        rep.add("err", f"{name} 폴더", f"경로에 한글 등 비ASCII 글자 — Open3D 가 이미지를 못 읽음. 영문 경로로 복사: {root}")
        return None
    try:
        fr = io_rgbd.load(inp)
    except (SystemExit, Exception) as e:   # noqa: BLE001 — 형식이 틀리면 reader 마다 다른 예외
        rep.add("err", f"{name} 읽기", f"{type(e).__name__}: {e}")
        return None
    n = len(fr.poses)
    rep.add("ok" if n >= 10 else "err", f"{name} 프레임", f"{n}장 (형식 {inp['format']})")
    if n < 2:
        return fr
    W, H = fr.intrinsic.width, fr.intrinsic.height
    fx = fr.intrinsic.intrinsic_matrix[0, 0]
    rep.add("ok", f"{name} 내부 파라미터", f"{W}×{H} · fx {fx:.1f} · 가로 화각 {np.degrees(2 * np.arctan(W / 2 / fx)):.0f}°")
    idx = np.linspace(0, n - 1, min(sample, n)).round().astype(int)
    bad_size, med, valid = 0, [], []
    for i in idx:
        c = o3d.io.read_image(fr.color[i])
        d = np.asarray(o3d.io.read_image(fr.depth[i]))
        if np.asarray(c).shape[:2] != (H, W) or d.shape[:2] != (H, W):
            bad_size += 1
        if d.dtype != np.uint16:
            rep.add("err", f"{name} 깊이 형식", f"{os.path.basename(fr.depth[i])} 가 {d.dtype} — 16bit PNG 여야 함")
            return fr
        m = d > 0
        valid.append(m.mean())
        if m.any():
            med.append(np.median(d[m]) / fr.depth_scale)
    rep.add("err" if bad_size else "ok", f"{name} 이미지 크기", f"{bad_size}/{len(idx)}장이 내부 파라미터 크기와 다름" if bad_size else f"표본 {len(idx)}장 모두 {W}×{H}")
    m = float(np.median(med)) if med else 0.0
    lvl = "ok" if 0.3 <= m <= 15 else ("warn" if 0.05 <= m <= 100 else "err")   # 1000배 어긋남 = 단위 오류, 15 m 넘음 = 먼 곳 위주(지붕 카메라 등)
    rep.add(lvl, f"{name} 깊이 단위", f"깊이 중앙값 {m:.2f} m (depth_scale {fr.depth_scale:g})"
            + {"ok": "", "warn": " — 먼 곳 위주 (멀리 보는 카메라면 정상, 아니면 단위 확인)", "err": " — 단위가 틀렸을 가능성 (mm 면 1000)"}[lvl])
    v = float(np.mean(valid))
    rep.add("ok" if v > 0.3 else "warn", f"{name} 깊이 채움", f"유효 화소 평균 {100 * v:.0f}%")
    P = np.array(fr.poses)
    t = P[:, :3, 3]
    step = np.linalg.norm(np.diff(t, axis=0), axis=1)
    cosang = np.clip((np.einsum("nij,nij->n", P[1:, :3, :3], P[:-1, :3, :3]) - 1) / 2, -1, 1)
    rot = np.degrees(np.arccos(cosang))
    lim = max(3.0, 10 * float(np.median(step)))           # 키프레임은 솎아져 있어 간격이 들쭉날쭉 — 중앙의 10배(최소 3 m) 넘게 튀는 곳만
    jumps = int((step > lim).sum() + (rot > 45).sum())
    lvl = "ok" if jumps <= 0.02 * len(step) else "warn"    # 회차가 바뀌는 곳 몇 개는 정상
    rep.add(lvl, f"{name} 자세 연속성",
            f"이동 중앙 {np.median(step) * 100:.0f} cm · 최대 {step.max():.2f} m · 회전 최대 {rot.max():.0f}° · {lim:.1f} m 넘게 튀거나 45° 넘게 도는 곳 {jumps}곳"
            + ("" if lvl == "ok" else " — 많음: 자세 순서 · 시각 짝 · 좌표계 확인"))
    rep.add("ok", f"{name} 경로", f"길이 {step.sum():.1f} m · 범위 x {np.ptp(t[:, 0]):.1f} × y {np.ptp(t[:, 1]):.1f} × z {np.ptp(t[:, 2]):.2f} m")
    return fr


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out")
    args = ap.parse_args()
    cfg = config.load(args.config)
    out = os.path.expanduser(args.out or cfg["out"])
    os.makedirs(out, exist_ok=True)
    rep = Report()
    print(f"입력 점검: {cfg.get('name')} ({args.config})")
    for k in ("name", "out"):
        if "CHANGE_ME" in str(cfg.get(k, "CHANGE_ME")):
            rep.add("err", f"설정 {k}", "틀의 CHANGE_ME 를 장소 이름으로 바꿔야 함")
    if "CHANGE_ME" in str(cfg["input"].get("root", "")):
        rep.add("err", "설정 input.root", "받은 데이터 폴더로 바꿔야 함")
        return finish(rep, out, cfg)
    fr = check_frames(rep, "주 키프레임", cfg["input"])
    for e in (cfg.get("colorize") or {}).get("extra_inputs", []) or []:
        check_frames(rep, f"보조 {e['name']}", e["input"], sample=10)
    root = os.path.expanduser(cfg["input"].get("root", ""))
    missing = [(k, p) for k, p in files_in(cfg, root) if not os.path.exists(p)]
    n_files = len(files_in(cfg, root))
    for k, p in missing:
        rep.add("err", f"파일 {k}", f"없음: {p}")
    if n_files and not missing:
        rep.add("ok", "설정의 파일", f"{n_files}개 모두 있음")
    geo = cfg.get("geometry")
    mesh_box = None
    if geo and os.path.exists(os.path.join(root, geo["mesh"])):
        m = o3d.io.read_triangle_mesh(os.path.join(root, geo["mesh"]))
        if not len(m.triangles):
            rep.add("err", "형상 메쉬", "삼각형 0개 (점군만 있는 PLY?)")
        else:
            lo, hi = m.get_min_bound(), m.get_max_bound()
            mesh_box = (lo, hi)
            rep.add("ok", "형상 메쉬", f"삼각형 {len(m.triangles):,} · 범위 {np.round(hi - lo, 1).tolist()} m · 색 {'있음' if m.has_vertex_colors() else '없음 (colorize 필요)'}")
            if not m.has_vertex_colors() and not cfg.get("colorize"):
                rep.add("err", "색", "메쉬에 색이 없는데 colorize 가 꺼져 있음")
            if fr is not None:
                t = np.array(fr.poses)[:, :3, 3]
                inside = np.all((t >= lo - 2) & (t <= hi + 2), axis=1).mean()
                rep.add("ok" if inside > 0.95 else "err", "자세 ↔ 메쉬 좌표",
                        f"카메라 위치 {100 * inside:.0f}% 가 메쉬 범위(±2 m) 안" + ("" if inside > 0.95 else " — 자세와 메쉬가 다른 좌표계 (camera_link ↔ optical, 다른 맵 회차?)"))
    elif not geo:
        rep.add("ok", "형상", "geometry 없음 → tsdf.py 로 메쉬를 만듦")
    if fr is not None:
        plot(fr, mesh_box, os.path.join(out, "check_input_path.png"))
    return finish(rep, out, cfg)


def plot(fr, mesh_box, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = "Malgun Gothic"         # 한글 (Windows). 없으면 네모로 나옴
    t = np.array(fr.poses)[:, :3, 3]
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(t[:, 0], t[:, 1], "-", lw=1, color="#3060c0")
    ax.plot(*t[0, :2], "go", label="시작")
    ax.plot(*t[-1, :2], "rs", label="끝")
    if mesh_box is not None:
        (x0, y0, _), (x1, y1, _) = mesh_box
        ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ls="--", color="gray", label="메쉬 범위"))
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.legend()
    ax.set_title(f"카메라 경로 {len(t)}장 (위에서)")
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def finish(rep, out, cfg):
    json.dump(dict(name=cfg.get("name"), errors=rep.errors, rows=rep.rows), open(os.path.join(out, "check_input.json"), "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    print(f"→ 오류 {rep.errors} · 경고 {sum(r['level'] == 'warn' for r in rep.rows)}  ({os.path.join(out, 'check_input.json')})")
    return 1 if rep.errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
