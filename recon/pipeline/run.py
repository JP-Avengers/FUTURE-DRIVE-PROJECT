"""한 명령으로: 입력 점검 → (메쉬 만들기) → USD → 구멍 점검 → 미리보기 → (D 패키지).

    python recon/pipeline/run.py --config recon/configs/<장소>.yaml [--out $OUT] [--package v0.1] [--no-preview] [--from build]

단계 (--from 으로 중간부터, --only 로 하나만)
  check     check_input.py — 오류가 있으면 여기서 멈춤 (--force 로 무시)
  tsdf      tsdf.py — config 에 geometry 가 없고 mesh_raw.ply 가 없을 때만 (--rebuild-mesh 로 다시)
  build     build_usd.py → <out>/usd/<name>_scene.usda · report.json
  visible   check_visibility.py → <out>/visibility/summary.json
  preview   Blender 로 로봇 카메라 높이 렌더 → <out>/preview/ (Blender 가 없으면 건너뜀. 경로: --blender 또는 환경변수 BLENDER)
  package   package.py — --package 판 이름을 줬을 때만 → <out>/handoff/<name>_<판>/ + zip

각 단계 출력은 <out>/run_log.txt 에 이어 씀. 결과 요약은 마지막에 화면과 <out>/run_summary.json.
"""
import argparse, csv, json, os, shutil, subprocess, sys, time

import numpy as np

import config
import io_rgbd

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(os.path.dirname(HERE), "tools")
STEPS = ["check", "tsdf", "build", "visible", "preview", "package"]
BLENDER_GUESS = [r"C:\Program Files\Blender Foundation\Blender 4.5\blender.exe", "/Applications/Blender.app/Contents/MacOS/Blender", "blender"]


def call(cmd, log, cwd):
    print("   $ " + " ".join(f'"{c}"' if " " in c else c for c in cmd), flush=True)
    t = time.time()
    with open(log, "a", encoding="utf-8") as fh:
        fh.write(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} {' '.join(cmd)}\n")
        fh.flush()
        p = subprocess.Popen(cmd, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace",
                             env=dict(os.environ, PYTHONIOENCODING="utf-8", PYTHONUNBUFFERED="1"))
        for line in p.stdout:
            fh.write(line)
            if not line.startswith(("Fra:", "Blender ", "Read ", "Saved:", " Time:")):   # Blender 진행 줄은 기록만
                print("     " + line.rstrip(), flush=True)
        p.wait()
    return p.returncode, round(time.time() - t, 1)


def find_blender(arg):
    for b in [arg, os.environ.get("BLENDER")] + BLENDER_GUESS:
        if b and (os.path.exists(b) or shutil.which(b)):
            return shutil.which(b) or b
    return None


def is_glb(cfg):
    return cfg["input"].get("format") == "glb"


def route_csv(cfg, out):
    """미리보기 카메라 자리: config 의 preview.route (x,y CSV) 가 있으면 그것, 없으면 입력 카메라 경로를 씬 좌표로 옮겨서."""
    r = (cfg.get("preview") or {}).get("route")
    if r:
        return os.path.expanduser(r)
    if is_glb(cfg):                                             # GLB: glb_to_usd.py 가 만든 가상 경로 (이미 씬 좌표)
        xy = np.array([np.array(P)[:2, 3] for P in json.load(open(os.path.join(out, "drive_path.json"), encoding="utf-8"))["poses"]])
    else:
        T = np.array(json.load(open(os.path.join(out, "transform.json"), encoding="utf-8"))["T_input_to_scene"])
        fr = io_rgbd.load(cfg["input"])
        xy = np.array([(T @ P)[:2, 3] for P in fr.poses])
    path = os.path.join(out, "preview_route.csv")
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["x", "y"])
        w.writerows(np.round(xy, 3).tolist())
    return path


def summary(out, cfg):
    s = dict(name=cfg["name"], out=out)
    p = os.path.join(out, "report.json")
    if os.path.exists(p):
        r = json.load(open(p, encoding="utf-8"))
        s["usd"] = r.get("usd", {}).get("scene")
        s["usd_ok"] = not r.get("usd", {}).get("problems")
        s["mesh_mb"] = r.get("usd", {}).get("mesh_mb")
        s["prims"] = r.get("usd", {}).get("prims")
        s["depth_vs_mesh_median_cm"] = (r.get("colorize") or {}).get("depth_vs_mesh_median_cm")
        s["collider_err"] = r.get("collider")
    p = os.path.join(out, "visibility", "summary.json")
    if os.path.exists(p):
        s["visibility"] = json.load(open(p, encoding="utf-8"))
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out")
    ap.add_argument("--from", dest="start", choices=STEPS, default="check")
    ap.add_argument("--only", choices=STEPS)
    ap.add_argument("--force", action="store_true", help="입력 점검 오류가 있어도 계속")
    ap.add_argument("--rebuild-mesh", action="store_true")
    ap.add_argument("--no-preview", action="store_true")
    ap.add_argument("--blender")
    ap.add_argument("--package", metavar="판", help="D 패키지 판 이름 (예: v0.1). 없으면 패키지 안 만듦")
    args = ap.parse_args()

    cfg_path = os.path.abspath(args.config)
    cfg = config.load(cfg_path)
    out = os.path.abspath(os.path.expanduser(args.out or cfg["out"]))
    os.makedirs(out, exist_ok=True)
    log = os.path.join(out, "run_log.txt")
    py = sys.executable
    steps = [args.only] if args.only else STEPS[STEPS.index(args.start):]
    common = ["--config", cfg_path, "--out", out]
    done, t0 = [], time.time()
    print(f"[run] {cfg['name']} → {out}")
    for st in steps:
        print(f"\n[{st}]", flush=True)
        if st == "check":
            rc, sec = call([py, os.path.join(HERE, "check_input.py")] + common, log, out)
            if rc and not args.force:
                raise SystemExit(f"입력 점검 오류 → 멈춤. {os.path.join(out, 'check_input.json')} 확인 (무시하려면 --force)")
        elif st == "tsdf":
            if cfg.get("geometry") or is_glb(cfg):
                print("   메쉬를 받음 (geometry · GLB) → 건너뜀")
                continue
            if os.path.exists(os.path.join(out, "mesh_raw.ply")) and not args.rebuild_mesh:
                print("   mesh_raw.ply 있음 → 건너뜀 (--rebuild-mesh 로 다시)")
                continue
            rc, sec = call([py, os.path.join(HERE, "tsdf.py")] + common, log, out)
        elif st == "build":
            rc, sec = call([py, os.path.join(HERE, "glb_to_usd.py" if is_glb(cfg) else "build_usd.py")] + common, log, out)
        elif st == "visible":
            rc, sec = call([py, os.path.join(HERE, "check_visibility.py")] + common, log, out)
        elif st == "preview":
            b = find_blender(args.blender)
            if args.no_preview or not b:
                print("   건너뜀" + ("" if args.no_preview else " — Blender 를 못 찾음 (--blender 경로 또는 환경변수 BLENDER)"))
                continue
            scene = summary(out, cfg).get("usd")
            rc, sec = call([b, "-b", "-P", os.path.join(TOOLS, "preview_blender.py"), "--", "--usd", scene, "--route", route_csv(cfg, out),
                            "--out", os.path.join(out, "preview"), "--overview"]
                           + [str(x) for x in (cfg.get("preview") or {}).get("args", [])], log, out)
        elif st == "package":
            if not args.package:
                continue
            rc, sec = call([py, os.path.join(HERE, "package.py")] + common + ["--version", args.package], log, out)
        if rc:
            raise SystemExit(f"[{st}] 실패 (종료 코드 {rc}) → {log}")
        done.append((st, sec))
    s = summary(out, cfg)
    s["steps"] = done
    s["minutes"] = round((time.time() - t0) / 60, 1)
    json.dump(s, open(os.path.join(out, "run_summary.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"\n[run] 끝 {s['minutes']}분 | " + " · ".join(f"{a} {b:.0f}s" for a, b in done))
    if s.get("usd"):
        print(f"      USD {s['usd']} ({s.get('mesh_mb')} MB, 검증 {'통과' if s.get('usd_ok') else '문제'})")


if __name__ == "__main__":
    main()
