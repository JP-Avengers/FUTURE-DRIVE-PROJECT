"""D 에게 넘길 패키지: <out>/handoff/<name>_<판>_<날짜>/ (usd · preview · transform.json · README_D.md) + zip · sha256.

    python recon/pipeline/package.py --config recon/configs/<장소>.yaml --version v0.1 [--out $OUT] [--changes "직전 판과 바뀐 것"]

README_D.md 는 report.json · visibility 결과로 채우고, '바뀐 것' · '확인 부탁' 은 --changes · config 의 package.ask 로.
씬 루트 제안(sim/scenes)은 config 의 package.scene_template (.usda, {name} {version} 을 바꿈)이 있을 때만 scene_suggestion/ 에.
보내기는 사람이 함 (서버 경로 · 계정은 README 에 적지 않음).
"""
import argparse, hashlib, json, os, shutil, time

import config


def sha12(path):
    h = hashlib.sha256()
    with open(path, "rb") as fh:
        for b in iter(lambda: fh.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()[:12]


def readme(cfg, ver, r, vis, changes, files):
    u = r.get("usd", {})
    prims = u.get("prims") or {}
    L = [f"# {cfg['name']} USD {ver} (C → D)", "", f"{time.strftime('%Y-%m-%d')} · 설정 `{os.path.basename(cfg.get('_path', ''))}` · 만든 도구 `recon/pipeline/run.py`", ""]
    L += ["## 바뀐 것", "", changes or "- (직전 판과 바뀐 것을 적어 주세요)", ""]
    L += ["## 들어 있는 것", "", "| 파일 | 내용 |", "| --- | --- |",
          f"| `usd/{os.path.basename(u.get('scene', ''))}` · `{os.path.basename(u.get('mesh', ''))}` | 씬 (m · Z-up) + 메쉬 {u.get('mesh_mb')} MB |",
          "| `transform.json` | 입력(B 맵) 좌표 → 씬 좌표 변환 |"]
    if files.get("preview"):
        L.append(f"| `preview/` | Blender 미리보기 {files['preview']}장 (로봇 카메라 높이, 흐린 날 · 해) — Isaac Sim 화면과 조명이 조금 다름 |")
    if files.get("scene"):
        L.append(f"| `scene_suggestion/{files['scene']}` | 씬 루트 제안 |")
    L += ["", "## prim", "", "| prim | 삼각형 | 용도 |", "| --- | ---: | --- |"]
    for k, n in prims.items():
        role = "충돌 (안 보임)" if k.endswith(("Collision", "Boundary")) else "보이기"
        L.append(f"| `{k}` | {n:,} | {role} |")
    L += ["", "## 점검 (C)", "", f"- USD 규격: {'통과' if not u.get('problems') else '문제 ' + str(u.get('problems'))} (Z-up · m · defaultPrim · 충돌 안 보임 · 텍스처 파일)"]
    c = r.get("colorize") or {}
    if c:
        L.append(f"- 키프레임 깊이 ↔ 메쉬: 중앙 {c.get('depth_vs_mesh_median_cm')} cm")
    col = r.get("collider") or {}
    if col:
        L.append(f"- 충돌 메쉬 오차: 95% {col.get('p95_cm')} cm" + (f" · 바닥 95% {col['floor_p95_cm']} cm" if "floor_p95_cm" in col else ""))
    if vis:
        L.append(f"- 주행 중 보이는 구멍: {json.dumps(vis.get('pct', vis), ensure_ascii=False)[:300]}")
    ask = (cfg.get("package") or {}).get("ask") or ["Isaac Sim 에서 열어 텍스처 · 충돌 확인", "충돌이 바뀌었으면 Nav2 지도 다시 만들기", "주행(X-Mobility · Nav2) 한 바퀴 결과"]
    L += ["", "## 확인 부탁 (D)", ""] + [f"- {a}" for a in ask] + [""]
    return "\n".join(L)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out")
    ap.add_argument("--version", required=True)
    ap.add_argument("--changes", help="README '바뀐 것' 에 넣을 글 (여러 줄은 \n)")
    args = ap.parse_args()
    cfg = config.load(args.config)
    cfg["_path"] = args.config
    out = os.path.expanduser(args.out or cfg["out"])
    r = json.load(open(os.path.join(out, "report.json"), encoding="utf-8"))
    if r.get("usd", {}).get("problems"):
        raise SystemExit(f"USD 검증 문제가 있어 패키지를 만들지 않음: {r['usd']['problems']}")
    vp = os.path.join(out, "visibility", "summary.json")
    vis = json.load(open(vp, encoding="utf-8")) if os.path.exists(vp) else None
    name = f"{cfg['name']}_{args.version}_{time.strftime('%Y-%m-%d')}"
    dst = os.path.join(out, "handoff", name)
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(os.path.join(out, "usd"), os.path.join(dst, "usd"))
    shutil.copy(os.path.join(out, "transform.json"), dst)
    files = {}
    pv = os.path.join(out, "preview")
    if os.path.isdir(pv):
        pngs = [f for f in os.listdir(pv) if f.endswith(".png")]
        os.makedirs(os.path.join(dst, "preview"))
        for f in pngs:
            shutil.copy(os.path.join(pv, f), os.path.join(dst, "preview", f))
        files["preview"] = len(pngs)
    tpl = (cfg.get("package") or {}).get("scene_template")
    if tpl:
        s = open(os.path.expanduser(tpl), encoding="utf-8").read().replace("{name}", cfg["name"]).replace("{version}", args.version)
        os.makedirs(os.path.join(dst, "scene_suggestion"))
        files["scene"] = f"{cfg['name']}_{args.version}.usda"
        open(os.path.join(dst, "scene_suggestion", files["scene"]), "w", encoding="utf-8").write(s)
    open(os.path.join(dst, "README_D.md"), "w", encoding="utf-8").write(readme(cfg, args.version, r, vis, (args.changes or "").replace("\n", "\n"), files))
    z = shutil.make_archive(dst, "zip", os.path.dirname(dst), name)
    print(f"패키지: {dst}\nzip: {z} · {os.path.getsize(z) / 1e6:.0f} MB · sha12 {sha12(z)}")


if __name__ == "__main__":
    main()
