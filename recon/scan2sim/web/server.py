"""scan2sim 웹 화면 — 브라우저에서 스캔 파일을 올리면 이 컴퓨터가 진단 · 보정 · 변환을 돌리고 결과를 보여 준다.

    cd recon
    python -m scan2sim.web.server [--port 8000] [--jobs <작업 폴더>] [--host 127.0.0.1] [--code <접속 코드>]

받는 파일: 아이폰 LiDAR 원본 zip (Stray Scanner) · GLB (스캔 앱 결과) · 맵 폴더를 묶은 zip
한 번에 한 건만 처리하고 나머지는 줄을 세운다. 작업마다 <jobs>/<작업 번호>/ 에 올린 파일(input/)과 결과(out/)를 둔다.
--host 0.0.0.0 이나 터널로 밖에 열 때는 --code 를 줄 것 — 링크를 아는 누구나 파일을 올려 이 컴퓨터에서 처리를 돌릴 수 있기 때문.

API (모두 JSON)
    POST /api/jobs {name, size}            → {id}                 올릴 자리 만들기
    PUT  /api/jobs/<id>/chunk?offset=N     → {received}           파일 조각 (차례대로)
    POST /api/jobs/<id>/start              → {status}             줄에 넣기
    GET  /api/jobs/<id>                    → 상태 · 단계별 결과 · 결함 목록
    GET  /api/jobs/<id>/preview.glb | map.png | report.md | bundle.zip
"""
import argparse
import glob
import io
import json
import os
import queue
import re
import subprocess
import sys
import threading
import time
import uuid
import zipfile

from flask import Flask, abort, jsonify, request, send_file, send_from_directory

HERE = os.path.dirname(os.path.abspath(__file__))
RECON = os.path.dirname(os.path.dirname(HERE))
STAGES = ["input", "convert", "nav2", "diagnose", "correct", "bundle", "verify", "report"]
LABEL = dict(input="입력 확인", convert="메쉬 · USD 만들기", nav2="Nav2 지도", diagnose="진단", correct="보정 · 다시 진단", bundle="묶음 만들기", verify="주행 검증", report="보고서")
EXT = (".zip", ".glb")
MAX_BYTES = 4 * 1024 ** 3

app = Flask(__name__, static_folder=os.path.join(HERE, "static"), static_url_path="/static")
CFG = dict(jobs=os.path.expanduser("~/scan2sim_jobs"), code=None)
JOBS, LOCK, Q = {}, threading.Lock(), queue.Queue()


def jdir(jid, *parts):
    if not re.fullmatch(r"[0-9a-f]{12}", jid or ""):
        abort(404)
    return os.path.join(CFG["jobs"], jid, *parts)


def job(jid):
    d = jdir(jid)
    with LOCK:
        j = JOBS.get(jid)
    if j is None:
        mp = os.path.join(d, "job.json")                        # 서버를 다시 켠 뒤에도 끝난 작업은 볼 수 있게
        if not os.path.exists(mp):
            abort(404)
        j = json.load(open(mp, encoding="utf-8"))
        if j["status"] in ("queued", "running", "uploading"):
            j["status"], j["error"] = "failed", "서버가 다시 켜져서 끊김"
        with LOCK:
            JOBS[jid] = j
    return j


def save(j):
    json.dump(j, open(os.path.join(CFG["jobs"], j["id"], "job.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)


@app.before_request
def gate():
    if CFG["code"] and request.path.startswith("/api/") and request.headers.get("X-Code", request.args.get("code")) != CFG["code"]:
        return jsonify(error="접속 코드가 맞지 않음"), 401


@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.get("/api/info")
def info():
    return jsonify(ok=True, stages=[dict(name=s, label=LABEL[s]) for s in STAGES], accept=EXT)


@app.post("/api/jobs")
def create():
    b = request.get_json(force=True, silent=True) or {}
    name, size = str(b.get("name", "")), int(b.get("size", 0))
    ext = os.path.splitext(name)[1].lower()
    if ext not in EXT:
        return jsonify(error=f"받는 파일은 {' · '.join(EXT)} 임"), 400
    if not 0 < size <= MAX_BYTES:
        return jsonify(error="파일 크기가 0 이거나 4 GB 를 넘음"), 400
    jid = uuid.uuid4().hex[:12]
    os.makedirs(jdir(jid, "input"))
    j = dict(id=jid, name=os.path.basename(name)[:120], size=size, received=0, status="uploading", created=time.time(), file="input" + ext)   # 경로에 한글이 있으면 Open3D 가 못 읽어 이름을 바꿔 둠
    with LOCK:
        JOBS[jid] = j
    save(j)
    return jsonify(id=jid)


@app.put("/api/jobs/<jid>/chunk")
def chunk(jid):
    j = job(jid)
    off = int(request.args.get("offset", -1))
    if j["status"] != "uploading" or off != j["received"]:
        return jsonify(error="조각 순서가 맞지 않음", received=j["received"]), 409
    n = 0
    with open(jdir(jid, "input", j["file"]), "ab") as f:
        while True:
            buf = request.stream.read(1 << 20)
            if not buf:
                break
            n += len(buf)
            if j["received"] + n > j["size"]:
                return jsonify(error="알린 크기보다 많이 옴"), 400
            f.write(buf)
    j["received"] += n
    return jsonify(received=j["received"])


@app.post("/api/jobs/<jid>/start")
def start(jid):
    j = job(jid)
    if j["status"] != "uploading" or j["received"] != j["size"]:
        return jsonify(error="파일이 다 오지 않음", received=j["received"]), 409
    j["status"], j["queued_at"] = "queued", time.time()
    save(j)
    Q.put(jid)
    return jsonify(status="queued")


def worker():
    while True:
        jid = Q.get()
        j = JOBS[jid]
        j["status"], j["started"] = "running", time.time()
        save(j)
        out = os.path.join(CFG["jobs"], jid, "out")
        try:
            with open(os.path.join(CFG["jobs"], jid, "run.txt"), "w", encoding="utf-8") as log:
                rc = subprocess.run([sys.executable, "-u", "-m", "scan2sim", os.path.join(CFG["jobs"], jid, "input", j["file"]), "--out", out], cwd=RECON, stdout=log, stderr=subprocess.STDOUT,
                                    env=dict(os.environ, PYTHONIOENCODING="utf-8")).returncode
            j["status"] = "done" if os.path.exists(os.path.join(out, "report.md")) else "failed"
            j["exit_code"] = rc
        except Exception as e:
            j["status"], j["error"] = "failed", f"{type(e).__name__}: {e}"
        j["finished"] = time.time()
        save(j)


def stage_results(out):
    res = {}
    for f in sorted(glob.glob(os.path.join(out, "stages", "*.json")), key=os.path.getmtime):
        try:
            d = json.load(open(f, encoding="utf-8"))
            res[d["stage"]] = d
        except Exception:
            pass                                                # 쓰는 중인 파일
    return res


def _json(path):
    return json.load(open(path, encoding="utf-8")) if os.path.exists(path) else None


@app.get("/api/jobs/<jid>")
def status(jid):
    j = dict(job(jid))
    out = jdir(jid, "out")
    res = stage_results(out) if j["status"] in ("running", "done", "failed") else {}
    st, running = [], j["status"] == "running"
    for s in STAGES:
        d = res.get(s)
        if d:
            st.append(dict(name=s, label=LABEL[s], status=d["status"], summary=d["summary"], messages=d["messages"], seconds=d["seconds"]))
        else:
            st.append(dict(name=s, label=LABEL[s], status="running" if running else "pending"))
            running = False
    j["stages"] = st
    j["queue_ahead"] = sum(1 for o in JOBS.values() if o["status"] in ("queued", "running") and o.get("queued_at", 0) < j.get("queued_at", 0)) if j["status"] == "queued" else 0
    if j["status"] in ("done", "failed"):
        j["defects_before"], j["defects_after"] = _json(os.path.join(out, "defects_before.json")), _json(os.path.join(out, "defects_after.json"))
        rep = _json(os.path.join(out, "report.json")) or {}
        j["verdict"] = rep.get("verdict") or next((s["status"] for s in st if s["name"] == "report"), None)
        j["has_bundle"] = os.path.isdir(os.path.join(out, "bundle", "usd"))
        d = res.get("diagnose", {}).get("metrics") or {}
        c = res.get("correct", {}).get("metrics") or {}
        j["metrics"] = dict(before=c.get("before") or d, after=c.get("after"), input=res.get("input", {}).get("metrics"))
    return jsonify(j)


@app.get("/api/jobs/<jid>/preview.glb")
def preview(jid):
    """보이는 메쉬(build/visual.ply, 씬 좌표 Z 위)를 화면용 GLB 로. 화면은 Y 가 위라서 돌려 둠 — 화면 (x, y, z) = 씬 (x, z, −y). 30만 삼각형까지 줄임."""
    job(jid)
    out = jdir(jid, "out")
    dst, src = os.path.join(out, "preview.glb"), os.path.join(out, "build", "visual.ply")
    if not os.path.exists(src):
        abort(404)
    if not os.path.exists(dst) or os.path.getmtime(dst) < os.path.getmtime(src):
        import numpy as np
        import open3d as o3d
        import trimesh
        m = o3d.io.read_triangle_mesh(src)
        if len(m.triangles) > 300_000:
            m = m.simplify_quadric_decimation(300_000)
        v = np.asarray(m.vertices)
        col = (np.asarray(m.vertex_colors) * 255).astype(np.uint8) if m.has_vertex_colors() else None
        tm = trimesh.Trimesh(np.c_[v[:, 0], v[:, 2], -v[:, 1]], np.asarray(m.triangles), vertex_colors=col, process=False)
        tm.export(dst)
    return send_file(dst, mimetype="model/gltf-binary", max_age=0)


@app.get("/api/jobs/<jid>/map.png")
def map_png(jid):
    job(jid)
    p = jdir(jid, "out", "bundle", "nav2", "map.pgm")
    if not os.path.exists(p):
        abort(404)
    from PIL import Image
    b = io.BytesIO()
    Image.open(p).save(b, "PNG")
    b.seek(0)
    return send_file(b, mimetype="image/png", max_age=0)


@app.get("/api/jobs/<jid>/report.md")
def report(jid):
    job(jid)
    p = jdir(jid, "out", "report.md")
    if not os.path.exists(p):
        abort(404)
    return send_file(p, mimetype="text/markdown; charset=utf-8", max_age=0, as_attachment=bool(request.args.get("dl")), download_name=f"scan2sim_report_{jid}.md")


@app.get("/api/jobs/<jid>/bundle.zip")
def bundle(jid):
    job(jid)
    out = jdir(jid, "out")
    b = os.path.join(out, "bundle")
    if not os.path.isdir(os.path.join(b, "usd")):
        abort(404)
    dst = os.path.join(out, "bundle.zip")
    if not os.path.exists(dst):
        tmp = dst + ".tmp"
        with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
            for root, _, files in os.walk(b):
                for f in files:
                    z.write(os.path.join(root, f), os.path.join("bundle", os.path.relpath(os.path.join(root, f), b)))
            for f in ("report.md", "defects_before.json", "defects_after.json"):
                if os.path.exists(os.path.join(out, f)):
                    z.write(os.path.join(out, f), f)
        os.replace(tmp, dst)
    return send_file(dst, as_attachment=True, download_name=f"scan2sim_{jid}.zip", max_age=0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--host", default="127.0.0.1", help="다른 컴퓨터에서 직접 들어오게 하려면 0.0.0.0 (터널을 쓰면 127.0.0.1 그대로)")
    ap.add_argument("--jobs", default=CFG["jobs"], help="작업 폴더 (올린 파일과 결과). 저장소 밖에 둘 것")
    ap.add_argument("--code", help="접속 코드 — 주면 화면에서 이 코드를 넣어야 올릴 수 있음")
    a = ap.parse_args()
    CFG.update(jobs=os.path.abspath(os.path.expanduser(a.jobs)), code=a.code)
    os.makedirs(CFG["jobs"], exist_ok=True)
    threading.Thread(target=worker, daemon=True).start()
    print(f"scan2sim 웹 화면: http://{a.host}:{a.port}  (작업 폴더 {CFG['jobs']})", flush=True)
    app.run(host=a.host, port=a.port, threaded=True)            # Flask 내장 서버 — 시연용 (한 번에 몇 명). 처리는 worker 한 줄이 차례로 함


if __name__ == "__main__":
    main()
