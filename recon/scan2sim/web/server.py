"""scan2sim 웹 화면 — 브라우저에서 스캔 파일을 올리면 이 컴퓨터가 진단 · 보정 · 변환을 돌리고 결과를 보여 준다.

    cd recon
    python -m scan2sim.web.server [--port 8000] [--jobs <작업 폴더>] [--host 127.0.0.1] [--code <접속 코드>] [--keep 5]

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
CFG = dict(jobs=os.path.expanduser("~/scan2sim_jobs"), code=None, keep=5)
JOBS, LOCK, Q = {}, threading.Lock(), queue.Queue()
PREVIEW_LOCK = threading.Lock()


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
        if j["status"] == "uploading":                          # 받은 만큼은 파일에 있음 — 화면이 거기서부터 이어 올림
            fp = os.path.join(d, "input", j["file"])
            j["received"] = min(os.path.getsize(fp), j["size"]) if os.path.exists(fp) else 0
        elif j["status"] in ("queued", "running"):              # recover() 가 못 살린 것 (이미 한 번 다시 돌렸음)
            j["status"], j["error"] = "failed", "서버가 다시 켜져서 끊김"
        with LOCK:
            j = JOBS.setdefault(jid, j)
    return j


def recover():
    """서버를 켤 때 한 번 — 꺼지기 전에 기다리던 · 돌던 작업을 줄에 다시 넣음 (작업마다 한 번만. 서버를 죽이는 작업이 끝없이 되돌지 않게)."""
    back = []
    for mp in glob.glob(os.path.join(CFG["jobs"], "*", "job.json")):
        try:
            j = json.load(open(mp, encoding="utf-8"))
        except Exception:
            continue
        if j.get("status") in ("queued", "running") and re.fullmatch(r"[0-9a-f]{12}", j.get("id", "")):
            if j.get("restarts", 0) >= 1:
                j["status"], j["error"], j["finished"] = "failed", "서버가 다시 켜져서 끊김 (두 번째)", time.time()
            else:
                j["status"], j["restarts"] = "queued", j.get("restarts", 0) + 1
                back.append(j)
            JOBS[j["id"]] = j
            save(j)
    for j in sorted(back, key=lambda j: j.get("queued_at", 0)):
        Q.put(j["id"])
    return len(back)


def tidy(keep):
    """작업 폴더 용량 줄이기 — 작업이 끝날 때마다 부름. 결과를 보는 데 쓰는 파일(preview.glb · bundle · report · 결함 목록 · 단계 결과)은 남김.
    · 끝난 작업 모두: 올린 zip 을 풀어 둔 사본(out/input_map/_unzip)을 지움 — 올린 파일(input/)과 같은 내용
    · 최근 keep 건보다 오래된 끝난 작업: 올린 파일(input/)과 중간 파일(out/input_map · out/build)도 지움. keep 이 0 보다 작으면 하지 않음"""
    import shutil
    done = []
    for mp in glob.glob(os.path.join(CFG["jobs"], "*", "job.json")):
        try:
            j = json.load(open(mp, encoding="utf-8"))
        except Exception:
            continue
        if j.get("status") in ("done", "failed") and j.get("finished"):
            done.append((j["finished"], os.path.dirname(mp)))
    for i, (_, d) in enumerate(sorted(done, reverse=True)):
        gone = [os.path.join(d, "out", "input_map", "_unzip")]
        if 0 <= keep <= i:
            gone += [os.path.join(d, "input"), os.path.join(d, "out", "input_map")]
            if os.path.exists(os.path.join(d, "out", "preview.glb")) or not os.path.isdir(os.path.join(d, "out", "build", "usd")):
                gone.append(os.path.join(d, "out", "build"))    # 화면용 파일이 아직 없으면 그것을 만들 USD 는 남겨 둠
        for g in gone:
            shutil.rmtree(g, ignore_errors=True)


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
        f.truncate(j["received"])                               # 앞 요청이 도중에 끊겼으면 받다 만 바이트가 남아 있음 — 잘라내고 이어 붙임
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
            # report 단계는 앞 단계가 실패해도 report.md 를 쓰므로, 종료 코드와 단계 결과도 봄 (엔진은 실패 단계가 있으면 1 로 끝남)
            bad = rc != 0 or any(d.get("status") == "fail" for d in stage_results(out).values())
            j["status"] = "done" if os.path.exists(os.path.join(out, "report.md")) and not bad else "failed"
            j["exit_code"] = rc
        except Exception as e:
            j["status"], j["error"] = "failed", f"{type(e).__name__}: {e}"
        j["finished"] = time.time()
        save(j)
        try:
            tidy(CFG["keep"])
        except Exception as e:
            app.logger.warning("작업 폴더 정리 실패: %s: %s", type(e).__name__, e)


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
    """화면용 GLB — 엔진이 묶음 단계 끝에 줄여서 써 둔 <out>/preview.glb 를 그대로 줌 (scan2sim/preview.py · 씬 → 화면 좌표 변환도 거기서 함).
    파일이 없거나 USD 보다 오래됐으면(preview.py 가 생기기 전에 돌린 작업 폴더) 같은 엔진 함수로 한 번 만들어 둠. USD 도 없으면 404."""
    job(jid)
    out = jdir(jid, "out")
    dst = os.path.join(out, "preview.glb")
    usd = sorted(glob.glob(os.path.join(out, "build", "usd", "*_mesh.usdc")))
    with PREVIEW_LOCK:                                          # 같은 파일을 두 요청이 함께 만들지 않게
        if usd and (not os.path.exists(dst) or os.path.getmtime(dst) < os.path.getmtime(usd[0])):
            try:
                from .. import preview as engine_preview
                engine_preview.write(usd[0], dst)
            except Exception as e:
                app.logger.warning("preview.glb 를 만들지 못함 (%s): %s: %s", jid, type(e).__name__, e)
    if not os.path.exists(dst):
        abort(404)
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
    ap.add_argument("--keep", type=int, default=CFG["keep"], help="올린 파일 · 중간 파일까지 남겨 둘 최근 작업 수. 더 오래된 끝난 작업은 결과만 남김 (-1 이면 지우지 않음)")
    a = ap.parse_args()
    CFG.update(jobs=os.path.abspath(os.path.expanduser(a.jobs)), code=a.code, keep=a.keep)
    os.makedirs(CFG["jobs"], exist_ok=True)
    n = recover()
    if n:
        print(f"꺼지기 전에 기다리던 · 돌던 작업 {n}건을 줄에 다시 넣음", flush=True)
    threading.Thread(target=worker, daemon=True).start()
    print(f"scan2sim 웹 화면: http://{a.host}:{a.port}  (작업 폴더 {CFG['jobs']})", flush=True)
    app.run(host=a.host, port=a.port, threaded=True)            # Flask 내장 서버 — 시연용 (한 번에 몇 명). 처리는 worker 한 줄이 차례로 함


if __name__ == "__main__":
    main()
