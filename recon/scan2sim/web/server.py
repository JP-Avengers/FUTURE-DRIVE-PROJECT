"""scan2sim 웹 화면 — 브라우저에서 스캔 파일을 올리면 이 컴퓨터가 진단 · 보정 · 변환을 돌리고 결과를 보여 준다.

    cd recon
    python -m scan2sim.web.server [--port 8000] [--jobs <작업 폴더>] [--host 127.0.0.1] [--code <접속 코드>] [--keep 5]
                                  [--samples <예시 파일 폴더>] [--timeout-min 20] [--max-wait 5] [--min-free-gb 20] [--ttl-hours 24] [--dev]

받는 파일: 아이폰 LiDAR 원본 zip (Stray Scanner) · GLB (스캔 앱 결과) · 맵 폴더를 묶은 zip
한 번에 한 건만 처리하고 나머지는 줄을 세운다. 작업마다 <jobs>/<작업 번호>/ 에 올린 파일(input/)과 결과(out/)를 둔다.
--host 0.0.0.0 이나 터널로 밖에 열 때는 --code 를 줄 것 — 링크를 아는 누구나 파일을 올려 이 컴퓨터에서 처리를 돌릴 수 있기 때문.

밖에 열어도 버티게 하는 것 (값은 위 선택 사항으로 바꿈, 0 이면 그 검사를 끔)
    · 처리 제한 시간: 엔진이 --timeout-min 분을 넘으면 죽이고 failed
    · 입력 검사: 줄에 넣기 전에 zip 은 항목 수 · 풀린 크기, GLB 는 앞머리를 봄 (파일을 풀지는 않음)
    · 줄 길이: 기다리는 작업이 --max-wait 건이면 새 작업을 429 로 거절
    · 디스크: 올릴 파일을 받고 나면 여유가 --min-free-gb 아래가 되는 경우 507 로 거절
    · 청소: 끝난 지 --ttl-hours 시간이 지난 작업 폴더와 그만큼 멈춰 있는 올리다 만 작업을 통째로 지움
    · waitress 가 설치돼 있으면 그것으로 켬 (없거나 --dev 면 Flask 내장 서버)

API (모두 JSON)
    POST /api/jobs {name, size}            → {id}                 올릴 자리 만들기
    PUT  /api/jobs/<id>/chunk?offset=N     → {received}           파일 조각 (차례대로)
    POST /api/jobs/<id>/start              → {status}             줄에 넣기
    GET  /api/jobs/<id>                    → 상태 · 단계별 결과 · 결함 목록
    GET  /api/jobs/<id>/preview.glb | map.png | report.md | bundle.zip
    GET  /api/samples                      → [{name, label, size}] 서버에 준비된 예시 파일 (--samples 폴더)
    POST /api/samples/<name>/jobs          → {id}                 예시 파일로 작업 만들기 (올리기 없이 바로 줄에 들어감)
    GET  /api/health                       → {ok, queue_length, running, free_gb}
"""
import argparse
import glob
import io
import json
import os
import queue
import re
import shutil
import signal
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
ZIP_MAX_FILES, ZIP_MAX_BYTES = 100_000, 6 * 1024 ** 3          # stray.py 의 ZIP_MAX_FILES · ZIP_MAX_BYTES 와 같은 값 (엔진이 받는 것을 서버가 막지 않게)
SAMPLE_NAME = r"[A-Za-z0-9][A-Za-z0-9._-]{0,79}"

app = Flask(__name__, static_folder=os.path.join(HERE, "static"), static_url_path="/static")
CFG = dict(jobs=os.path.expanduser("~/scan2sim_jobs"), code=None, keep=5, samples=None, timeout_min=20.0, max_wait=5, min_free_gb=20.0, ttl_hours=24.0)
JOBS, LOCK, Q = {}, threading.Lock(), queue.Queue()
PREVIEW_LOCK = threading.Lock()
BUNDLE_LOCK = threading.Lock()


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


def purge(ttl_hours):
    """오래된 작업 폴더를 통째로 지움 — 서버를 켤 때 · 작업이 끝날 때 · 10분마다 부름. ttl_hours 가 0 이하면 하지 않음.
    · 끝난 작업(done · failed): 끝난 때부터 ttl_hours 시간이 지난 것
    · 올리다 만 작업(uploading): 마지막 조각을 받은 때(없으면 만든 때)부터 ttl_hours 시간이 지난 것
    기다리는 · 도는 작업과 job.json 이 없는 폴더는 건드리지 않음. 지운 작업 번호를 돌려줌"""
    if ttl_hours <= 0:
        return []
    cut, gone = time.time() - ttl_hours * 3600, []
    for mp in glob.glob(os.path.join(CFG["jobs"], "*", "job.json")):
        d = os.path.dirname(mp)
        jid = os.path.basename(d)
        if not re.fullmatch(r"[0-9a-f]{12}", jid):
            continue
        with LOCK:
            j = JOBS.get(jid)
        try:
            j = j or json.load(open(mp, encoding="utf-8"))
        except Exception:
            continue
        st = j.get("status")
        if st in ("done", "failed"):
            last = j.get("finished") or os.path.getmtime(mp)
        elif st == "uploading":
            fp = os.path.join(d, "input", j.get("file", ""))
            last = os.path.getmtime(fp) if os.path.isfile(fp) else j.get("created", os.path.getmtime(mp))
        else:
            continue
        if last < cut:
            with LOCK:
                JOBS.pop(jid, None)
            shutil.rmtree(d, ignore_errors=True)
            gone.append(jid)
    return gone


def save(j):
    json.dump(j, open(os.path.join(CFG["jobs"], j["id"], "job.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)


def waiting():
    with LOCK:
        return sum(1 for o in JOBS.values() if o["status"] == "queued")


def free_bytes():
    return shutil.disk_usage(CFG["jobs"]).free


def busy(size):
    """새 작업을 받을 수 없으면 (HTTP 상태, 사용자에게 보일 문장), 받을 수 있으면 None — 큰 파일을 다 올린 뒤가 아니라 올리기 전에 거절함."""
    n = waiting()
    if 0 < CFG["max_wait"] <= n:
        return 429, f"기다리는 작업이 {n}건이라 지금은 받을 수 없음 — 몇 분 뒤 다시 올릴 것"
    if CFG["min_free_gb"] > 0 and free_bytes() - size < CFG["min_free_gb"] * 1024 ** 3:
        return 507, "서버 디스크 여유가 모자라 지금은 받을 수 없음 — 서버를 맡은 사람에게 알릴 것"
    return None


def check_input(path):
    """줄에 넣기 전 검사 — 문제가 있으면 사용자에게 보일 문장, 없으면 None. 파일을 풀지 않고 목록 · 앞머리만 봄.
    zip 안의 경로가 폴더 밖을 가리키는지 · Stray 촬영본으로서 맞는지는 엔진(stray.py)이 본다."""
    if path.lower().endswith(".glb"):
        with open(path, "rb") as f:
            if f.read(4) != b"glTF":
                return "GLB 파일이 아님 (파일 앞머리가 glTF 가 아님) — 스캔 앱에서 GLB 로 다시 내보내 올릴 것"
        return None
    try:
        with zipfile.ZipFile(path) as z:
            infos = z.infolist()
    except (zipfile.BadZipFile, OSError, ValueError, NotImplementedError):
        return "zip 파일을 열 수 없음 — zip 이 아니거나 올리는 중에 깨짐. 다시 올릴 것"
    if len(infos) > ZIP_MAX_FILES:
        return f"zip 안의 파일이 {len(infos):,}개로 너무 많음 (최대 {ZIP_MAX_FILES:,}개)"
    total = sum(i.file_size for i in infos)
    if total > ZIP_MAX_BYTES:
        return f"zip 을 풀면 {total / 1024 ** 3:.1f} GB 로 너무 큼 (최대 {ZIP_MAX_BYTES // 1024 ** 3} GB)"
    return None


def enqueue(j):
    """다 받은 파일을 검사하고 줄에 넣음. 검사에 걸리면 작업을 failed 로 두고 그 문장을 돌려줌 (통과면 None)."""
    msg = check_input(os.path.join(CFG["jobs"], j["id"], "input", j["file"]))
    if msg:
        j["status"], j["error"], j["finished"] = "failed", msg, time.time()
        save(j)
        return msg
    j["status"], j["queued_at"] = "queued", time.time()
    save(j)
    Q.put(j["id"])
    return None


def samples():
    """--samples 폴더의 예시 파일 목록 [{name, label, size}]. 이름표는 그 폴더의 samples.json {파일 이름: 이름표} 에서, 없으면 확장자를 뺀 파일 이름."""
    d = CFG["samples"]
    if not d or not os.path.isdir(d):
        return []
    try:
        labels = _json(os.path.join(d, "samples.json")) or {}
    except Exception:
        labels = {}
    res = []
    for f in sorted(os.listdir(d)):
        p = os.path.join(d, f)
        if re.fullmatch(SAMPLE_NAME, f) and os.path.splitext(f)[1].lower() in EXT and os.path.isfile(p):
            res.append(dict(name=f, label=str(labels.get(f) or os.path.splitext(f)[0]), size=os.path.getsize(p)))
    return res


@app.before_request
def gate():
    if CFG["code"] and request.path.startswith("/api/") and request.headers.get("X-Code", request.args.get("code")) != CFG["code"]:
        return jsonify(error="접속 코드가 맞지 않음"), 401


@app.get("/")
def index():
    return send_from_directory(app.static_folder, "index.html")


@app.get("/api/info")
def info():
    return jsonify(ok=True, stages=[dict(name=s, label=LABEL[s]) for s in STAGES], accept=EXT, max_bytes=MAX_BYTES, samples=len(samples()))


@app.get("/api/health")
def health():
    with LOCK:
        running = any(o["status"] == "running" for o in JOBS.values())
    return jsonify(ok=True, queue_length=waiting(), running=running, free_gb=round(free_bytes() / 1024 ** 3, 1))


@app.get("/api/samples")
def sample_list():
    return jsonify(samples())


@app.post("/api/samples/<name>/jobs")
def sample_job(name):
    s = next((s for s in samples() if s["name"] == name), None)   # 목록에 있는 이름만 — 경로를 직접 만들지 않음
    if s is None:
        abort(404)
    no = busy(s["size"])
    if no:
        return jsonify(error=no[1]), no[0]
    ext = os.path.splitext(name)[1].lower()
    jid = uuid.uuid4().hex[:12]
    os.makedirs(jdir(jid, "input"))
    src, dst = os.path.join(CFG["samples"], name), jdir(jid, "input", "input" + ext)
    try:
        os.link(src, dst)                                       # 같은 디스크면 복사 없이. 작업 폴더를 지워도 예시 파일은 남음
    except OSError:
        shutil.copyfile(src, dst)
    j = dict(id=jid, name=name, size=s["size"], received=s["size"], status="uploading", created=time.time(), file="input" + ext, sample=name)
    with LOCK:
        JOBS[jid] = j
    msg = enqueue(j)
    if msg:
        return jsonify(error=msg, id=jid), 400
    return jsonify(id=jid)


@app.post("/api/jobs")
def create():
    b = request.get_json(force=True, silent=True) or {}
    try:
        name, size = str(b.get("name", "")), int(b.get("size", 0))
    except (TypeError, ValueError, AttributeError):
        return jsonify(error="name · size 를 읽을 수 없음"), 400
    ext = os.path.splitext(name)[1].lower()
    if ext not in EXT:
        return jsonify(error=f"받는 파일은 {' · '.join(EXT)} 임"), 400
    if not 0 < size <= MAX_BYTES:
        return jsonify(error="파일 크기가 0 이거나 4 GB 를 넘음"), 400
    no = busy(size)
    if no:
        return jsonify(error=no[1]), no[0]
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
    try:
        off = int(request.args.get("offset", -1))
    except ValueError:
        return jsonify(error="offset 은 숫자여야 함", received=j["received"]), 400
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
    msg = enqueue(j)
    if msg:
        return jsonify(error=msg), 400
    return jsonify(status="queued")


def kill_tree(p):
    """엔진과 엔진이 띄운 프로세스(Nav2 지도 · 파이프라인 스크립트)를 함께 죽임.
    엔진을 새 프로세스 묶음으로 띄우지 않는 까닭: deploy/stop.sh 가 서버의 묶음째 끄는데, 묶음이 다르면 엔진만 남아 계속 돎."""
    if os.name == "nt":
        subprocess.run(["taskkill", "/F", "/T", "/PID", str(p.pid)], capture_output=True)
    else:
        kids = {}
        for d in os.listdir("/proc") if os.path.isdir("/proc") else []:
            if d.isdigit():
                try:
                    with open(f"/proc/{d}/stat", encoding="utf-8", errors="replace") as f:
                        kids.setdefault(int(f.read().rsplit(")", 1)[1].split()[1]), []).append(int(d))
                except (OSError, IndexError, ValueError):
                    pass                                        # 그사이 끝난 프로세스
        todo, pids = [p.pid], []
        while todo:
            x = todo.pop()
            pids.append(x)
            todo += kids.get(x, [])
        for x in pids:
            try:
                os.kill(x, signal.SIGKILL)
            except OSError:
                pass
    try:
        p.wait(timeout=10)
    except subprocess.TimeoutExpired:
        pass


def worker():
    while True:
        jid = Q.get()
        with LOCK:
            j = JOBS.get(jid)
        if j is None:
            continue
        j["status"], j["started"] = "running", time.time()
        save(j)
        out = os.path.join(CFG["jobs"], jid, "out")
        limit = CFG["timeout_min"] * 60 if CFG["timeout_min"] > 0 else None
        try:
            with open(os.path.join(CFG["jobs"], jid, "run.txt"), "w", encoding="utf-8") as log:
                p = subprocess.Popen([sys.executable, "-u", "-m", "scan2sim", os.path.join(CFG["jobs"], jid, "input", j["file"]), "--out", out], cwd=RECON, stdout=log, stderr=subprocess.STDOUT,
                                     env=dict(os.environ, PYTHONIOENCODING="utf-8"))
                try:
                    rc = p.wait(timeout=limit)
                except subprocess.TimeoutExpired:
                    kill_tree(p)
                    rc = None
                    log.write(f"\n[서버] 처리 제한 시간 {CFG['timeout_min']:g}분을 넘어 엔진을 멈춤\n")
            if rc is None:
                final, j["timed_out"] = "failed", True
                j["error"] = f"처리가 {CFG['timeout_min']:g}분을 넘어 멈춤 — 촬영본이 너무 길거나 넓을 수 있음. 더 짧게 나눠 찍어 다시 올릴 것"
            else:
                # report 단계는 앞 단계가 실패해도 report.md 를 쓰므로, 종료 코드와 단계 결과도 봄 (엔진은 실패 단계가 있으면 1 로 끝남)
                bad = rc != 0 or any(d.get("status") == "fail" for d in stage_results(out).values())
                final = "done" if os.path.exists(os.path.join(out, "report.md")) and not bad else "failed"
            j["exit_code"] = rc
        except Exception as e:
            final, j["error"] = "failed", f"{type(e).__name__}: {e}"
        try:
            make_bundle(out)                                    # 내려받기를 처음 누를 때가 아니라 지금 만들어 둠 (큰 맵이면 첫 요청이 오래 걸림)
        except Exception as e:
            app.logger.warning("bundle.zip 을 미리 만들지 못함 (%s): %s: %s", jid, type(e).__name__, e)
        j["status"], j["finished"] = final, time.time()   # 상태와 끝난 때를 같이 씀 — bundle.zip 을 만드는 동안은 running 으로 보임 (R21)
        save(j)
        try:
            tidy(CFG["keep"])
            purge(CFG["ttl_hours"])
        except Exception as e:
            app.logger.warning("작업 폴더 정리 실패: %s: %s", type(e).__name__, e)


def sweeper():
    while True:
        time.sleep(600)
        try:
            gone = purge(CFG["ttl_hours"])
            if gone:
                print(f"오래된 작업 {len(gone)}건을 지움", flush=True)
        except Exception as e:
            app.logger.warning("오래된 작업 지우기 실패: %s: %s", type(e).__name__, e)


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
    j["queue_ahead"] = sum(1 for o in list(JOBS.values()) if o["status"] in ("queued", "running") and o.get("queued_at", 0) < j.get("queued_at", 0)) if j["status"] == "queued" else 0
    if j["status"] in ("done", "failed"):
        j["defects_before"], j["defects_after"] = _json(os.path.join(out, "defects_before.json")), _json(os.path.join(out, "defects_after.json"))
        rep = _json(os.path.join(out, "report.json")) or {}
        v = rep.get("verdict") or next((s["status"] for s in st if s["name"] == "report"), None)
        if j["status"] == "failed" and v in (None, "pending", "running", "skip"):
            v = "fail"                                          # 입력 검사 · 제한 시간 · 재시작으로 끝난 작업은 report 가 없음 — 이유는 error 에 있음 (R21)
        j["verdict"] = v
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


def make_bundle(out):
    """<out>/bundle.zip 을 만들어 두고 그 경로를 돌려줌 (이미 있으면 그대로). 묶음(bundle/usd)이 없으면 None."""
    b = os.path.join(out, "bundle")
    if not os.path.isdir(os.path.join(b, "usd")):
        return None
    dst = os.path.join(out, "bundle.zip")
    with BUNDLE_LOCK:                                           # 작업이 끝날 때와 내려받기 요청이 같은 파일을 함께 만들지 않게
        if not os.path.exists(dst):
            tmp = dst + ".tmp"
            with zipfile.ZipFile(tmp, "w", zipfile.ZIP_DEFLATED) as z:
                for root, _, files in os.walk(b):
                    for f in files:
                        z.write(os.path.join(root, f), os.path.join("bundle", os.path.relpath(os.path.join(root, f), b)))
                for f in ("report.md", "defects_before.json", "defects_after.json"):
                    if os.path.exists(os.path.join(out, f)):
                        z.write(os.path.join(out, f), os.path.join("bundle", f))    # 맨 위가 폴더 하나(bundle/) — sim/add_site.py 가 그 안을 맵 패키지로 봄
            os.replace(tmp, dst)
    return dst


@app.get("/api/jobs/<jid>/bundle.zip")
def bundle(jid):
    job(jid)
    dst = make_bundle(jdir(jid, "out"))
    if dst is None:
        abort(404)
    return send_file(dst, as_attachment=True, download_name=f"scan2sim_{jid}.zip", max_age=0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--host", default="127.0.0.1", help="다른 컴퓨터에서 직접 들어오게 하려면 0.0.0.0 (터널을 쓰면 127.0.0.1 그대로)")
    ap.add_argument("--jobs", default=CFG["jobs"], help="작업 폴더 (올린 파일과 결과). 저장소 밖에 둘 것")
    ap.add_argument("--code", help="접속 코드 — 주면 화면에서 이 코드를 넣어야 올릴 수 있음")
    ap.add_argument("--keep", type=int, default=CFG["keep"], help="올린 파일 · 중간 파일까지 남겨 둘 최근 작업 수. 더 오래된 끝난 작업은 결과만 남김 (-1 이면 올린 파일 · 중간 파일을 지우지 않음. zip 을 풀어 둔 사본은 늘 지우고, 폴더를 통째로 지우는 것은 --ttl-hours 가 따로 함)")
    ap.add_argument("--samples", help="예시 파일(.zip · .glb) 폴더 — 주면 /api/samples 로 알려 주고, 올리기 없이 그 파일로 작업을 만들 수 있음. 이름표는 폴더의 samples.json {파일 이름: 이름표}")
    ap.add_argument("--timeout-min", type=float, default=CFG["timeout_min"], help="엔진 처리 제한 시간(분). 넘으면 죽이고 failed (0 이면 끝날 때까지 기다림)")
    ap.add_argument("--max-wait", type=int, default=CFG["max_wait"], help="기다리는 작업이 이 수면 새 작업을 429 로 거절 (0 이면 제한 없음)")
    ap.add_argument("--min-free-gb", type=float, default=CFG["min_free_gb"], help="올릴 파일을 받고 나면 작업 폴더 디스크 여유가 이 값(GB) 아래가 되는 경우 507 로 거절 (0 이면 안 봄)")
    ap.add_argument("--ttl-hours", type=float, default=CFG["ttl_hours"], help="끝난 지 이 시간이 지난 작업 폴더와 그만큼 멈춰 있는 올리다 만 작업을 통째로 지움 (0 이면 지우지 않음)")
    ap.add_argument("--dev", action="store_true", help="waitress 가 있어도 Flask 내장 서버로 켬")
    a = ap.parse_args()
    CFG.update(jobs=os.path.abspath(os.path.expanduser(a.jobs)), code=a.code, keep=a.keep, samples=os.path.abspath(os.path.expanduser(a.samples)) if a.samples else None,
               timeout_min=a.timeout_min, max_wait=a.max_wait, min_free_gb=a.min_free_gb, ttl_hours=a.ttl_hours)
    os.makedirs(CFG["jobs"], exist_ok=True)
    if CFG["samples"]:
        print(f"예시 파일 {len(samples())}개 ({CFG['samples']})", flush=True)
    n = recover()
    if n:
        print(f"꺼지기 전에 기다리던 · 돌던 작업 {n}건을 줄에 다시 넣음", flush=True)
    gone = purge(CFG["ttl_hours"])
    if gone:
        print(f"오래된 작업 {len(gone)}건을 지움", flush=True)
    threading.Thread(target=worker, daemon=True).start()
    threading.Thread(target=sweeper, daemon=True).start()
    serve = None
    if not a.dev:
        try:
            from waitress import serve
        except ImportError:
            print("waitress 가 없어 Flask 내장 서버로 켬 (pip install -r recon/requirements.txt)", flush=True)
    print(f"scan2sim 웹 화면: http://{a.host}:{a.port}  (작업 폴더 {CFG['jobs']} · {'waitress' if serve else 'Flask 내장 서버'})", flush=True)
    if serve:                                                   # 처리는 worker 한 줄이 차례로 함. 여기 스레드는 요청 받기(올리기 · 상태 · 내려받기)만
        serve(app, host=a.host, port=a.port, threads=16, max_request_body_size=MAX_BYTES, channel_timeout=300, ident="scan2sim")
    else:
        app.run(host=a.host, port=a.port, threaded=True)        # Flask 내장 서버 — 혼자 돌려 볼 때


if __name__ == "__main__":
    main()
