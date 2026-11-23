"""전체 흐름 시험 — 서버를 띄워 실제 사용자처럼 파일을 올리고, 끝날 때까지 기다린 뒤 결과가 약속(docs/CONTRACT.md)에 맞는지 본다.

    cd recon
    python -m scan2sim.tests.e2e <파일(zip · glb)> [--truth room_truth.json] [--server http://… --code …] [--json 결과.json]

--server 를 안 주면 빈 포트에 서버를 새로 띄우고(임시 작업 폴더 · 무작위 접속 코드) 끝나면 끈다.
--truth 를 주면 (samples/make_room.py 가 쓴 정답) 넣어 둔 결함마다 같은 종류의 결함이 정답 자리에서 --tol 안에 잡혔는지 본다.
검사 항목: 접속 코드 없이 401 · 조각 순서가 틀리면 409 · 상태가 done · 상태 응답 필드 · 결함 목록 필드 · 결과 파일 4개(preview.glb · map.png · report.md · bundle.zip)
종료 코드: 0 = 모두 통과 · 1 = 하나라도 실패 · 2 = 서버가 안 뜸 / 시간 초과
"""
import argparse
import io
import json
import math
import os
import secrets
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
import zipfile

RECON = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CHUNK = 8 * 1024 * 1024
JOB_KEYS = ("id", "name", "size", "received", "status", "stages", "queue_ahead")
DONE_KEYS = ("verdict", "has_bundle", "metrics", "defects_before", "defects_after")
STAGE_KEYS = ("name", "label", "status")
DEFECT_KEYS = ("id", "type", "title", "severity", "fixable", "center_scene", "center_input", "radius_m", "extent_m", "samples", "basis", "action")


class Api:
    def __init__(self, base, code=None):
        self.base, self.code = base.rstrip("/"), code

    def call(self, method, path, body=None, ctype=None, code=True, raw=False, timeout=300):
        h = {}
        if code and self.code:
            h["X-Code"] = self.code
        if ctype:
            h["Content-Type"] = ctype
        if isinstance(body, (dict, list)):
            body, h["Content-Type"] = json.dumps(body).encode(), "application/json"
        req = urllib.request.Request(self.base + path, data=body, method=method, headers=h)
        try:
            with urllib.request.urlopen(req, timeout=timeout) as r:
                data = r.read()
                return r.status, (data if raw else json.loads(data or b"null"))
        except urllib.error.HTTPError as e:
            data = e.read()
            try:
                return e.code, json.loads(data)
            except Exception:
                return e.code, data


class Checks:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail=""):
        self.rows.append(dict(name=name, ok=bool(ok), detail=str(detail)))
        print(f"  {'통과' if ok else '실패'}  {name}" + (f" — {detail}" if detail else ""), flush=True)
        return ok

    @property
    def ok(self):
        return all(r["ok"] for r in self.rows)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def start_server(jobs, code, log):
    port = free_port()
    p = subprocess.Popen([sys.executable, "-m", "scan2sim.web.server", "--port", str(port), "--jobs", jobs, "--code", code], cwd=RECON,
                         stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONIOENCODING="utf-8"))
    api = Api(f"http://127.0.0.1:{port}", code)
    for _ in range(60):
        if p.poll() is not None:
            break
        try:
            if api.call("GET", "/api/info", timeout=2)[0] == 200:
                return p, api
        except Exception:
            pass
        time.sleep(0.5)
    p.kill()
    return None, None


def upload(api, path, C):
    size = os.path.getsize(path)
    st, r = api.call("POST", "/api/jobs", dict(name=os.path.basename(path), size=size))
    if not C.add("올릴 자리 만들기", st == 200 and isinstance(r, dict) and "id" in r, f"HTTP {st} {r if st != 200 else ''}"):
        return None
    jid = r["id"]
    st, r = api.call("PUT", f"/api/jobs/{jid}/chunk?offset=5", b"x", ctype="application/octet-stream")
    C.add("조각 순서가 틀리면 409", st == 409 and isinstance(r, dict) and r.get("received") == 0, f"HTTP {st}")
    t = time.time()
    with open(path, "rb") as f:
        off = 0
        while off < size:
            buf = f.read(CHUNK)
            st, r = api.call("PUT", f"/api/jobs/{jid}/chunk?offset={off}", buf, ctype="application/octet-stream")
            if st != 200 or r.get("received") != off + len(buf):
                C.add("조각 올리기", False, f"offset {off}: HTTP {st} {r}")
                return None
            off += len(buf)
    C.add("조각 올리기", True, f"{size / 1e6:.1f} MB · {math.ceil(size / CHUNK)}조각 · {time.time() - t:.1f}초")
    st, r = api.call("POST", f"/api/jobs/{jid}/start")
    C.add("줄에 넣기", st == 200 and r.get("status") == "queued", f"HTTP {st} {r}")
    return jid


def wait(api, jid, timeout, C):
    t0, seen = time.time(), None
    while time.time() - t0 < timeout:
        st, j = api.call("GET", f"/api/jobs/{jid}")
        if st != 200:
            C.add("상태 조회", False, f"HTTP {st}")
            return None
        run = next((s["name"] for s in j.get("stages", []) if s.get("status") == "running"), None)
        if run != seen:
            print(f"    … {j['status']} {run or ''} ({time.time() - t0:.0f}초)", flush=True)
            seen = run
        if j["status"] in ("done", "failed"):
            j["_seconds"] = round(time.time() - t0, 1)
            return j
        time.sleep(2)
    C.add("시간 안에 끝남", False, f"{timeout}초 초과")
    return None


def check_job(j, C):
    C.add("상태가 done", j["status"] == "done", f"{j['status']} · {j.get('error', '')} · 판정 {j.get('verdict')}")
    miss = [k for k in JOB_KEYS + DONE_KEYS if k not in j]
    C.add("상태 응답 필드", not miss, f"없음 {miss}" if miss else "")
    bad = [s.get("name") for s in j.get("stages", []) if any(k not in s for k in STAGE_KEYS) or s["status"] not in ("pass", "warn", "fail", "skip")]
    C.add("단계 8개 · 판정 값", len(j.get("stages", [])) == 8 and not bad, " · ".join(f"{s['name']} {s['status']}" for s in j.get("stages", [])))
    db = j.get("defects_before") or {}
    ds = db.get("defects")
    ok = isinstance(ds, list) and all(not [k for k in DEFECT_KEYS if k not in d] for d in ds) and all(k in db for k in ("phase", "T_input_to_scene"))
    C.add("결함 목록 필드", ok, f"보정 전 {len(ds or [])}곳 · 보정 뒤 {len((j.get('defects_after') or {}).get('defects') or []) if j.get('defects_after') else '없음'}")
    failed = [s for s in j.get("stages", []) if s.get("status") == "fail"]
    if not C.allow_fail:                                          # 서버는 report.md 만 있으면 done 으로 둠 — 엔진 단계의 실패는 따로 봄
        C.add("엔진 단계에 실패 없음", not failed, " / ".join(f"{s['name']}: {s.get('summary')} {s.get('messages', [])[:2]}" for s in failed))
    return ds or []


def check_files(api, jid, C):
    st, b = api.call("GET", f"/api/jobs/{jid}/preview.glb", raw=True, timeout=600)
    C.add("preview.glb", st == 200 and b[:4] == b"glTF", f"HTTP {st} · {len(b) / 1e6:.1f} MB" if st == 200 else f"HTTP {st}")
    st, b = api.call("GET", f"/api/jobs/{jid}/map.png", raw=True)
    C.add("map.png", st == 200 and b[:8] == b"\x89PNG\r\n\x1a\n", f"HTTP {st}")
    st, b = api.call("GET", f"/api/jobs/{jid}/report.md", raw=True)
    C.add("report.md", st == 200 and len(b) > 100, f"HTTP {st}")
    st, b = api.call("GET", f"/api/jobs/{jid}/bundle.zip", raw=True, timeout=600)
    names = zipfile.ZipFile(io.BytesIO(b)).namelist() if st == 200 else []
    usd = [n for n in names if n.endswith("_mesh.usdc")]
    C.add("bundle.zip 에 USD", bool(usd) and any(n.endswith("scene_root.usda") for n in names), f"HTTP {st} · 파일 {len(names)}개 · {usd[:1]}")
    return names


def check_truth(defects, truth, tol, C):
    """정답의 결함마다 같은 종류 중 가장 가까운 것을 찾음. 정답 형식: {"defects": [{"type", "center_scene": [x, y, z]}]}"""
    res = []
    for t in truth.get("defects", []):
        if "center_scene" not in t:
            continue
        same = [d for d in defects if d["type"] == t["type"]]
        dist = min((math.dist(d["center_scene"][:2], t["center_scene"][:2]) for d in same), default=None)
        ok = dist is not None and dist <= tol
        C.add(f"정답 결함 {t['type']} @({t['center_scene'][0]:.1f}, {t['center_scene'][1]:.1f})", ok,
              f"가장 가까운 것 {dist:.2f} m" if dist is not None else "같은 종류가 없음")
        res.append(dict(type=t["type"], truth=t["center_scene"], dist_m=None if dist is None else round(dist, 3), ok=ok))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("--truth")
    ap.add_argument("--tol", type=float, default=0.5, help="정답 자리와의 거리 허용 (m, 바닥 위 평면 거리)")
    ap.add_argument("--server", help="이미 떠 있는 서버 주소 (안 주면 새로 띄움)")
    ap.add_argument("--code", help="--server 의 접속 코드")
    ap.add_argument("--timeout", type=int, default=1200)
    ap.add_argument("--json", help="결과를 이 파일에")
    ap.add_argument("--keep", action="store_true", help="새로 띄운 서버의 작업 폴더를 지우지 않음")
    ap.add_argument("--allow-fail", action="store_true", help="엔진 단계가 실패해도 시험 실패로 보지 않음 (일부러 나쁘게 찍은 촬영본처럼 실패가 맞는 입력)")
    a = ap.parse_args()
    C, srv, tmp = Checks(), None, None
    C.allow_fail = a.allow_fail
    t0 = time.time()
    print(f"e2e: {a.input}", flush=True)
    if a.server:
        api = Api(a.server, a.code)
    else:
        tmp = tempfile.mkdtemp(prefix="scan2sim_e2e_")
        log = open(os.path.join(tmp, "server.txt"), "w", encoding="utf-8")
        srv, api = start_server(os.path.join(tmp, "jobs"), secrets.token_hex(4), log)
        if not srv:
            print(f"서버가 뜨지 않음 — {tmp}/server.txt", flush=True)
            sys.exit(2)
        print(f"  서버 {api.base} · 작업 폴더 {tmp}/jobs", flush=True)
    out = dict(input=os.path.abspath(a.input))
    try:
        if api.code:
            st, _ = api.call("GET", "/api/info", code=False)
            C.add("접속 코드 없이 401", st == 401, f"HTTP {st}")
        jid = upload(api, a.input, C)
        j = wait(api, jid, a.timeout, C) if jid else None
        if j:
            out.update(job=jid, status=j["status"], verdict=j.get("verdict"), seconds=j["_seconds"],
                       stages={s["name"]: [s["status"], s.get("seconds")] for s in j.get("stages", [])})
            defects = check_job(j, C)
            out["defects"] = [dict(type=d["type"], center_scene=d["center_scene"], samples=d["samples"]) for d in defects]
            if j["status"] == "done":
                check_files(api, jid, C)
            if a.truth:
                out["truth"] = check_truth(defects, json.load(open(a.truth, encoding="utf-8")), a.tol, C)
    finally:
        if srv:
            srv.terminate()
            srv.wait(10)
            if not a.keep:
                shutil.rmtree(tmp, ignore_errors=True)
    out.update(ok=C.ok, checks=C.rows, total_s=round(time.time() - t0, 1))
    if a.json:
        json.dump(out, open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"e2e {'통과' if C.ok else '실패'} — 검사 {sum(r['ok'] for r in C.rows)}/{len(C.rows)} · {out['total_s']}초", flush=True)
    sys.exit(0 if C.ok else 1)


if __name__ == "__main__":
    main()
