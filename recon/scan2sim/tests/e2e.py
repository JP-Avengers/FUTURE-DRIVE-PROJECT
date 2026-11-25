"""전체 흐름 시험 — 서버를 띄워 실제 사용자처럼 파일을 올리고, 끝날 때까지 기다린 뒤 결과가 약속(docs/CONTRACT.md)에 맞는지 본다.

    cd recon
    python -m scan2sim.tests.e2e <파일(zip · glb)> [--truth 정답.json] [--server http://… --code …] [--json 결과.json] [--allow-fail]

--server 를 안 주면 빈 포트에 서버를 새로 띄우고(임시 작업 폴더 · 무작위 접속 코드) 끝나면 엔진까지 함께 끈다.
--truth 를 주면 (samples/make_room.py · make_stray.py 가 쓴 정답) 넣어 둔 결함마다 같은 종류의 결함이 잡혔는지 본다.
  정답에 넓이(size_m)가 있으면 결함 자리가 넣은 넓이(씬의 사각형)에서 --tol-area 안(안이면 0)인지, 없으면 정답 가운데에서 --tol 안인지.
  정답 자리는 결함의 center_input(glTF 좌표, Y 위)을 이 작업의 T_input_to_scene 으로 옮겨 씬 좌표로 견준다.
  Stray 입력(zip)이면 그 앞에 stray.py 와 같은 Y_UP_TO_Z_UP 을 먼저 곱한다 (입력 좌표 = input_map 세계, docs/CONTRACT.md 1장).
  정답 하나에 잡힌 결함 하나만 짝지음(같은 결함이 정답 둘을 맞추지 못함). 정답이 있는데 견줄 것이 하나도 없으면 실패.
검사 항목: 접속 코드 없이 401 · 조각 순서가 틀리면 409 · 상태가 done · 상태 응답 필드 · 결함 목록 필드(보정 전 · 뒤)
           · 엔진 단계에 실패 없음 · 결과 파일 4개(preview.glb · map.png · report.md · bundle.zip)
--allow-fail: 일부러 나쁘게 찍은 입력처럼 엔진이 실패하는 것이 맞을 때. 단계 실패를 허용하고, 실패한 단계 뒤의 결과 파일은 404 가 맞는지 본다.
종료 코드: 0 = 모두 통과 · 1 = 하나라도 실패 · 2 = 서버가 안 뜸 · 연결이 끊김 · 시간 초과
"""
import argparse
import io
import json
import math
import os
import secrets
import shutil
import signal
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
OK = ("pass", "warn")
Y_UP_TO_Z_UP = [[1.0, 0, 0, 0], [0, 0, -1, 0], [0, 1, 0, 0], [0, 0, 0, 1]]     # stray.py 와 같은 값


class Broken(Exception):
    """서버와 연결이 끊김 · 시간 초과 — 종료 코드 2"""


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
        except (urllib.error.URLError, TimeoutError, ConnectionError) as e:
            raise Broken(f"{method} {path}: {e}") from e


class Checks:
    def __init__(self, allow_fail=False):
        self.rows, self.allow_fail = [], allow_fail

    def add(self, name, ok, detail=""):
        self.rows.append(dict(name=name, ok=bool(ok), detail=str(detail)))
        print(f"  {'통과' if ok else '실패'}  {name}" + (f" — {detail}" if detail else ""), flush=True)
        return ok

    @property
    def ok(self):
        return bool(self.rows) and all(r["ok"] for r in self.rows)


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def start_server(jobs, code, log):
    """서버를 새 프로세스 묶음으로 띄움 — 끌 때 서버가 띄운 엔진과 그 아래 파이프라인까지 한꺼번에 끄기 위해"""
    port = free_port()
    group = dict(creationflags=subprocess.CREATE_NEW_PROCESS_GROUP) if os.name == "nt" else dict(start_new_session=True)
    # --min-free-gb 0: 서버 기본은 올린 뒤 디스크 여유 20 GB 아래면 507 (D 9b2c95f) — 시험은 디스크가 작은 PC 에서도 돌아야 함
    p = subprocess.Popen([sys.executable, "-m", "scan2sim.web.server", "--port", str(port), "--jobs", jobs, "--code", code, "--min-free-gb", "0"], cwd=RECON,
                         stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONIOENCODING="utf-8"), **group)
    api = Api(f"http://127.0.0.1:{port}", code)
    for _ in range(60):
        if p.poll() is not None:
            break
        try:
            if api.call("GET", "/api/info", timeout=2)[0] == 200:
                return p, api
        except Broken:
            pass
        time.sleep(0.5)
    stop_server(p)
    return None, None


def stop_server(p):
    """서버 프로세스 묶음(서버 · 엔진 · 파이프라인)을 SIGTERM 으로, 10초 안에 안 끝나면 SIGKILL 로 끔.
    Windows 에는 프로세스 묶음 신호(killpg · SIGKILL)가 없어 taskkill /T 로 자식까지 끔 (C 가 Windows 에서 찾음, 11/24)."""
    if os.name == "nt":
        if p.poll() is None:
            subprocess.run(["taskkill", "/T", "/F", "/PID", str(p.pid)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            pass
        return
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(p.pid, sig)
        except ProcessLookupError:
            p.poll()
            return
        end = time.time() + 10
        while time.time() < end:
            p.poll()                                            # 서버가 끝났으면 거둬야 묶음이 비워짐
            try:
                os.killpg(p.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.3)


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
            if st != 200 or not isinstance(r, dict) or r.get("received") != off + len(buf):
                C.add("조각 올리기", False, f"offset {off}: HTTP {st} {r}")
                return None
            off += len(buf)
    C.add("조각 올리기", True, f"{size / 1e6:.1f} MB · {math.ceil(size / CHUNK)}조각 · {time.time() - t:.1f}초")
    st, r = api.call("POST", f"/api/jobs/{jid}/start")
    if not C.add("줄에 넣기", st == 200 and isinstance(r, dict) and r.get("status") == "queued", f"HTTP {st} {r}"):
        return None
    return jid


def wait(api, jid, timeout):
    t0, seen = time.time(), None
    while time.time() - t0 < timeout:
        st, j = api.call("GET", f"/api/jobs/{jid}")
        if st != 200:
            raise Broken(f"상태 조회 HTTP {st}")
        run = next((s["name"] for s in j.get("stages", []) if s.get("status") == "running"), None)
        if run != seen:
            print(f"    … {j['status']} {run or ''} ({time.time() - t0:.0f}초)", flush=True)
            seen = run
        if j["status"] in ("done", "failed"):
            j["_seconds"] = round(time.time() - t0, 1)
            return j
        time.sleep(2)
    raise Broken(f"{timeout}초 안에 끝나지 않음")


def defect_list_ok(d, phase):
    """결함 목록 파일 하나가 약속대로인지 → (맞음, 결함 목록)"""
    if not isinstance(d, dict):
        return False, []
    ds = d.get("defects")
    ok = (isinstance(ds, list) and d.get("phase") == phase and "T_input_to_scene" in d
          and all(isinstance(x, dict) and not [k for k in DEFECT_KEYS if k not in x] for x in ds))
    return ok, ds if isinstance(ds, list) else []


def stage_status(j):
    return {s.get("name"): s.get("status") for s in j.get("stages", [])}


def check_job(j, C):
    st = stage_status(j)
    C.add("상태가 done", j["status"] == "done", f"{j['status']} {j.get('error', '')} · 판정 {j.get('verdict')}".strip())
    miss = [k for k in JOB_KEYS + DONE_KEYS if k not in j]
    C.add("상태 응답 필드", not miss, f"없음 {miss}" if miss else "")
    bad = [s.get("name") for s in j.get("stages", []) if any(k not in s for k in STAGE_KEYS) or s["status"] not in ("pass", "warn", "fail", "skip")]
    C.add("단계 8개 · 판정 값", len(st) == 8 and not bad, " · ".join(f"{k} {v}" for k, v in st.items()))
    failed = [s for s in j.get("stages", []) if s.get("status") == "fail"]
    if not C.allow_fail:                                        # 엔진 단계의 실패를 직접 봄 (서버도 R2 부터 실패면 failed 로 둠)
        C.add("엔진 단계에 실패 없음", not failed, " / ".join(f"{s['name']}: {s.get('summary')} {s.get('messages', [])[:2]}" for s in failed))
    defects = []
    if st.get("diagnose") in OK:
        ok, defects = defect_list_ok(j.get("defects_before"), "before")
        C.add("결함 목록 필드(보정 전)", ok, f"{len(defects)}곳")
        if st.get("correct") in OK:
            ok2, after = defect_list_ok(j.get("defects_after"), "after")
            C.add("결함 목록 필드(보정 뒤)", ok2, f"{len(after)}곳")
        else:
            C.add("보정을 안 했으면 보정 뒤 목록 없음", j.get("defects_after") is None, f"correct {st.get('correct')}")
    else:
        C.add("진단을 못 했으면 결함 목록 없음", j.get("defects_before") is None, f"diagnose {st.get('diagnose')}")
    return defects


def check_files(api, jid, j, C):
    """결과 파일 — 만들어진 단계까지는 있어야 하고, 실패 · 건너뛴 단계의 것은 404 여야 함"""
    st = stage_status(j)
    want = dict(preview=st.get("convert") in OK, map=st.get("nav2") in OK, report=True, bundle=st.get("bundle") in OK)
    s, b = api.call("GET", f"/api/jobs/{jid}/preview.glb", raw=True, timeout=900)
    C.add("preview.glb", (s == 200 and b[:4] == b"glTF") if want["preview"] else s == 404,
          (f"HTTP {s} · {len(b) / 1e6:.1f} MB" if s == 200 else f"HTTP {s}") + ("" if want["preview"] else " (convert 가 안 돼 404 가 맞음)"))
    s, b = api.call("GET", f"/api/jobs/{jid}/map.png", raw=True)
    C.add("map.png", (s == 200 and b[:8] == b"\x89PNG\r\n\x1a\n") if want["map"] else s == 404, f"HTTP {s}")
    s, b = api.call("GET", f"/api/jobs/{jid}/report.md", raw=True)
    C.add("report.md", s == 200 and len(b) > 100, f"HTTP {s}")
    s, b = api.call("GET", f"/api/jobs/{jid}/bundle.zip", raw=True, timeout=900)
    if not want["bundle"]:
        C.add("bundle.zip", s == 404, f"HTTP {s} (bundle 이 안 돼 404 가 맞음)")
        return []
    try:
        names = zipfile.ZipFile(io.BytesIO(b)).namelist() if s == 200 else []
    except zipfile.BadZipFile:
        C.add("bundle.zip", False, "zip 이 깨짐")
        return []
    usd = [n for n in names if n.endswith("_mesh.usdc")]
    C.add("bundle.zip 에 USD", bool(usd) and any(n.endswith("scene_root.usda") for n in names), f"HTTP {s} · 파일 {len(names)}개 · {usd[:1]}")
    return names


def _to_scene(p_in, T, stray_input):
    p = list(p_in[:3]) + [1.0]
    if stray_input:
        p = [sum(Y_UP_TO_Z_UP[r][c] * p[c] for c in range(4)) for r in range(4)]
    return [sum(T[r][c] * p[c] for c in range(4)) for r in range(2)]


def truth_scene(t, T, stray_input):
    """정답 결함의 씬 좌표 (x, y) · 넣은 넓이의 씬 네 모서리 (정답에 size_m 이 없으면 None) · 구한 방법.
    center_input 이 있으면 이 작업의 T_input_to_scene 으로 옮김. size_m 은 씬 (x, y) 크기 — glTF 에서는 (x, −z) 방향 (make_room 좌표 규약)"""
    hx, hy = (t["size_m"][0] / 2, t["size_m"][1] / 2) if "size_m" in t else (None, None)
    corners = ((-1, -1), (1, -1), (1, 1), (-1, 1))
    if "center_input" in t and T is not None:
        c = t["center_input"]
        quad = [_to_scene([c[0] + sx * hx, c[1], c[2] - sy * hy], T, stray_input) for sx, sy in corners] if hx is not None else None
        return _to_scene(c, T, stray_input), quad, "center_input × T_input_to_scene" + (" (Y_UP_TO_Z_UP 먼저)" if stray_input else "")
    for k in ("center_scene", "center_scene_expected"):
        if k in t:
            c = t[k]
            quad = [[c[0] + sx * hx, c[1] + sy * hy] for sx, sy in corners] if hx is not None else None
            return list(c[:2]), quad, k
    return None, None, None


def quad_dist(p, q):
    """점 p 에서 볼록 사각형 q (모서리 네 개, 차례대로) 까지의 평면 거리 — 안이면 0"""
    e = list(zip(q, q[1:] + q[:1]))
    s = [(b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) for a, b in e]
    if min(s) >= 0 or max(s) <= 0:
        return 0.0

    def seg(a, b):
        ux, uy = b[0] - a[0], b[1] - a[1]
        u = max(0.0, min(1.0, ((p[0] - a[0]) * ux + (p[1] - a[1]) * uy) / ((ux * ux + uy * uy) or 1.0)))
        return math.dist(p, (a[0] + u * ux, a[1] + u * uy))
    return min(seg(a, b) for a, b in e)


def check_truth(defects, truth, T, stray_input, tol, tol_area, C):
    """정답마다 같은 종류의 결함 하나를 1:1 짝지음. 정답에 넓이(size_m)가 있으면 결함 자리가 넣은 넓이에서 tol_area 안인지,
    없으면 정답 가운데에서 tol 안인지 봄. 결함 자리(center_scene)는 걸린 표본의 평균이라 넓이 안의 어디든 맞음 —
    얇은 판(double_floor)은 카메라를 향한 두 가장자리만 표본에 걸려 가운데에서 0.12~0.41 m 벗어남 (11/24, Stray 3 seed × 깊이 규칙 2개).
    가운데까지 거리(dist_m)는 늘 함께 남김 (흐름 비교용)."""
    want = [t for t in truth.get("defects", []) if isinstance(t, dict) and "type" in t]
    pairs = []
    for i, t in enumerate(want):
        xy, quad, how = truth_scene(t, T, stray_input)
        if xy is None:
            C.add(f"정답 결함 {t['type']}", False, "정답에 자리(center_input · center_scene)가 없음")
            continue
        for k, d in enumerate(defects):
            if d["type"] == t["type"]:
                dc = math.dist(d["center_scene"][:2], xy)
                pairs.append((quad_dist(d["center_scene"][:2], quad) if quad else dc, dc, i, k))
        t["_xy"], t["_quad"], t["_how"] = xy, quad, how
    pairs.sort()
    used_t, used_d, match = set(), set(), {}
    for key, dc, i, k in pairs:
        if i not in used_t and k not in used_d:
            used_t.add(i), used_d.add(k)
            match[i] = (key, dc)
    res = []
    for i, t in enumerate(want):
        if "_xy" not in t:
            continue
        m, lim = match.get(i), (tol_area if t["_quad"] else tol)
        ok = m is not None and m[0] <= lim
        what = ("짝지을 같은 종류 결함이 없음" if m is None else
                f"넣은 넓이까지 {m[0]:.2f} m (허용 {lim:g}) · 가운데까지 {m[1]:.2f} m" if t["_quad"] else f"짝지은 결함까지 {m[1]:.2f} m (허용 {lim:g})")
        C.add(f"정답 결함 {t['type']} @({t['_xy'][0]:.2f}, {t['_xy'][1]:.2f})", ok, what + f" · {t['_how']}")
        res.append(dict(type=t["type"], truth_xy=[round(v, 3) for v in t["_xy"]], dist_m=None if m is None else round(m[1], 3),
                        dist_area_m=None if m is None or not t["_quad"] else round(m[0], 3), ok=ok))
    if want and not res:
        C.add("정답과 견줌", False, "정답 결함이 있는데 견준 것이 없음")
    if not want:
        print("    (정답 파일에 결함이 없음 — 결함 위치는 견주지 않음)", flush=True)
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input")
    ap.add_argument("--truth")
    ap.add_argument("--tol", type=float, default=0.5, help="정답에 넓이가 없을 때: 정답 가운데와의 거리 허용 (m, 바닥 위 평면 거리)")
    ap.add_argument("--tol-area", type=float, default=0.25, help="정답에 넓이(size_m)가 있을 때: 넣은 넓이와의 거리 허용 (m, 넓이 안이면 0)")
    ap.add_argument("--server", help="이미 떠 있는 서버 주소 (안 주면 새로 띄움)")
    ap.add_argument("--code", help="--server 의 접속 코드")
    ap.add_argument("--timeout", type=int, default=1200)
    ap.add_argument("--json", help="결과를 이 파일에")
    ap.add_argument("--keep", action="store_true", help="새로 띄운 서버의 작업 폴더를 지우지 않음")
    ap.add_argument("--allow-fail", action="store_true", help="엔진 단계가 실패하는 것이 맞는 입력 (일부러 나쁘게 찍은 촬영본 등)")
    a = ap.parse_args()
    for f in [a.input] + ([a.truth] if a.truth else []):
        if not os.path.isfile(f):
            print(f"e2e: 파일이 없음 — {f}", flush=True)
            sys.exit(2)
    C, srv, tmp, rc = Checks(a.allow_fail), None, None, None
    t0 = time.time()
    print(f"e2e: {a.input}", flush=True)
    out = dict(input=os.path.abspath(a.input))
    try:
        if a.server:
            api = Api(a.server, a.code)
        else:
            tmp = tempfile.mkdtemp(prefix="scan2sim_e2e_")
            log = open(os.path.join(tmp, "server.txt"), "w", encoding="utf-8")
            srv, api = start_server(os.path.join(tmp, "jobs"), secrets.token_hex(4), log)
            if not srv:
                raise Broken(f"서버가 뜨지 않음 — {tmp}/server.txt")
            print(f"  서버 {api.base} · 작업 폴더 {tmp}/jobs", flush=True)
        if api.code:
            st, _ = api.call("GET", "/api/info", code=False)
            C.add("접속 코드 없이 401", st == 401, f"HTTP {st}")
        jid = upload(api, a.input, C)
        if jid:
            j = wait(api, jid, a.timeout)
            out.update(job=jid, status=j["status"], verdict=j.get("verdict"), seconds=j["_seconds"],
                       stages={s["name"]: [s["status"], s.get("seconds")] for s in j.get("stages", [])})
            defects = check_job(j, C)
            out["defects"] = [dict(type=d["type"], center_scene=d["center_scene"], samples=d["samples"]) for d in defects]
            check_files(api, jid, j, C)
            if a.truth:
                T = (j.get("defects_before") or {}).get("T_input_to_scene")
                out["truth"] = check_truth(defects, json.load(open(a.truth, encoding="utf-8")), T, a.input.lower().endswith(".zip"), a.tol, a.tol_area, C)
    except Broken as e:
        print(f"  중단 — {e}", flush=True)
        out["broken"], rc = str(e), 2
    finally:
        if srv:
            stop_server(srv)
            if not a.keep:
                shutil.rmtree(tmp, ignore_errors=True)
        out.update(ok=C.ok and rc is None, checks=C.rows, total_s=round(time.time() - t0, 1))
        if a.json:
            json.dump(out, open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"e2e {'통과' if out['ok'] else '실패'} — 검사 {sum(r['ok'] for r in C.rows)}/{len(C.rows)} · {out['total_s']}초", flush=True)
    sys.exit(rc if rc is not None else (0 if out["ok"] else 1))


if __name__ == "__main__":
    main()
