"""접속 코드 틀린 횟수 제한 — 서버를 --code-tries 3 으로 새로 띄워 확인한다 (엔진은 안 돌림, 몇 초).

    cd recon
    python -m scan2sim.tests.code_tries

검사 항목: health 의 addr_header · X-Forwarded-For 로는 구분하지 않음 · 코드 없이 401(세지 않음) · 틀린 코드 3번째에 막힘 · 막힌 주소는 맞는 코드도 429 · 터널이 적어 준 다른 주소는 따로 셈 · 0 이면 안 셈
종료 코드: 0 = 통과 · 1 = 실패
"""
import json
import os
import subprocess
import sys
import tempfile
import urllib.error
import urllib.request

from scan2sim.tests import e2e


def get(port, code=None, ip=None):
    h = {}
    if code is not None:
        h["X-Code"] = code
    if ip:
        h["CF-Connecting-IP"] = ip
    try:
        with urllib.request.urlopen(urllib.request.Request(f"http://127.0.0.1:{port}/api/info", headers=h), timeout=10) as r:
            return r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def get_path(port, path, code, xff=None):
    h = {"X-Code": code}
    if xff:
        h["X-Forwarded-For"] = xff
    try:
        with urllib.request.urlopen(urllib.request.Request(f"http://127.0.0.1:{port}{path}", headers=h), timeout=10) as r:
            return r.status, json.loads(r.read() or b"{}")
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read() or b"{}")


def serve(tries):
    tmp = tempfile.mkdtemp(prefix="s2s_tries_")
    log = open(os.path.join(tmp, "server.log"), "w")
    port = e2e.free_port()
    group = dict(creationflags=subprocess.CREATE_NEW_PROCESS_GROUP) if os.name == "nt" else dict(start_new_session=True)
    p = subprocess.Popen([sys.executable, "-m", "scan2sim.web.server", "--port", str(port), "--jobs", os.path.join(tmp, "jobs"), "--code", "246810", "--min-free-gb", "0",
                          "--code-tries", str(tries)], cwd=e2e.RECON, stdout=log, stderr=subprocess.STDOUT, env=dict(os.environ, PYTHONIOENCODING="utf-8"), **group)
    api = e2e.Api(f"http://127.0.0.1:{port}", "246810")
    for _ in range(60):
        if p.poll() is not None:
            break
        try:
            if api.call("GET", "/api/info", timeout=2)[0] == 200:
                return p, port, log
        except e2e.Broken:
            pass
        import time
        time.sleep(0.5)
    e2e.stop_server(p)
    return None, None, log


def main():
    ok = []

    def add(name, cond, detail=""):
        ok.append(bool(cond))
        print(f"  {'통과' if cond else '실패'}  {name}" + (f" — {detail}" if detail else ""))

    p, port, log = serve(3)
    if not p:
        print("서버가 뜨지 않음")
        return 1
    try:
        A, B = "203.0.113.7", "203.0.113.8"                     # 터널이 적어 주는 주소 흉내 (문서용 주소 대역)
        for _ in range(5):
            st, _r = get(port, ip=A)
        add("코드 없이 401 · 여러 번 해도 세지 않음", st == 401, f"HTTP {st}")
        st, _r = get(port, "246810", A)
        add("그 뒤 맞는 코드는 200", st == 200, f"HTTP {st}")
        s1, _r = get(port, "000000", A)
        s2, _r = get(port, "000001", A)
        s3, _r = get(port, "000002", A)
        add("틀린 코드 1 · 2 · 3번째는 401", (s1, s2, s3) == (401, 401, 401), f"HTTP {s1} {s2} {s3}")
        st, r = get(port, "246810", A)
        add("3번 틀린 주소는 맞는 코드도 429", st == 429 and "뒤에" in r.get("error", ""), f"HTTP {st} {r.get('error')}")
        st, _r = get(port, "246810", B)
        add("다른 주소는 그대로 200", st == 200, f"HTTP {st}")
        st, _r = get(port, "246810")
        add("이 컴퓨터 화면(터널 주소 없음)도 그대로 200", st == 200, f"HTTP {st}")
        st, r = get_path(port, "/api/health", "246810")
        add("health 의 addr_header 가 CF-Connecting-IP", st == 200 and r.get("addr_header") == "CF-Connecting-IP", f"HTTP {st} {r.get('addr_header')}")
        st, _r = get(port, ip=A)
        add("막힌 주소가 코드 없이 물으면 401 (화면이 코드 칸을 보이게)", st == 401, f"HTTP {st}")
    finally:
        e2e.stop_server(p)
        log.close()
    p, port, log = serve(2)
    if not p:
        print("서버가 뜨지 않음 (addr_header null)")
        return 1
    try:
        st, r = get_path(port, "/api/health", "246810")
        add("터널로 온 요청이 없으면 addr_header 는 null", st == 200 and r.get("addr_header") is None, f"{r.get('addr_header')}")
        # X-Forwarded-For 는 방문자 주소로 쓰지 않음 (waitress 3 이 지우고, 보낸 쪽이 지어낼 수 있음) — 그것만 달라도 같은 주소로 셈
        get_path(port, "/api/info", "000000", "198.51.100.4")
        get_path(port, "/api/info", "000001", "198.51.100.5")
        s1, _r = get_path(port, "/api/info", "246810", "198.51.100.6")
        st, r = get_path(port, "/api/health", "246810", "198.51.100.7")
        add("X-Forwarded-For 만 바꿔서는 다른 주소로 세지 않음", s1 == 429 and r.get("addr_header") is None, f"HTTP {s1} · {r.get('addr_header')}")
    finally:
        e2e.stop_server(p)
        log.close()
    p, port, log = serve(0)
    if not p:
        print("서버가 뜨지 않음 (--code-tries 0)")
        return 1
    try:
        for i in range(6):
            st, _r = get(port, f"00000{i}", "203.0.113.9")
        st2, _r = get(port, "246810", "203.0.113.9")
        add("--code-tries 0 이면 세지 않음", st == 401 and st2 == 200, f"틀린 6번째 HTTP {st} · 맞는 코드 HTTP {st2}")
    finally:
        e2e.stop_server(p)
        log.close()
    print("CODE_TRIES_OK" if all(ok) else "CODE_TRIES_FAIL")
    return 0 if all(ok) else 1


if __name__ == "__main__":
    sys.exit(main())
