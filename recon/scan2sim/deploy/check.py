"""켜져 있는 scan2sim 서버에 파일 하나를 올려 끝까지 돌려 보고 결과를 받아 봄 — 화면이 하는 것과 같은 순서 · 같은 조각 크기.

    ~/scan2sim_env/bin/python recon/scan2sim/deploy/check.py <촬영본.zip 또는 .glb>
    ~/scan2sim_env/bin/python recon/scan2sim/deploy/check.py <파일> --url https://<이름>.trycloudflare.com   # 터널을 지나서

--url 을 안 주면 이 컴퓨터의 서버(http://127.0.0.1:8000), --code 를 안 주면 ~/scan2sim_run/code 를 씀.
표준 라이브러리만 씀. 끝나면 마지막 줄에 CHECK_OK 또는 CHECK_FAIL.
"""
import argparse
import json
import os
import sys
import time
import urllib.error
import urllib.request

print = __import__("functools").partial(print, flush=True)            # 로그 파일로 돌려도 바로 보이게

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument("file")
ap.add_argument("--url", default=f"http://127.0.0.1:{os.environ.get('S2S_PORT', '8000')}")
ap.add_argument("--code")
ap.add_argument("--chunk-mb", type=float, default=8, help="조각 크기 (화면은 8). 터널이 큰 요청을 막는지 볼 때 바꿔 봄")
ap.add_argument("--timeout-min", type=float, default=60)
a = ap.parse_args()

code_file = os.path.join(os.environ.get("S2S_RUN", os.path.expanduser("~/scan2sim_run")), "code")
code = a.code or (open(code_file).read().strip() if os.path.exists(code_file) else "")
base = a.url.rstrip("/")
UA = "scan2sim-check"                                           # 기본 값(Python-urllib)은 Cloudflare 가 막을 수 있음


def call(path, method="GET", data=None, ctype=None, raw=False, timeout=600):
    h = {"User-Agent": UA}
    if code:
        h["X-Code"] = code
    if ctype:
        h["Content-Type"] = ctype
    try:
        body = urllib.request.urlopen(urllib.request.Request(base + path, data=data, headers=h, method=method), timeout=timeout).read()
    except urllib.error.HTTPError as e:
        raise SystemExit(f"{method} {path} → {e.code} {e.read()[:300].decode('utf-8', 'replace')}\nCHECK_FAIL")
    return body if raw else json.loads(body)


size = os.path.getsize(a.file)
print(f"서버 {base} · 파일 {os.path.basename(a.file)} {size / 1e6:.1f} MB · 조각 {a.chunk_mb:g} MB")
t0 = time.time()
jid = call("/api/jobs", "POST", json.dumps(dict(name=os.path.basename(a.file), size=size)).encode(), "application/json")["id"]
off, n = 0, int(a.chunk_mb * 1024 * 1024)
with open(a.file, "rb") as f:
    while off < size:
        buf = f.read(n)
        got = call(f"/api/jobs/{jid}/chunk?offset={off}", "PUT", buf, "application/octet-stream")["received"]
        assert got == off + len(buf), f"서버가 받은 크기 {got} ≠ 보낸 크기 {off + len(buf)}"
        off = got
t_up = time.time() - t0
print(f"올리기 {t_up:.1f}초 ({size / 1e6 / max(t_up, 1e-3):.1f} MB/s) · 작업 번호 {jid}")
call(f"/api/jobs/{jid}/start", "POST", b"")

seen, t1 = {}, time.time()
while True:
    j = call(f"/api/jobs/{jid}")
    for s in j["stages"]:
        if s["status"] not in ("pending", "running") and s["name"] not in seen:
            seen[s["name"]] = s["status"]
            print(f"  {s['name']:9s} {s['status']:5s} {s.get('seconds', '')} s  {s.get('summary', '')}")
    if j["status"] in ("done", "failed"):
        break
    if time.time() - t1 > a.timeout_min * 60:
        raise SystemExit(f"{a.timeout_min:g}분 안에 끝나지 않음\nCHECK_FAIL")
    time.sleep(2)
t_run = time.time() - t1
print(f"처리 {t_run:.0f}초 · 상태 {j['status']} · 판정 {j.get('verdict')} · 묶음 {'있음' if j.get('has_bundle') else '없음'}")

got = {}
for name in ["report.md", "map.png", "preview.glb"] + (["bundle.zip"] if j.get("has_bundle") else []):
    t = time.time()
    try:
        b = call(f"/api/jobs/{jid}/{name}", raw=True)
        got[name] = len(b)
        print(f"  {name:12s} {len(b) / 1e6:8.2f} MB  {time.time() - t:.1f}초")
    except SystemExit as e:
        print(f"  {name:12s} 못 받음 — {str(e).splitlines()[0]}")

ok = j["status"] == "done" and j.get("has_bundle") and all(got.get(k) for k in ("report.md", "preview.glb", "bundle.zip"))
print("CHECK_OK" if ok else "CHECK_FAIL")
sys.exit(0 if ok else 1)
