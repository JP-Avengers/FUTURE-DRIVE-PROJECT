"""Stray 입력의 경계 사례 시험 — 일부러 망가뜨린 촬영본 · zip, 중간에 멈춘 변환 · 메쉬 다시 만들기를 가상 촬영본으로 만들어
stray · stray_mesh · stray_fix · stray_quality 가 받을 것은 받고, 못 받을 것은 traceback 없이 한국어 문구(CaptureError)로 거절하는지 본다.
서버 · 엔진은 띄우지 않음.

    cd recon
    python -m scan2sim.tests.stray_edge [--fixtures <폴더>] [--work <폴더>] [--json 결과.json]

시험 자료: samples/make_room.py · make_stray.py 로 만든 6 초짜리 가상 촬영본 (180장 · 가상의 방). 실제 촬영본은 쓰지 않음.
  --fixtures 를 주면 그 폴더에 만들고 다음에는 다시 씀 (run.sh), 안 주면 임시 폴더(TMPDIR 아래)에 만들고 지움 (약 15초 더 걸림).
  망가뜨린 촬영본은 가상 촬영본의 파일을 링크하고 (안 되면 복사) 고칠 파일만 새로 씀.
--work: 망가뜨린 촬영본 · 변환 결과를 둘 폴더 (안 주면 임시 폴더를 만들고 끝나면 지움)
검사
  입력      빠진 파일 · 빈 depth/ · 짧은 촬영 · odometry.csv 깨진 줄 (열 모자람 · 글자 · NaN · 빈 칸 · 사원수 길이 · 시각 거꾸로) · 깊이 · 신뢰도
            장 수 · frame 번호의 깊이 파일이 모자람 · 영상 (짧음 · 4:3 아님 · 깨짐) · camera_matrix (2×2 · NaN · 글자 · 주점 밖 · 초점 0)
            → CaptureError 문구. 신뢰도 없음 · 깊이 1장 모자람 · frame 열이 차례가 아님은 주의만 (변환은 됨)
  zip       './' · '\\' 이름은 받음 · 폴더 밖 경로 ('../' · '..\\' · 절대) 는 CaptureError 이고 아무것도 안 풂 · 촬영 여러 개 · 깨진 zip ·
            깨진 내용 (CRC) · __MACOSX · 너무 큼 (ZIP_MAX_FILES 를 잠시 줄여서) · 촬영이 없는 zip · 촬영이 둘 든 폴더 · 한 겹 아래 촬영
  멈춘 변환  키프레임 중간에 멈추면 예전 stray_meta.json · trajectory · 보정 · 품질 파일이 남지 않고, 다시 돌리면 .writing 없이 끝남
  짝짓기    odometry.csv 줄이 빠져 frame 번호가 건너뛴 촬영 (10줄 · 2줄) — 키프레임 깊이가 frame 번호 파일과 화소까지 같고 stray_quality._depth 가
            남기는 화소가 stray.py 와 같음. 줄 번호로 짝지었다면 깊이가 달랐음도 확인 (시험이 헛돌지 않게)
  깨진 장   첫 깊이 파일 (깨짐 · 8비트) · 신뢰도 크기 · 가운데 키프레임 깊이 크기 → CaptureError, 가운데 키프레임 깊이가 깨짐 → 그 장만 빼고 주의
  메쉬      읽기 전용 맵 폴더에 --no-fix 로 만들기 · stray_quality 가 pose_fix.json 을 sha256 으로 확인 (새 빌드 · input_map 의 같은 바이트 ·
            손으로 고친 파일 · 멈춘 다시 만들기 두 방향 · sha256 없는 예전 빌드 · 예전 빌드인데 보정 없이 만든 메쉬)
  CLI       --tilt / --no-tilt 와 --fix · --no-fix 조합 (stray_fix.tilt_fix · split_opts, stray_mesh · stray_fix 는 종료 코드 2) ·
            stray CLI 가 짧은 촬영에 traceback 없이 종료 코드 1
종료 코드: 0 = 모두 통과 · 1 = 하나라도 실패 · 2 = 시험 자료를 만들지 못함
"""
import os

os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")              # 일부러 깨뜨린 그림마다 cv2 가 찍는 경고를 줄임 (cv2 를 읽기 전에)

import argparse  # noqa: E402
import json  # noqa: E402
import shutil  # noqa: E402
import stat  # noqa: E402
import subprocess  # noqa: E402
import sys  # noqa: E402
import tempfile  # noqa: E402
import time  # noqa: E402
import traceback  # noqa: E402
import zipfile  # noqa: E402

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from .. import stray, stray_fix, stray_mesh, stray_quality as sq  # noqa: E402

RECON = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SAMPLES = os.path.join(RECON, "scan2sim", "samples")
NAME = "synth_edge"
FIXTURE = ["--seconds", "6", "--render-width", "160", "--seed", "0", "--name", NAME]   # 180장 · 색은 작게 그려 키움 (약 15초)
MIN_ROWS = 150                                                  # 이보다 짧은 시험 자료면 시험이 가정한 줄 번호가 없음
ENV = dict(os.environ, PYTHONIOENCODING="utf-8")


def quiet(*_a, **_k):
    pass


def js(d, n=200):
    return json.dumps(d, ensure_ascii=False, default=str)[:n]


class Checks:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail=""):
        ok = bool(ok)
        self.rows.append(dict(name=name, ok=ok, detail=str(detail)))
        print(f"  {'통과' if ok else '실패'}  {name}" + (f" — {detail}" if detail != "" else ""), flush=True)
        return ok

    def skip(self, name, why):
        self.rows.append(dict(name=name, ok=None, detail=why))
        print(f"  건너뜀  {name} — {why}", flush=True)

    def error(self, name, fn, must=(), exc=stray.CaptureError):
        """fn() 이 exc 를 내고 그 문구에 must 가 모두 있으면 통과. 오류가 안 나거나 다른 예외 (= CLI 에서 traceback) 면 실패"""
        must = (must,) if isinstance(must, str) else tuple(must)
        try:
            r = fn()
        except exc as e:
            miss = [m for m in must if m not in str(e)]
            return self.add(name, not miss, (f"문구에 {miss} 없음: " if miss else "") + str(e)[:220])
        except Exception as e:
            return self.add(name, False, f"다른 예외 {type(e).__name__}: {str(e)[:200]}")
        return self.add(name, False, f"오류가 안 남 ({str(r)[:120]})")

    @property
    def ok(self):
        done = [r for r in self.rows if r["ok"] is not None]
        return bool(done) and all(r["ok"] for r in done)


def _link(src, dst):
    """파일 하나를 링크 (안 되면 복사 — 윈도우)"""
    try:
        os.symlink(src, dst)
    except (OSError, NotImplementedError):
        shutil.copy2(src, dst)


def _chmod_dirs(root, mode):
    for d, _, _ in os.walk(root):
        try:
            os.chmod(d, mode)
        except OSError:
            pass


def _rmtree(p):
    if os.path.isdir(p):
        _chmod_dirs(p, stat.S_IRWXU)                            # 앞 회차가 읽기 전용으로 남겼어도 지울 수 있게
    shutil.rmtree(p, ignore_errors=True)


def _png(img):
    return cv2.imencode(".png", img)[1].tobytes()


def _video(n, w, h):
    """n 장 · w×h mp4 를 쓰는 함수 (Fake.make 의 files 에)"""
    def write(p):
        vw = cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*"mp4v"), 30, (w, h))
        for i in range(n):
            vw.write(np.full((h, w, 3), i % 255, np.uint8))
        vw.release()
    return write


class Fake:
    """가상 촬영본 src 를 고쳐 쓴 촬영 폴더를 root 아래에 만듦 (원본 파일은 링크 — 고칠 파일만 새로 씀)"""

    def __init__(self, src, root):
        self.src, self.root = src, root
        self.names = sorted(os.listdir(src))
        lines = open(os.path.join(src, "odometry.csv"), encoding="utf-8").read().splitlines()
        self.hdr, self.body = lines[0], [s for s in lines[1:] if s.strip()]
        self.n = len(self.body)
        self.od = np.atleast_2d(np.genfromtxt(os.path.join(src, "odometry.csv"), delimiter=",", skip_header=1, usecols=range(13)))

    def make(self, name, skip=(), odo=None, depth=None, conf=None, files=None):
        """skip = 뺄 파일 · 폴더 이름, odo = odometry.csv 데이터 줄 목록, depth · conf = 남길 frame 번호 (None = 모두),
        files = {상대 경로: bytes · str · 함수(경로)} 새로 쓸 파일 (depth/000000.png 처럼 폴더 안 파일도)"""
        d = os.path.join(self.root, name)
        os.makedirs(d)
        files = dict(files or {})
        if odo is not None:
            files["odometry.csv"] = "\n".join([self.hdr] + list(odo)) + "\n"
        for n in self.names:
            if n in skip or n in files:
                continue
            p = os.path.join(self.src, n)
            if not os.path.isdir(p):
                _link(p, os.path.join(d, n))
                continue
            os.makedirs(os.path.join(d, n))
            sel = {"depth": depth, "confidence": conf}.get(n)
            keep = None if sel is None else {f"{i:06d}.png" for i in sel}
            for f in sorted(os.listdir(p)):
                if (keep is None or f in keep) and f"{n}/{f}" not in files:
                    _link(os.path.join(p, f), os.path.join(d, n, f))
        for rel, v in files.items():
            p = os.path.join(d, *rel.split("/"))
            os.makedirs(os.path.dirname(p), exist_ok=True)
            if callable(v):
                v(p)
            else:
                with open(p, "wb") as f:
                    f.write(v.encode("utf-8") if isinstance(v, str) else v)
        return d

    def col(self, row, k, v):
        """데이터 줄 row 의 k 번째 열을 v 로 바꾼 줄 목록"""
        L = list(self.body)
        c = L[row].split(",")
        c[k] = v
        L[row] = ",".join(c)
        return L

    def line(self, row, s):
        L = list(self.body)
        L[row] = s
        return L


# ── 입력 확인 ────────────────────────────────────────────────────────────

def check_inputs(C, F, out):
    n, a = F.n, F.n // 2
    swap = list(F.body)
    swap[a], swap[a + 1] = swap[a + 1], swap[a]
    drop10 = [s for k, s in enumerate(F.body) if not a <= k < a + 10]
    K = lambda s: dict(files={"camera_matrix.csv": s})       # noqa: E731
    cases = [
        ("rgb.mp4 없음", dict(skip=("rgb.mp4",)), ("필요한 파일이 없음", "rgb.mp4")),
        ("odometry.csv 없음", dict(skip=("odometry.csv",)), ("필요한 파일이 없음", "odometry.csv")),
        ("camera_matrix.csv 없음", dict(skip=("camera_matrix.csv",)), ("필요한 파일이 없음", "camera_matrix.csv")),
        ("depth/ 없음", dict(skip=("depth",)), ("필요한 파일이 없음", "depth/")),
        ("빈 depth/", dict(depth=[]), "깊이 파일(.png)이 없음"),
        ("짧은 촬영 (50줄)", dict(odo=F.body[:50], depth=range(50), conf=range(50)), "너무 짧음"),
        ("odometry 열 모자람", dict(odo=F.line(a, "abc,def,ghi")), "열이 3개뿐"),
        ("odometry 글자 값", dict(odo=F.col(a, 3, " x1.0")), "숫자가 아닌 값"),
        ("odometry NaN", dict(odo=F.col(a, 5, " nan")), "NaN"),
        ("odometry 빈 칸 (사원수)", dict(odo=F.col(a, 6, " ")), "숫자가 아닌 값"),
        ("odometry 사원수 길이 ≠ 1", dict(odo=F.col(a, 8, " 5.0")), "사원수"),
        ("odometry 시각 거꾸로 (두 줄 바뀜)", dict(odo=swap), "시각이 앞 줄보다 커지지 않음"),
        ("깊이 30장 모자람", dict(depth=range(n - 30)), ("depth/", "30개 다름")),
        ("신뢰도 30장 모자람", dict(conf=range(n - 30)), ("confidence/", "30개 다름")),
        ("frame 번호 건너뜀 + 그 번호의 깊이 5장 없음", dict(odo=drop10, depth=[i for i in range(n) if not a + 20 <= i < a + 25]),
         ("depth/", "frame 번호의 파일이 5개 없음")),
        ("영상이 짧음 (50장)", dict(files={"rgb.mp4": _video(50, 160, 120)}), ("rgb.mp4 가 50장뿐",)),
        ("영상이 4:3 아님 (16:9)", dict(files={"rgb.mp4": _video(n, 160, 90)}), "4:3 이 아님"),
        ("영상이 깨짐", dict(files={"rgb.mp4": b"\0" * 5000}), "rgb.mp4 를 열 수 없음"),
        ("camera_matrix 2×2", K("1,0\n0,1\n"), "camera_matrix.csv 를 읽지 못함"),
        ("camera_matrix NaN", K("1,0,0\n0,nan,0\n0,0,1\n"), "camera_matrix.csv 를 읽지 못함"),
        ("camera_matrix 글자", K("a,b,c\n"), "camera_matrix.csv 를 읽지 못함"),
        ("camera_matrix 주점이 영상 밖", K("1335.0, 0.0, 3000\n0.0, 1335.0, 719.5\n0.0, 0.0, 1.0\n"), "주점"),
        ("camera_matrix 초점거리 0", K("0.0, 0.0, 959.5\n0.0, 0.0, 719.5\n0.0, 0.0, 1.0\n"), "초점거리가 0 이하"),
    ]
    for k, (name, kw, must) in enumerate(cases):
        d = F.make(f"in{k:02d}", **kw)
        C.error(f"입력: {name} → CaptureError", lambda: stray.convert(d, os.path.join(out, f"in{k:02d}"), log=quiet), must)

    d = F.make("in_noconf", skip=("confidence",))                # 주의만 — 변환은 됨
    m = stray.convert(d, os.path.join(out, "in_noconf"), log=quiet)
    w = m["checks"]["warnings"]
    C.add("입력: confidence/ 없음 → 주의만 · 깊이 규칙 no_confidence 로 변환", m["depth_rule"]["name"] == "no_confidence" and m["keyframes"] >= 2
          and any("confidence/ 가 없음" in s for s in w), f"규칙 {m['depth_rule']['name']} · 키프레임 {m['keyframes']} · {js(w, 120)}")
    errs, warns, _ = stray.validate(F.make("in_depth_m1", depth=range(n - 1)))
    C.add("입력: 깊이 1장 모자람 → 오류 없이 주의", not errs and any("1개 다름" in s for s in warns), js(errs + warns, 160))
    same = [",".join([c[0], " 000007"] + c[2:]) for c in (s.split(",") for s in F.body)]
    errs, warns, info = stray.validate(F.make("in_framecol", odo=same))
    C.add("입력: frame 열이 차례대로 커지지 않음 → 줄 순서로 짝짓고 주의", not errs and info.get("depth_pairing") == "row"
          and any("줄 순서로 짝지음" in s for s in warns), js(errs + warns, 160))


# ── zip · 폴더 찾기 ──────────────────────────────────────────────────────

def check_zips(C, Z):
    os.makedirs(Z)

    def mkzip(name, entries, how=zipfile.ZIP_STORED):
        p = os.path.join(Z, name)
        with zipfile.ZipFile(p, "w", how) as z:
            for arc, data in entries:
                z.writestr(arc, data)
        return p

    w = lambda name: os.path.join(Z, "u_" + name)               # noqa: E731  — 풀 곳
    small = [("rgb.mp4", "x"), ("odometry.csv", "x"), ("camera_matrix.csv", "x"), ("depth/000000.png", "x")]
    isf = lambda *p: os.path.isfile(os.path.join(*p))          # noqa: E731

    r = stray.find_capture(mkzip("dot.zip", [("./", "")] + [("./" + n, d) for n, d in small]), w("dot"))
    C.add("zip: './' 이름 → 맨 위 촬영으로 풂", r and isf(r, "rgb.mp4") and isf(r, "depth", "000000.png"), r)
    r = stray.find_capture(mkzip("bs.zip", [("cap1\\" + n.replace("/", "\\"), d) for n, d in small]), w("bs"))
    C.add("zip: '\\' 이름 (윈도우 Compress-Archive) → 폴더로 풂", r and os.path.basename(r) == "cap1" and isf(r, "depth", "000000.png"), r)
    r = stray.find_capture(shutil.make_archive(os.path.join(Z, "ma"), "zip", root_dir=w("dot")), w("ma"))
    C.add("zip: shutil.make_archive (맨 위에 파일) → 풂", r and isf(r, "odometry.csv") and isf(r, "depth", "000000.png"), r)

    evil_abs = os.path.join(Z, "evil_abs.txt").replace("\\", "/")
    for k, (label, bad) in enumerate((("'../'", "../evil.txt"), ("'..\\'", "..\\evil.txt"), ("절대 경로", evil_abs))):
        C.error(f"zip: 폴더 밖 경로 {label} → CaptureError", lambda: stray.find_capture(mkzip(f"out{k}.zip", small + [(bad, "x")]), w(f"out{k}")), "밖")
    left = [p for p in (os.path.join(Z, "evil.txt"), evil_abs) if os.path.exists(p)] + [
        f"u_out{k}/{f}" for k in range(3) if os.path.isdir(w(f"out{k}")) for f in os.listdir(w(f"out{k}"))]
    C.add("zip: 밖 경로가 하나라도 있으면 아무것도 안 풂", not left, f"남은 것 {left}" if left else "")

    two = [(f"{c}/{f}", "x") for c in ("capA", "capB") for f in ("odometry.csv", "camera_matrix.csv")]
    C.error("zip: 촬영 폴더 둘 → CaptureError", lambda: stray.find_capture(mkzip("two.zip", two), w("two")), "여러 개")
    p = os.path.join(Z, "broken.zip")
    open(p, "wb").write(b"PK\x03\x04garbage")
    C.error("zip: 깨진 zip → CaptureError", lambda: stray.find_capture(p, w("broken")), "zip 파일을 열 수 없음")
    p = mkzip("crc.zip", [("cap/rgb.mp4", b"0123456789" * 100)] + [("cap/" + n, d) for n, d in small[1:]])
    raw = bytearray(open(p, "rb").read())
    raw[raw.index(b"0123456789" * 10) + 50] ^= 0xFF                 # 내용 한 바이트 (CRC 가 안 맞음)
    open(p, "wb").write(bytes(raw))
    C.error("zip: 내용이 깨짐 (CRC) → CaptureError", lambda: stray.find_capture(p, w("crc")), "zip 을 풀지 못함")
    nodepth = [("cap/" + n, d) for n, d in small[:3]]
    C.error("zip: depth/ 가 빠진 촬영 → 풀고 변환에서 CaptureError",
            lambda: stray.convert(stray.find_capture(mkzip("nodepth.zip", nodepth), w("nodepth")), os.path.join(Z, "o_nodepth"), log=quiet),
            ("필요한 파일이 없음", "depth/"))
    mac = [(f"outer/cap/{f}", "x") for f in ("odometry.csv", "camera_matrix.csv")] + [(f"__MACOSX/outer/cap/._{f}", "x") for f in ("odometry.csv", "rgb.mp4")]
    r = stray.find_capture(mkzip("mac.zip", mac), w("mac"))
    C.add("zip: macOS 가 묶은 zip (__MACOSX · ._ 파일, 두 겹 아래) → 그 촬영 폴더", r and r.replace("\\", "/").endswith("outer/cap"), r)
    keep = stray.ZIP_MAX_FILES
    stray.ZIP_MAX_FILES = 3
    try:
        C.error("zip: 파일 수가 한도를 넘음 (ZIP_MAX_FILES 를 잠시 3) → CaptureError", lambda: stray.find_capture(mkzip("bomb.zip", small), w("bomb")), "너무 큼")
    finally:
        stray.ZIP_MAX_FILES = keep
    r = stray.find_capture(mkzip("none.zip", [("a.txt", "x"), ("b/c.txt", "x")]), w("none"))
    C.add("zip: 촬영이 없는 zip → None (CLI: 촬영 폴더가 아님)", r is None, r)

    for sub in ("a", "b"):                                      # 촬영 둘이 든 폴더 · 한 겹 아래 촬영
        for top in ("two_dirs", "nest"):
            if top == "nest" and sub == "b":
                continue
            os.makedirs(os.path.join(Z, top, sub))
            for f in ("odometry.csv", "camera_matrix.csv"):
                open(os.path.join(Z, top, sub, f), "w").write("x")
    C.error("폴더: 촬영 폴더 둘이 든 폴더 → CaptureError", lambda: stray.find_capture(os.path.join(Z, "two_dirs")), "여러 개")
    r = stray.find_capture(os.path.join(Z, "nest"))
    C.add("폴더: 한 겹 아래 촬영 폴더 → 그 폴더", r == os.path.join(Z, "nest", "a"), r)


# ── 멈춘 변환 ────────────────────────────────────────────────────────────

def check_interrupt(C, F, base):
    """base 에 가상 촬영본을 변환 (뒤 시험이 씀) → 예전 결과를 둔 채 키프레임 중간에 멈춤 → 다시 변환. → stray_meta dict"""
    m0 = stray.convert(F.src, base, log=quiet)
    C.add("기준: 가상 촬영본이 주의 없이 변환됨", m0["keyframes"] >= 10 and not m0["checks"]["warnings"] and m0["depth_pairing"] == "frame",
          f"{F.n}장 중 키프레임 {m0['keyframes']}장 · 깊이 규칙 {m0['depth_rule']['name']} · 주의 {m0['checks']['warnings']}")
    for f in ("pose_fix.json", "floor_fix.json", "stray_quality.json"):
        open(os.path.join(base, f), "w").write("{}")
    orig, calls, stop = stray.depth_keep, [0], max(2, m0["keyframes"] // 2)

    def boom(*a, **k):
        calls[0] += 1
        if calls[0] > stop:
            raise KeyboardInterrupt("멈춤 흉내")
        return orig(*a, **k)

    stray.depth_keep = boom
    stopped = False
    try:
        stray.convert(F.src, base, log=quiet)
    except KeyboardInterrupt as e:
        stopped = "흉내" in str(e)
        if not stopped:
            raise
    finally:
        stray.depth_keep = orig
    left = [f for f in stray.STALE if os.path.exists(os.path.join(base, f))]
    C.add("멈춘 변환: 키프레임 중간에 멈춘 뒤 예전 stray_meta · trajectory · 보정 · 품질 파일이 남지 않음", stopped and not left,
          f"{stop}장째에서 멈춤" + (f" · 남은 것 {left}" if left else "") if stopped else "멈춤 흉내가 일어나지 않음")
    m = stray.convert(F.src, base, log=quiet)
    writing = [f for f in os.listdir(base) if f.endswith(".writing")]
    C.add("멈춘 변환: 다시 돌리면 stray_meta.json 이 생기고 .writing 이 남지 않음 · 처음과 같은 키프레임",
          os.path.exists(os.path.join(base, "stray_meta.json")) and not writing and m["keyframes"] == m0["keyframes"],
          f"키프레임 {m['keyframes']}" + (f" · {writing}" if writing else ""))
    return m


# ── frame 번호 짝짓기 ────────────────────────────────────────────────────

def check_pairing(C, F, out):
    a = F.n // 2
    for name, drop in (("10줄 빠짐", set(range(a, a + 10))), ("2줄 빠짐", {a, a + 1})):
        tag = f"gap{len(drop)}"
        d = F.make(tag, odo=[s for k, s in enumerate(F.body) if k not in drop])
        errs, warns, info = stray.validate(d)
        C.add(f"짝짓기 {name}: validate 가 오류 없이 frame 번호로 짝짓고 건너뛴 번호 {len(drop)}개를 주의로",
              not errs and info.get("depth_pairing") == "frame" and any(f"건너뛴 번호 {len(drop)}개" in s for s in warns), js(errs + warns, 160))
        mp = os.path.join(out, tag)
        m = stray.convert(d, mp, log=quiet)
        od = np.genfromtxt(os.path.join(d, "odometry.csv"), delimiter=",", skip_header=1, usecols=range(13))
        tr = np.atleast_2d(np.loadtxt(os.path.join(mp, "trajectory_camera_optical.txt"), comments="#"))
        cap = sq._load(d, m, "camera_matrix")
        bad_val = bad_mask = tot = shifted = row_diff = 0
        for r in tr:
            i = int(np.argmin(np.abs(od[:, 0] - r[0])))
            f = int(od[i, 1])
            kd = cv2.imread(os.path.join(mp, "keyframes", "depth", f"{int(r[8])}.png"), cv2.IMREAD_UNCHANGED)
            zs = cv2.imread(os.path.join(d, "depth", f"{f:06d}.png"), cv2.IMREAD_UNCHANGED)
            keep = kd > 0
            bad_val += int((kd[keep] != zs[keep]).sum())
            _, ok = sq._depth(d, i, m["select"]["max_depth_m"], 1, m["depth_rule"], cap)
            bad_mask += int((ok != keep).sum())
            tot += kd.size
            if i != f:                                          # 예전처럼 줄 번호로 짝지었다면 읽었을 파일
                shifted += 1
                zr = cv2.imread(os.path.join(d, "depth", f"{i:06d}.png"), cv2.IMREAD_UNCHANGED)
                row_diff += int((kd[keep] != zr[keep]).sum())
        C.add(f"짝짓기 {name}: 키프레임 깊이 = frame 번호 파일의 깊이 (키프레임 {len(tr)}장)", m["depth_pairing"] == "frame" and bad_val == 0,
              f"짝짓기 {m['depth_pairing']} · 값이 다른 화소 {bad_val}")
        C.add(f"짝짓기 {name}: stray_quality._depth 가 남긴 화소 = stray.py 가 남긴 화소", bad_mask == 0, f"다른 화소 {bad_mask} / {tot}")
        C.add(f"짝짓기 {name}: 시험이 헛돌지 않음 — 줄 번호 ≠ frame 번호인 키프레임이 있고 줄 번호로 짝지었다면 깊이가 달랐음",
              shifted > 0 and row_diff > 0, f"{shifted}/{len(tr)}장 · 달랐을 화소 {row_diff}")


# ── 깨진 장 ──────────────────────────────────────────────────────────────

def check_broken(C, F, out, base, bm):
    conv = lambda d: lambda: stray.convert(d, os.path.join(out, os.path.basename(d)), log=quiet)   # noqa: E731
    small = _png(np.full((96, 128), 2, np.uint8))
    d = F.make("bad_conf_size", files={f"confidence/{f}": small for f in os.listdir(os.path.join(F.src, "confidence"))})
    C.error("깨진 장: 신뢰도 크기 ≠ 깊이 크기 → CaptureError", conv(d), ("confidence/", "크기"))
    d = F.make("bad_depth_first", files={"depth/000000.png": b"not a png"})
    C.error("깨진 장: 첫 깊이 파일이 깨짐 → CaptureError", conv(d), ("depth/000000.png", "읽지 못함"))
    d = F.make("bad_depth_8bit", files={"depth/000000.png": _png(np.full((192, 256), 100, np.uint8))})
    C.error("깨진 장: 첫 깊이 파일이 8비트 → CaptureError", conv(d), ("depth/000000.png", "16비트"))

    tr = np.atleast_2d(np.loadtxt(os.path.join(base, "trajectory_camera_optical.txt"), comments="#"))
    kf = int(F.od[int(np.argmin(np.abs(F.od[:, 0] - tr[len(tr) // 2, 0]))), 1])   # 가운데 키프레임의 frame 번호
    d = F.make("bad_depth_mid_size", files={f"depth/{kf:06d}.png": _png(np.full((96, 128), 1000, np.uint16))})
    C.error(f"깨진 장: 가운데 키프레임 깊이(depth/{kf:06d}.png) 크기가 다름 → CaptureError", conv(d), (f"depth/{kf:06d}.png", "첫 장"))
    d = F.make("bad_depth_mid", files={f"depth/{kf:06d}.png": b"broken"})
    m = stray.convert(d, os.path.join(out, "bad_depth_mid"), log=quiet)
    w = [s for s in m["checks"]["warnings"] if "읽지 못해" in s]
    C.add(f"깨진 장: 가운데 키프레임 깊이(depth/{kf:06d}.png)가 깨짐 → 그 장만 빼고 주의", w and m["keyframes"] == bm["keyframes"] - 1,
          f"키프레임 {bm['keyframes']} → {m['keyframes']} · {js(w, 140)}")


# ── 메쉬 · 자세 보정 info ────────────────────────────────────────────────

def fake_fix(times, poses):
    """늘 고치는 보정 (가상의 방은 수평 보정이 관문에서 건너뛰므로) — 높이 1 cm 내림. stray_mesh.build(pose_fix=…) 꼴"""
    new = np.array(poses, dtype=float, copy=True)
    new[:, 2, 3] -= 0.01
    t = [float(x) for x in times]
    return new, dict(method="chain", applied=True, status="applied", reasons=[], flags=[], keyframes=len(t),
                     steps=[dict(method="floor_cells", kind="floor", applied=True, status="applied", t=t, dz=[0.01] * len(t),
                                 tilt=None, reasons=[], flags=[])])


class StopBeforeFix:
    """with 안에서 stray_mesh 가 pose_fix.json 을 쓰려 하면 멈춤 (tsdf_meta.json · mesh_raw.ply 는 이미 바꿔 놓은 뒤) → .stopped"""

    def __enter__(self):
        self.stopped, self.orig = False, stray_mesh._write_bytes

        def stop(path, data):
            if os.path.basename(path) == stray_mesh.FIX_FILE:
                raise KeyboardInterrupt("pose_fix.json 쓰기 전에 멈춤 흉내")
            return self.orig(path, data)
        stray_mesh._write_bytes = stop
        return self

    def __exit__(self, et, e, tb):
        stray_mesh._write_bytes = self.orig
        self.stopped = et is KeyboardInterrupt and "흉내" in str(e)
        return self.stopped


def check_readonly(C, base, root):
    mp, bd = os.path.join(root, "ro_map"), os.path.join(root, "ro_build")
    name = "메쉬: 읽기 전용 맵 폴더에 --no-fix → 메쉬 · build/pose_fix.json 은 만들고 맵 폴더는 건너뛰며 한 줄 남김"
    shutil.copytree(base, mp)
    _chmod_dirs(mp, stat.S_IRUSR | stat.S_IXUSR)
    try:
        try:
            open(os.path.join(mp, ".probe"), "w").close()
            os.remove(os.path.join(mp, ".probe"))
            return C.skip(name, "폴더를 읽기 전용으로 만들 수 없음 (root · 윈도우)")
        except OSError:
            pass
        logs = []
        stray_mesh.build(mp, bd, fix=None, log=logs.append)
        msg = next((s for s in logs if "쓰지 못함" in s), "")
        C.add(name, os.path.exists(os.path.join(bd, "mesh_raw.ply")) and os.path.exists(os.path.join(bd, stray_mesh.FIX_FILE))
              and msg and not os.path.exists(os.path.join(mp, stray_mesh.FIX_FILE)), msg[:160] or "맵 폴더에 못 쓴다는 줄이 없음")
    finally:
        _chmod_dirs(mp, stat.S_IRWXU)


def check_fix_sha(C, base, root):
    mp, bd = os.path.join(root, "res", "input_map"), os.path.join(root, "res", "build")   # 결과 폴더 꼴 (input_map/ · build/)
    shutil.copytree(base, mp)
    build = lambda **kw: stray_mesh.build(mp, bd, log=quiet, **kw)   # noqa: E731
    look = lambda: sq._pose_fix(mp, bd)                        # noqa: E731
    note = lambda st: st.get("note") or ""                     # noqa: E731
    tp = os.path.join(bd, "tsdf_meta.json")

    build(pose_fix=fake_fix)
    info, st = look()
    moved = None
    if info is not None:
        fr = stray_mesh.read_map(mp)
        d = stray_fix.apply_fix(fr["times"], fr["poses"], info) - fr["poses"]
        moved = bool(np.allclose(d[:, 2, 3], -0.01) and np.abs(np.delete(d.reshape(len(d), -1), 11, axis=1)).max() < 1e-12)
    C.add("보정 sha256: 새 빌드 → build/pose_fix.json 을 확인해 적용 (키프레임 높이만 −1 cm)", info is not None and st.get("file") == "build/pose_fix.json" and moved,
          f"{st.get('file')} · 높이만 −1 cm {moved}")
    os.remove(os.path.join(bd, stray_mesh.FIX_FILE))
    info, st = look()
    C.add("보정 sha256: build/pose_fix.json 이 없으면 input_map 의 같은 바이트를 씀", info is not None and st.get("file") == "input_map/pose_fix.json", st.get("file"))

    build(pose_fix=fake_fix)
    for p in (os.path.join(bd, stray_mesh.FIX_FILE), os.path.join(mp, stray_mesh.FIX_FILE)):
        open(p, "ab").write(b" ")                               # JSON 으로는 같게 읽히지만 바이트가 다름
    info, st = look()
    C.add("보정 sha256: 손으로 고친 pose_fix.json (build · input_map 둘 다) → 적용 안 함 · note", info is None and "sha256" in note(st), note(st)[:160])

    build(pose_fix=fake_fix)
    with StopBeforeFix() as s:
        build(fix=None)
    info, st = look()
    C.add("보정 sha256: 고치지 않는 다시 만들기가 pose_fix.json 을 쓰기 전에 멈춤 (예전 고친 info 가 남음) → 적용 안 함",
          s.stopped and info is None and st.get("applied") is False, ("" if s.stopped else "멈춤 흉내가 일어나지 않음 · ") + js(st, 160))

    build(fix=None)
    with StopBeforeFix() as s:
        build(pose_fix=fake_fix)
    info, st = look()
    C.add("보정 sha256: 고치는 다시 만들기가 pose_fix.json 을 쓰기 전에 멈춤 (예전 끈 info 가 남음) → 적용 안 함 · note",
          s.stopped and info is None and "sha256" in note(st), ("" if s.stopped else "멈춤 흉내가 일어나지 않음 · ") + note(st)[:160])

    build(pose_fix=fake_fix)                                    # 예전 빌드 꼴: tsdf_meta.pose_fix 에 sha256 없음 · input_map/pose_fix.json
    tm = json.load(open(tp, encoding="utf-8"))
    tm["pose_fix"].pop("sha256")
    json.dump(tm, open(tp, "w", encoding="utf-8"))
    os.remove(os.path.join(bd, stray_mesh.FIX_FILE))
    info, st = look()
    C.add("보정 sha256: 예전 빌드 꼴 (sha256 없음) → input_map/pose_fix.json 적용", info is not None and st.get("file") == "input_map/pose_fix.json", st.get("file"))
    tm["pose_fix"]["applied"] = False
    json.dump(tm, open(tp, "w", encoding="utf-8"))
    info, st = look()
    C.add("보정 sha256: 예전 빌드인데 메쉬는 보정 없이 만듦 → input_map 의 고친 info 를 적용 안 함 · note", info is None and "고치지 않고" in note(st), note(st)[:160])


# ── --tilt · CLI ─────────────────────────────────────────────────────────

def _cli(mod, args):
    p = subprocess.run([sys.executable, "-m", f"scan2sim.{mod}"] + args, cwd=RECON, capture_output=True, text=True,
                       encoding="utf-8", errors="replace", env=ENV, timeout=300)
    return p.returncode, p.stdout + p.stderr


def check_cli(C, F, out, base):
    tf = stray_fix.tilt_fix
    C.add("--tilt: 주지 않으면 보정 이름 그대로 (xy)", tf("xy", None) == "xy")
    logs = []
    r = tf("xy", True, explicit=False, log=logs.append, who="mesh")
    C.add("--tilt: --fix 없이 주면 높이 단계를 더해 floor+xy 로 · 한 줄 남김", r == "floor+xy" and len(logs) == 1, f"{r} · {logs[:1]}")
    C.add("--no-tilt: --fix floor+xy 와 같이 주면 그대로", tf("floor+xy", False, explicit=True) == "floor+xy")
    C.add("--tilt: --fix floor 와 같이 주면 그대로", tf("floor", True, explicit=True) == "floor")
    C.error("--tilt + --fix xy → ValueError (--fix floor+xy 를 알려 줌)", lambda: tf("xy", True, explicit=True), "--fix floor+xy", exc=ValueError)
    C.error("--no-tilt + --no-fix → ValueError", lambda: tf(None, False), ("--no-tilt", "끄면"), exc=ValueError)
    C.error("보정 설정: 이 보정(xy)에 없는 단계(floor)의 설정 → ValueError", lambda: stray_fix.split_opts(("xy",), {"floor": {"tilt": True}}),
            "floor", exc=ValueError)
    C.error("보정 설정: 모르는 설정 이름 → ValueError", lambda: stray_fix.split_opts(("floor",), {"floor": {"tilt_typo": True}}), "tilt_typo", exc=ValueError)
    C.add("보정 설정: 예전 꼴 dict(tilt=True) 는 높이 설정으로", stray_fix.split_opts(("floor", "xy"), {"tilt": True}) == {"floor": {"tilt": True}})

    for k, (mod, args, must) in enumerate((("stray_mesh", ["--no-fix", "--tilt"], "쓸 데가 없음"),
                                           ("stray_mesh", ["--fix", "xy", "--tilt"], "--fix floor+xy"),
                                           ("stray_fix", ["--fix", "xy", "--no-tilt"], "--fix floor+xy"))):
        o = os.path.join(out, f"cli{k}")
        rc, txt = _cli(mod, [base] + (["--out", o] if mod == "stray_mesh" else []) + args)
        last = (txt.strip().splitlines() or [""])[-1]
        C.add(f"CLI: {mod} {' '.join(args)} → 종료 코드 2 · 아무것도 안 만듦", rc == 2 and must in txt and "Traceback" not in txt and not os.path.exists(o),
              f"종료 코드 {rc} · {last[:160]}")
    d = F.make("cli_short", odo=F.body[:50], depth=range(50), conf=range(50))
    rc, txt = _cli("stray", [d, "--out", os.path.join(out, "cli_short")])
    last = (txt.strip().splitlines() or [""])[-1]
    C.add("CLI: stray 에 짧은 촬영 → 종료 코드 1 · '[stray] 실패:' 문구 · traceback 없음",
          rc == 1 and "[stray] 실패:" in txt and "너무 짧음" in txt and "Traceback" not in txt, f"종료 코드 {rc} · {last[:160]}")


# ── 차례 ────────────────────────────────────────────────────────────────

def make_fixture(d):
    """samples/make_room.py · make_stray.py 로 가상 촬영본 zip 을 d 에 만듦 (있으면 그대로) → zip 경로 · 못 만들면 None"""
    zp = os.path.join(d, NAME + ".zip")
    if os.path.isfile(zp):
        return zp
    os.makedirs(d, exist_ok=True)
    t = time.time()
    room = os.path.join(d, "room")
    for cmd in ([sys.executable, os.path.join(SAMPLES, "make_room.py"), "--out", room, "--seed", "0"],
                [sys.executable, os.path.join(SAMPLES, "make_stray.py"), "--room", room, "--out", d] + FIXTURE):
        p = subprocess.run(cmd, cwd=RECON, capture_output=True, text=True, encoding="utf-8", errors="replace", env=ENV)
        if p.returncode or (cmd[1].endswith("make_stray.py") and not os.path.isfile(zp)):
            print(f"  시험 자료를 만들지 못함 — {os.path.basename(cmd[1])} 종료 코드 {p.returncode}\n" + (p.stdout + p.stderr)[-1500:], flush=True)
            return None
    print(f"  시험 자료: {' '.join(FIXTURE)} 가상 촬영본을 만듦 ({time.time() - t:.0f}초) → {zp}", flush=True)
    return zp


def section(C, name, fn, *args):
    """시험 묶음 하나 — 예상 못 한 예외는 그 묶음의 실패로 남기고 다음 묶음으로"""
    try:
        return fn(*args)
    except Exception as e:
        traceback.print_exc()
        C.add(f"{name}: 끝까지 돎", False, f"{type(e).__name__}: {str(e)[:200]}")
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--fixtures", help="가상 촬영본을 둘 폴더 (있으면 다시 만들지 않음). 안 주면 임시 폴더에 만들고 지움")
    ap.add_argument("--work", help="망가뜨린 촬영본 · 변환 결과를 둘 폴더 (안 주면 임시 폴더를 만들고 끝나면 지움)")
    ap.add_argument("--json", help="결과를 이 파일에")
    a = ap.parse_args()
    t0 = time.time()
    C, rc = Checks(), None
    tmp = tempfile.mkdtemp(prefix="scan2sim_edge_")
    work = os.path.abspath(a.work) if a.work else tmp
    T = os.path.join(work, "t")
    _rmtree(T)
    os.makedirs(T)
    out = os.path.join(T, "out")
    zp = None
    try:
        zp = make_fixture(os.path.abspath(a.fixtures) if a.fixtures else os.path.join(tmp, "fixture"))
        print(f"stray_edge: {zp}", flush=True)
        src = stray.find_capture(zp, os.path.join(T, "unzip")) if zp else None
        F = Fake(src, os.path.join(T, "cases")) if src else None
        if F is None or F.n < MIN_ROWS:
            print(f"  중단 — 시험 자료가 없거나 짧음 ({src}, {F.n if F else 0}줄 < {MIN_ROWS})", flush=True)
            rc = 2
        else:
            section(C, "입력", check_inputs, C, F, out)
            section(C, "zip", check_zips, C, os.path.join(T, "zip"))
            base = os.path.join(out, "base")
            bm = section(C, "멈춘 변환", check_interrupt, C, F, base)
            section(C, "짝짓기", check_pairing, C, F, out)
            if bm is None:
                C.add("기준 변환", False, "가상 촬영본을 변환하지 못해 깨진 장 · 메쉬 · CLI 시험을 건너뜀")
            else:
                section(C, "깨진 장", check_broken, C, F, out, base, bm)
                section(C, "읽기 전용 맵", check_readonly, C, base, os.path.join(T, "mesh"))
                section(C, "보정 sha256", check_fix_sha, C, base, os.path.join(T, "mesh"))
                section(C, "CLI", check_cli, C, F, out, base)
    except stray.CaptureError as e:
        print(f"  중단 — 시험 자료를 풀지 못함: {e}", flush=True)
        rc = 2
    finally:
        _rmtree(tmp)                                            # 임시 폴더 (--fixtures · --work 를 안 줬을 때 쓴 것) — --work 의 결과는 남김
        done = [r for r in C.rows if r["ok"] is not None]
        ok = C.ok and rc is None
        secs = round(time.time() - t0, 1)
        if a.json:
            json.dump(dict(fixture=zp, ok=ok, checks=C.rows, total_s=secs), open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    skipped = len(C.rows) - len(done)
    print(f"stray_edge {'통과' if ok else '실패'} — 검사 {sum(r['ok'] for r in done)}/{len(done)}" + (f" · 건너뜀 {skipped}" if skipped else "")
          + f" · {secs}초", flush=True)
    sys.exit(rc if rc is not None else (0 if ok else 1))


if __name__ == "__main__":
    main()
