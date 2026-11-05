#!/usr/bin/env python3
r"""X-Mobility 학습 형식(xm_v1 등)을 녹화 묶음 단위로 다시 나눠 새 폴더에 만든다 — 원본은 읽기만 한다.

왜: xm_v1(v1.0)은 val 에 반시계 녹화만(경로 지시 직진 · 좌), test 에 시계 녹화만(직진 · 우) 있어서
    val_loss 로 고른 ckpt 는 우회전을 안 보고, 오프라인 test 에는 좌회전이 없다.
    --preset v1.1 = Nav2 장애물 녹화 둘을 맞바꿈 — 1014_123928_nav2_ccw(콘 · 반시계) val→test,
    1014_115111_nav2_cw(슬라럼 · 시계) test→val. 자동 교사 묶음은 그대로.

하는 일:
  점검  원본에 train · val · test 가 있음, split 폴더 안은 <split>/<시나리오>/run_<묶음>_<NN>.pqt 뿐,
        한 묶음은 한 split 에만, index/<묶음>.json 의 split = 파일 위치, 옮길 묶음이 있음(파일 이름 또는 index),
        대상 폴더가 없음(덮어쓰지 않음) · 원본 안이 아님, 옮긴 뒤 빈 split 없음. 하나라도 걸리면 아무것도 쓰지 않음
  복사  .pqt 를 새 배치로(옮기지 않은 것은 같은 split) — 기본 shutil.copy2, --link hard 면 하드 링크(같은 디스크만).
        index/<묶음>.json 은 옮긴 묶음만 split · files[].file 을 고치고 resplit_from 을 더함, 나머지는 그대로 복사.
        그 밖의 최상위 파일(README.md · export.log 등)도 그대로 복사 — 내용은 원본 나눔 기준
  확인  대상 파일마다 sha256 = 원본 파일 sha256, 대상을 다시 훑어 묶음마다 split 하나 · 계획과 같음
  기록  <대상>/RESPLIT.json(마지막에 씀 — 없으면 중간에 끊긴 것) — 원본 경로 · 원본 sha12 · 대상 sha12(RESPLIT.json 뺌) ·
        옮김 · split × 시나리오별 묶음 · 파일 · 행 · 경로 지시 수(pyarrow 있으면 parquet 에서, 없으면 행만 index 에서) · 시각
  출력  MANIFEST 한 줄 초안(sha12 = RESPLIT.json 까지 포함한 대상 폴더), --check 면 e2e/check_xm_dataset.py 를 대상에 돌림

폴더 sha 는 MANIFEST.md 디렉터리 방법을 LC_ALL=C 순서로 계산한 값과 같다:
  (cd <폴더> && find . -type f -print0 | LC_ALL=C sort -z | xargs -0 sha256sum) | sha256sum | cut -c1-12
  LC_ALL=C 없이 en_US.UTF-8 로캘에서 돌리면 대문자 이름(README.md · RESPLIT.json)의 순서가 바뀌어 값이 달라진다
  (C · C.UTF-8 · ko_KR.UTF-8 은 C 순서와 같음 — glibc 2.39 sort 로 확인).

사용(PowerShell, Windows — 레포 폴더에서):
  python e2e\resplit_xm.py C:\Users\hennr\Downloads\xm_v1_unzip\xm_v1 C:\Users\hennr\Downloads\xm_v1_unzip\xm_v1.1 --preset v1.1 --check
  python e2e\resplit_xm.py <원본> <대상> --preset v1.1 --dry-run          # 계획 · 원본 sha 만, 쓰지 않음
사용(bash, server207):
  ~/xmobility_test/venv/bin/python e2e/resplit_xm.py data/teacher/xm_v1 data/teacher/xm_v1.1 --preset v1.1 --link hard --check
  python3 e2e/resplit_xm.py <원본> <대상> --move <묶음>=<train|val|test> [--move ...]
필요: 표준 라이브러리만. pyarrow 가 있으면 행 · 경로 지시 수를 parquet 에서 셈. --check 는 numpy · pandas · pyarrow · pillow.
종료 코드: 0 성공 · 1 복사 확인 실패 또는 --check 오류(못 돌림 포함) · 2 거부(아무것도 쓰지 않음).
"""
import argparse
import datetime
import difflib
import hashlib
import importlib.util
import json
import math
import os
import platform
import re
import shutil
import subprocess
import sys
from collections import Counter, defaultdict
from pathlib import Path

try:
    import pyarrow.parquet as pq
except ImportError:  # 행 수는 index json 에서, 경로 지시 수는 생략
    pq = None

SPLITS = ("train", "val", "test")
RUN_RE = re.compile(r"^run_(.+)_(\d+)\.pqt$")      # e2e/check_xm_dataset.py 와 같음
TOOL = "e2e/resplit_xm.py"
RECORD = "RESPLIT.json"
CMD_NAME = {0: "직진", 1: "좌", 2: "우"}
PRESETS = {
    "v1.1": dict(
        moves=[("1014_123928_nav2_ccw", "val", "test", "콘 · 반시계"),
               ("1014_115111_nav2_cw", "test", "val", "슬라럼 · 시계")],
        name="xm_parking_r3", version="v1.1", path="data/teacher/xm_v1.1/",
        base="xm_parking_r3 v1.0", base_sha12="9350296bfd8f",      # MANIFEST 11.2 행 — server207 에서 계산한 값
        note="Nav2 장애물 녹화 val ↔ test 맞바꿈"),
}


class Refuse(Exception):
    pass


# ---------------------------------------------------------------- 폴더 sha (MANIFEST 디렉터리 방법)
def file_sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _ckey(rel):
    return rel.encode("utf-8", "surrogateescape")       # LC_ALL=C sort = 바이트 순서


def list_files(root):
    """find . -type f 와 같은 목록(심볼릭 링크 제외) — '/' 로 이은 상대 경로, C 순서."""
    out = []
    for dp, _, fns in os.walk(root):
        for fn in fns:
            p = os.path.join(dp, fn)
            if not os.path.islink(p) and os.path.isfile(p):
                out.append(os.path.relpath(p, root).replace(os.sep, "/"))
    return sorted(out, key=_ckey)


def hash_files(root, rels):
    return {rel: file_sha256(os.path.join(root, *rel.split("/"))) for rel in rels}


def tree_sha(hashes):
    """{상대 경로: sha256} → `xargs -0 sha256sum | sha256sum` 의 값(GNU 이름 이스케이프 포함)."""
    h = hashlib.sha256()
    for rel in sorted(hashes, key=_ckey):
        name = "./" + rel
        if any(c in name for c in "\\\n\r"):
            name = name.replace("\\", "\\\\").replace("\n", "\\n").replace("\r", "\\r")
            line = f"\\{hashes[rel]}  {name}\n"
        else:
            line = f"{hashes[rel]}  {name}\n"
        h.update(line.encode("utf-8", "surrogateescape"))
    return h.hexdigest()


def human(n):
    """du -h 처럼(1024 단위, 올림) — 파일 크기 합이라 du 와 조금 다를 수 있음."""
    v, i, units = float(n), 0, ("B", "K", "M", "G", "T")
    while v >= 1024 and i < len(units) - 1:
        v, i = v / 1024, i + 1
    if i == 0:
        return f"{n}B"
    return f"{math.ceil(v * 10) / 10:.1f}{units[i]}" if v < 10 else f"{math.ceil(v)}{units[i]}"


# ---------------------------------------------------------------- 원본 훑기
def scan(root):
    """→ dict(files=[{split, scenario, name, bundle, rel}], index={묶음: (rel, data)}, other=[rel], errors, warnings)"""
    inv = dict(files=[], index={}, other=[], errors=[], warnings=[])
    err, warn = inv["errors"].append, inv["warnings"].append
    for sp in SPLITS:
        d = root / sp
        if not d.is_dir():
            err(f"{sp}/ 없음")
            continue
        for sc in sorted(d.iterdir()):
            if sc.is_symlink() or not sc.is_dir():
                err(f"{sp}/{sc.name}: split 폴더 안의 시나리오 폴더가 아닌 항목")
                continue
            for f in sorted(sc.iterdir()):
                m = RUN_RE.match(f.name)
                if f.is_symlink() or not f.is_file() or not m:
                    err(f"{sp}/{sc.name}/{f.name}: run_<묶음>_<NN>.pqt 가 아님 — 묶음을 알 수 없음")
                    continue
                inv["files"].append(dict(split=sp, scenario=sc.name, name=f.name, bundle=m.group(1),
                                         rel=f"{sp}/{sc.name}/{f.name}"))
    idx = root / "index"
    if idx.is_dir():
        for f in sorted(idx.iterdir()):
            rel = f"index/{f.name}"
            if f.is_symlink() or not f.is_file():
                err(f"{rel}: 일반 파일이 아님")
            elif f.suffix != ".json":
                inv["other"].append(rel)
            else:
                try:
                    inv["index"][f.stem] = (rel, json.loads(f.read_text(encoding="utf-8")))
                except (ValueError, UnicodeDecodeError) as e:
                    err(f"{rel}: JSON 읽기 실패 {e}")
    elif idx.exists() or idx.is_symlink():
        err("index: 폴더가 아님")
    else:
        warn("index/ 없음 — index json 은 건너뜀")
    for e in sorted(root.iterdir()):
        if e.name in SPLITS or e.name == "index":
            continue
        if e.is_symlink() or not (e.is_file() or e.is_dir()):
            err(f"{e.name}: 일반 파일 · 폴더가 아님(심볼릭 링크 등)")
        else:
            inv["other"].append(e.name)
    return inv


def bundle_layout(inv):
    """묶음 → 현재 split(파일 위치, 없으면 index). 묶음이 여러 split 이나 index 와 다른 곳에 있으면 오류."""
    err, warn = inv["errors"].append, inv["warnings"].append
    where = defaultdict(set)
    for f in inv["files"]:
        where[f["bundle"]].add(f["split"])
    cur = {}
    for b, sps in sorted(where.items()):
        if len(sps) > 1:
            err(f"묶음 {b} 가 여러 split 에 있음 {sorted(sps)} — 원본부터 평가 누수")
        cur[b] = sorted(sps)[0]
    for b, (rel, data) in sorted(inv["index"].items()):
        isp = data.get("split")
        if b in where:
            if isp not in where[b]:
                err(f"{rel}: split={isp} 인데 파일은 {sorted(where[b])} 에 있음")
            listed = {str(x.get("file", "")).replace("\\", "/").rsplit("/", 1)[-1] for x in data.get("files", [])}
            actual = {f["name"] for f in inv["files"] if f["bundle"] == b}
            if listed != actual:
                warn(f"{rel}: files 목록 {len(listed)}개 ≠ 실제 파일 {len(actual)}개")
        else:
            if isp not in SPLITS:
                err(f"{rel}: 파일도 없고 split 값({isp})도 train · val · test 가 아님")
                continue
            if data.get("runs"):
                warn(f"{rel}: runs={data.get('runs')} 인데 .pqt 파일이 없음")
            cur[b] = isp
    for b in sorted(where):
        if b not in inv["index"]:
            warn(f"index/{b}.json 없음 — 이 묶음은 index 없이 파일만 옮김")
    return cur


# ---------------------------------------------------------------- 행 · 경로 지시 수
def file_stats(path, index_rows):
    """→ (rows, Counter(route_command) 또는 None, 출처)"""
    if pq is not None:
        try:
            rows = pq.read_metadata(path).num_rows
            rc = None
            if "route_command" in pq.read_schema(path).names:
                col = pq.read_table(path, columns=["route_command"]).column(0).to_pylist()
                rc = Counter(int(x) for x in col if x is not None)
            return rows, rc, "parquet"
        except Exception as e:  # noqa: BLE001 — 깨진 파일은 점검기가 잡음, 여기선 수만 못 셈
            print(f"  경고: {path.name} 읽기 실패({type(e).__name__}) — 행 수는 index 값", file=sys.stderr)
    return index_rows, None, "index" if index_rows is not None else None


def group_counts(files, key):
    """files 의 key(f) 별 묶음 · 파일 · 행 · 경로 지시 합."""
    g = {}
    for f in files:
        k = key(f)
        s = g.setdefault(k, dict(bundles=set(), files=0, rows=0, route_command=Counter(), _rc_ok=True, _rows_ok=True))
        s["bundles"].add(f["bundle"])
        s["files"] += 1
        if f["rows"] is None:
            s["_rows_ok"] = False
        else:
            s["rows"] += f["rows"]
        if f["rc"] is None:
            s["_rc_ok"] = False
        else:
            s["route_command"].update(f["rc"])
    out = {}
    for k in sorted(g, key=lambda k: (SPLITS.index(k.split("/")[0]), k)):     # train · val · test 순
        s = g[k]
        out[k] = dict(bundles=sorted(s["bundles"]), files=s["files"], rows=s["rows"] if s["_rows_ok"] else None,
                      route_command={str(c): s["route_command"][c] for c in sorted(s["route_command"])} if s["_rc_ok"] else None)
    return out


def fmt_counts(s):
    t = f"묶음 {len(s['bundles']):2d} · 파일 {s['files']:3d} · 행 {s['rows'] if s['rows'] is not None else '-':>6}"
    if s["route_command"] is not None:
        t += " · 지시 " + " ".join(f"{CMD_NAME.get(int(c), c)} {n}" for c, n in s["route_command"].items())
    return t


# ---------------------------------------------------------------- index 다시 쓰기
def resplit_rel(rel, new_split):
    parts = str(rel).replace("\\", "/").split("/")
    if parts and parts[0] in SPLITS:
        parts[0] = new_split
    return "/".join(parts)


def rewrite_index(data, old, new, src_sha12, how):
    mark = dict(split=old, src_sha12=src_sha12, by=how)
    if "resplit_from" in data:
        mark["prev"] = data["resplit_from"]
    out = {}
    for k, v in data.items():
        if k == "resplit_from":
            continue
        if k == "split":
            out["split"], out["resplit_from"] = new, mark
        elif k == "files" and isinstance(v, list):
            out["files"] = [dict(x, file=resplit_rel(x["file"], new)) if isinstance(x, dict) and "file" in x else x for x in v]
        else:
            out[k] = v
    if "split" not in out:
        out["split"], out["resplit_from"] = new, mark
    return out


def write_json(path, obj):
    with open(path, "w", encoding="utf-8", newline="\n") as f:     # Windows 에서도 LF — 어디서 만들든 같은 바이트
        json.dump(obj, f, ensure_ascii=False, indent=1)


# ---------------------------------------------------------------- 계획
def parse_moves(a, cur):
    """→ [(묶음, 새 split, 기대 split 또는 None, 설명)] — 문제는 Refuse 로."""
    errs, moves = [], []
    if a.preset:
        for b, fr, to, label in PRESETS[a.preset]["moves"]:
            moves.append((b, to, fr, label))
    for s in a.move:
        b, eq, to = s.partition("=")
        b, to = b.strip(), to.strip()
        if not eq or not b or to not in SPLITS:
            errs.append(f"--move {s!r}: <묶음>=<train|val|test> 꼴이 아님")
            continue
        moves.append((b, to, None, ""))
    seen = Counter(m[0] for m in moves)
    for b, n in seen.items():
        if n > 1:
            errs.append(f"묶음 {b} 를 두 번 옮기려 함(--preset 과 --move 겹침 포함)")
    for b, to, fr, _ in moves:
        if b not in cur:
            near = difflib.get_close_matches(b, list(cur), n=3, cutoff=0.8)
            errs.append(f"묶음 {b} 없음(파일 이름 · index 둘 다)" + (f" — 비슷한 이름 {near}" if near else ""))
        elif fr is not None and cur[b] != fr:
            errs.append(f"--preset {a.preset}: {b} 는 {fr} 에 있어야 하는데 {cur[b]} 에 있음 — 원본이 v1.0 이 맞는지 확인")
    if not moves and not errs:
        errs.append("옮길 것이 없음 — --preset 또는 --move 를 주세요")
    if errs:
        raise Refuse("\n".join(errs))
    return moves


def main():
    for s in (sys.stdout, sys.stderr):
        try:
            s.reconfigure(errors="replace")
        except (AttributeError, ValueError):
            pass
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("src", help="원본 폴더(xm_v1 등) — 읽기만 함")
    ap.add_argument("dst", help="새 폴더 — 없어야 함")
    ap.add_argument("--preset", choices=sorted(PRESETS), help="정해 둔 옮김(v1.1: Nav2 장애물 녹화 val ↔ test)")
    ap.add_argument("--move", action="append", default=[], metavar="묶음=split", help="묶음 하나를 옮김(여러 번 가능)")
    ap.add_argument("--link", choices=("copy", "hard"), default="copy",
                    help=".pqt 를 복사(기본) 또는 하드 링크(같은 디스크, 공간 안 씀 — 대상 .pqt 를 고치면 원본도 바뀜)")
    ap.add_argument("--dry-run", action="store_true", help="점검 · 계획 · 원본 sha 만, 아무것도 쓰지 않음")
    ap.add_argument("--check", action="store_true", help="끝나면 e2e/check_xm_dataset.py 를 대상에 돌림")
    a = ap.parse_args()

    src, dst = Path(a.src), Path(a.dst)
    try:
        if not src.is_dir():
            raise Refuse(f"원본 폴더 없음: {src}")
        if dst.exists() or dst.is_symlink():
            hint = "" if (dst / RECORD).exists() else f" ({RECORD} 이 없으면 이전 실행이 중간에 끊긴 것 — 지우고 다시)"
            raise Refuse(f"대상이 이미 있음: {dst} — 덮어쓰지 않음, 다른 이름을 주세요{hint}")
        sr, dr = src.resolve(), dst.resolve()
        if dr == sr or sr in dr.parents:
            raise Refuse(f"대상이 원본 안에 있음: {dst} — 원본 폴더가 바뀌므로 거부")
        inv = scan(src)
        cur = bundle_layout(inv)
        if inv["errors"]:
            raise Refuse("원본 점검 실패:\n" + "\n".join(inv["errors"]))
        moves = parse_moves(a, cur)
    except Refuse as e:
        for line in str(e).splitlines():
            print(f"거부: {line}", file=sys.stderr)
        print("아무것도 쓰지 않음.", file=sys.stderr)
        return 2

    warnings = list(inv["warnings"])
    new = dict(cur)
    for b, to, _, _ in moves:
        if cur[b] == to:
            warnings.append(f"--move {b}={to}: 이미 {to} 에 있음 — 바뀌지 않음")
        new[b] = to
    if all(cur[b] == to for b, to, _, _ in moves):
        print("거부: 모든 옮김이 제자리 — 바뀌는 것 없음\n아무것도 쓰지 않음.", file=sys.stderr)
        return 2

    # 파일별 행 · 경로 지시
    idx_rows = {}
    for b, (_, data) in inv["index"].items():
        for x in data.get("files", []) if isinstance(data.get("files"), list) else []:
            if isinstance(x, dict) and "file" in x:
                idx_rows[(b, str(x["file"]).replace("\\", "/").rsplit("/", 1)[-1])] = x.get("rows")
    rows_src = set()
    for f in inv["files"]:
        f["rows"], f["rc"], how_rows = file_stats(src / f["split"] / f["scenario"] / f["name"], idx_rows.get((f["bundle"], f["name"])))
        rows_src.add(how_rows)
        f["new_split"] = new[f["bundle"]]
        f["new_rel"] = f"{f['new_split']}/{f['scenario']}/{f['name']}"
    rows_source = rows_src.pop() if len(rows_src) == 1 else ("mixed" if rows_src else None)

    empty = [sp for sp in SPLITS if not any(f["new_split"] == sp for f in inv["files"])]
    if empty:
        print(f"거부: 옮긴 뒤 빈 split {empty} — 로더가 빈 split 을 못 씀\n아무것도 쓰지 않음.", file=sys.stderr)
        return 2

    before = group_counts(inv["files"], lambda f: f"{f['split']}/{f['scenario']}")
    after = group_counts(inv["files"], lambda f: f"{f['new_split']}/{f['scenario']}")
    sp_before = group_counts(inv["files"], lambda f: f["split"])
    sp_after = group_counts(inv["files"], lambda f: f["new_split"])
    for sp in ("val", "test"):
        rc = sp_after[sp]["route_command"]
        if rc is not None:
            miss = [CMD_NAME[c] for c in (1, 2) if rc.get(str(c), 0) == 0]
            if miss:
                warnings.append(f"옮긴 뒤 {sp} 에 경로 지시 {' · '.join(miss)} 없음")

    how = " ".join([TOOL] + ([f"--preset {a.preset}"] if a.preset else []) + [f"--move {s}" for s in a.move])
    move_rows = []
    for b, to, _, label in moves:
        fs = [f for f in inv["files"] if f["bundle"] == b]
        rows = sum(f["rows"] for f in fs) if all(f["rows"] is not None for f in fs) else None
        scen = sorted({f["scenario"] for f in fs}) or [inv["index"].get(b, ("", {}))[1].get("scenario")]
        move_rows.append(dict(bundle=b, scenario=",".join(str(s) for s in scen), **{"from": cur[b]}, to=to,
                              files=len(fs), rows=rows, label=label))

    print(f"원본  {src}")
    print(f"대상  {dst}  ({'하드 링크' if a.link == 'hard' else '복사'}{', dry-run' if a.dry_run else ''})")
    for m in move_rows:
        print(f"옮김  {m['bundle']:22s} {m['scenario']:12s} {m['from']:5s} → {m['to']:5s} 파일 {m['files']} · 행 {m['rows'] if m['rows'] is not None else '-'}"
              + (f"  ({m['label']})" if m["label"] else ""))
    print(f"split 별 (행 출처: {rows_source or '없음'}{'' if pq else ' — pyarrow 없음, 경로 지시 수 생략'})")
    for sp in SPLITS:
        print(f"  {sp:5s} 이전 {fmt_counts(sp_before.get(sp, dict(bundles=[], files=0, rows=0, route_command=None)))}")
        print(f"  {'':5s} 이후 {fmt_counts(sp_after[sp])}")
    print("이후 split × 시나리오")
    for k, s in after.items():
        names = "  " + ", ".join(s["bundles"]) if len(s["bundles"]) <= 3 else ""
        print(f"  {k:24s} {fmt_counts(s)}{names}")
    for w in warnings:
        print(f"  경고: {w}")

    print("원본 sha 계산 중 ...", flush=True)
    src_rels = list_files(src)
    src_hash = hash_files(src, src_rels)
    src_sha = tree_sha(src_hash)
    src_bytes = sum((src / r).stat().st_size for r in src_rels)
    preset = PRESETS.get(a.preset)
    print(f"원본 sha12 {src_sha[:12]} · 파일 {len(src_rels)} · {human(src_bytes)}"
          + (f"  (MANIFEST {preset['base']} {preset['base_sha12']}: {'같음' if src_sha[:12] == preset['base_sha12'] else '다름'})"
             if preset else ""))
    if a.dry_run:
        print("dry-run — 아무것도 쓰지 않음.")
        return 0

    # ------------------------------------------------------------ 쓰기
    plan = {}            # 대상 상대 경로 → 원본 상대 경로(그대로) 또는 None(새로 씀)
    for f in inv["files"]:
        plan[f["new_rel"]] = f["rel"]
    for rel in inv["other"]:
        if (src / rel).is_dir():
            for r in list_files(src / rel):
                plan[f"{rel}/{r}"] = f"{rel}/{r}"
        elif rel == RECORD:                          # 원본도 다시 나눈 것 — 그 기록은 이름을 바꿔 남김
            plan[f"RESPLIT_{src_sha[:12]}.json"] = rel
        else:
            plan[rel] = rel
    rewritten = {}
    for b, (rel, data) in inv["index"].items():
        if new[b] != cur[b]:
            rewritten[rel] = rewrite_index(data, cur[b], new[b], src_sha[:12], how)
            plan[rel] = None
        else:
            plan[rel] = rel
    try:
        dst.mkdir(parents=True, exist_ok=False)
        print(f"쓰는 중 ... 파일 {len(plan)}", flush=True)
        for drel, srel in sorted(plan.items()):
            dp = dst.joinpath(*drel.split("/"))
            dp.parent.mkdir(parents=True, exist_ok=True)
            if srel is None:
                write_json(dp, rewritten[drel])
            elif a.link == "hard" and drel.endswith(".pqt"):
                os.link(src.joinpath(*srel.split("/")), dp)
            else:
                shutil.copy2(src.joinpath(*srel.split("/")), dp)
    except BaseException as e:
        print(f"중간에 멈춤 — {dst} 는 불완전({RECORD} 없음). 지우고 다시 실행하세요.", file=sys.stderr)
        if a.link == "hard" and isinstance(e, OSError):
            print("  --link hard 는 원본과 같은 디스크(NTFS · ext4 등)에서만 됨 — 아니면 기본 복사로.", file=sys.stderr)
        raise

    # ------------------------------------------------------------ 확인
    print("대상 확인 중 ...", flush=True)
    problems = []
    dst_rels = list_files(dst)
    if set(dst_rels) != set(plan):
        problems.append(f"대상 파일 목록이 계획과 다름: 더 있음 {sorted(set(dst_rels) - set(plan))[:5]} · 빠짐 {sorted(set(plan) - set(dst_rels))[:5]}")
    dst_hash = hash_files(dst, dst_rels)
    for drel, srel in plan.items():
        if srel is not None and dst_hash.get(drel) != src_hash.get(srel):
            problems.append(f"{drel}: sha256 가 원본 {srel} 와 다름")
    inv2 = scan(dst)
    cur2 = bundle_layout(inv2)
    problems += inv2["errors"]
    for b, sp in new.items():
        if cur2.get(b) != sp:
            problems.append(f"묶음 {b}: 대상에서 {cur2.get(b)} — 계획은 {sp}")
    if problems:
        for p in problems:
            print(f"오류: {p}", file=sys.stderr)
        print(f"확인 실패 — {dst} 는 쓰지 마세요({RECORD} 안 씀).", file=sys.stderr)
        return 1
    data_sha = tree_sha(dst_hash)
    dst_bytes = sum((dst / r).stat().st_size for r in dst_rels)

    rec = dict(
        tool=TOOL, how=how, preset=a.preset, link=a.link,
        created=datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        python=sys.version.split()[0], platform=platform.platform(),
        src=dict(path=str(sr), sha12=src_sha[:12], files=len(src_rels), bytes=src_bytes),
        dst=dict(path=str(dr), sha12_without_record=data_sha[:12], files=len(dst_rels), bytes=dst_bytes),
        sha_method="(cd <폴더> && find . -type f -print0 | LC_ALL=C sort -z | xargs -0 sha256sum) | sha256sum | cut -c1-12"
                   f" — dst 의 sha12 · files · bytes 는 {RECORD} 을 뺀 값(find . -type f ! -path ./{RECORD}), 같은 원본이면 어디서 만들든 같음",
        moves=move_rows,
        rows_source=rows_source,
        counts=dict(before=before, after=after, splits_before=sp_before, splits_after=sp_after),
        index_rewritten=sorted(rewritten), copied_as_is=sorted(inv["other"]),
        warnings=warnings,
    )
    write_json(dst / RECORD, rec)
    full = dict(dst_hash)
    full[RECORD] = file_sha256(dst / RECORD)
    dst_sha = tree_sha(full)
    print(f"완료: {dst}  sha12 {dst_sha[:12]}(전체) · {data_sha[:12]}({RECORD} 뺌) · 파일 {len(full)} · {human(dst_bytes)}")
    if inv["other"]:
        print(f"  그대로 복사한 기타 파일: {', '.join(sorted(inv['other']))} — 내용은 원본 나눔 기준(새 나눔은 {RECORD})")

    # ------------------------------------------------------------ MANIFEST 초안
    today = datetime.date.today()
    rows_txt = " · ".join(f"{sp} {sp_after[sp]['rows']:,}" for sp in SPLITS) + "장" \
        if all(sp_after[sp]["rows"] is not None for sp in SPLITS) else \
        " · ".join(f"{sp} 파일 {sp_after[sp]['files']}" for sp in SPLITS)
    moved_txt = " · ".join(f"{m['bundle']}{'(' + m['label'] + ')' if m['label'] else ''} {m['from']}→{m['to']}"
                           for m in move_rows if m["from"] != m["to"])
    rc_txt = ""
    if all(sp_after[sp]["route_command"] is not None for sp in ("val", "test")):
        rc_txt = ", 경로 지시(직진/좌/우) " + " · ".join(
            f"{sp} " + "/".join(str(sp_after[sp]["route_command"].get(str(c), 0)) for c in (0, 1, 2)) for sp in ("val", "test"))
    note = (f"{preset['note'] + ' — ' if preset else ''}{moved_txt}{', 자동 교사 그대로' if a.preset == 'v1.1' else ''}. "
            f"{rows_txt}(녹화 단위){rc_txt}. run 파일 {len(inv['files'])}개, {RECORD} 뺀 sha {data_sha[:12]}")
    if preset and src_sha[:12] != preset["base_sha12"]:
        note += f", 로컬 원본 sha {src_sha[:12]}"
    origin = (f"{preset['base']}({preset['base_sha12']}) → {how}" if preset else f"원본 {src.name}({src_sha[:12]}) → {how}")
    name = preset["name"] if preset else "xm_parking_r3"
    version = preset["version"] if preset else "v?.?"
    path = preset["path"] if preset else f"data/teacher/{dst.name}/"
    print("\nMANIFEST 초안(한 줄):")
    print(f"| {today.month}.{today.day} | `dataset` | `{name}` | {version} | `{path}` | {human(dst_bytes)} | {dst_sha[:12]} "
          f"| `{origin}` | E | `{note}` |")
    print(f"  · sha12 는 {RECORD} 까지 포함한 대상 폴더 전체, LC_ALL=C 순서. 서버에서 확인:")
    print(f"    (cd {path.rstrip('/')} && find . -type f -print0 | LC_ALL=C sort -z | xargs -0 sha256sum) | sha256sum | cut -c1-12")
    print(f"  · {RECORD} 에 시각 · 경로가 들어가서 전체 sha 는 실행마다 다름 — 같은 원본에서 다시 만들었는지는 {RECORD} 뺀 sha({data_sha[:12]})로 비교")
    if preset and src_sha[:12] != preset["base_sha12"]:
        print(f"  · 로컬 원본 sha {src_sha[:12]} ≠ MANIFEST {preset['base']} {preset['base_sha12']}(server207 에서 계산). "
              "zip 에 빠지거나 더해진 파일(README.md · export.log 등), 또는 서버 sort 로캘(LC_ALL=C 아님) 차이일 수 있음")
    print(f"  · 크기는 파일 크기 합(du -sh 와 조금 다를 수 있음){', 하드 링크라 실제 디스크는 index · 기타 파일만' if a.link == 'hard' else ''}")

    if not a.check:
        return 0
    checker = Path(__file__).resolve().parent / "check_xm_dataset.py"
    missing = [m for m in ("numpy", "pandas", "pyarrow", "PIL") if importlib.util.find_spec(m) is None]
    if missing:
        print(f"\n--check 못 돌림 — 이 python({sys.executable})에 {missing} 없음. 대상 폴더는 완성됨.", file=sys.stderr)
        return 1
    print(f"\n점검: {sys.executable} {checker} {dst}", flush=True)
    rc = subprocess.run([sys.executable, str(checker), str(dst)]).returncode
    print(f"점검 종료 코드 {rc}")
    return 0 if rc == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
