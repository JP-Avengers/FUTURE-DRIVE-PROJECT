"""보정 단계가 바닥 구멍 판만 더하는지 (통합 기록 R16 · C 29a05ba, C_TRACK C14) — 바닥 구멍만 넣은 가상의 방 GLB 로 엔진을 돌림 (서버 없이, 약 30초).

    cd recon
    python -m scan2sim.tests.patch_only [--room <room.glb 가 든 폴더>] [--work <폴더>] [--json 결과.json]

시험 자료: samples/make_room.py --seed 0 --defects no_visible_floor (보이는 바닥 구멍 1 m² 하나만 — 두 겹 바닥이 없어야 판만 더하는 길을 탐).
  --room 폴더에 그 방이 있으면 그대로 씀. 없거나 만들다 멈춘 것(room_truth.json 이 없거나 깨짐)이면 다시 만듦.
  다른 결함이 든 방이면 멈춤(종료 코드 2) — 기본 방(결함 셋)은 두 겹 바닥이 있어 보정이 늘 처음부터 다시 만들므로 이 길을 안 탐
--work: 엔진 결과 · 다시 만든 빌드를 둘 폴더 (안 주면 임시 폴더를 만들고 끝나면 지움)
검사
  (a) 엔진이 끝까지 감 (종료 코드 0) · 보정 전 진단에 no_visible_floor 가 있고 double_floor 는 없음
  (b) 보정 단계: 실패 아님 · patch_only 참 · fixed 에 no_visible_floor · 판 칸 > 0 · 보정 뒤 진단의 결함 목록에 no_visible_floor 없음 ·
      "판만 기존 USD 에 더함" 메시지
  (c) 빌드: report.json 의 floor_patch.patch_only 참 · USD 검증 문제 없음 · prim 에 FloorPatch · build/floor_patch.ply 있음
  (d) 같은 보정 설정을 처음부터 다시 만든 빌드 (pipeline/glb_to_usd.py --config <보정 설정> --out <다른 폴더>) 와 견줌 —
      판만 더한 결과가 전체를 다시 만든 것과 같아야 함 (C: "prim 순서만 다름"):
      메쉬 USD 의 Mesh prim 이 경로까지 같고, prim 마다 쓴 속성(points · faceVertexIndices · normals · displayColor · doubleSided …)의
      이름과 값이 같음 · floor_patch.ply 의 정점 · 삼각형 · 색이 같음.
      (판 칸 수 · report.json 의 충돌 물체 수는 견주지 않음 — 판만 더할 때는 같은 마스크를 읽고 충돌을 다시 재지 않아 구조상 늘 같음)
종료 코드: 0 = 모두 통과 · 1 = 하나라도 실패 · 2 = 시험 자료를 만들지 못함
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

from scan2sim.tests.e2e import Checks

RECON = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SAMPLES = os.path.join(RECON, "scan2sim", "samples")
PIPE = os.path.join(RECON, "pipeline")
ENV = dict(os.environ, PYTHONIOENCODING="utf-8")


def _json(p):
    try:
        return json.load(open(p, encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _ply(path):
    import open3d as o3d
    m = o3d.io.read_triangle_mesh(path)
    return np.asarray(m.vertices), np.asarray(m.triangles), np.asarray(m.vertex_colors)


def _usd_meshes(path):
    """메쉬 USD → {prim 경로: {속성 이름: 값}} — Mesh prim 마다 쓴(authored) 속성 모두. 값은 배열이면 numpy"""
    from pxr import Usd, UsdGeom
    out = {}
    st = Usd.Stage.Open(path)
    for p in st.Traverse():
        if p.IsA(UsdGeom.Mesh):
            vals = {}
            for a in p.GetAuthoredAttributes():
                v = a.Get()
                try:
                    vals[a.GetName()] = np.asarray(v) if hasattr(v, "__len__") and not isinstance(v, str) else v
                except (TypeError, ValueError):
                    vals[a.GetName()] = v
            out[str(p.GetPath())] = vals
    return out


def _same(a, b):
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        a, b = np.asarray(a), np.asarray(b)
        return a.shape == b.shape and np.array_equal(a, b)
    return a == b


def room(d):
    """d 에 바닥 구멍만 넣은 방 — 있으면 그대로, 없거나 만들다 멈춘 것이면 다시 만듦. → (room.glb 경로 또는 None, 까닭)"""
    glb, tp = os.path.join(d, "room.glb"), os.path.join(d, "room_truth.json")
    t = _json(tp)
    if os.path.isfile(glb) and t is not None and t.get("defect_types") != ["no_visible_floor"]:
        return None, f"{d} 의 방에는 다른 결함이 있음 (defect_types {t.get('defect_types')}) — 그 폴더를 지우거나 다른 --room 을 줄 것"
    if not os.path.isfile(glb) or t is None:                   # 없음 · make_room 이 room.glb 만 쓰고 멈춘 것
        for f in (glb, tp):
            if os.path.exists(f):
                os.remove(f)
        r = subprocess.run([sys.executable, os.path.join(SAMPLES, "make_room.py"), "--out", d, "--seed", "0", "--defects", "no_visible_floor"], cwd=RECON)
        t = _json(tp)
        if r.returncode or not os.path.isfile(glb) or not t or t.get("defect_types") != ["no_visible_floor"]:
            return None, f"make_room.py 로 바닥 구멍만 넣은 방을 만들지 못함 (종료 코드 {r.returncode})"
    return glb, ""


def check(work, room_dir, C):
    """검사를 C 에 쌓음. 시험 자료를 못 만들면 2, 아니면 None"""
    glb, why = room(room_dir)
    if not glb:
        print(f"시험 자료를 만들지 못함: {why}")
        return 2
    out = os.path.join(work, "run")
    shutil.rmtree(out, ignore_errors=True)
    r = subprocess.run([sys.executable, "-m", "scan2sim", glb, "--out", out], cwd=RECON, capture_output=True, text=True, env=ENV)
    # (a) 엔진 · 보정 전 진단
    C.add("엔진이 끝까지 감 (종료 코드 0)", r.returncode == 0, f"종료 코드 {r.returncode}" + ("" if r.returncode == 0 else f" · {(r.stdout + r.stderr)[-400:]}"))
    cor = _json(os.path.join(out, "stages", "04_correct.json")) or {}
    cm = cor.get("metrics") or {}
    before = (cm.get("before") or (_json(os.path.join(out, "stages", "03_diagnose.json")) or {}).get("metrics") or {}).get("defects") or {}
    C.add("보정 전 진단에 no_visible_floor 있음 · double_floor 없음 (판만 더하는 길의 조건)",
          before.get("no_visible_floor", 0) >= 1 and not before.get("double_floor"), f"{before}")
    # (b) 보정 단계
    aft = cm.get("after") or {}
    patch = cm.get("floor_patch") or {}
    C.add("보정 단계가 실패가 아님", cor.get("status") in ("pass", "warn"), f"{cor.get('status')} · {cor.get('summary')}")
    C.add("보정 단계 수치 patch_only 참", cm.get("patch_only") is True, f"{cm.get('patch_only')}")
    C.add("fixed 에 no_visible_floor · 판 칸 > 0", "no_visible_floor" in (cm.get("fixed") or []) and (patch.get("cells") or 0) > 0,
          f"fixed {cm.get('fixed')} · 판 {patch.get('cells')}칸 {patch.get('area_m2')} m²")
    C.add("보정 뒤 진단에 결함 목록이 있고 no_visible_floor 는 없음", isinstance(aft.get("defects"), dict) and not aft["defects"].get("no_visible_floor"),
          f"after 의 defects {aft.get('defects')}")
    C.add("보정 단계 메시지에 '판만 기존 USD 에 더함'", any("판만 기존 USD 에 더함" in m for m in cor.get("messages") or []), "")
    # (c) 빌드
    build = os.path.join(out, "build")
    rep = _json(os.path.join(build, "report.json")) or {}
    usd = rep.get("usd") or {}
    prims = usd.get("prims") or {}
    C.add("report.json 의 floor_patch.patch_only 참", (rep.get("floor_patch") or {}).get("patch_only") is True, f"{rep.get('floor_patch')}")
    C.add("USD 검증 문제 없음 · prim 에 FloorPatch", not usd.get("problems") and any(k.endswith("/FloorPatch") and v > 0 for k, v in prims.items()),
          f"problems {usd.get('problems')} · prims {prims}")
    pp = os.path.join(build, "floor_patch.ply")
    C.add("build/floor_patch.ply 있음", os.path.isfile(pp), pp)
    # (d) 같은 보정 설정으로 처음부터 다시 만든 빌드와 견줌
    cfgs = sorted(glob.glob(os.path.join(build, "*_corrected.yaml")))
    full = os.path.join(work, "full_rebuild")
    shutil.rmtree(full, ignore_errors=True)
    if not cfgs:
        C.add("보정 설정(*_corrected.yaml)이 빌드 폴더에 있음", False, build)
        return None
    rf = subprocess.run([sys.executable, os.path.join(PIPE, "glb_to_usd.py"), "--config", cfgs[0], "--out", full], cwd=RECON, capture_output=True, text=True, env=ENV)
    rep2 = _json(os.path.join(full, "report.json")) or {}
    usd2 = rep2.get("usd") or {}
    C.add("같은 보정 설정을 처음부터 다시 만듦 (glb_to_usd.py, 종료 코드 0 · 판만 더한 빌드가 아님)",
          rf.returncode == 0 and bool(rep2) and not (rep2.get("floor_patch") or {}).get("patch_only"),
          f"종료 코드 {rf.returncode}" + ("" if rf.returncode == 0 else f" · {(rf.stdout + rf.stderr)[-400:]}"))
    m1p, m2p = usd.get("mesh"), usd2.get("mesh")
    if not (m1p and m2p and os.path.isfile(m1p) and os.path.isfile(m2p)):
        C.add("두 빌드의 메쉬 USD 가 있음", False, f"{m1p} · {m2p}")
    else:
        A, B = _usd_meshes(m1p), _usd_meshes(m2p)
        C.add("Mesh prim 이 경로까지 같음 (순서는 안 봄)", sorted(A) == sorted(B), f"판만 {sorted(A)} · 처음부터 {sorted(B)}")
        diff = []
        for k in sorted(set(A) & set(B)):
            if set(A[k]) != set(B[k]):
                diff.append(f"{k} 속성 이름 {sorted(set(A[k]) ^ set(B[k]))}")
            diff += [f"{k}.{n}" for n in sorted(set(A[k]) & set(B[k])) if not _same(A[k][n], B[k][n])]
        fp = next((k for k in A if k.endswith("/FloorPatch")), None)
        C.add("prim 마다 쓴 속성의 이름 · 값이 같음 (FloorPatch 의 점 · 면 · 법선 · 색 · 양면 포함)", not diff and fp is not None,
              (f"다름: {diff[:6]}" if diff else f"prim {len(A)}개 · FloorPatch 속성 {sorted(A[fp]) if fp else '없음'}"))
    pp2 = os.path.join(full, "floor_patch.ply")
    if os.path.isfile(pp) and os.path.isfile(pp2):
        (v1, t1, k1), (v2, t2, k2) = _ply(pp), _ply(pp2)
        same = v1.shape == v2.shape and t1.shape == t2.shape and k1.shape == k2.shape
        dv = float(np.abs(v1 - v2).max()) if same and len(v1) else None
        dk = float(np.abs(k1 - k2).max()) if same and len(k1) else None
        C.add("floor_patch.ply 의 정점 · 삼각형 · 색이 같음", same and len(t1) > 0 and np.array_equal(t1, t2) and dv == 0.0 and dk == 0.0,
              f"정점 {len(v1)} · {len(v2)} · 삼각형 {len(t1)} · {len(t2)} · 정점 차 최대 {dv} · 색 차 최대 {dk}")
    else:
        C.add("floor_patch.ply 가 두 빌드에 다 있음", False, f"{os.path.isfile(pp)} · {os.path.isfile(pp2)}")
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--room", help="바닥 구멍만 넣은 방이 든 (또는 만들) 폴더 (안 주면 --work 아래 room_hole)")
    ap.add_argument("--work", help="결과 폴더 (안 주면 임시 폴더, 끝나면 지움)")
    ap.add_argument("--json", help="검사 결과를 쓸 파일 (실패 · 시험 자료 없음에도 씀)")
    a = ap.parse_args()
    tmp = None if a.work else tempfile.mkdtemp(prefix="s2s_patch_")
    work = os.path.abspath(os.path.expanduser(a.work or tmp))
    os.makedirs(work, exist_ok=True)
    room_dir = os.path.abspath(os.path.expanduser(a.room)) if a.room else os.path.join(work, "room_hole")
    C, rc = Checks(), 1
    try:
        rc = check(work, room_dir, C) or (0 if C.ok else 1)
        print("PATCH_ONLY_OK" if rc == 0 else "PATCH_ONLY_FAIL")
        return rc
    finally:
        if a.json:
            json.dump(dict(ok=rc == 0, exit_code=rc, checks=C.rows, room=room_dir, work=work), open(a.json, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
        if tmp:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
