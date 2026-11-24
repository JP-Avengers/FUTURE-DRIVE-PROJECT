"""화면용 GLB (<out>/preview.glb) — 엔진이 마지막에 줄여서 써 두고, 서버는 그 파일을 주기만 한다 (D 11.24).

    cd recon
    python -m scan2sim.preview <…_mesh.usdc> <preview.glb> [--max-tris 300000] [--max-tex 2048]

USD 의 보이는 prim 만 옮긴다 (충돌 · 경계벽 · 안 보이는 것은 뺌). 사진 텍스처가 묶인 prim 은 st 와 텍스처째, 나머지는 정점 색으로.
좌표: 화면은 Y 가 위 — 화면 (x, y, z) = 씬 (x, z, −y). 서버가 하던 변환과 같다 (docs/CONTRACT.md 1-1).

줄이는 방법 — 같은 입력이면 같은 파일이 나온다 (무작위를 쓰지 않음)
- 정점 색 prim: 삼각형 수의 합이 max_tris 를 넘으면, 넘는 prim 의 정점을 격자 칸으로 모아 줄인다 (칸을 3 cm 부터 1.25배씩 키움).
  quadric 은 형상이 더 좋지만 방 하나(220만 삼각형)에 수십 초라 화면용에는 쓰지 않음.
- 텍스처 prim: 형상은 그대로 두고(uv 를 지켜야 함) 텍스처만 긴 변 max_tex 화소 안으로 줄여 JPEG 로 넣는다.
"""
import argparse
import io
import os
import time

import numpy as np


def _cluster(v, f, col, cell):
    """정점을 cell 칸으로 모음 (칸 안 정점의 평균 자리 · 평균 색). 찌그러진 · 겹친 삼각형은 뺌. → (v, f, col)"""
    key = np.floor(v / cell).astype(np.int64)
    key -= key.min(0)
    dim = key.max(0) + 1
    flat = (key[:, 0] * dim[1] + key[:, 1]) * dim[2] + key[:, 2]
    _, inv, cnt = np.unique(flat, return_inverse=True, return_counts=True)
    inv = inv.ravel()
    nv = np.zeros((len(cnt), 3))
    np.add.at(nv, inv, v)
    nv /= cnt[:, None]
    nc = None
    if col is not None:
        nc = np.zeros((len(cnt), 3))
        np.add.at(nc, inv, col)
        nc /= cnt[:, None]
    nf = inv[f]
    nf = nf[(nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])]
    if len(nf):
        nf = nf[np.unique(np.sort(nf, axis=1), axis=0, return_index=True)[1]]
        nf = nf[np.lexsort((nf[:, 2], nf[:, 1], nf[:, 0]))]      # 순서를 정해 둠 — 같은 입력이면 같은 파일
    return nv, nf, nc


def write(usd_mesh, dst, max_tris=300_000, max_tex=2048):
    """USD 메쉬 파일 → 화면용 GLB. → dict(prims, triangles, triangles_in, mb, seconds, cell_m)"""
    import trimesh
    from PIL import Image
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade
    t0 = time.time()
    st = Usd.Stage.Open(usd_mesh)
    parts = []                                                  # (이름, v, f, 정점 색 또는 None, uv 또는 None, 텍스처 경로 또는 None)
    for p in st.Traverse():
        if not p.IsA(UsdGeom.Mesh) or p.HasAPI(UsdPhysics.CollisionAPI) or UsdGeom.Imageable(p).ComputeVisibility() == "invisible":
            continue
        m = UsdGeom.Mesh(p)
        v = np.array(m.GetPointsAttr().Get(), float)
        f = np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
        if len(f) == 0:
            continue
        pv, tex = UsdGeom.PrimvarsAPI(p), None
        mat = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial()[0]
        if mat:
            for c in Usd.PrimRange(mat.GetPrim()):
                inp = UsdShade.Shader(c).GetInput("file") if c.IsA(UsdShade.Shader) else None
                if inp and inp.Get():
                    tp = os.path.normpath(os.path.join(os.path.dirname(usd_mesh), inp.Get().path))
                    tex = tp if os.path.exists(tp) else None
        uv, dc = pv.GetPrimvar("st"), pv.GetPrimvar("displayColor")
        if tex and uv and uv.GetInterpolation() == "vertex":
            parts.append((p.GetName(), v, f, None, np.array(uv.Get(), float), tex))
        else:
            col = np.clip(np.array(dc.Get(), float), 0, 1) if dc and dc.GetInterpolation() == "vertex" and len(dc.Get() or []) == len(v) else None
            parts.append((p.GetName(), v, f, col, None, None))    # displayColor 는 선형 값 — glTF 정점 색도 선형
    n_in = sum(len(x[2]) for x in parts)
    plain = [k for k, x in enumerate(parts) if x[5] is None]
    budget = max_tris - sum(len(x[2]) for x in parts if x[5] is not None)   # 텍스처 prim 은 그대로 두므로 그만큼 뺌
    cell = None
    if plain and sum(len(parts[k][2]) for k in plain) > max(budget, 0):
        total = sum(len(parts[k][2]) for k in plain)
        cell = 0.03
        for _ in range(16):
            new = {}
            for k in plain:
                nm, v, f, col, _, _ = parts[k]
                share = max(budget, 20_000) * len(f) / total      # prim 마다 원래 크기 비율대로
                new[k] = _cluster(v, f, col, cell) if len(f) > share else (v, f, col)
            if sum(len(x[1]) for x in new.values()) <= max(budget, 20_000):
                break
            cell *= 1.25
        for k, (v, f, col) in new.items():
            parts[k] = (parts[k][0], v, f, col, None, None)
    sc = trimesh.Scene()
    for nm, v, f, col, uv, tex in parts:
        if len(f) == 0:
            continue
        if tex:
            img = Image.open(tex)
            if max(img.size) > max_tex:
                s = max_tex / max(img.size)
                buf = io.BytesIO()
                img.convert("RGB").resize((max(1, round(img.size[0] * s)), max(1, round(img.size[1] * s))), Image.LANCZOS).save(buf, "JPEG", quality=85)
                buf.seek(0)
                img = Image.open(buf)                             # JPEG 형식을 지닌 채로 — 그냥 넘기면 PNG 로 들어가 커짐
            vis = trimesh.visual.TextureVisuals(uv=uv, material=trimesh.visual.material.PBRMaterial(baseColorTexture=img, metallicFactor=0.0, roughnessFactor=1.0))
        elif col is not None:
            vis = trimesh.visual.ColorVisuals(vertex_colors=(col * 255).astype(np.uint8))
        else:
            vis = None
        sc.add_geometry(trimesh.Trimesh(np.c_[v[:, 0], v[:, 2], -v[:, 1]], f, visual=vis, process=False), geom_name=nm)
    tmp = dst + ".tmp"
    open(tmp, "wb").write(sc.export(file_type="glb"))
    os.replace(tmp, dst)
    return dict(prims=len(parts), triangles=int(sum(len(x[2]) for x in parts)), triangles_in=int(n_in), mb=round(os.path.getsize(dst) / 1e6, 1),
                seconds=round(time.time() - t0, 1), cell_m=round(cell, 3) if cell else None)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("usd_mesh")
    ap.add_argument("dst")
    ap.add_argument("--max-tris", type=int, default=300_000)
    ap.add_argument("--max-tex", type=int, default=2048)
    a = ap.parse_args()
    print(write(a.usd_mesh, a.dst, a.max_tris, a.max_tex))


if __name__ == "__main__":
    main()
