"""맵 비교 도구(map_compare) 공용 함수: 메시 읽기(PLY · OBJ · GLB/glTF), 광선 장면, 검증 회차 depth 일치.
직접 실행하지 않음. mc_register.py · map_compare.py · mc_rigidval.py · mc_synth.py 가 가져다 씀.
GLB 는 open3d(ASSIMP)가 못 읽는 경우가 있어 numpy 로 직접 읽는다(Draco · meshopt 압축 GLB 는 못 읽음 → Polycam 에서 압축 없이 내보내거나 PLY/OBJ 로)."""
import json, re, struct, base64, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy.spatial.transform import Rotation as Rot

EXPORTS = Path.home() / 'b_slam/exports'


# ---------------------------------------------------------------- 메시 읽기 · 쓰기
def _q2m(q):  # glTF 쿼터니언 xyzw → 3x3
    return Rot.from_quat(q).as_matrix()


def _node_matrix(nd):
    if 'matrix' in nd: return np.array(nd['matrix'], float).reshape(4, 4).T   # glTF 는 열 우선
    M = np.eye(4); S = np.diag(nd.get('scale', [1, 1, 1]))
    M[:3, :3] = _q2m(nd.get('rotation', [0, 0, 0, 1])) @ S; M[:3, 3] = nd.get('translation', [0, 0, 0]); return M


def read_gltf(path):
    """GLB/glTF → (V float64 Nx3, T int64 Mx3, C float Nx3 또는 None). 장면 노드 변환(matrix · TRS)을 적용한 세계 좌표. 삼각형(mode 4)만."""
    path = Path(path); raw = path.read_bytes(); bins = {}
    if raw[:4] == b'glTF':
        _, ln = struct.unpack_from('<II', raw, 4); off = 12; js = None
        while off < ln:
            cl, ct = struct.unpack_from('<II', raw, off); data = raw[off + 8:off + 8 + cl]
            if ct == 0x4E4F534A: js = json.loads(data.decode('utf-8'))
            elif ct == 0x004E4942: bins[0] = data
            off += 8 + cl
    else: js = json.loads(raw.decode('utf-8'))
    bad = {'KHR_draco_mesh_compression', 'EXT_meshopt_compression', 'KHR_mesh_quantization'} & set(js.get('extensionsRequired', []) + js.get('extensionsUsed', []))
    if bad: raise RuntimeError(f'{path.name}: 압축 확장 {sorted(bad)} 은 지원 안 함 — 압축 없이 다시 내보내거나 PLY/OBJ 로')
    for k, b in enumerate(js.get('buffers', [])):
        if 'uri' in b:
            u = b['uri']; bins[k] = base64.b64decode(u.split(',', 1)[1]) if u.startswith('data:') else (path.parent / u).read_bytes()
    CT = {5120: 'i1', 5121: 'u1', 5122: 'i2', 5123: 'u2', 5125: 'u4', 5126: 'f4'}; NC = {'SCALAR': 1, 'VEC2': 2, 'VEC3': 3, 'VEC4': 4}

    def acc(i):
        a = js['accessors'][i]; bv = js['bufferViews'][a['bufferView']]; dt = np.dtype('<' + CT[a['componentType']]); n = NC[a['type']]
        st = bv.get('byteStride') or dt.itemsize * n; buf = bins[bv.get('buffer', 0)]
        x = np.ndarray((a['count'], n), dt, buffer=buf, offset=bv.get('byteOffset', 0) + a.get('byteOffset', 0), strides=(st, dt.itemsize)).copy()
        if a.get('normalized') and dt.kind in 'iu': x = x.astype(np.float64) / np.iinfo(dt).max
        return x
    Vs, Ts, Cs, nv = [], [], [], 0

    def walk(ni, M):
        nonlocal nv
        nd = js['nodes'][ni]; M = M @ _node_matrix(nd)
        if 'mesh' in nd:
            for pr in js['meshes'][nd['mesh']]['primitives']:
                if pr.get('mode', 4) != 4: continue
                P = acc(pr['attributes']['POSITION']).astype(np.float64); P = P @ M[:3, :3].T + M[:3, 3]
                I = acc(pr['indices']).reshape(-1, 3).astype(np.int64) if 'indices' in pr else np.arange(len(P)).reshape(-1, 3)
                if np.linalg.det(M[:3, :3]) < 0: I = I[:, ::-1]
                C = acc(pr['attributes']['COLOR_0'])[:, :3].astype(np.float64) if 'COLOR_0' in pr['attributes'] else None
                Vs.append(P); Ts.append(I + nv); Cs.append(C); nv += len(P)
        for c in nd.get('children', []): walk(c, M)
    sc = js.get('scene', 0); roots = js['scenes'][sc]['nodes'] if js.get('scenes') else range(len(js['nodes']))
    for r in roots: walk(r, np.eye(4))
    if not Vs: raise RuntimeError(f'{path.name}: 삼각형 메시가 없음')
    C = np.vstack(Cs) if all(c is not None for c in Cs) else None
    return np.vstack(Vs), np.vstack(Ts), C


def write_glb(path, V, T, node_translation=None):
    """시험용 최소 GLB 쓰기(위치 float32 + 인덱스 uint32, 노드 1개, 이동만)."""
    V = np.asarray(V, np.float32); I = np.asarray(T, np.uint32); bv = V.tobytes(); bi = I.tobytes(); pad = lambda b, c=b'\0': b + c * (-len(b) % 4)
    bv = pad(bv); binc = bv + pad(bi)
    js = {'asset': {'version': '2.0'}, 'scene': 0, 'scenes': [{'nodes': [0]}], 'nodes': [{'mesh': 0}], 'buffers': [{'byteLength': len(binc)}],
          'bufferViews': [{'buffer': 0, 'byteOffset': 0, 'byteLength': V.nbytes, 'target': 34962}, {'buffer': 0, 'byteOffset': len(bv), 'byteLength': I.nbytes, 'target': 34963}],
          'accessors': [{'bufferView': 0, 'componentType': 5126, 'count': len(V), 'type': 'VEC3', 'min': V.min(0).tolist(), 'max': V.max(0).tolist()},
                        {'bufferView': 1, 'componentType': 5125, 'count': I.size, 'type': 'SCALAR'}],
          'meshes': [{'primitives': [{'attributes': {'POSITION': 0}, 'indices': 1, 'mode': 4}]}]}
    if node_translation is not None: js['nodes'][0]['translation'] = [float(x) for x in node_translation]
    jb = pad(json.dumps(js).encode(), b' '); out = struct.pack('<4sII', b'glTF', 2, 12 + 8 + len(jb) + 8 + len(binc)) + struct.pack('<II', len(jb), 0x4E4F534A) + jb + struct.pack('<II', len(binc), 0x004E4942) + binc
    Path(path).write_bytes(out)


def load_mesh(path):
    """→ (V, T, C 또는 None). .ply/.obj/.stl 은 open3d, .glb/.gltf 는 read_gltf."""
    p = str(path)
    if p.lower().endswith(('.glb', '.gltf')):
        try: return read_gltf(p)
        except RuntimeError: raise
        except Exception as e:
            print(f'  [GLB 직접 읽기 실패: {e}] open3d 로 다시 시도')
    m = o3d.io.read_triangle_mesh(p)
    if len(m.triangles) == 0: raise RuntimeError(f'{p}: 삼각형을 읽지 못함')
    C = np.asarray(m.vertex_colors) if m.has_vertex_colors() else None
    return np.asarray(m.vertices).copy(), np.asarray(m.triangles).astype(np.int64), (None if C is None else C.copy())


def save_ply(path, V, T, C=None):
    m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.asarray(V, np.float64)), o3d.utility.Vector3iVector(np.asarray(T, np.int32)))
    if C is not None: m.vertex_colors = o3d.utility.Vector3dVector(np.asarray(C, np.float64))
    m.compute_vertex_normals(); o3d.io.write_triangle_mesh(str(path), m)


def tri_normals(V, T, idx=None):
    t = T if idx is None else T[idx]; n = np.cross(V[t[:, 1]] - V[t[:, 0]], V[t[:, 2]] - V[t[:, 0]]); a = np.linalg.norm(n, axis=1)
    return n / np.maximum(a, 1e-12)[:, None], 0.5 * a


def vertex_normals(V, T):
    n = np.cross(V[T[:, 1]] - V[T[:, 0]], V[T[:, 2]] - V[T[:, 0]]); vn = np.zeros_like(V)
    for k in range(3): np.add.at(vn, T[:, k], n)
    return vn / np.maximum(np.linalg.norm(vn, axis=1), 1e-12)[:, None]


def scene_of(V, T):
    tm = o3d.t.geometry.TriangleMesh(); tm.vertex.positions = o3d.core.Tensor(np.ascontiguousarray(V, np.float32)); tm.triangle.indices = o3d.core.Tensor(np.ascontiguousarray(T, np.int32))
    sc = o3d.t.geometry.RaycastingScene(); sc.add_triangles(tm); return sc


# ---------------------------------------------------------------- 자세 · 키프레임
def load_poses(path):
    """rows: timestamp x y z qx qy qz qw id (카메라 광학 좌표 → 세계)."""
    c = np.loadtxt(path, comments='#', ndmin=2); return c[np.argsort(c[:, 0])]


_KCACHE = {}


def calib(kdir, i):
    key = (str(kdir), i)
    if key not in _KCACHE:
        _KCACHE[key] = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (Path(kdir) / f'kf_calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
    return _KCACHE[key]


def depth_agree(sc, poses, kdir, step=4, zmin=0.4, zmax=3.5):
    """in_eval.py (1) 과 같은 계산을 키프레임별로: 화소 step 간격, 유효 depth zmin~zmax, 메시 렌더 depth 와 차이.
    → list of (id, cam xyz, |차| 배열(메시가 닿은 유효 화소), 채운 비율)"""
    out = []
    for r in poses:
        i = int(r[8]); k = calib(kdir, i); d = cv2.imread(str(Path(kdir) / f'kf_depth/{i}.png'), -1).astype(np.float32) / 1000
        v, u = np.mgrid[2:d.shape[0]:step, 2:d.shape[1]:step]; z = d[v, u]; ok = (z > zmin) & (z < zmax)
        dc = np.stack([(u - k[2]) / k[0], (v - k[5]) / k[4], np.ones(u.shape)], -1).reshape(-1, 3); nrm = np.linalg.norm(dc, axis=1); dw = (dc / nrm[:, None]) @ Rot.from_quat(r[4:8]).as_matrix().T
        t = sc.cast_rays(o3d.core.Tensor(np.c_[np.broadcast_to(r[1:4], dw.shape), dw].astype(np.float32)))['t_hit'].numpy(); zr = (t / nrm).reshape(z.shape); hit = np.isfinite(zr)
        out.append((i, r[1:4].copy(), np.abs(zr - z)[ok & hit], float(np.mean(hit[ok])) if ok.any() else np.nan))
    return out


def agree_summary(res):
    if not res: return dict(n=0, med_cm=np.nan, in5=np.nan, in10=np.nan, fill=np.nan)
    d = np.concatenate([x[2] for x in res]) if res else np.zeros(0)
    if len(d) == 0: return dict(n=len(res), med_cm=np.nan, in5=np.nan, in10=np.nan, fill=float(np.nanmean([x[3] for x in res])))
    return dict(n=len(res), med_cm=float(np.median(d) * 100), in5=float(np.mean(d < 0.05)), in10=float(np.mean(d < 0.10)), fill=float(np.nanmean([x[3] for x in res])))
