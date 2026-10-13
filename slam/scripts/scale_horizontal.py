#!/usr/bin/env python3
"""맵 수평 배율 보정: 월드 좌표 (x, y) × s, z 그대로 (z = 중력 반대 방향).
 - PLY(binary/ascii 상관없이 binary_little_endian만 지원): 꼭짓점 x,y 배율, 법선은 S^-1 적용 후 정규화. 나머지 속성·면·기타 element 보존
 - TUM 궤적(.txt): 2·3열(x, y) 배율, 자세는 그대로
 - 키프레임: 자세(traj_camera_id)로 카메라 좌표의 위쪽 u를 구해 p' = (s·I + (1-s)·u·uᵀ) p 를 정확히 적용하고
   depth·RGB를 같은 내부 파라미터로 다시 투영 (calib yaml 그대로). 단순 depth×s·fy×s 근사는 피치 5° 때문에 30m에서 약 20cm 틀려 쓰지 않음
사용: scale_horizontal.py <src_export_dir> <dst_export_dir> <s>     (exports/<run> 구조: cloud_cloud.ply, traj_*.txt, keyframes/kf_*)
      scale_horizontal.py --ply <src.ply> <dst.ply> <s>
      scale_horizontal.py --traj <src.txt> <dst.txt> <s>"""
import sys, re, shutil
from pathlib import Path
import numpy as np, cv2

PT = {'char': 'i1', 'int8': 'i1', 'uchar': 'u1', 'uint8': 'u1', 'short': 'i2', 'int16': 'i2', 'ushort': 'u2', 'uint16': 'u2',
      'int': 'i4', 'int32': 'i4', 'uint': 'u4', 'uint32': 'u4', 'float': 'f4', 'float32': 'f4', 'double': 'f8', 'float64': 'f8'}


def scale_ply(src, dst, s):
    raw = Path(src).read_bytes(); end = raw.index(b'end_header') + len(b'end_header')
    end = raw.index(b'\n', end) + 1; header = raw[:end].decode('ascii', 'replace'); lines = header.splitlines()
    assert any(l.startswith('format binary_little_endian') for l in lines), f'{src}: binary_little_endian 아님'
    elems = []; cur = None
    for l in lines:
        t = l.split()
        if t and t[0] == 'element': cur = [t[1], int(t[2]), []]; elems.append(cur)
        elif t and t[0] == 'property' and cur is not None: cur[2].append(t[1:])
    assert elems[0][0] == 'vertex', '첫 element가 vertex가 아님'
    _, n, props = elems[0]
    assert all(p[0] != 'list' for p in props)
    dt = np.dtype([(p[1], '<' + PT[p[0]]) for p in props])
    V = np.frombuffer(raw, dt, count=n, offset=end).copy()
    V['x'] *= s; V['y'] *= s
    if 'nx' in V.dtype.names:
        N = np.stack([V['nx'] / s, V['ny'] / s, V['nz']], 1).astype(np.float64)
        N /= np.maximum(np.linalg.norm(N, axis=1, keepdims=True), 1e-12)
        V['nx'], V['ny'], V['nz'] = N[:, 0], N[:, 1], N[:, 2]
    rest = raw[end + n * dt.itemsize:]
    Path(dst).write_bytes(raw[:end] + V.tobytes() + rest)
    return n


def scale_traj(src, dst, s):
    out = []
    for l in Path(src).read_text().splitlines():
        if not l.strip() or l.startswith('#'): out.append(l); continue
        t = l.split(); t[1] = f'{float(t[1]) * s:.6f}'; t[2] = f'{float(t[2]) * s:.6f}'; out.append(' '.join(t))
    Path(dst).write_text('\n'.join(out) + '\n')


def fmt(v):
    return f'{int(v)}.' if v == int(v) and abs(v) < 1e6 else f'{v:.16e}'


def scale_calib(src, dst, s):
    t = Path(src).read_text()
    def fix(name, idx):
        nonlocal t
        m = re.search(name + r':.*?data: \[([^\]]+)\]', t, re.S)
        vals = [float(x) for x in m.group(1).replace('\n', ' ').split(',')]
        vals[idx] *= s
        t = t[:m.start(1)] + ' ' + ', '.join(fmt(v) for v in vals) + ' ' + t[m.end(1):]
    fix('camera_matrix', 4); fix('projection_matrix', 5)
    Path(dst).write_text(t)


def qR(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def calib_K(path):
    t = Path(path).read_text()
    a = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', t, re.S).group(1).replace('\n', ' ').split(',')]
    return a[0], a[4], a[2], a[5]


def warp_keyframe(rgb_src, dep_src, rgb_dst, dep_dst, K, R_wc, s):
    """월드 수평 배율 S=diag(s,s,1)을 카메라 좌표에서 정확히 적용: p' = A p, A = s·I + (1-s)·u·uᵀ (u = 카메라 좌표의 위쪽).
    내부 파라미터는 그대로 두고 depth·RGB를 다시 투영(가까운 점 우선)."""
    fx, fy, cx, cy = K
    d = cv2.imread(str(dep_src), cv2.IMREAD_UNCHANGED).astype(np.float64) / 1000; c = cv2.imread(str(rgb_src))
    H, W = d.shape; u_up = R_wc.T @ np.array([0, 0, 1.0]); A = s * np.eye(3) + (1 - s) * np.outer(u_up, u_up)
    v, u = np.nonzero(d > 0); z = d[v, u]
    P = np.c_[(u - cx) * z / fx, (v - cy) * z / fy, z] @ A.T
    un = np.round(fx * P[:, 0] / P[:, 2] + cx).astype(int); vn = np.round(fy * P[:, 1] / P[:, 2] + cy).astype(int)
    ok = (P[:, 2] > 0) & (un >= 0) & (un < W) & (vn >= 0) & (vn < H)
    order = np.argsort(-P[ok, 2])  # 먼 점부터 써서 가까운 점이 덮어씀
    Z = np.zeros((H, W)); C = c.copy(); hit = np.zeros((H, W), bool)
    vi, ui, zi = vn[ok][order], un[ok][order], P[ok, 2][order]
    Z[vi, ui] = zi; C[vi, ui] = c[v[ok][order], u[ok][order]]; hit[vi, ui] = True
    # 구멍 메우기는 하지 않음: 원본 키프레임 depth에도 정렬(업샘플)로 생긴 규칙적인 빈칸이 있어, 메우면 데이터가 2배로 부풀어 원본과 달라짐
    cv2.imwrite(str(dep_dst), np.where(hit, np.clip(np.round(Z * 1000), 1, 65535), 0).astype(np.uint16))
    cv2.imwrite(str(rgb_dst), C, [cv2.IMWRITE_JPEG_QUALITY, 95])


if __name__ == '__main__':
    a = sys.argv[1:]
    if a[0] == '--ply': print(scale_ply(a[1], a[2], float(a[3])), 'vertices'); sys.exit()
    if a[0] == '--traj': scale_traj(a[1], a[2], float(a[3])); sys.exit()
    S, D, s = Path(a[0]), Path(a[1]), float(a[2])
    if D.exists(): sys.exit(f'이미 존재: {D} (덮어쓰지 않음)')
    D.mkdir(parents=True)
    for f in S.iterdir():
        if f.suffix == '.ply' and f.name == 'cloud_cloud.ply': print(f.name, scale_ply(f, D / f.name, s))
        elif f.suffix == '.txt' and f.name.startswith('traj_'): scale_traj(f, D / f.name, s)
    K = S / 'keyframes'
    for sub in ('kf_rgb', 'kf_depth', 'kf_calib'): (D / 'keyframes' / sub).mkdir(parents=True)
    tr = np.loadtxt(S / 'traj_camera_id_camera_poses.txt', comments='#', ndmin=2)  # 광학 좌표 자세 + id
    pose = {int(r[8]): qR(*r[4:8]) for r in tr}
    n = m = 0
    for f in sorted((K / 'kf_calib').iterdir()):
        shutil.copy2(f, D / 'keyframes/kf_calib' / f.name)  # 내부 파라미터는 그대로
        i = int(f.stem); rgb, dep = K / f'kf_rgb/{i}.jpg', K / f'kf_depth/{i}.png'
        if i in pose and rgb.exists() and dep.exists():
            warp_keyframe(rgb, dep, D / f'keyframes/kf_rgb/{i}.jpg', D / f'keyframes/kf_depth/{i}.png', calib_K(f), pose[i], s); n += 1
        else:  # 그래프에 없는 키프레임: 자세가 없어 쓰이지 않음 → 그대로 복사
            for x in (rgb, dep):
                if x.exists(): shutil.copy2(x, D / 'keyframes' / ('kf_rgb' if x.suffix == '.jpg' else 'kf_depth') / x.name)
            m += 1
    print(f'키프레임 재투영 {n}개, 자세 없어 그대로 복사 {m}개')
    (D / 'SCALE.txt').write_text(f'source={S}\nhorizontal_scale={s}\n')
    print(f'키프레임 {n}개, → {D}')
