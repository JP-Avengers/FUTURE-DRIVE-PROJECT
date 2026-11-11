"""map_v0.5f 조립: map_v0.5e_island 전체(하드링크) + 둘레 실험층(nr3 비강체 배치 손 촬영 덧붙임, 섬 제외) + 지역별 정확도 그림.
사용: build_v05f.py <island_pkg> <perimeter_added.ply> <out_pkg> <readme.md> <changes_head.md>"""
import sys, os, shutil, hashlib, numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy import ndimage
S, ADD, O, RD, CH = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3]), Path(sys.argv[4]), Path(sys.argv[5])
if O.exists(): raise SystemExit(f'이미 있음: {O}')
O.mkdir(parents=True)
for p in sorted(S.iterdir()):
    if p.name in ('README.md', 'CHANGES.md', 'checksums.sha256'): continue
    if p.is_dir(): shutil.copytree(p, O / p.name, copy_function=os.link)
    else: os.link(p, O / p.name)
F = np.load(S / 'floor_height.npz'); lo, c, conf = F['lo'], float(F['cell']), F['conf']
isl = ndimage.binary_dilation(conf == 4, iterations=5)                       # 섬(정식판에 이미 있음) + 0.5m
m = o3d.io.read_triangle_mesh(ADD); V = np.asarray(m.vertices); T = np.asarray(m.triangles)
ix = np.clip(((V[:, 0] - lo[0]) / c).astype(int), 0, conf.shape[0] - 1); iy = np.clip(((V[:, 1] - lo[1]) / c).astype(int), 0, conf.shape[1] - 1)
ib = (V[:, 0] > -38) & (V[:, 0] < -7) & (V[:, 1] > -17) & (V[:, 1] < -2)      # 가운데 섬 구역(정식판 담당) 통째로 제외
keep = ~(isl[ix, iy] | ib)[T].any(1); m.triangles = o3d.utility.Vector3iVector(T[keep]); m.remove_unreferenced_vertices(); m.compute_vertex_normals()
o3d.io.write_triangle_mesh(str(O / 'hand_perimeter_nr3_EXPERIMENTAL.ply'), m)
V = np.asarray(m.vertices); x, y = V[:, 0], V[:, 1]
reg = np.select([(x >= -58) & (x <= -24) & (y <= -12), (x > -24) & (x < 0) & (y < -18), (x < -5) & (y > -12)], [0, 1, 2], 3)
names = ['남쪽·남서 0.2~0.4m(검증)', 'IT1 입구 1~2m', '북쪽·북서 1~3m', '북동·동쪽 미검증']
cols = np.array([[0, 200, 0], [0, 200, 255], [0, 140, 255], [0, 0, 255]], np.uint8)       # BGR
Q = np.load(S / 'outer/ground_ortho.npz'); o = np.ascontiguousarray(np.transpose(Q['rgb'], (1, 0, 2))[::-1][:, :, ::-1]); qlo = Q['lo']; H = o.shape[0]
img = (o * 0.55).astype(np.uint8); u = ((x - qlo[0]) / 0.1).astype(int); v = H - 1 - ((y - qlo[1]) / 0.1).astype(int); ok = (u >= 0) & (u < o.shape[1]) & (v >= 0) & (v < H)
img[v[ok], u[ok]] = cols[reg[ok]]
ys, xs = np.where(img.sum(2) > 0); img = img[max(ys.min() - 10, 0):ys.max() + 10, max(xs.min() - 10, 0):xs.max() + 10]
img = cv2.resize(img, None, fx=2, fy=2, interpolation=cv2.INTER_NEAREST)
from PIL import Image, ImageDraw, ImageFont
im = Image.fromarray(img[:, :, ::-1]); d = ImageDraw.Draw(im); f = ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf', 22)
for k, n in enumerate(names): d.rectangle([12, 12 + k * 32, 34, 34 + k * 32], fill=tuple(int(t) for t in cols[k][::-1])); d.text((42, 10 + k * 32), n, font=f, fill=(255, 255, 255))
im.save(O / 'qa/v05e/hand_perimeter_accuracy.png')
print('둘레 실험층: 삼각형', len(m.triangles), '| 지역별 꼭짓점 비율', {n: f'{np.mean(reg == k):.0%}' for k, n in enumerate(names)})
shutil.copy2(RD, O / 'README.md')
(O / 'CHANGES.md').write_text(CH.read_text(encoding='utf-8') + '\n' + (S / 'CHANGES.md').read_text(encoding='utf-8').split('\n', 3)[-1], encoding='utf-8')
def sha(p):
    h = hashlib.sha256()
    with open(p, 'rb') as fh:
        for b in iter(lambda: fh.read(1 << 20), b''): h.update(b)
    return h.hexdigest()
L = [f'{sha(p)}  {p.relative_to(O).as_posix()}' for p in sorted(O.rglob('*')) if p.is_file() and p.name != 'checksums.sha256']
(O / 'checksums.sha256').write_text('\n'.join(L) + '\n', encoding='utf-8'); print('파일', len(L), '→', O)
