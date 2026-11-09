"""주차 차·나무·기둥 위치 목록 (에셋으로 바꿔 넣기용). 정리된 물체 메시(objects_only.ply) 꼭짓점을 바닥 높이 지도 기준 높이로 나눠 위에서 본 덩어리로 묶음.
  차: 바닥 위 0.3~2.2m 점, 0.2m 칸 덩어리 → 최소 넓이 사각형. 높이 1.1~2.1m, 길이 3.5~6m, 폭 1.5~2.6m.
      옆으로 붙어 선 여러 대는 긴 변을 2.5m(주차칸 폭) 단위로 나눔.
  나무: 바닥 위 2.5m 넘는 점의 0.5m 칸 최고 높이 지도에서 3.5m 창 안의 꼭대기(3m 넘음) 하나 = 한 그루. 수관 지름 = 이웃 꼭대기 거리(3~8m로 제한).
  기둥(가로등 등): 바닥 위 2.5m 넘는 덩어리 중 넓이 1㎡ 미만.
출력: objects.csv (type,id,x,y,z_floor,yaw_deg,length_m,width_m,height_m,note), objects.png
좌표는 B 맵 좌표(m). yaw는 긴 변 방향(+x 기준 반시계, 도). 주차 차는 10/5·10/9·10/10 촬영이 섞여 있음.
사용: object_list.py <objects_only.ply> <floor_height.npz> <ortho.npz> <out_dir>"""
import sys, csv, json
import numpy as np, open3d as o3d, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial import cKDTree
M, FL, OR, O = sys.argv[1], sys.argv[2], sys.argv[3], Path(sys.argv[4]); O.mkdir(parents=True, exist_ok=True)
F = np.load(FL); G, lo, cell = F['grid'].astype(float), F['lo'], float(F['cell'])
def fh(xy): return ndimage.map_coordinates(G, [(xy[:, 0] - lo[0]) / cell - 0.5, (xy[:, 1] - lo[1]) / cell - 0.5], order=1, mode='nearest')
V = np.asarray(o3d.io.read_triangle_mesh(M).vertices); h = V[:, 2] - fh(V[:, :2])
x0, y0 = V[:, 0].min() - 1, V[:, 1].min() - 1
def blobs(sel, c, minpts, close=1):
    ix = ((V[sel, 0] - x0) / c).astype(int); iy = ((V[sel, 1] - y0) / c).astype(int); H = np.zeros((ix.max() + 2, iy.max() + 2), np.int32)
    np.add.at(H, (ix, iy), 1); occ = H >= minpts
    if close: occ = ndimage.binary_closing(occ, iterations=close)
    lab, n = ndimage.label(occ, np.ones((3, 3))); return lab, n, ix, iy
rows = []
# 차
sel = (h > 0.3) & (h < 2.2); lab, n, ix, iy = blobs(sel, 0.2, 4); hs = h[sel]; li = lab[ix, iy]
for k in range(1, n + 1):
    cells = np.argwhere(lab == k)
    if len(cells) * 0.04 < 3.0: continue
    hk = hs[li == k]
    if len(hk) < 50: continue
    ht = np.percentile(hk, 97)
    if not (1.1 <= ht <= 2.1): continue
    pts = (cells + 0.5) * 0.2 + [x0, y0]; (cx, cy), (w, l), ang = cv2.minAreaRect(pts.astype(np.float32))
    if w > l: w, l = l, w                                     # OpenCV: 첫 변(width)이 ang 방향 → 긴 변 방향 = ang
    else: ang = ang + 90                                      # 긴 변이 두 번째 변이면 ang + 90
    th = np.radians(ang); ax_l, ax_w = np.array([np.cos(th), np.sin(th)]), np.array([-np.sin(th), np.cos(th)])
    if 3.5 <= l <= 6.2 and 1.5 <= w <= 2.6: cars = [((cx, cy), l, w, ang)]
    elif 3.5 <= w <= 6.2 and l > 2.6:                         # 옆으로 붙은 여러 대: 긴 변 = 대수 × 2.5m, 차 길이 = 짧은 변
        nc = max(2, int(round(l / 2.5))); cars = [((cx, cy) + ax_l * (l * ((j + 0.5) / nc - 0.5)), w, l / nc, ang + 90) for j in range(nc)]
    elif 3.5 <= l <= 6.2 and 2.6 < w <= 3.4: cars = [((cx, cy), l, min(w, 2.2), ang)]
    else: continue
    for (c_, L, Wd, A) in cars:
        rows.append(dict(type='car', x=c_[0], y=c_[1], yaw_deg=A % 180, length_m=L, width_m=min(Wd, 2.2), height_m=ht, note=f'덩어리 {len(cells)*0.04:.1f}㎡' + (f', {len(cars)}대로 나눔' if len(cars) > 1 else '')))
# 기둥: 2.5m 넘는 작은 덩어리. 나무: 2.5m 넘는 점의 0.5m 칸 최고 높이 지도(수관 높이 지도)에서 3.5m 안 꼭대기 = 한 그루
sel = h > 2.5; lab, n, ix, iy = blobs(sel, 0.5, 2); hs = h[sel]; li = lab[ix, iy]
for k in range(1, n + 1):
    cells = np.argwhere(lab == k); A = len(cells) * 0.25; hk = hs[li == k]
    if A < 1.0 and np.percentile(hk, 95) > 3.0:
        cx, cy = ((cells.mean(0) + 0.5) * 0.5 + [x0, y0]); rows.append(dict(type='pole', x=cx, y=cy, yaw_deg=0, length_m=0.3, width_m=0.3, height_m=float(hk.max()), note='가로등·기둥 추정'))
CHM = np.zeros(lab.shape); np.maximum.at(CHM, (ix, iy), hs); big = np.isin(lab, [k for k in range(1, n + 1) if (lab == k).sum() * 0.25 >= 2.0])
CHM[~big] = 0; pk = (CHM == ndimage.maximum_filter(CHM, size=7)) & (CHM > 3.0)
pki = np.argwhere(pk)
if len(pki):
    pxy = (pki + 0.5) * 0.5 + [x0, y0]; dnn = cKDTree(pxy).query(pxy, k=2)[0][:, 1] if len(pxy) > 1 else np.full(len(pxy), 8.0)
    for (i_, j_), (px_, py_), dn in zip(pki, pxy, dnn):
        A = (lab == lab[i_, j_]).sum() * 0.25; rad = float(np.clip(dn / 2, 1.5, 4.0))
        rows.append(dict(type='tree', x=px_, y=py_, yaw_deg=0, length_m=2 * rad, width_m=2 * rad, height_m=float(CHM[i_, j_]),
                         note=f'수관 꼭대기 (지름 = length, 이웃 꼭대기 거리로 추정), 덩어리 {A:.0f}㎡' + (' — 건물 일부일 수 있음' if A > 150 else '')))
for i, r in enumerate(rows): r['id'] = i + 1; r['z_floor'] = float(fh(np.array([[r['x'], r['y']]]))[0])
keys = ['type', 'id', 'x', 'y', 'z_floor', 'yaw_deg', 'length_m', 'width_m', 'height_m', 'note']
with open(O / 'objects.csv', 'w', newline='', encoding='utf-8') as f:
    w_ = csv.DictWriter(f, keys); w_.writeheader()
    for r in rows: w_.writerow({k: (round(r[k], 2) if isinstance(r[k], float) else r[k]) for k in keys})
cnt = {t: sum(r['type'] == t for r in rows) for t in ('car', 'tree', 'pole')}; print('목록', cnt)
# 그림: 정사 영상 위에 표시
Q = np.load(OR); rgb = Q['rgb'][..., ::-1].copy(); ql, qc = Q['lo'], float(Q['cell']); img = np.transpose(rgb, (1, 0, 2))[::-1].copy()
img[(img == 0).all(2)] = 255
def px(x, y): return int((x - ql[0]) / qc), img.shape[0] - 1 - int((y - ql[1]) / qc)
col = {'car': (0, 0, 255), 'tree': (0, 160, 0), 'pole': (255, 0, 255), 'tall_other': (255, 128, 0)}
for r in rows:
    if r['type'] == 'car':
        box = cv2.boxPoints(((r['x'], r['y']), (r['length_m'], r['width_m']), r['yaw_deg'])); cv2.polylines(img, [np.array([px(*p) for p in box], np.int32)], True, col['car'], 2)
    else:
        cv2.circle(img, px(r['x'], r['y']), max(3, int(r['length_m'] / 2 / qc)), col[r['type']], 2)
    cv2.putText(img, str(r['id']), px(r['x'], r['y']), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)
ys, xs = np.where((img < 250).any(2)); img = img[ys.min():ys.max(), xs.min():xs.max()]
cv2.imwrite(str(O / 'objects.png'), img); json.dump(cnt, open(O / 'objects.json', 'w'))
