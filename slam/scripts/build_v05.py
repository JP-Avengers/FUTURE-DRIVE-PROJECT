"""map_v0.5 조립: v0.4b 패키지의 키프레임·궤적·현장 자료는 그대로(하드링크), 형상·점군을 v0.5 정리본으로, 바닥 높이 지도·지도 밖 바닥·점검 자료 추가.
점군 정리: 바닥 밑 점(메시와 같은 기준), 차로(본네트 궤적 2m·D 기준 경로 2.5m 안)의 바닥 위 25cm 이하 점과 정리된 메시에서 8cm 넘게 떨어진 점, 외딴 점(0.2m 안 이웃 3개 미만).
사용: build_v05.py <v04b_pkg> <v05build_dir> <out_pkg> <route_csv>"""
import sys, os, shutil, json
import numpy as np, open3d as o3d
from pathlib import Path
from scipy import ndimage
from scipy.spatial import cKDTree
S, W, O, RC = map(Path, sys.argv[1:5])
if O.exists(): raise SystemExit(f'이미 있음: {O}')
O.mkdir(parents=True); (O / 'qa').mkdir()
for p in sorted(S.iterdir()):
    if p.name.startswith(('keyframes_', 'trajectory_')) or p.name in ('example_tsdf.py', 'site_measurements.md', 'site_photos'):
        if p.is_dir(): shutil.copytree(p, O / p.name, copy_function=os.link)
        else: os.link(p, O / p.name)
shutil.copy2(W / 'geometry_v05.ply', O / 'geometry_clean_5cm.ply')
for f in ('objects_only.ply', 'ground_extended_0.5m.ply', 'floor_height.npz', 'floor_height.png', 'floor_height.json'): shutil.copy2(W / f, O / f)
for f in ('removed_noise.ply', 'removed_below.ply', 'clean_report.json'): shutil.copy2(W / f, O / 'qa' / f)
for f in ('floor_map.png', 'route_profile.png', 'summary.json'): shutil.copy2(W / 'check' / f, O / 'qa' / f'route_check_{f}')

F = np.load(W / 'floor_height.npz'); G, lo, cell, conf = F['grid'].astype(float), F['lo'], float(F['cell']), F['conf']
def fh(xy): return ndimage.map_coordinates(G, [(xy[:, 0] - lo[0]) / cell - 0.5, (xy[:, 1] - lo[1]) / cell - 0.5], order=1, mode='nearest')
def fc(xy):
    i = np.clip(((xy[:, 0] - lo[0]) / cell).astype(int), 0, G.shape[0] - 1); j = np.clip(((xy[:, 1] - lo[1]) / cell).astype(int), 0, G.shape[1] - 1); return conf[i, j]
pc = o3d.io.read_point_cloud(str(S / 'cloud_3cm.ply')); X = np.asarray(pc.points); n0 = len(X)
h = X[:, 2] - fh(X[:, :2]); c = fc(X[:, :2])
below = h < np.select([c >= 2, c == 1], [-0.05, -0.15], -0.80)
T = np.vstack([np.loadtxt(S / f'trajectory_bonnet_{d}_camera_link.txt', comments='#', ndmin=2)[:, 1:3] for d in ('cw', 'ccw')])
R = np.loadtxt(RC, delimiter=',', skiprows=1)[:, 2:4]
lane = (cKDTree(T).query(X[:, :2], distance_upper_bound=2.0)[0] <= 2.0) | (cKDTree(R).query(X[:, :2], distance_upper_bound=2.5)[0] <= 2.5)
Vk = np.asarray(o3d.io.read_triangle_mesh(str(W / 'objects_only.ply')).vertices)
dk = cKDTree(Vk).query(X, distance_upper_bound=0.08)[0]
plate = lane & (h > 0.06) & (h <= 0.25); unsup = lane & (h > 0.06) & (h < 2.5) & ~np.isfinite(dk)
keep = ~(below | plate | unsup)
nb = cKDTree(X[keep]).query_ball_point(X[keep], 0.2, return_length=True); iso = np.zeros(n0, bool); iso[np.flatnonzero(keep)[nb < 4]] = True; keep &= ~iso
out = pc.select_by_index(np.flatnonzero(keep)); o3d.io.write_point_cloud(str(O / 'cloud_3cm.ply'), out)
rep = dict(cloud_in=n0, cloud_out=int(keep.sum()), below=int(below.sum()), lane_plate=int((plate & ~below).sum()), lane_unsupported=int((unsup & ~plate & ~below).sum()), isolated=int(iso.sum()))
json.dump(rep, open(O / 'qa' / 'cloud_report.json', 'w'), indent=1); print('점군', json.dumps(rep, ensure_ascii=False))
print('→', O)
