"""바닥(회색) + 주차선(검정) + 궤적(초록) 탑뷰, 5m 격자 + 10m마다 좌표 라벨. topgrid.py <run> <out.png>"""
import sys, numpy as np, open3d as o3d, cv2
from pathlib import Path
run, out = sys.argv[1], sys.argv[2]; E = Path.home()/f'b_slam/exports/{run}'; s = 12
pc = o3d.io.read_point_cloud(str(E/'cloud_cloud.ply')); P = np.asarray(pc.points); C = (np.asarray(pc.colors)*255).astype(int)
T = np.loadtxt(E/'traj_optimized_poses.txt', ndmin=2)[:, 1:4]
i = np.concatenate([np.argmin(((P[j:j+20000, None, :2]-T[None, :, :2])**2).sum(-1), 1) for j in range(0, len(P), 20000)]); dz = T[i, 2]-P[:, 2]
ground = (dz > 1.0) & (dz < 2.2); white = ground & (C.mean(1) > 120) & (C.max(1)-C.min(1) < 40)
lo = np.floor(np.r_[P[ground][:, :2], T[:, :2]].min(0)/10)*10 - 5; hi = np.ceil(np.r_[P[ground][:, :2], T[:, :2]].max(0)/10)*10 + 5
W, H = ((hi-lo)*s).astype(int)
img = np.full((H, W, 3), 255, np.uint8)
def px(xy): return ((xy[:, 0]-lo[0])*s).astype(int).clip(0, W-1), (H-1-(xy[:, 1]-lo[1])*s).astype(int).clip(0, H-1)
u, v = px(P[ground]); img[v, u] = (210, 210, 210)
u, v = px(P[white]); img[v, u] = (0, 0, 0)
for x in np.arange(lo[0], hi[0]+1, 5):
    c = int((x-lo[0])*s); cv2.line(img, (c, 0), (c, H), (255, 160, 160) if x % 10 else (230, 60, 60), 1)
    if x % 10 == 0: cv2.putText(img, f'{x:.0f}', (c+2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 0, 0), 1)
for y in np.arange(lo[1], hi[1]+1, 5):
    r = int(H-1-(y-lo[1])*s); cv2.line(img, (0, r), (W, r), (255, 160, 160) if y % 10 else (230, 60, 60), 1)
    if y % 10 == 0: cv2.putText(img, f'{y:.0f}', (2, r-2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 0, 0), 1)
u, v = px(T); 
for a, b, c, d in zip(u[:-1], v[:-1], u[1:], v[1:]):
    if abs(a-c)+abs(b-d) < 60: cv2.line(img, (a, b), (c, d), (0, 160, 0), 1)
cv2.circle(img, (u[0], v[0]), 5, (255, 0, 0), -1)
cv2.putText(img, run, (W-80, H-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
cv2.imwrite(out, img); print(run, 'lo', lo, 'hi', hi, 'px/m', s, img.shape)
np.save(E/'paint_xy.npy', P[white][:, :2]); np.save(E/'ground_xyz.npy', P[ground])
