"""feature_* 강체 정합 확인: v0.5 바닥 정사 영상 위에 회차별 정합된 주차선 점(색)과 궤적(검정).
사용: fe_overlay.py <out.jpg> <run>... [--align _x]   (정합 결과 폴더 align_<run><접미사>/T_src_to_ref.npy, 기본 _x)"""
import sys, numpy as np, cv2
from pathlib import Path
a = sys.argv[1:]; SUF = '_x'
if '--align' in a: i = a.index('--align'); SUF = a[i + 1]; del a[i:i + 2]
out, runs = a[0], a[1:]
E = Path.home() / 'b_slam/exports'; P = Path.home() / 'b_slam/deliver/map_v0.5'
Q = np.load(P / 'outer/ground_ortho.npz'); rgb, lo, c = Q['rgb'], Q['lo'], float(Q['cell'])
img = np.transpose(rgb, (1, 0, 2))[::-1][:, :, ::-1].copy(); img[(img == 0).all(2)] = 255; img = (img * 0.6 + 255 * 0.4).astype(np.uint8)
H = img.shape[0]
def px(xy): return np.c_[((xy[:, 0] - lo[0]) / c).astype(int), H - 1 - ((xy[:, 1] - lo[1]) / c).astype(int)]
cols = [(0, 0, 255), (0, 160, 0), (255, 0, 0), (0, 200, 255), (255, 0, 255), (0, 120, 255)]
for k, r in enumerate(runs):
    f = E / f'align_{r}{SUF}/T_src_to_ref.npy'
    if not f.exists(): print(r, 'T 없음:', f); continue
    T = np.load(f)
    p = np.load(E / f'{r}_paint_1-12m/paint_xy.npy')[::7]; p2 = p @ T[:2, :2].T + T[:2, 3]
    t = np.loadtxt(E / r / 'traj_optimized_poses.txt', comments='#', ndmin=2)[:, 1:3]; t2 = t @ T[:2, :2].T + T[:2, 3]
    for q in px(p2): cv2.circle(img, tuple(int(v) for v in q), 1, cols[k % len(cols)], -1)
    cv2.polylines(img, [px(t2).reshape(-1, 1, 2).astype(np.int32)], False, (0, 0, 0), 2)
    q0 = px(t2[:1])[0]; cv2.putText(img, r, (int(q0[0]) + 4, int(q0[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6, cols[k % len(cols)], 2)
    print(r, 'T', np.round(T, 3).tolist())
ys, xs = np.where((img < 250).any(2)); img = img[max(ys.min() - 20, 0):ys.max() + 20, max(xs.min() - 20, 0):xs.max() + 20]
cv2.imwrite(out, img)
