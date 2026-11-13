"""실내 회차 위에서 본 그림: 점군(벽 = 바닥 위 0.3~1.8m 점은 진하게, 바닥은 옅게) + 궤적(시간 순 색: 파랑→빨강) + 5m 격자.
회차 좌표 그대로(정합 전). 복도 폭 추정(궤적 수직 방향 벽 사이 거리)도 출력.
사용: in_top.py <out.png> <run> [<run> ...]"""
import sys, numpy as np, open3d as o3d, cv2
from pathlib import Path
E = Path.home() / 'b_slam/exports'; out, runs = sys.argv[1], sys.argv[2:]; C = 0.05; tiles = []
for run in runs:
    pc = o3d.io.read_point_cloud(str(E / run / 'cloud_cloud.ply')); X = np.asarray(pc.points); col = np.asarray(pc.colors)
    tr = np.loadtxt(E / run / 'traj_optimized_poses.txt', comments='#', ndmin=2); P = tr[:, 1:4]
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))]
    zf = np.interp(np.arange(len(X)) * 0, [0], [0])                     # 자리 표시
    from scipy.spatial import cKDTree
    _, k = cKDTree(P[:, :2]).query(X[:, :2]); h = X[:, 2] - (P[k, 2] - 1.2)        # 카메라 높이 약 1.2m 가정 → 바닥 기준 높이
    lo = X[:, :2].min(0) - 1; n = ((X[:, :2].max(0) + 1 - lo) / C).astype(int) + 1
    img = np.full((n[1], n[0], 3), 255, np.uint8)
    def put(m, c_):
        u = ((X[m, 0] - lo[0]) / C).astype(int); v = n[1] - 1 - ((X[m, 1] - lo[1]) / C).astype(int); img[v, u] = c_
    put((h > -0.2) & (h < 0.2), (225, 225, 225)); put((h > 0.3) & (h < 1.8), (60, 60, 60))
    for x in np.arange(np.ceil(lo[0] / 5) * 5, lo[0] + n[0] * C, 5): cv2.line(img, (int((x - lo[0]) / C), 0), (int((x - lo[0]) / C), n[1] - 1), (200, 220, 255), 1); cv2.putText(img, f'{x:.0f}', (int((x - lo[0]) / C) + 2, 12), 0, 0.4, (0, 120, 255), 1)
    for y in np.arange(np.ceil(lo[1] / 5) * 5, lo[1] + n[1] * C, 5): cv2.line(img, (0, n[1] - 1 - int((y - lo[1]) / C)), (n[0] - 1, n[1] - 1 - int((y - lo[1]) / C)), (200, 220, 255), 1); cv2.putText(img, f'{y:.0f}', (2, n[1] - 3 - int((y - lo[1]) / C)), 0, 0.4, (0, 120, 255), 1)
    for i in range(len(P) - 1):
        t = i / max(len(P) - 2, 1); c_ = (int(255 * (1 - t)), 40, int(255 * t))
        a_ = (int((P[i, 0] - lo[0]) / C), n[1] - 1 - int((P[i, 1] - lo[1]) / C)); b_ = (int((P[i + 1, 0] - lo[0]) / C), n[1] - 1 - int((P[i + 1, 1] - lo[1]) / C)); cv2.line(img, a_, b_, c_, 2)
    for d_ in range(0, int(s[-1]) + 1, 10):
        j = int(np.argmin(np.abs(s - d_))); q = (int((P[j, 0] - lo[0]) / C), n[1] - 1 - int((P[j, 1] - lo[1]) / C)); cv2.circle(img, q, 4, (0, 160, 0), -1); cv2.putText(img, f'{d_}', (q[0] + 5, q[1] - 4), 0, 0.45, (0, 120, 0), 1)
    # 복도 폭: 궤적 점마다 진행 방향에 수직인 양쪽 벽(높이 0.5~1.5m 점)까지 거리
    W = X[(h > 0.5) & (h < 1.5), :2]; wt = cKDTree(W); ws = []
    for i in range(5, len(P) - 5, 5):
        d = P[i + 5, :2] - P[i - 5, :2]; nrm = np.linalg.norm(d)
        if nrm < 0.5: continue
        d /= nrm; nv = np.array([-d[1], d[0]]); idx = wt.query_ball_point(P[i, :2], 4.0)
        if len(idx) < 50: continue
        rel = W[idx] - P[i, :2]; along = rel @ d; lat = rel @ nv; m = np.abs(along) < 0.25
        L_, R_ = lat[m & (lat > 0.3)], lat[m & (lat < -0.3)]
        if len(L_) > 5 and len(R_) > 5: ws.append(np.percentile(L_, 10) - np.percentile(R_, 90))
    ws = np.array(ws); wtxt = f'폭 중앙 {np.median(ws):.2f}m (10~90% {np.percentile(ws, 10):.2f}~{np.percentile(ws, 90):.2f}, n={len(ws)})' if len(ws) else '폭 측정 불가'
    print(f'{run}: 점 {len(X):,}, 경로 {s[-1]:.1f}m, 범위 {np.ptp(X[:, 0]):.0f}×{np.ptp(X[:, 1]):.0f}m, z 범위(궤적) {P[:, 2].min():+.2f}~{P[:, 2].max():+.2f} | 복도 {wtxt}')
    cv2.putText(img, f'{run}  path {s[-1]:.0f} m  (blue=start, red=end, green=10 m marks)', (40, n[1] - 10), 0, 0.5, (0, 0, 0), 1); tiles.append(img)
w = max(t.shape[1] for t in tiles); tiles = [np.pad(t, ((4, 4), (0, w - t.shape[1]), (0, 0)), constant_values=255) for t in tiles]
img = np.vstack(tiles); sc = min(1.0, 2200 / max(img.shape[:2])); cv2.imwrite(out, cv2.resize(img, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA))
