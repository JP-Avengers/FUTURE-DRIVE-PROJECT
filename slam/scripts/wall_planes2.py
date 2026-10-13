"""중앙값 depth(wall_median.py 결과)에서 정면을 향한 평면을 순서대로 최대 5개 찾음(RANSAC, 문턱 3cm+0.5%·Z, 법선이 광축과 35° 이내).
평면별: 카메라→평면 수직 거리, 법선 각도, 화면 비율, 화면 위치(가운데 픽셀). 그림: 평면별 색칠.
사용: wall_planes2.py <median_dir> <name>...   (~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, cv2
rng = np.random.default_rng(0)
M = Path(sys.argv[1]); COL = [(0, 0, 255), (0, 200, 0), (255, 0, 0), (0, 200, 255), (255, 0, 255)]
for name in sys.argv[2:]:
    f = np.load(M / f'{name}.npz'); d, K = f['med'], f['K']
    vv, uu = np.mgrid[0:480, 0:640]; ok = np.isfinite(d) & (d > 0.5) & (d < 14) & (f['frac'] > 0.5)
    P = np.c_[(uu[ok] - K[2]) * d[ok] / K[0], (vv[ok] - K[5]) * d[ok] / K[4], d[ok]]; U, V = uu[ok], vv[ok]
    idx = np.arange(len(P)); vis = cv2.cvtColor(np.clip(np.nan_to_num(d) / 12 * 255, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR); out = []
    for j in range(5):
        if len(idx) < 2000: break
        Q = P[idx]; best = None
        for _ in range(600):
            s = Q[rng.choice(len(Q), 3, replace=False)]; n = np.cross(s[1] - s[0], s[2] - s[0]); nn = np.linalg.norm(n)
            if nn < 1e-9: continue
            n /= nn
            if abs(n[2]) < np.cos(np.radians(35)): continue
            off = s[0] @ n; k = np.abs(Q @ n - off) < 0.03 + 0.005 * abs(off)
            if best is None or k.sum() > best[0]: best = (k.sum(), k)
        if best is None or best[0] < 1500: break
        k = best[1]
        for _ in range(3):
            X = Q[k]; c = X.mean(0); n = np.linalg.svd(X - c, full_matrices=False)[2][-1]; dist = abs(c @ n)
            k = np.abs((Q - c) @ n) < 0.03 + 0.005 * dist
        sel = idx[k]; res = np.abs((P[sel] - c) @ n)
        out.append((dist, np.degrees(np.arccos(abs(n[2]))), len(sel) / ok.size, int(np.median(U[sel])), int(np.median(V[sel])), np.std(res) * 100))
        vis[V[sel], U[sel]] = COL[j]; cv2.putText(vis, f'{j}: {dist:.3f}m', (8, 24 + 24 * j), cv2.FONT_HERSHEY_SIMPLEX, .7, COL[j], 2)
        idx = idx[~k]
    cv2.imwrite(str(M / f'{name}_planes.png'), vis)
    print(f'{name:34s} ' + ' | '.join(f'#{j} {o[0]:6.3f}m 각 {o[1]:4.1f}° 화면 {o[2]*100:3.0f}% 중심({o[3]},{o[4]}) 잔차 {o[5]:.1f}cm' for j, o in enumerate(out)))
