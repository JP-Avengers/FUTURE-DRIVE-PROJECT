#!/usr/bin/env python3
"""주차선(바닥 흰 페인트) 2D 정합: 새 맵(src)을 기준 맵(ref, v0.3 좌표계)에 yaw + 수평 이동으로 맞춤.
1) 전수 탐색: 두 페인트를 0.2m 격자 밀도 영상(가우시안 흐림)으로 만들고 yaw 0~359°(1°)마다 FFT 상관 → 최고점
   (180° 반대 후보·다른 위치 후보와 점수 비교로 확실성 판정)
2) 정밀: yaw ±2°(0.1°) 재탐색 후 페인트 점 2D ICP(잘린 최소제곱, yaw+이동)
3) 검증: src 페인트 → 가장 가까운 ref 페인트 거리(겹치는 곳), 구역별
출력: exports/<out>/T_src_to_ref_2d.npy (3x3), report.txt, overlay.png(검정 ref, 빨강 src)
사용: align_paint2d.py <src_paint_dir> <ref_paint_dir> <out_dir_name>   (exports/ 아래 이름, 각 폴더에 paint_xy.npy)"""
import sys
from pathlib import Path
import numpy as np, cv2
from scipy.spatial import cKDTree
from scipy.ndimage import gaussian_filter
H = Path.home() / 'b_slam/exports'
S = np.load(H / sys.argv[1] / 'paint_xy.npy'); R = np.load(H / sys.argv[2] / 'paint_xy.npy')
out = H / sys.argv[3]; out.mkdir(exist_ok=True)
rng = np.random.default_rng(0)
S = S[rng.choice(len(S), min(len(S), 150000), replace=False)]; R = R[rng.choice(len(R), min(len(R), 150000), replace=False)]
c = 0.2
def raster(P, lo, shape):
    g = np.zeros(shape); u = ((P - lo) / c).astype(int); k = (u[:, 0] >= 0) & (u[:, 0] < shape[0]) & (u[:, 1] >= 0) & (u[:, 1] < shape[1])
    np.add.at(g, (u[k, 0], u[k, 1]), 1); g = np.minimum(g, 5); return gaussian_filter(g, 1.0)
sc = S.mean(0); S0 = S - sc
rad = np.abs(S0).max() + 2
lo = R.min(0) - rad; hi = R.max(0) + rad; shape = tuple(((hi - lo) / c).astype(int) + 1)
FR = np.fft.rfft2(raster(R, lo, shape))
def score_at(yaw):
    t = np.radians(yaw); Rm = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    g = raster(S0 @ Rm.T, np.array([-rad, -rad]), shape)  # src 중심이 격자 (rad, rad)에 오게
    cc = np.fft.irfft2(FR * np.conj(np.fft.rfft2(g)), s=shape)
    k = np.unravel_index(np.argmax(cc), cc.shape)
    return cc[k], wrap(k), cc
def wrap(k):  # 원형 상관의 음수 이동 처리
    return tuple(int(ki - n) if ki > n - rad / c else int(ki) for ki, n in zip(k, shape))
res = []
for yaw in np.arange(0, 360, 1.0):
    s, k, _ = score_at(yaw); res.append((s, yaw, k))
res.sort(key=lambda r: -r[0]); best = res[0]
alt = next(r for r in res if abs((r[1] - best[1] + 180) % 360 - 180) > 10)
fine = [(score_at(y)[0], y) for y in np.arange(best[1] - 2, best[1] + 2.01, 0.1)]
yaw0 = max(fine)[1]; s0, k0, cc = score_at(yaw0)
shift = np.array(k0) * c + lo - np.array([-rad, -rad])          # 격자 이동 → 월드
t = np.radians(yaw0); Rm = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
tvec = shift - Rm @ sc
# 같은 yaw에서 다른 위치(2m 이상 떨어진) 두 번째 봉우리
cc2 = np.roll(cc, (-k0[0], -k0[1]), (0, 1)); r0 = int(2 / c); cc2[:r0, :r0] = cc2[:r0, -r0:] = cc2[-r0:, :r0] = cc2[-r0:, -r0:] = -np.inf
s_pos = cc2.max()
# ICP (2D, 잘린 최소제곱)
tree = cKDTree(R)
for it in range(40):
    X = S @ Rm.T + tvec; d, j = tree.query(X)
    thr = max(np.percentile(d, 60), 0.15); m = d < thr
    A, B = X[m], R[j[m]]; ma, mb = A.mean(0), B.mean(0)
    U, _, Vt = np.linalg.svd((A - ma).T @ (B - mb)); dR = Vt.T @ U.T
    if np.linalg.det(dR) < 0: Vt[-1] *= -1; dR = Vt.T @ U.T
    Rm = dR @ Rm; tvec = dR @ tvec + (mb - dR @ ma)
X = S @ Rm.T + tvec; d, _ = tree.query(X)
near = d < 1.0
yaw = np.degrees(np.arctan2(Rm[1, 0], Rm[0, 0]))
T = np.eye(3); T[:2, :2] = Rm; T[:2, 2] = tvec; np.save(out / 'T_src_to_ref_2d.npy', T)
lines = [f'전수 탐색: 최고 yaw {best[1]:.0f}° 점수 {best[0]:.0f} | 다른 각도 최고 {alt[1]:.0f}° 점수 {alt[0]:.0f} (비 {best[0]/alt[0]:.2f}) | 같은 각도 다른 위치 비 {s0/s_pos:.2f}',
         f'최종: yaw {yaw:.2f}°, 이동 ({tvec[0]:.2f}, {tvec[1]:.2f}) m',
         f'src 페인트 중 ref 1m 안에 있는 비율 {100*near.mean():.0f}%, 그 거리 중앙 {np.median(d[near])*100:.1f}cm, 80% {np.percentile(d[near],80)*100:.1f}cm']
print('\n'.join(lines)); (out / 'report.txt').write_text('\n'.join(lines) + '\n')
# 겹쳐 그리기
P = np.vstack([R, X]); lo2 = P.min(0) - 1; s = 20; w, h = ((P.max(0) + 1 - lo2) * s).astype(int) + 1
img = np.full((h, w, 3), 255, np.uint8)
for Q, col in ((R, (0, 0, 0)), (X, (0, 0, 230))):
    u = ((Q[:, 0] - lo2[0]) * s).astype(int); v = (h - 1 - (Q[:, 1] - lo2[1]) * s).astype(int); img[v, u] = col
for x in np.arange(np.ceil(lo2[0] / 5) * 5, P.max(0)[0], 5):
    X0 = int((x - lo2[0]) * s); img[:, X0] = (200, 200, 255); cv2.putText(img, f'{x:.0f}', (X0 + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 0, 200), 1)
for y in np.arange(np.ceil(lo2[1] / 5) * 5, P.max(0)[1], 5):
    Y0 = int(h - 1 - (y - lo2[1]) * s); img[Y0, :] = (200, 200, 255); cv2.putText(img, f'{y:.0f}', (2, Y0 - 2), cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 0, 200), 1)
cv2.imwrite(str(out / 'overlay.png'), img)
