"""2D 전수 정합: run20(이동) → run18(고정). 바닥 점유 + 주차선(가중 3) 상관, 회전 1° × FFT 이동. 상위 후보와 겹친 그림 출력"""
import numpy as np, cv2
from pathlib import Path
from numpy.fft import rfft2, irfft2
H = Path.home()/'b_slam/exports'; res = 0.2
def grids(run):
    G = np.load(H/run/'ground_xyz.npy')[:, :2]; Pn = np.load(H/run/'paint_xy.npy'); return G, Pn
G0, P0 = grids('run18'); G1, P1 = grids('run20')
lo = np.r_[G0.min(0)] - 45; size = 512  # 102m 칸
def raster(G, Pn, origin):
    a = np.zeros((size, size), np.float32); b = np.zeros_like(a)
    for M, X in ((a, G), (b, Pn)):
        ij = np.floor((X - origin) / res).astype(int); ok = (ij >= 0).all(1) & (ij < size).all(1)
        np.add.at(M, (ij[ok, 1], ij[ok, 0]), 1)
    a = (a > 0).astype(np.float32); b = cv2.dilate((b > 0).astype(np.float32), np.ones((3, 3)))
    return a, b
A0, B0 = raster(G0, P0, lo)
F0a, F0b = rfft2(A0), rfft2(B0)
c1 = G1.mean(0); res_list = []
for yaw in range(0, 360, 1):
    t = np.radians(yaw); R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    g = (G1 - c1) @ R.T; p = (P1 - c1) @ R.T
    o = np.array([-51.2, -51.2]); A1, B1 = raster(g, p, o)
    cg = irfft2(F0a * np.conj(rfft2(A1)), s=A0.shape); cp = irfft2(F0b * np.conj(rfft2(B1)), s=A0.shape)
    sc = cg / A1.sum() + 3 * cp / max(B1.sum(), 1)
    k = np.unravel_index(np.argmax(sc), sc.shape); res_list.append((sc[k], yaw, k, cg[k] / A1.sum(), cp[k] / max(B1.sum(), 1)))
res_list.sort(key=lambda r: -r[0])
top = []
for r in res_list:
    if all(min(abs(r[1] - q[1]), 360 - abs(r[1] - q[1])) > 8 for q in top): top.append(r)
    if len(top) == 4: break
out = []
for sc, yaw, (dy, dx), fg, fp in top:
    # 이동량: 점 x → x' = R(x - c1) + o + shift*res 를 run18 격자 원점 lo 기준으로
    shift = np.array([dx if dx < size // 2 else dx - size, dy if dy < size // 2 else dy - size]) * res
    t = np.radians(yaw); R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    tr = lo + shift - np.array([-51.2, -51.2]) * 0 + np.array([51.2, 51.2]) * 0  # 표시용
    # 실제 변환: x18 = R (x20 - c1) + (-51.2,-51.2) + lo - (-51.2,-51.2)?? → 아래에서 직접 검증
    print(f'yaw {yaw:3d}°  점수 {sc:.3f} (바닥 겹침 {fg*100:.0f}%, 주차선 겹침 {fp*100:.0f}%)')
    out.append((yaw, shift))
np.save('/home/owner7/b_slam/exports/tmp/coarse_top.npy', np.array([[y, s[0], s[1]] for y, s in out]))
# 겹친 그림: run18 회색/검정, run20 빨강/파랑
tiles = []
for (yaw, shift), (sc, *_ ) in zip(out, top):
    t = np.radians(yaw); R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
    # 격자 관계: A1 칸 (i) = (g - o)/res, A0 칸 = (X - lo)/res, 상관 최대 shift: A0(u) ≈ A1(u - shift) → X = g - o + lo + shift
    f = lambda X: (X - c1) @ R.T + np.array([51.2, 51.2]) + lo + shift
    img = np.full((size, size, 3), 255, np.uint8)
    def put(X, col):
        ij = np.floor((X - lo) / res).astype(int); ok = (ij >= 0).all(1) & (ij < size).all(1); img[size - 1 - ij[ok, 1], ij[ok, 0]] = col
    put(G0, (200, 200, 200)); put(f(G1), (180, 180, 255)); put(P0, (0, 0, 0)); put(f(P1), (255, 0, 0))
    cv2.putText(img, f'yaw {yaw}  score {sc:.2f}', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2); tiles.append(img)
    np.save(f'/home/owner7/b_slam/exports/tmp/coarse_T_{yaw}.npy', np.r_[yaw, c1, lo + shift + 51.2])
cv2.imwrite('/home/owner7/b_slam/exports/tmp/coarse.png', np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:4])]))
