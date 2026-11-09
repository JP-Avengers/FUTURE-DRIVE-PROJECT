"""바닥 정사 영상과 구역 지도: 키프레임 사진의 바닥 화소를 바닥 높이 지도 위로 투영(광선-바닥 교차).
칸(바닥 높이 지도와 같은 10cm)마다 가장 가까이에서 본 화소 몇 개의 중앙값 색. depth가 바닥보다 가까우면(차·사람 등에 가림) 쓰지 않음.
구역: HSV로 아스팔트(무채색, 흰 선 포함)·풀(초록)·붉은 벽돌 보도·기타, 5x5 다수결로 다듬음.
출력: ground_ortho.png(위가 +y, 오른쪽이 +x, 1px = 1칸), ground_ortho.npz(rgb[ix, iy], dist, lo, cell), ground_zones.npz(zone[ix, iy]), ground_zones.png
투영 거리: 본네트 12m, 지붕 --maxd(15m). 그보다 먼 화소는 바닥에 길게 늘어져 쓸 수 없음. 작은 빈틈은 둘레 색으로 메움(inpaint).
사용: ground_ortho.py <pkg_dir> <floor_height.npz> <out_dir> [--maxd 15] [--step 2]"""
import argparse, re, json
import numpy as np, cv2
from pathlib import Path
from scipy import ndimage
from scipy.spatial.transform import Rotation as Rot

ap = argparse.ArgumentParser(); ap.add_argument('pkg'); ap.add_argument('floor'); ap.add_argument('out')
ap.add_argument('--maxd', type=float, default=15.0); ap.add_argument('--step', type=int, default=2); a = ap.parse_args()
P, O = Path(a.pkg), Path(a.out); O.mkdir(parents=True, exist_ok=True)
F = np.load(a.floor); G, lo, cell = F['grid'].astype(np.float32), F['lo'], float(F['cell']); nx, ny = G.shape
def fh(x, y): return ndimage.map_coordinates(G, [(x - lo[0]) / cell - 0.5, (y - lo[1]) / cell - 0.5], order=1, mode='nearest')
K = 3                                                # 칸마다 가까운 화소 3개까지
bd = np.full((nx * ny, K), np.inf, np.float32); bc = np.zeros((nx * ny, K, 3), np.uint8)
sets = [('bonnet_cw', 12.0), ('bonnet_ccw', 12.0), ('roof_cw', a.maxd), ('roof_ccw', a.maxd)]   # 먼 화소는 바닥에 길게 늘어져 줄무늬가 됨
nk = 0
for name, maxd in sets:
    Kd = P / f'keyframes_{name}'
    for r in np.loadtxt(P / f'trajectory_{name}_camera_optical.txt', comments='#', ndmin=2):
        i = int(r[8]); ip, dp = Kd / f'rgb/{i}.jpg', Kd / f'depth/{i}.png'
        if not ip.exists(): continue
        k = [float(x) for x in re.search(r'camera_matrix:.*?data: \[([^\]]+)\]', (Kd / f'calib/{i}.yaml').read_text(), re.S).group(1).replace('\n', ' ').split(',')]
        im = cv2.imread(str(ip)); D = cv2.imread(str(dp), cv2.IMREAD_UNCHANGED) if dp.exists() else None; h_, w_ = im.shape[:2]
        v, u = np.mgrid[0:h_:a.step, 0:w_:a.step]; u = u.ravel().astype(np.float32); v = v.ravel().astype(np.float32)
        Rm = Rot.from_quat(r[4:8]).as_matrix(); c = r[1:4]
        d = np.c_[(u - k[2]) / k[0], (v - k[5]) / k[4], np.ones_like(u)] @ Rm.T; d /= np.linalg.norm(d, axis=1, keepdims=True)
        dn = d[:, 2] < -0.02                                             # 아래를 향한 광선만
        u, v, d = u[dn], v[dn], d[dn]
        z0 = fh(np.array([c[0]]), np.array([c[1]]))[0]; t = (z0 - c[2]) / d[:, 2]
        for _ in range(3):                                               # 바닥 높이 지도와 교차 (고정점 반복)
            p = c + t[:, None] * d; t = (fh(p[:, 0], p[:, 1]) - c[2]) / d[:, 2]
        p = c + t[:, None] * d; ok = (t > 0.5) & (t < maxd)
        if D is not None:                                                # depth가 바닥보다 확실히 가까우면 가림 → 제외
            dz = D[v.astype(int), u.astype(int)].astype(np.float32) / 1000.0                  # depth = 광축 방향 거리
            zc = t / np.linalg.norm(np.c_[(u - k[2]) / k[0], (v - k[5]) / k[4], np.ones_like(u)], axis=1)   # 바닥 교차점의 광축 방향 거리
            ok &= ~((dz > 0.3) & (dz < zc - 0.25 - 0.03 * zc))
        ix = ((p[:, 0] - lo[0]) / cell).astype(int); iy = ((p[:, 1] - lo[1]) / cell).astype(int); ok &= (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
        cid = ix[ok] * ny + iy[ok]; dist = t[ok].astype(np.float32); col = im[v[ok].astype(int), u[ok].astype(int)]
        o = np.lexsort((dist, cid)); cid, dist, col = cid[o], dist[o], col[o]
        first = np.r_[True, cid[1:] != cid[:-1]]; cid, dist, col = cid[first], dist[first], col[first]   # 이 사진에서 칸마다 가장 가까운 화소 하나
        # 칸별 상위 K 갱신: 새 값이 현재 최댓값보다 작으면 그 자리에 넣음
        worst = bd[cid].argmax(1); repl = dist < bd[cid, worst]
        bd[cid[repl], worst[repl]] = dist[repl]; bc[cid[repl], worst[repl]] = col[repl]; nk += 1
print(f'키프레임 {nk}장 투영')
has = np.isfinite(bd).sum(1); rgb = np.zeros((nx * ny, 3), np.uint8)
for kk in range(1, K + 1):
    m = has == kk
    if m.any(): rgb[m] = np.median(np.where(np.isfinite(bd[m])[:, :, None], bc[m].astype(float), np.nan), axis=1) if kk > 1 else bc[m][np.arange(m.sum()), np.argmin(bd[m], 1)]
dist = np.where(has > 0, np.nanmin(np.where(np.isfinite(bd), bd, np.nan), 1), np.nan).reshape(nx, ny)
rgb = rgb.reshape(nx, ny, 3); cov = has.reshape(nx, ny) > 0
# 작은 빈틈(닫힘 연산으로 메워지는 곳)은 둘레 색으로 메움
cl_ = ndimage.binary_closing(cov, iterations=3) & ~cov
if cl_.any():
    rgb = cv2.inpaint(rgb, cl_.astype(np.uint8), 3, cv2.INPAINT_TELEA); cov = cov | cl_
print(f'빈틈 메움 {cl_.sum():,}칸')
# 구역 분류 (BGR → HSV, OpenCV H 0~180)
hsv = cv2.cvtColor(rgb.reshape(1, -1, 3), cv2.COLOR_BGR2HSV).reshape(nx, ny, 3).astype(float); Hh, Ss, Vv = hsv[..., 0] * 2, hsv[..., 1] / 255, hsv[..., 2] / 255
zone = np.zeros((nx, ny), np.uint8)                                      # 0 없음, 1 아스팔트, 2 풀, 3 붉은 보도, 4 기타
zone[cov] = 4
zone[cov & ((Ss < 0.22) | ((Hh > 180) & (Hh < 260) & (Ss < 0.40)))] = 1        # 무채색 + 하늘빛이 비친 푸르스름한 아스팔트
zone[cov & (Hh >= 45) & (Hh <= 170) & (Ss >= 0.18) & (Vv > 0.12)] = 2
zone[cov & ((Hh < 25) | (Hh > 330)) & (Ss >= 0.22) & (Vv > 0.2)] = 3
lab = np.zeros((5, nx, ny), np.float32)
for zi in range(5): lab[zi] = ndimage.uniform_filter((zone == zi).astype(np.float32), 5)
zs = lab.argmax(0).astype(np.uint8); zs[~cov] = 0
np.savez_compressed(O / 'ground_ortho.npz', rgb=rgb[..., ::-1], dist=dist.astype(np.float32), lo=lo, cell=cell)
np.savez_compressed(O / 'ground_zones.npz', zone=zs, lo=lo, cell=cell, names=np.array(['없음', '아스팔트', '풀', '붉은 보도', '기타']))
img = np.transpose(rgb, (1, 0, 2))[::-1].copy(); img[~np.transpose(cov)[::-1]] = 255; cv2.imwrite(str(O / 'ground_ortho.png'), img)
pal = np.array([[255, 255, 255], [90, 90, 90], [60, 170, 60], [60, 70, 200], [200, 180, 150]], np.uint8)
cv2.imwrite(str(O / 'ground_zones.png'), pal[np.transpose(zs)[::-1]])
area = {n: round(float((zs == i).sum() * cell * cell)) for i, n in enumerate(['없음', '아스팔트', '풀', '붉은 보도', '기타'])}
json.dump(dict(keyframes=nk, covered_m2=round(float(cov.sum() * cell * cell)), zone_m2=area), open(O / 'ground_ortho.json', 'w'), ensure_ascii=False, indent=1)
print('덮인 넓이', round(float(cov.sum() * cell * cell)), '㎡, 구역', area)
