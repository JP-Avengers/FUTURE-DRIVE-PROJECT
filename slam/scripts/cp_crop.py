"""같은 세계 영역을 정사 영상과 손 촬영 BEV에서 잘라 같은 축척(1m 격자, 5m 좌표)으로 나란히 + 반투명 겹침.
사용: cp_crop.py <bev_dir> <run> <x0> <x1> <y0> <y1> <out.png> [--scale 3]"""
import sys, numpy as np, cv2
from pathlib import Path
a = sys.argv; D, run = Path(a[1]), a[2]; x0, x1, y0, y1 = map(float, a[3:7]); out = a[7]
SC = int(a[a.index('--scale') + 1]) if '--scale' in a else 3; C = 0.1
def crop(img_path, lo):
    img = cv2.imread(str(img_path)); H = img.shape[0]
    c0 = int(round((x0 - lo[0]) / C)); c1 = int(round((x1 - lo[0]) / C)); r0 = H - 1 - int(round((y1 - lo[1]) / C)); r1 = H - 1 - int(round((y0 - lo[1]) / C))
    outi = np.zeros((r1 - r0, c1 - c0, 3), np.uint8)
    rs, cs = slice(max(r0, 0), min(r1, H)), slice(max(c0, 0), min(c1, img.shape[1]))
    outi[rs.start - r0:rs.stop - r0, cs.start - c0:cs.stop - c0] = img[rs, cs]; return outi
Q = np.load(Path.home() / 'b_slam/deliver/map_v0.5/outer/ground_ortho.npz'); qlo = Q['lo']
o = np.ascontiguousarray(np.transpose(Q['rgb'], (1, 0, 2))[::-1][:, :, ::-1]); cv2.imwrite('/tmp/_ortho_raw.png', o)
b = np.load(D / f'{run}_bev.npz')
A = crop('/tmp/_ortho_raw.png', qlo); B = crop(D / f'{run}_bev.png', b['lo'])
def grid(img):
    img = cv2.resize(img, None, fx=SC, fy=SC, interpolation=cv2.INTER_NEAREST); H, W = img.shape[:2]
    for x in np.arange(np.ceil(x0), x1 + 0.01, 1.0):
        u = int((x - x0) / C * SC); col = (0, 165, 255) if x % 5 == 0 else (70, 70, 70); cv2.line(img, (u, 0), (u, H - 1), col, 1)
        if x % 5 == 0: cv2.putText(img, f'{x:.0f}', (u + 2, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1)
    for y in np.arange(np.ceil(y0), y1 + 0.01, 1.0):
        v = H - 1 - int((y - y0) / C * SC); col = (0, 165, 255) if y % 5 == 0 else (70, 70, 70); cv2.line(img, (0, v), (W - 1, v), col, 1)
        if y % 5 == 0: cv2.putText(img, f'{y:.0f}', (2, v - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 220, 255), 1)
    return img
mb = (B.sum(2) > 0)[..., None]; blend = np.where(mb, (A * 0.5 + B * 0.5), A).astype(np.uint8)
cv2.imwrite(out.replace('.png', '_ortho.png'), grid(A)); cv2.imwrite(out.replace('.png', '_bev.png'), grid(B)); cv2.imwrite(out.replace('.png', '_blend.png'), grid(blend))
print('저장', out)
