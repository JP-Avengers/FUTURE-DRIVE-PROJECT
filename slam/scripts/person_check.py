"""키프레임 컬러에서 사람 검출(OpenCV HOG 보행자). 검출 프레임 목록과 검출 상자 그림(최대 24장 모아 한 장).
사용: person_check.py <run> <out.png>   (~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, cv2
run, out = sys.argv[1], sys.argv[2]
K = Path.home() / f'b_slam/exports/{run}/keyframes/kf_rgb'
hog = cv2.HOGDescriptor(); hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
hits = []
for f in sorted(K.iterdir(), key=lambda p: int(p.stem)):
    im = cv2.imread(str(f)); big = cv2.resize(im, None, fx=1.5, fy=1.5)
    rects, w = hog.detectMultiScale(big, winStride=(8, 8), padding=(8, 8), scale=1.05)
    w = np.ravel(w); keep = [(r / 1.5).astype(int) for r, s in zip(rects, w) if s > 0.8]
    if keep: hits.append((f.stem, im, keep, float(w.max())))
print(f'{run}: 키프레임 {len(list(K.iterdir()))}장 중 사람 의심 {len(hits)}장: ' + ', '.join(f'{h[0]}({h[3]:.1f})' for h in hits[:40]))
tiles = []
for stem, im, rs, s in sorted(hits, key=lambda h: -h[3])[:24]:
    im = im.copy()
    for x, y, w_, h_ in rs: cv2.rectangle(im, (x, y), (x + w_, y + h_), (0, 0, 255), 2)
    cv2.putText(im, f'{stem} {s:.1f}', (6, 22), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 255), 2); tiles.append(cv2.resize(im, (320, 240)))
if tiles:
    while len(tiles) % 4: tiles.append(np.zeros_like(tiles[0]))
    cv2.imwrite(out, np.vstack([np.hstack(tiles[i:i + 4]) for i in range(0, len(tiles), 4)]))
