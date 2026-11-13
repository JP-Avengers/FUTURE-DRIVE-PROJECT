"""DB 전체 노드(끊긴 조각 포함)의 키프레임 사진 모음판: 노드 id · 시각 · 맵 조각 · 오도메트리 위치. 사용: in_sheet.py <run> <out.jpg> [n=24]"""
import sys, sqlite3, struct, numpy as np, cv2
from pathlib import Path
run, out = sys.argv[1], sys.argv[2]; n = int(sys.argv[3]) if len(sys.argv) > 3 else 24
c = sqlite3.connect(str(Path.home() / f'maps/{run}.db')); rows = [x for x in c.execute('select id, map_id, stamp, pose from Node order by id') if x[3] is not None and len(bytes(x[3])) == 48]
K = Path.home() / f'b_slam/exports/{run}/keyframes/kf_rgb'; t0 = rows[0][2]; tiles = []
for q in np.linspace(0, len(rows) - 1, n).astype(int):
    i, mp, st, ps = rows[q]; P = np.array(struct.unpack('12f', bytes(ps))).reshape(3, 4); im = cv2.imread(str(K / f'{i}.jpg'))
    if im is None: im = np.zeros((480, 640, 3), np.uint8)
    cv2.putText(im, f'id{i} t+{st - t0:.0f}s map{mp} ({P[0, 3]:+.1f},{P[1, 3]:+.1f})', (6, 28), 0, 0.8, (0, 0, 255), 2); tiles.append(im[::2, ::2])
tiles += [np.zeros_like(tiles[0])] * (-len(tiles) % 6)
cv2.imwrite(out, np.vstack([np.hstack(tiles[k:k + 6]) for k in range(0, len(tiles), 6)]), [cv2.IMWRITE_JPEG_QUALITY, 80]); print(run, len(rows), '노드')
