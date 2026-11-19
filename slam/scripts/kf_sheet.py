"""키프레임 RGB 모음판. 사용: kf_sheet.py <out.jpg> <run> <node_index_from> <to> <step> [cols]"""
import sys, sqlite3, numpy as np, cv2
from pathlib import Path
out, run, a, b, s = sys.argv[1], sys.argv[2], *map(int, sys.argv[3:6]); C = int(sys.argv[6]) if len(sys.argv) > 6 else 6
rows = sqlite3.connect(str(Path.home() / f'maps/{run}.db')).execute('select id, stamp from Node order by id').fetchall(); K = Path.home() / f'b_slam/exports/{run}/keyframes/kf_rgb'
ims = []
for j in range(a, min(b, len(rows)), s):
    i, t = rows[j]; f = K / f'{i}.jpg'
    im = cv2.resize(cv2.imread(str(f)), (240, 180)) if f.exists() else np.zeros((180, 240, 3), np.uint8)
    cv2.putText(im, f'{j} t{t - rows[0][1]:.0f}s', (4, 16), 0, 0.5, (0, 255, 255), 2); ims.append(im)
ims += [np.zeros_like(ims[0])] * (-len(ims) % C)
cv2.imwrite(out, np.vstack([np.hstack(ims[i:i + C]) for i in range(0, len(ims), C)]), [cv2.IMWRITE_JPEG_QUALITY, 80])
