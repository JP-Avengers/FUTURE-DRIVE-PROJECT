"""벽 녹화: 원본 depth 픽셀별 시간 중앙값(3프레임마다, 0 제외) + 컬러 한 장을 저장하고, 영역 고르기용 그림(격자마다 중앙값 depth 숫자).
사용: wall_median.py <out_dir> <capture_dir>...   (~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, cv2
from rosbags.highlevel import AnyReader
NS = '/camera/camera'; out = Path(sys.argv[1]); out.mkdir(exist_ok=True)
for cap in sys.argv[2:]:
    name = Path(cap).name; D = []; img = None
    with AnyReader([Path(cap) / 'bag']) as r:
        K = next(r.deserialize(raw, c.msgtype).k for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic == f'{NS}/depth/camera_info']))
        i = 0
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic in (f'{NS}/depth/image_rect_raw', f'{NS}/color/image_raw')]):
            m = r.deserialize(raw, c.msgtype)
            if 'color' in c.topic:
                if img is None: img = cv2.cvtColor(np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3), cv2.COLOR_RGB2BGR)
                continue
            i += 1
            if i % 3: continue
            D.append(np.frombuffer(m.data, np.uint16).reshape(m.height, m.width))
    D = np.array(D, dtype=np.float32); D[D == 0] = np.nan
    med = np.nanmedian(D, 0) / 1000; frac = np.mean(~np.isnan(D), 0)
    np.savez_compressed(out / f'{name}.npz', med=med, frac=frac, K=np.array(K), n=len(D))
    v = cv2.applyColorMap(np.clip(np.nan_to_num(med) / 12 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO); v[np.isnan(med)] = 0
    for y in range(40, 480, 60):
        for x in range(40, 640, 80):
            z = med[y, x]; cv2.circle(v, (x, y), 3, (255, 255, 255), -1)
            s = '-' if np.isnan(z) else f'{z:.2f}'; cv2.putText(v, s, (x - 22, y - 6), cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 0, 0), 3); cv2.putText(v, s, (x - 22, y - 6), cv2.FONT_HERSHEY_SIMPLEX, .42, (255, 255, 255), 1)
    for x in range(0, 640, 80): cv2.line(img, (x, 0), (x, 479), (255, 255, 255), 1); cv2.putText(img, str(x), (x + 2, 470), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1)
    for y in range(0, 480, 60): cv2.line(img, (0, y), (639, y), (255, 255, 255), 1); cv2.putText(img, str(y), (2, y + 12), cv2.FONT_HERSHEY_SIMPLEX, .4, (255, 255, 255), 1)
    t = np.hstack([img, v]); cv2.putText(t, f'{name} (frames {len(D)})', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 0), 3); cv2.putText(t, f'{name} (frames {len(D)})', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 1)
    cv2.imwrite(str(out / f'{name}_grid.png'), t); print(name, len(D))
