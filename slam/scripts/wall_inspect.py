"""벽 녹화 점검: 회차별 가운데 프레임 컬러·depth 나란히(가운데 25% 상자 표시), IMU 피치·자이로 흔들림, 가운데 depth 분포.
사용: wall_inspect.py <out.png> <capture_dir>...   (~/tsdf_env)"""
import sys
from pathlib import Path
import numpy as np, cv2
from rosbags.highlevel import AnyReader
NS = '/camera/camera'
out, caps = sys.argv[1], sys.argv[2:]
tiles = []
for cap in caps:
    bag = Path(cap) / 'bag'; name = Path(cap).name
    with AnyReader([bag]) as r:
        dur = (r.end_time - r.start_time) / 1e9; mid = r.start_time + (r.end_time - r.start_time) // 2
        acc, gyr = [], []
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic in (f'{NS}/accel/sample', f'{NS}/gyro/sample')]):
            m = r.deserialize(raw, c.msgtype)
            (acc if 'accel' in c.topic else gyr).append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z] if 'accel' in c.topic else [m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z])
        A, G = np.array(acc), np.array(gyr)
        g = A.mean(0); pitch = np.degrees(np.arctan2(-g[2], -g[1]))  # 광학 좌표(y 아래, z 앞): 아래로 숙이면 +
        shake = np.degrees(np.linalg.norm(G - G.mean(0), axis=1)).std()
        img = dep = None
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic in (f'{NS}/color/image_raw', f'{NS}/depth/image_rect_raw')]):
            if t < mid: continue
            m = r.deserialize(raw, c.msgtype)
            if 'color' in c.topic and img is None:
                img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3); img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else img
            if 'depth' in c.topic and dep is None: dep = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).astype(float) / 1000
            if img is not None and dep is not None: break
    h, w = dep.shape; roi = dep[int(h * .375):int(h * .625), int(w * .375):int(w * .625)]; z = roi[roi > 0.2]
    hist = np.histogram(z, np.arange(0, 15.01, 0.25))[0]; top = np.argsort(hist)[::-1][:3]
    print(f'{name:36s} {dur:5.0f}s  피치 {pitch:+5.1f}°(아래+)  자이로 흔들림 {shake:5.2f}°/s  가운데 depth 유효 {len(z)/roi.size*100:3.0f}% 중앙 {np.median(z):.2f}m  최빈 ' + ', '.join(f'{k*0.25:.2f}~{k*0.25+0.25:.2f}m({hist[k]/len(z)*100:.0f}%)' for k in top))
    dc = cv2.applyColorMap(np.clip(dep / 12 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_TURBO); dc[dep == 0] = 0
    for im in (img, dc):
        cv2.rectangle(im, (int(w * .375), int(h * .375)), (int(w * .625), int(h * .625)), (255, 255, 255), 2)
    t = np.hstack([img, dc]); cv2.putText(t, f'{name}  pitch {pitch:+.1f}  shake {shake:.2f}', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .7, (255, 255, 255), 3); cv2.putText(t, f'{name}  pitch {pitch:+.1f}  shake {shake:.2f}', (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 0, 0), 1)
    tiles.append(cv2.resize(t, (640, 240)))
cv2.imwrite(out, np.vstack(tiles)); print('→', out)
