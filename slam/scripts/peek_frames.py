"""11/8 촬영 기록마다 컬러 사진 3장(시작+15s, 중간, 끝-15s)을 뽑아 한 장의 모음판으로. 원본은 읽기만.
사용: peek_frames.py <out.jpg> <capture_dir>..."""
import sys, numpy as np, cv2
from pathlib import Path
from rosbags.highlevel import AnyReader
out, dirs = sys.argv[1], sys.argv[2:]
rows = []
for d in dirs:
    bag = Path(d) / 'bag'; name = Path(d).name.rsplit('_2026', 1)[0]
    with AnyReader([bag]) as r:
        con = [c for c in r.connections if c.topic == '/camera/camera/color/image_raw']
        t0, t1 = r.start_time, r.end_time; want = [t0 + 15e9, (t0 + t1) / 2, t1 - 15e9]; got = []
        for c, ts, raw in r.messages(connections=con, start=int(want[0])):
            if len(got) < 3 and ts >= want[len(got)]:
                m = r.deserialize(raw, c.msgtype); im = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
                im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR) if m.encoding.lower().startswith('rgb') else im.copy()
                im = cv2.resize(im, (320, 240)); cv2.putText(im, f'{name} +{(ts - t0) / 1e9:.0f}s', (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2); got.append(im)
                if len(got) == 3: break
            elif len(got) < 3 and len(got) and ts < want[len(got)]:
                continue
        while len(got) < 3: got.append(np.zeros((240, 320, 3), np.uint8))
    rows.append(np.hstack(got)); print(name, 'ok')
cv2.imwrite(out, np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 80])
