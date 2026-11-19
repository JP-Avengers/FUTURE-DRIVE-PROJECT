"""촬영 회차 훑어보기(원본은 읽기만): 일정 간격 창마다 가속도로 카메라 기울기(위 +), 정렬 depth 통계, 컬러 썸네일.
사용: cap_peek.py <out_prefix> <capture_dir>... [--step 6] [--thumb 20]"""
import sys, json, numpy as np, cv2
from pathlib import Path
from rosbags.highlevel import AnyReader
a = sys.argv; opt = lambda k, d: float(a[a.index(k) + 1]) if k in a else d
STEP, TH = opt('--step', 6.0), opt('--thumb', 20.0)
out = a[1]; dirs = [x for i, x in enumerate(a[2:], 2) if not x.startswith('--') and not a[i - 1].startswith('--')]
rows, rep = [], {}
for d in dirs:
    bag = Path(d) / 'bag'; name = Path(d).name.rsplit('_2026', 1)[0]
    with AnyReader([bag]) as r:
        cs = {c.topic: c for c in r.connections}
        acc, col, dep = cs['/camera/camera/accel/sample'], cs['/camera/camera/color/image_raw'], cs['/camera/camera/aligned_depth_to_color/image_raw']
        t0, t1 = r.start_time, r.end_time; recs, thumbs = [], []; nextth = t0
        for t in np.arange(t0 + 1e9, t1 - 0.5e9, STEP * 1e9):
            A, img, dimg = [], None, None
            for c, ts, raw in r.messages(connections=[acc, col, dep], start=int(t), stop=int(t + 0.3e9)):
                m = r.deserialize(raw, c.msgtype)
                if c is acc: A.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
                elif c is col and img is None: img = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
                elif c is dep and dimg is None: dimg = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width)
            if not A or img is None or dimg is None: continue
            u = np.mean(A, 0); u = u / np.linalg.norm(u)
            pitch = float(np.degrees(np.arcsin(np.clip(u[2], -1, 1)))); roll = float(np.degrees(np.arctan2(u[0], -u[1])))
            z = dimg.astype(np.float32) / 1000; v = z > 0.1; top = v[:160]
            recs.append(dict(t=float((t - t0) / 1e9), pitch=pitch, roll=roll, valid=float(v.mean()), top_valid=float(top.mean()),
                             med=float(np.median(z[v])) if v.any() else 0.0))
            if t >= nextth:
                im = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) if img.shape[2] == 3 else img[..., :3]
                im = cv2.resize(im, (256, 192)); cv2.putText(im, f'{(t - t0) / 1e9:.0f}s p{pitch:+.0f}', (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
                thumbs.append(im); nextth = t + TH * 1e9
    P = np.array([x['pitch'] for x in recs]); R = np.array([x['roll'] for x in recs])
    rep[name] = dict(dur_s=(t1 - t0) / 1e9, n=len(recs), pitch_med=float(np.median(P)), pitch_p10=float(np.percentile(P, 10)), pitch_p90=float(np.percentile(P, 90)),
                     roll_med=float(np.median(R)), valid_med=float(np.median([x['valid'] for x in recs])), top_valid_med=float(np.median([x['top_valid'] for x in recs])),
                     depth_med=float(np.median([x['med'] for x in recs])), series=recs)
    print(f"{name:12s} {rep[name]['dur_s']:5.0f}s 창 {len(recs):3d} | 기울기 중앙 {rep[name]['pitch_med']:+5.1f}° (10~90% {rep[name]['pitch_p10']:+5.1f}~{rep[name]['pitch_p90']:+5.1f}) 롤 {rep[name]['roll_med']:+4.1f}° | depth 유효 {rep[name]['valid_med']:.0%} 위 1/3 유효 {rep[name]['top_valid_med']:.0%} 거리 중앙 {rep[name]['depth_med']:.2f}m", flush=True)
    W = 6; thumbs += [np.zeros_like(thumbs[0])] * (-len(thumbs) % W)
    lab = np.zeros((24, 256 * W, 3), np.uint8); cv2.putText(lab, name, (4, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    rows.append(np.vstack([lab] + [np.hstack(thumbs[i:i + W]) for i in range(0, len(thumbs), W)]))
    cv2.imwrite(f'{out}_{name}.jpg', rows[-1], [cv2.IMWRITE_JPEG_QUALITY, 80])
json.dump(rep, open(f'{out}.json', 'w'), indent=1)
