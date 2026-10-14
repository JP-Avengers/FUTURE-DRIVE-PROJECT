"""시나리오를 한 장에 — Nav2 경로 계획용 지도(화단 채움) 위에 기준선, 장애물 시나리오의 장애물(숫자), 구간 시나리오의 구간(글자).

    python3 eval/scenarios/plot_scenarios.py [eval/scenarios/parking_r3_*.yaml ...] --map data/nav2/parking_v0.4b_r3/map_keepout.yaml --out <png>
장애물 자리는 sim/make_obstacles.py 와 같은 계산(기준선 길이 s · 옆 오프셋 lateral). 구간은 기준선 바깥으로 띄운 굵은 선 + 화살표
(출발 점 → 목표), 겹치지 않게 1.3 · 2.1 · 2.9 · 3.7 m 로 번갈아 띄움.
"""
import argparse
import csv
import glob
import math

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont

ap = argparse.ArgumentParser()
ap.add_argument("scenarios", nargs="*")
ap.add_argument("--map", default="data/nav2/parking_v0.4b_r3/map_keepout.yaml")
ap.add_argument("--out", default="logs/nav2/scenarios_r3.png")
ap.add_argument("--box", default="-41,-25,4,4", help="자를 범위 x0,y0,x1,y1 (m)")
ap.add_argument("--scale", type=int, default=3, help="0.05 m 칸 하나를 몇 px 로")
a = ap.parse_args()
files = a.scenarios or (sorted(glob.glob("eval/scenarios/parking_r3_obs_*.yaml")) + sorted(glob.glob("eval/scenarios/parking_r3_straight_*.yaml"))
                        + sorted(glob.glob("eval/scenarios/parking_r3_curve_*.yaml")))

my = yaml.safe_load(open(a.map))
res, (ox, oy) = float(my["resolution"]), [float(v) for v in my["origin"][:2]]
raw = open(a.map.rsplit("/", 1)[0] + "/" + my["image"], "rb").read()
hdr = raw.split(b"\n", 3); W, H = map(int, hdr[1].split())
img = np.frombuffer(hdr[3], np.uint8).reshape(H, W)
x0, y0, x1, y1 = (float(v) for v in a.box.split(","))
c0, c1 = int((x0 - ox) / res), int((x1 - ox) / res)
r0, r1 = H - 1 - int((y1 - oy) / res), H - 1 - int((y0 - oy) / res)
crop = img[r0:r1, c0:c1]
rgb = np.stack([crop] * 3, -1).astype(np.uint8)
rgb[crop == 205] = (225, 228, 233)
rgb[crop == 254] = (250, 250, 248)
rgb[crop == 0] = (120, 124, 130)
im = Image.fromarray(rgb).resize((rgb.shape[1] * a.scale, rgb.shape[0] * a.scale), Image.NEAREST)
d = ImageDraw.Draw(im)


def px(x, y):
    return ((x - ox) / res - c0) * a.scale, (H - 1 - (y - oy) / res - r0) * a.scale


try:
    font = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc", 26, index=1)
    small = ImageFont.truetype("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc", 20, index=1)
except OSError:
    font = small = ImageFont.load_default()

route = None
legend = []   # (글, 색)
SEGCOL = [(40, 90, 200), (30, 150, 70), (130, 60, 170), (0, 140, 140), (200, 120, 0)]
k = n_seg = 0
for f in files:
    sc = yaml.safe_load(open(f))
    R = np.array([(float(r["s"]), float(r["x"]), float(r["y"])) for r in csv.DictReader(open(sc["route"])) if r.get("run", "0") == "0"])
    if route is None:
        route = R
        pts = [px(x, y) for _, x, y in R]
        d.line(pts, fill=(205, 40, 40), width=3)
        sx, sy = px(R[0, 1], R[0, 2])
        d.ellipse([sx - 9, sy - 9, sx + 9, sy + 9], outline=(205, 40, 40), width=3)
        d.text((sx + 12, sy - 14), "출발", font=small, fill=(205, 40, 40))
    if "s" in (sc.get("goal") or {}):   # 구간 시나리오 — 시계는 진행 방향 왼쪽(+), 반시계는 오른쪽(−)이 바깥
        lab, col = chr(ord("A") + n_seg), SEGCOL[n_seg % len(SEGCOL)]
        off = (1.3 + 0.8 * (n_seg % 4)) * (1 if sc.get("direction", "cw") == "cw" else -1)
        P = R[(R[:, 0] >= float(sc["start"]["s"])) & (R[:, 0] <= float(sc["goal"]["s"]))]
        th = np.arctan2(np.gradient(P[:, 2]), np.gradient(P[:, 1]))
        pts = [px(x - off * math.sin(q), y + off * math.cos(q)) for (_, x, y), q in zip(P, th)]
        d.line(pts, fill=col, width=7)
        (ex, ey), (bx, by) = pts[-1], pts[-4]
        g = math.atan2(ey - by, ex - bx)
        d.polygon([(ex + 18 * math.cos(g), ey + 18 * math.sin(g)), (ex + 14 * math.cos(g + 2.4), ey + 14 * math.sin(g + 2.4)),
                   (ex + 14 * math.cos(g - 2.4), ey + 14 * math.sin(g - 2.4))], fill=col)
        sx, sy = pts[0]
        d.ellipse([sx - 8, sy - 8, sx + 8, sy + 8], fill=col)
        d.text((sx + 10, sy - 34), lab, font=font, fill=col)
        legend.append((f"{lab} {sc['name'].replace('parking_r3_', '')} — {sc.get('direction', 'cw')} s {sc['start']['s']} → {sc['goal']['s']}", col))
        n_seg += 1
        continue
    k += 1
    for o in sc["obstacles"]:
        i = int(np.clip(np.searchsorted(R[:, 0], float(o["s"])), 1, len(R) - 1))
        (s0, xa, ya), (s1, xb, yb) = R[i - 1], R[i]
        t = (float(o["s"]) - s0) / max(s1 - s0, 1e-9); th = math.atan2(yb - ya, xb - xa); lat = float(o.get("lateral", 0.0))
        x, y = xa + t * (xb - xa) - lat * math.sin(th), ya + t * (yb - ya) + lat * math.cos(th)
        col = tuple(int(255 * c) for c in o.get("color", [0.55, 0.38, 0.22]))
        cx, cy = px(x, y); r = max(float(o["size"][0]) / 2 / res * a.scale, 7)
        if o.get("type", "box") == "box":
            hx, hy = float(o["size"][0]) / 2, float(o["size"][1]) / 2
            yaw = th + math.radians(float(o.get("yaw_deg", 0.0)))
            poly = [px(x + u * math.cos(yaw) - v * math.sin(yaw), y + u * math.sin(yaw) + v * math.cos(yaw)) for u, v in [(hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy)]]
            d.polygon(poly, fill=col, outline=(20, 20, 20))
        else:
            d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col, outline=(20, 20, 20))
        d.text((cx + 10, cy - 34), str(k), font=font, fill=(20, 20, 20))
    legend.append((f"{k} {sc['name'].replace('parking_r3_obs_', '')} — 장애물 {len(sc['obstacles'])}개", (20, 20, 20)))
d.rectangle([4, 4, 10 + max(d.textlength(s_, font=small) for s_, _ in legend) + 8, 12 + 26 * len(legend)], fill=(255, 255, 255), outline=(160, 160, 160))
y = 8
for line, col in legend:
    d.text((10, y), line, font=small, fill=col); y += 26
im.save(a.out)
print("saved", a.out, im.size)
