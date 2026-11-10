"""동기에게 보낼 손 촬영 경로 그리기판: v0.5 바닥 정사 영상(5m 격자·좌표·주요 지점 이름) + 아래쪽 안내 문구.
사용: route_sheet.py <out.png> <font.ttf>"""
import sys, numpy as np, cv2
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
OUT, FONT = sys.argv[1], sys.argv[2]; C = 0.1; SC = 2
X0, X1, Y0, Y1 = -62, 16, -36, 26
Q = np.load(Path.home() / 'b_slam/deliver/map_v0.5/outer/ground_ortho.npz'); qlo = Q['lo']
o = np.ascontiguousarray(np.transpose(Q['rgb'], (1, 0, 2))[::-1])           # RGB, 위 +y
H = o.shape[0]; c0, c1 = int((X0 - qlo[0]) / C), int((X1 - qlo[0]) / C); r0, r1 = H - int((Y1 - qlo[1]) / C), H - int((Y0 - qlo[1]) / C)
crop = np.zeros((r1 - r0, c1 - c0, 3), np.uint8); rs, cs = slice(max(r0, 0), min(r1, H)), slice(max(c0, 0), min(c1, o.shape[1]))
crop[rs.start - r0:rs.stop - r0, cs.start - c0:cs.stop - c0] = o[rs, cs]
empty = crop.sum(2) == 0; crop = np.clip(crop.astype(np.float32) * 1.15 + 10, 0, 255).astype(np.uint8); crop[empty] = (235, 235, 235)
img = Image.fromarray(cv2.resize(crop, None, fx=SC, fy=SC, interpolation=cv2.INTER_NEAREST)); W, Hh = img.size
d = ImageDraw.Draw(img); f_s = ImageFont.truetype(FONT, 15); f_m = ImageFont.truetype(FONT, 20); f_l = ImageFont.truetype(FONT, 28)
px = lambda x, y: ((x - X0) / C * SC, (Y1 - y) / C * SC)
for x in range(int(np.ceil(X0 / 5) * 5), X1 + 1, 5):
    u, _ = px(x, 0); d.line([(u, 0), (u, Hh)], fill=(90, 140, 220) if x % 10 == 0 else (160, 185, 230), width=1)
    if x % 10 == 0: d.text((u + 3, 3), f'{x}', font=f_s, fill=(30, 60, 160))
for y in range(int(np.ceil(Y0 / 5) * 5), Y1 + 1, 5):
    _, v = px(0, y); d.line([(0, v), (W, v)], fill=(90, 140, 220) if y % 10 == 0 else (160, 185, 230), width=1)
    if y % 10 == 0: d.text((3, v + 2), f'{y}', font=f_s, fill=(30, 60, 160))
marks = [('IT1 입구·계단', -8, -27.5), ('가운데 섬', -24, -9), ('북쪽 보도', -24, 17), ('북서 보도(가로수 구덩이)', -59, -2),
         ('남서 둥근 화단', -54, -21), ('동쪽 주차칸', 6, 3), ('남쪽 주차칸 줄', -38, -24)]
for t, x, y in marks:
    u, v = px(x, y); bb = d.textbbox((u, v), t, font=f_m); d.rectangle([bb[0] - 4, bb[1] - 2, bb[2] + 4, bb[3] + 2], fill=(255, 255, 255)); d.text((u, v), t, font=f_m, fill=(200, 20, 20))
u0, v0 = px(X0 + 2, Y0 + 2); d.line([(u0, v0), (u0 + 10 / C * SC, v0)], fill=(0, 0, 0), width=5); d.text((u0, v0 - 26), '10 m', font=f_m, fill=(0, 0, 0))
guide = ['11/8 손 촬영 경로 그려 주기 (FutureDrive · B 맵)',
         '1. 회차마다 색을 바꿔 걸은 길을 선으로 그려 주세요: 422wall, 422wall2, centerparking, IT1wall, It1west, onetake',
         '2. 출발점 ●, 끝점 ■, 방향을 바꾼(돌아선) 곳 ▲ 를 표시하고, 회차 이름을 옆에 적어 주세요.',
         '3. 지나간 곳의 특징을 짧게 적어 주세요.  예) "가로수 구덩이 3개 지나서 꺾음", "계단 앞에서 멈춤", "화단 따라 걸음"',
         '4. 대략이어도 괜찮아요. 확실한 구간은 실선, 기억이 애매한 구간은 점선으로 그려 주세요.',
         '5. 혹시 현장에 갈 일이 있으면 줄자로 몇 곳만 재 주세요(가장 도움이 됨):',
         '     · 주차장 연석 → 가장 가까운 가로수 구덩이 가장자리   · 북쪽 보도 폭(연석 → 화단)',
         '     · IT1 계단 윗변 → 가장 가까운 주차칸 선                  · 남서 둥근 화단 연석 → 옆 주차칸 선',
         '격자 한 칸 = 5 m (굵은 선 10 m, 숫자는 맵 좌표). 회색 빈 곳은 차량 촬영 자료가 없는 곳이라 그 위에 그려도 됩니다.']
pad = 18; lh = 34; panel = Image.new('RGB', (W, pad * 2 + lh * len(guide) + 10), (255, 255, 255)); pd = ImageDraw.Draw(panel)
for i, t in enumerate(guide): pd.text((pad, pad + i * lh), t, font=f_l if i == 0 else f_m, fill=(20, 20, 20) if i else (180, 0, 0))
sheet = Image.new('RGB', (W, Hh + panel.size[1]), (255, 255, 255)); sheet.paste(img, (0, 0)); sheet.paste(panel, (0, Hh)); sheet.save(OUT)
print(OUT, sheet.size)
