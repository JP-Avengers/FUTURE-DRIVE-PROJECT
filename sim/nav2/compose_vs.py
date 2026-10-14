"""X-Mobility vs Nav2 나란히 시연 영상 — 두 시연 영상(1920×1080, 같은 배속)을 한 화면 좌우로 합성.
python -I compose_vs.py <왼쪽 X-Mobility.mp4> <오른쪽 Nav2.mp4> <출력.mp4> <제목> <왼쪽 결과 한 줄> <오른쪽 결과 한 줄>
각 영상에서 3인칭 화면 · 주행 수치 두 줄 · 앞 카메라 · 위에서 본 지도를 잘라 960 폭 칸에 다시 놓는다. 짧은 쪽은 마지막 장면을 멈춰 둔다.
"""
import sys
import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

L, R, OUT, TITLE, RES_L, RES_R = sys.argv[1:7]
REG = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
BOLD = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
f = lambda p, s: ImageFont.truetype(p, s, index=1)  # noqa: E731
FT, FH, FS, FE, FE2 = f(BOLD, 30), f(BOLD, 24), f(REG, 19), f(BOLD, 40), f(BOLD, 30)
BG = (24, 26, 30)
HEAD = [("X-Mobility — E2E 사전학습 모델(제로샷)", "카메라 1장 + 속도 + 앞쪽 경로 → 바로 주행 명령", (255, 210, 63)),
        ("Nav2 — ROS 2 표준 규칙 기반", "3D LiDAR + 지도로 위치 추정(AMCL) → 경로 계획 → 추종", (120, 220, 255))]


def panel(fr, k, d, base):
    """fr: 원본 BGR 프레임 → base(RGB PIL)의 k번째 칸(0 왼쪽 · 1 오른쪽)에 붙임."""
    x0 = k * 960
    rgb = cv2.cvtColor(fr, cv2.COLOR_BGR2RGB)
    third = cv2.resize(rgb[60:780, 0:1280], (960, 540), interpolation=cv2.INTER_AREA)
    stats = cv2.resize(rgb[792:912, 0:1280], (960, 90), interpolation=cv2.INTER_AREA)
    front = cv2.resize(rgb[60:540, 1280:1920], (440, 330), interpolation=cv2.INTER_AREA)
    mini = cv2.resize(rgb[625:1080, 1280:1920], (464, 330), interpolation=cv2.INTER_AREA)
    base.paste(Image.fromarray(third), (x0, 116))
    base.paste(Image.fromarray(stats), (x0, 656))
    base.paste(Image.fromarray(front), (x0 + 16, 748))
    base.paste(Image.fromarray(mini), (x0 + 480, 748))


def frame(fl, fr):
    base = Image.new("RGB", (1920, 1080), BG)
    d = ImageDraw.Draw(base)
    d.text((24, 10), TITLE, font=FT, fill=(255, 255, 255))
    for k, fr_ in enumerate((fl, fr)):
        name, sub, col = HEAD[k]
        d.rectangle((k * 960, 56, k * 960 + 959, 114), fill=(36, 39, 45))
        d.text((k * 960 + 16, 60), name, font=FH, fill=col)
        d.text((k * 960 + 16, 89), sub, font=FS, fill=(220, 220, 220))
        panel(fr_, k, d, base)
    d.line((960, 56, 960, 1080), fill=(90, 90, 90), width=3)
    return base


def is_end(fr):
    """원본의 끝 화면(가운데 검은 띠에 결과 글자) — 3인칭 화면 가운데 띠 왼쪽 끝이 거의 검정이면."""
    return fr[340:520, 0:30].mean() < 6


cl, cr = cv2.VideoCapture(L), cv2.VideoCapture(R)
fps = cl.get(cv2.CAP_PROP_FPS) or 30
w = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*"mp4v"), fps, (1920, 1080))
lastl = lastr = None
n = 0
while True:
    okl, fl = cl.read()
    okr, fr = cr.read()
    endl, endr = okl and is_end(fl), okr and is_end(fr)
    if okl and not endl:
        lastl = fl
    if okr and not endr:
        lastr = fr
    if (not okl or endl) and (not okr or endr):
        break
    img = frame(lastl, lastr)
    w.write(cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR))
    n += 1
    if n == 600:
        img.save(OUT.replace(".mp4", "_f600.jpg"), quality=88)
# 끝 화면 4 s — 두 결과를 나란히
end = frame(lastl, lastr)
d = ImageDraw.Draw(end)
d.rectangle((0, 380, 1920, 640), fill=(0, 0, 0))
d.text((40, 400), "같은 맵 · 같은 기준선 한 바퀴 결과", font=FE, fill=(255, 255, 255))
d.text((40, 470), "X-Mobility  " + RES_L, font=FE2, fill=HEAD[0][2])
d.text((40, 530), "Nav2  " + RES_R, font=FE2, fill=HEAD[1][2])
for _ in range(int(4 * fps)):
    w.write(cv2.cvtColor(np.asarray(end), cv2.COLOR_RGB2BGR))
    n += 1
end.save(OUT.replace(".mp4", "_end.jpg"), quality=90)
w.release()
print(f"VS_DONE frames={n} video_s={n / fps:.1f}")
