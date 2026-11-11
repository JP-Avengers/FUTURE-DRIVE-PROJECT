"""차 배치 화면 만들기 — 빌드 폴더의 바닥 텍스처 위에서 차를 끌어 놓고 돌린 뒤 파일로 저장하는 HTML 한 장 (브라우저에서 엶, 인터넷 불필요).

    python recon/tools/car_editor.py --out <빌드 폴더> [--cars <처음 차 목록.json>] [--html <저장할 곳>]

  - 바탕 = <out>/floor_tex/floor_color.jpg (4 cm/화소로 줄여 넣음), 주차칸 = <out>/floor_tex/stalls.json, 촬영 경로 = <out>/preview_route.csv
  - 처음 차 = --cars (없으면 <out>/report.json 의 car_models.slots_xy_yawdeg_l_h)
  - 화면에서 '저장' → cars_manual.json (내려받기 폴더). 그 파일을 설정의 cars.models.manual_file 에 적으면 빌드가 그대로 놓음 (pipeline/cars.py)
  - 파일 형식: {"cars": [{"x", "y", "yaw_deg", "length", "height", "color"}]} — 좌표는 맵 좌표(m), yaw 는 차 길이 방향
보완(만든 것): 손으로 놓은 차는 측정이 아님 — README 에 밝힐 것.
"""
import argparse
import base64
import io
import json
import os

import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None
COLORS = {"white": [0.85, 0.85, 0.83], "silver": [0.62, 0.63, 0.64], "gray": [0.35, 0.36, 0.38], "black": [0.10, 0.10, 0.11],
          "blue": [0.18, 0.24, 0.36], "red": [0.45, 0.12, 0.12]}

HTML = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><title>차 배치</title>
<style>
 html,body{margin:0;height:100%;background:#1c1d1f;color:#e8e8e8;font:14px/1.5 "Malgun Gothic",system-ui,sans-serif;overflow:hidden}
 #bar{position:fixed;left:0;top:0;right:0;height:46px;background:#2a2c30;display:flex;align-items:center;gap:8px;padding:0 12px;z-index:2;border-bottom:1px solid #444}
 #bar button,#bar select,#bar input{font:inherit;background:#3a3d42;color:#eee;border:1px solid #555;border-radius:5px;padding:4px 10px}
 #bar button:hover{background:#4a4e55;cursor:pointer}
 #bar .pri{background:#2f6fd0;border-color:#2f6fd0}
 #bar label{display:flex;align-items:center;gap:4px}
 #bar input[type=number]{width:58px}
 #info{margin-left:auto;color:#bbb;white-space:nowrap}
 #help{position:fixed;right:12px;bottom:12px;background:#2a2c30e6;border:1px solid #444;border-radius:8px;padding:10px 14px;max-width:330px;z-index:2;font-size:13px}
 #help b{color:#fff} #help kbd{background:#444;border-radius:3px;padding:0 5px}
 canvas{position:fixed;left:0;top:46px;cursor:grab}
 #warn{color:#ffb84d}
</style></head><body>
<div id="bar">
 <button id="add">차 추가</button><button id="del">지우기</button>
 <label>길이<input id="len" type="number" min="3" max="6" step="0.1" value="4.6">m</label>
 <label>색<select id="col"></select></label>
 <label><input id="snap" type="checkbox" checked>주차칸에 붙이기</label>
 <label><input id="guide" type="checkbox" checked>주차칸 표시</label>
 <button id="load">불러오기</button><input id="file" type="file" accept=".json" style="display:none">
 <button id="save" class="pri">저장 (cars_manual.json)</button>
 <span id="info"></span>
</div>
<canvas id="cv"></canvas>
<div id="help">
 <b>쓰는 법</b><br>
 차를 <b>끌어서</b> 옮김 · 차 앞쪽 <b>동그라미</b>를 끌어 돌림<br>
 <kbd>Q</kbd> <kbd>E</kbd> 5°씩 돌림 (<kbd>Shift</kbd> 1°) · <kbd>R</kbd> 90° 돌림<br>
 <kbd>Del</kbd> 지우기 · <kbd>Ctrl</kbd>+<kbd>Z</kbd> 되돌리기 · <kbd>Ctrl</kbd>+<kbd>D</kbd> 복사<br>
 빈 곳 <b>두 번 누름</b> = 차 추가 (주차칸이면 칸에 맞춰)<br>
 점선 차(뺀 차) <b>두 번 누름</b> = 다시 넣기<br>
 바탕을 끌면 이동 · 휠로 확대<br>
 <span id="warn"></span>
</div>
<script>
const D = __DATA__;
const cv = document.getElementById('cv'), g = cv.getContext('2d');
const img = new Image(); img.src = D.image;
const W_CAR = D.width_m, PAL = D.colors;
let cars = D.cars.map(c => Object.assign({on: true, height: 1.5, color: 'white'}, c));
let sel = -1, view = {s: 1, ox: 0, oy: 0}, drag = null, undo = [];
const colSel = document.getElementById('col');
for (const k in PAL) { const o = document.createElement('option'); o.value = k; o.textContent = {white:'흰색',silver:'은색',gray:'회색',black:'검정',blue:'파랑',red:'빨강'}[k] || k; colSel.appendChild(o); }
try { const s = localStorage.getItem('car_editor_' + D.name); if (s) { const j = JSON.parse(s); if (j && j.length && confirm('지난번에 하던 배치가 있습니다. 이어서 할까요?')) cars = j; } } catch (e) {}
function keep() { try { localStorage.setItem('car_editor_' + D.name, JSON.stringify(cars)); } catch (e) {} }
function push() { undo.push(JSON.stringify(cars)); if (undo.length > 100) undo.shift(); }
// 맵 좌표(m) ↔ 화면
function w2s(x, y) { return [(x - D.x0) / D.mpp * view.s + view.ox, (D.y1 - y) / D.mpp * view.s + view.oy]; }
function s2w(px, py) { return [D.x0 + (px - view.ox) / view.s * D.mpp, D.y1 - (py - view.oy) / view.s * D.mpp]; }
function fit() { cv.width = innerWidth; cv.height = innerHeight - 46; if (!fit.done && img.width) { view.s = Math.min(cv.width / img.width, cv.height / img.height); view.ox = (cv.width - img.width * view.s) / 2; view.oy = (cv.height - img.height * view.s) / 2; fit.done = true; } draw(); }
function corners(c, L, Wd) { const a = c.yaw_deg * Math.PI / 180, u = [Math.cos(a), Math.sin(a)], v = [-u[1], u[0]]; L = L || c.length; Wd = Wd || W_CAR;
  return [[-1,-1],[1,-1],[1,1],[-1,1]].map(k => [c.x + k[0]*u[0]*L/2 + k[1]*v[0]*Wd/2, c.y + k[0]*u[1]*L/2 + k[1]*v[1]*Wd/2]); }
function handle(c) { const a = c.yaw_deg * Math.PI / 180; return [c.x + Math.cos(a) * (c.length / 2 + 0.7), c.y + Math.sin(a) * (c.length / 2 + 0.7)]; }
function inside(c, x, y) { const a = c.yaw_deg * Math.PI / 180, dx = x - c.x, dy = y - c.y, p = dx*Math.cos(a) + dy*Math.sin(a), q = -dx*Math.sin(a) + dy*Math.cos(a); return Math.abs(p) < c.length/2 && Math.abs(q) < W_CAR/2; }
function pathDist(c) { let m = 1e9; for (const p of D.path) { const a = c.yaw_deg*Math.PI/180, dx = p[0]-c.x, dy = p[1]-c.y, u = Math.abs(dx*Math.cos(a)+dy*Math.sin(a)) - c.length/2, v = Math.abs(-dx*Math.sin(a)+dy*Math.cos(a)) - W_CAR/2; m = Math.min(m, Math.hypot(Math.max(u,0), Math.max(v,0))); } return m; }
function overlap(a, b) { // 분리축 검사
  const A = corners(a), B = corners(b);
  for (const P of [A, B]) for (let i = 0; i < 2; i++) { const ax = [P[i+1][0]-P[i][0], P[i+1][1]-P[i][1]]; const pa = A.map(p => p[0]*ax[0]+p[1]*ax[1]), pb = B.map(p => p[0]*ax[0]+p[1]*ax[1]); if (Math.max(...pa) <= Math.min(...pb) || Math.max(...pb) <= Math.min(...pa)) return false; } return true; }
function poly(pts) { g.beginPath(); pts.forEach((p, i) => { const s = w2s(p[0], p[1]); i ? g.lineTo(s[0], s[1]) : g.moveTo(s[0], s[1]); }); g.closePath(); }
function draw() {
  g.setTransform(1,0,0,1,0,0); g.fillStyle = '#1c1d1f'; g.fillRect(0,0,cv.width,cv.height);
  g.imageSmoothingEnabled = view.s < 3; g.drawImage(img, view.ox, view.oy, img.width*view.s, img.height*view.s);
  if (document.getElementById('guide').checked) { g.lineWidth = 1; g.strokeStyle = 'rgba(90,200,255,.55)';
    for (const s of D.stalls) { poly(corners({x:s[0], y:s[1], yaw_deg:s[2]*180/Math.PI}, s[4], s[3])); g.stroke(); } }
  g.lineWidth = 1.5; g.strokeStyle = 'rgba(80,255,120,.8)'; g.setLineDash([6,4]); g.beginPath(); D.path.forEach((p,i) => { const s = w2s(p[0],p[1]); i ? g.lineTo(s[0],s[1]) : g.moveTo(s[0],s[1]); }); g.stroke(); g.setLineDash([]);
  let nOn = 0, nBad = 0, nNear = 0;
  cars.forEach((c, i) => {
    poly(corners(c));
    if (!c.on) { g.setLineDash([5,4]); g.strokeStyle = 'rgba(255,255,255,.45)'; g.lineWidth = 1.2; g.stroke(); g.setLineDash([]); return; }
    nOn++;
    const bad = cars.some((o, j) => j !== i && o.on && overlap(c, o)), near = pathDist(c) < D.path_clear_m;
    if (bad) nBad++; if (near) nNear++;
    const rgb = PAL[c.color] || PAL.white; g.fillStyle = `rgba(${rgb.map(v => Math.round(v*255)).join(',')},.88)`; g.fill();
    g.lineWidth = i === sel ? 3 : 1.5; g.strokeStyle = bad ? '#ff4d4d' : near ? '#ffb84d' : i === sel ? '#ffe14d' : '#000'; g.stroke();
    const a = c.yaw_deg*Math.PI/180, f = w2s(c.x + Math.cos(a)*c.length*0.32, c.y + Math.sin(a)*c.length*0.32), ct = w2s(c.x, c.y);   // 앞쪽 표시
    g.beginPath(); g.moveTo(ct[0], ct[1]); g.lineTo(f[0], f[1]); g.strokeStyle = 'rgba(255,255,255,.7)'; g.lineWidth = 1; g.stroke();
    if (i === sel) { const h = handle(c), hs = w2s(h[0], h[1]); g.beginPath(); g.arc(hs[0], hs[1], 7, 0, 7); g.fillStyle = '#ffe14d'; g.fill(); g.strokeStyle = '#000'; g.stroke(); }
  });
  document.getElementById('info').textContent = `차 ${nOn}대` + (sel >= 0 && cars[sel] ? ` · 고른 차 (${cars[sel].x.toFixed(1)}, ${cars[sel].y.toFixed(1)}) ${((cars[sel].yaw_deg%360+360)%360).toFixed(0)}°` : '');
  document.getElementById('warn').textContent = (nBad ? `빨강 = 서로 겹친 차 ${nBad}대. ` : '') + (nNear ? `주황 = 촬영 경로(초록 점선)에서 ${D.path_clear_m} m 안 ${nNear}대 — 로봇 길을 막을 수 있음` : '');
}
function pick(x, y) { for (let i = cars.length - 1; i >= 0; i--) if (inside(cars[i], x, y)) return i; return -1; }
function stallAt(x, y) { let b = null; for (const s of D.stalls) { const c = {x:s[0], y:s[1], yaw_deg:s[2]*180/Math.PI, length:s[4]}; const a = s[2], dx = x-s[0], dy = y-s[1]; if (Math.abs(dx*Math.cos(a)+dy*Math.sin(a)) < s[4]/2 && Math.abs(-dx*Math.sin(a)+dy*Math.cos(a)) < s[3]/2) b = s; } return b; }
function snapTo(c) { if (!document.getElementById('snap').checked) return; const s = stallAt(c.x, c.y); if (!s) return;
  let d = s[2]*180/Math.PI; const diff = ((c.yaw_deg - d) % 360 + 540) % 360 - 180; if (Math.abs(diff) > 90) d += 180;   // 머리 방향은 가까운 쪽
  const a = s[2], dx = c.x - s[0], dy = c.y - s[1], along = dx*Math.cos(a) + dy*Math.sin(a), lim = Math.max(0, (s[4] - c.length)/2 - 0.2), t = Math.max(-lim, Math.min(lim, along));
  c.x = s[0] + t*Math.cos(a); c.y = s[1] + t*Math.sin(a); c.yaw_deg = d; }
function mouse(e) { const r = cv.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }
cv.addEventListener('mousedown', e => { const [px, py] = mouse(e), [x, y] = s2w(px, py);
  if (sel >= 0 && cars[sel] && cars[sel].on) { const h = handle(cars[sel]), hs = w2s(h[0], h[1]); if (Math.hypot(hs[0]-px, hs[1]-py) < 11) { push(); drag = {t:'rot'}; return; } }
  const i = pick(x, y);
  if (i >= 0 && cars[i].on) { sel = i; push(); drag = {t:'move', dx: x - cars[i].x, dy: y - cars[i].y}; document.getElementById('len').value = cars[i].length.toFixed(1); colSel.value = cars[i].color; }
  else { if (i < 0) sel = -1; drag = {t:'pan', px, py, ox: view.ox, oy: view.oy}; cv.style.cursor = 'grabbing'; }
  draw(); });
addEventListener('mousemove', e => { if (!drag) return; const [px, py] = mouse(e), [x, y] = s2w(px, py);
  if (drag.t === 'pan') { view.ox = drag.ox + px - drag.px; view.oy = drag.oy + py - drag.py; }
  else if (drag.t === 'move') { cars[sel].x = x - drag.dx; cars[sel].y = y - drag.dy; }
  else if (drag.t === 'rot') { let a = Math.atan2(y - cars[sel].y, x - cars[sel].x) * 180 / Math.PI; if (!e.shiftKey) a = Math.round(a / 5) * 5; cars[sel].yaw_deg = a; }
  draw(); });
addEventListener('mouseup', () => { if (drag && drag.t === 'move') snapTo(cars[sel]); if (drag && drag.t !== 'pan') keep(); drag = null; cv.style.cursor = 'grab'; draw(); });
cv.addEventListener('dblclick', e => { const [px, py] = mouse(e), [x, y] = s2w(px, py), i = pick(x, y); push();
  if (i >= 0) { cars[i].on = !cars[i].on ? true : cars[i].on; sel = i; }
  else { const c = {x, y, yaw_deg: 0, length: +document.getElementById('len').value || 4.6, height: 1.5, color: colSel.value, on: true}; const s = stallAt(x, y); if (s) { c.x = s[0]; c.y = s[1]; c.yaw_deg = s[2]*180/Math.PI; } cars.push(c); sel = cars.length - 1; }
  keep(); draw(); });
cv.addEventListener('wheel', e => { e.preventDefault(); const [px, py] = mouse(e), k = e.deltaY < 0 ? 1.15 : 1/1.15, s = Math.max(0.2, Math.min(40, view.s * k)); view.ox = px - (px - view.ox) * s / view.s; view.oy = py - (py - view.oy) * s / view.s; view.s = s; draw(); }, {passive: false});
addEventListener('keydown', e => { if (e.target.tagName === 'INPUT' && e.target.type === 'number') return;
  if (e.ctrlKey && e.key.toLowerCase() === 'z') { if (undo.length) { cars = JSON.parse(undo.pop()); sel = -1; keep(); draw(); } e.preventDefault(); return; }
  if (sel < 0 || !cars[sel]) return; const c = cars[sel], st = e.shiftKey ? 1 : 5;
  if (e.ctrlKey && e.key.toLowerCase() === 'd') { push(); const a = (c.yaw_deg + 90) * Math.PI / 180; cars.push(Object.assign({}, c, {x: c.x + Math.cos(a)*2.5, y: c.y + Math.sin(a)*2.5})); sel = cars.length - 1; e.preventDefault(); }
  else if (e.key === 'q' || e.key === 'Q') { push(); c.yaw_deg += st; } else if (e.key === 'e' || e.key === 'E') { push(); c.yaw_deg -= st; }
  else if (e.key === 'r' || e.key === 'R') { push(); c.yaw_deg += 90; }
  else if (e.key === 'Delete' || e.key === 'Backspace') { push(); cars.splice(sel, 1); sel = -1; }
  else if (e.key.startsWith('Arrow')) { push(); const d = e.shiftKey ? 0.02 : 0.1; c.x += e.key === 'ArrowRight' ? d : e.key === 'ArrowLeft' ? -d : 0; c.y += e.key === 'ArrowUp' ? d : e.key === 'ArrowDown' ? -d : 0; e.preventDefault(); }
  else return; keep(); draw(); });
document.getElementById('add').onclick = () => { push(); const [x, y] = s2w(cv.width/2, cv.height/2); cars.push({x, y, yaw_deg: 0, length: +document.getElementById('len').value || 4.6, height: 1.5, color: colSel.value, on: true}); sel = cars.length - 1; keep(); draw(); };
document.getElementById('del').onclick = () => { if (sel >= 0) { push(); cars.splice(sel, 1); sel = -1; keep(); draw(); } };
document.getElementById('len').onchange = e => { if (sel >= 0) { push(); cars[sel].length = Math.max(3, Math.min(6, +e.target.value || 4.6)); keep(); draw(); } };
colSel.onchange = () => { if (sel >= 0) { push(); cars[sel].color = colSel.value; keep(); draw(); } };
document.getElementById('guide').onchange = draw;
document.getElementById('load').onclick = () => document.getElementById('file').click();
document.getElementById('file').onchange = e => { const f = e.target.files[0]; if (!f) return; f.text().then(t => { try { const j = JSON.parse(t); push(); cars = (j.cars || j).map(c => Object.assign({on: true, height: 1.5, color: 'white'}, c)); sel = -1; keep(); draw(); } catch (err) { alert('읽을 수 없는 파일입니다'); } }); e.target.value = ''; };
document.getElementById('save').onclick = () => { const out = {name: D.name, note: '손으로 놓은 차 (recon/tools/car_editor.py) — 맵 좌표 m, yaw_deg = 차 길이 방향', width_m: W_CAR,
    cars: cars.filter(c => c.on).map(c => ({x: +c.x.toFixed(3), y: +c.y.toFixed(3), yaw_deg: +(((c.yaw_deg % 360) + 360) % 360).toFixed(1), length: +c.length.toFixed(2), height: c.height || 1.5, color: c.color}))};
  const a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([JSON.stringify(out, null, 1)], {type: 'application/json'})); a.download = 'cars_manual.json'; a.click(); setTimeout(() => URL.revokeObjectURL(a.href), 2000); };
img.onload = fit; addEventListener('resize', fit);
</script></body></html>
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, help="빌드 폴더")
    ap.add_argument("--cars", help="처음 차 목록 json ({cars: [{x, y, yaw_deg, length, height, color, on}]})")
    ap.add_argument("--html", help="저장할 곳 (기본 <out>/car_editor.html)")
    ap.add_argument("--mpp", type=float, default=0.04, help="바탕 그림 화소 크기 m")
    a = ap.parse_args()
    out = os.path.expanduser(a.out)
    mt = np.load(os.path.join(out, "floor_tex", "bake_meta.npz"))
    x0, x1, y0, y1 = (float(v) for v in mt["region"])
    im = Image.open(os.path.join(out, "floor_tex", "floor_color.jpg")).convert("RGB")
    im = im.resize((int(round((x1 - x0) / a.mpp)), int(round((y1 - y0) / a.mpp))), Image.LANCZOS)
    buf = io.BytesIO()
    im.save(buf, "JPEG", quality=82)
    sj = os.path.join(out, "floor_tex", "stalls.json")
    stalls = json.load(open(sj, encoding="utf-8"))["stalls"] if os.path.exists(sj) else []
    rp = os.path.join(out, "preview_route.csv")
    path = [[float(v) for v in ln.split(",")[:2]] for ln in open(rp).read().split("\n")[1:] if ln.strip()][::2] if os.path.exists(rp) else []
    rep = json.load(open(os.path.join(out, "report.json"), encoding="utf-8"))
    cm = rep.get("car_models", {})
    if a.cars:
        cars = json.load(open(a.cars, encoding="utf-8"))
        cars = cars.get("cars", cars)
    else:
        rng = np.random.default_rng(0)
        names = list(COLORS)
        cars = [dict(x=x, y=y, yaw_deg=yd, length=L, height=H, color=names[int(rng.integers(len(names)))], on=True) for x, y, yd, L, H in cm.get("slots_xy_yawdeg_l_h", [])]
    data = dict(name=rep.get("name", "map"), image="data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode(), x0=x0, y1=y1, mpp=a.mpp,
                stalls=stalls, path=path, cars=cars, colors=COLORS, width_m=cm.get("width_m", 1.8), path_clear_m=cm.get("path_clear_m", 1.0))
    html = a.html or os.path.join(out, "car_editor.html")
    open(html, "w", encoding="utf-8").write(HTML.replace("__DATA__", json.dumps(data, ensure_ascii=False)))
    print(f"차 배치 화면: {html} ({os.path.getsize(html) / 1e6:.1f} MB) · 차 {len(cars)}대 · 주차칸 {len(stalls)}개")


if __name__ == "__main__":
    main()
