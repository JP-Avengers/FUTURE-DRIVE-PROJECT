"""최종 대응점 목록 조립: 다듬은 결과(cps_v1_ref, cps_arc_ref)에서 확인한 것만 + 섬(fecen) 자동 ±1.2m 창(선명·경계 아님) → cps_final.json"""
import json, sys
from pathlib import Path
D = Path.home() / 'b_slam/exports/cp_nr'; v1 = {c['name']: c for c in json.load(open(D / 'cps_v1_ref.json', encoding='utf-8'))}
arc = {c['name']: c for c in json.load(open(D / 'cps_arc_ref.json', encoding='utf-8'))}
out = []
def add(c, **kw): c = dict(c); c.update(kw); out.append(c)
add(v1['grate'], w=1.0, group='grate'); add(v1['islSW'], w=0.5, group='islSW'); add(v1['islS_b'], w=1.0, group='islS_b')
for s in ('s0', 's3', 's4'): add(v1[f'pits_{s}'], w=1.0, group='pits')
add(arc['arc_s0'], w=0.7, group='arc'); add(arc['arc_s3'], w=0.7, group='arc')
for s in ('s0', 's1'): add(v1[f'curbS_{s}'], w=1.0, group='curbS')
for s in ('s0', 's5'): add(v1[f'stair_{s}'], w=1.0, group='stair')
for s in ('s0', 's3', 's4'):                                   # 북쪽 보도: 상관이 약해 사람이 읽은 값(-3.5m) 그대로, 가중치 낮게
    c = dict(v1[f'Nwalk_{s}']); c['off_n'] = -3.5; c['dx'], c['dy'] = 0.35, -3.48; add(c, w=0.3, group='Nwalk')
auto = json.load(open(Path.home() / 'b_slam/exports/cp_diag/r12/fems5_s2_fecen_cp_local.json', encoding='utf-8'))
k = 0
for r in auto:
    if 'dx' in r and not r['edge'] and r['r_best'] - r['second'] >= 0.05 and r['r_best'] >= 0.2:
        out.append(dict(name=f'isl_auto{k}', bev='fems5_s2_fecen', x=r['x'], y=r['y'], dx=r['dx'], dy=r['dy'], type='point', w=0.5, group=f'isl_auto{k}')); k += 1
json.dump(out, open(D / 'cps_final.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
print('대응점', len(out), '(섬 자동', k, ')'); [print(f"  {c['name']:10s} {c['type']:5s} w{c['w']} " + (f"d=({c['dx']:+.2f},{c['dy']:+.2f})" if c['type'] == 'point' else f"n방향 {c['off_n']:+.2f}")) for c in out]
