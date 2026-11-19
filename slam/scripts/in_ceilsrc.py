"""[천장판] 회차 키프레임마다 높이를 무엇으로 정했는지(in_localize_ceil.py 와 같은 규칙: 천장 1 > 바닥 2 > 없음 0 = 이웃 보간)를 x 2 m 구간별로 세고 npz 로 저장.
천장 · 바닥을 둘 다 본 키프레임이 있으면 '바닥 높이 + 천장 거리' = 천장 높이도 출력.
사용: in_ceilsrc.py <run(예 ic30b)> <export(예 ic30bF)> <out.npz> [--ex ~/b_slam/exports/indoor1119]"""
import sys, numpy as np
from pathlib import Path
a = sys.argv; E = Path.home() / 'b_slam/exports'; EXD = Path(a[a.index('--ex') + 1]) if '--ex' in a else E / 'indoor1119'
ex = np.load(EXD / f'explore_{a[1]}.npz')
ang_ = lambda X, Y: np.degrees(np.arccos(np.clip((X * Y).sum(-1), -1, 1)))
upi = ex['up_imu']; okc = np.isfinite(ex['ce_d']) & (ang_(ex['ce_n'], upi) < 8); okf0 = np.isfinite(ex['fl_h']) & (ang_(ex['fl_n'], upi) < 8)
src = np.where(okc, 1, np.where(okf0, 2, 0)); ids = ex['ids']
print(f'{a[1]}: 노드 {len(ids)} | 천장 {okc.sum()} · 바닥만 {(okf0 & ~okc).sum()} · 둘 다 {(okc & okf0).sum()} · 없음 {(src == 0).sum()}')
both = okc & okf0
if both.any():
    hc = ex['fl_h'][both] + ex['ce_d'][both]; print('  둘 다 본 키프레임 천장 높이: 중앙 %.3f · 25~75%% %.3f~%.3f (n %d)' % (np.median(hc), *np.percentile(hc, [25, 75]), len(hc)))
F = np.loadtxt(E / a[2] / 'traj_camera_id_camera_poses.txt', ndmin=2); fid = F[:, 8].astype(int); pos = {int(i): j for j, i in enumerate(ids)}
s = np.array([src[pos[i]] for i in fid])
for lo in range(-2, 46, 2):
    k = (F[:, 1] >= lo) & (F[:, 1] < lo + 2)
    if k.any(): print(f'  x {lo}~{lo+2}: 키프레임 {k.sum()} (천장 {(s[k]==1).sum()} · 바닥 {(s[k]==2).sum()} · 없음 {(s[k]==0).sum()}) | 카메라 z {F[k,3].min():.2f}~{F[k,3].max():.2f}')
np.savez(a[3], id=fid, src=s, xyz=F[:, 1:4]); print('→', a[3])
