"""기준 회차 하나의 v0.5 정합을 묶음 맵(fems5)을 거쳐 다른 회차로 옮김.
타임스탬프로 단독 회차 키프레임 ↔ fems5 키프레임 대응 → 강체(Umeyama, 축척 없음) 단독→fems5 변환과 잔차(휨 정도)
T_fems5→v05 = T_ref_run @ inv(A_ref), 각 회차 T_run = T_fems5→v05 @ A_run → align_<run>_ms/T_src_to_ref.npy
사용: fems_transfer.py <ref_run> <T_ref.npy> [--ms fems5] [--runs fe422a,fe422b,fecen,feit1,feit1w,feone]"""
import sys, numpy as np
from pathlib import Path
a = sys.argv; ref, tref = a[1], np.load(a[2])
MS = a[a.index('--ms') + 1] if '--ms' in a else 'fems5'
RUNS = (a[a.index('--runs') + 1] if '--runs' in a else 'fe422a,fe422b,fecen,feit1,feit1w,feone').split(',')
E = Path.home() / 'b_slam/exports'
def load(r): x = np.loadtxt(E / r / 'traj_optimized_poses.txt', comments='#', ndmin=2); return x[:, 0], x[:, 1:4]
ts_m, p_m = load(MS)
def rigid(src, dst):
    cs, cd = src.mean(0), dst.mean(0); U, S, Vt = np.linalg.svd((src - cs).T @ (dst - cd)); D = np.eye(3); D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T; T = np.eye(4); T[:3, :3] = R; T[:3, 3] = cd - R @ cs; return T
A = {}
for r in RUNS:
    ts, p = load(r); sel = np.where((ts_m >= ts[0]) & (ts_m <= ts[-1]))[0]       # 이 회차 시간대의 fems5 키프레임
    gap = np.min(np.abs(ts[None, :] - ts_m[sel, None]), axis=1) if len(sel) else np.array([])
    sel = sel[gap < 1.5]                                                          # 단독 궤적 키프레임 사이(1.5s 안)만 보간
    if len(sel) < 10: print(r, '대응 부족', len(sel)); continue
    pi = np.stack([np.interp(ts_m[sel], ts, p[:, k]) for k in range(3)], 1); pm = p_m[sel]; ok = np.ones(len(sel), bool); j = sel
    T = rigid(pi, pm); res = np.linalg.norm(pi @ T[:3, :3].T + T[:3, 3] - pm, axis=1)
    yaw = np.degrees(np.arctan2(T[1, 0], T[0, 0])); A[r] = T
    print(f'{r:7s} 대응 {len(sel):3d}/{len(ts)} | 단독→{MS} 잔차(휨) 중앙 {np.median(res):.2f}m 95% {np.percentile(res, 95):.2f}m 최대 {res.max():.2f}m | 요 {yaw:+.1f}°')
Tms = tref @ np.linalg.inv(A[ref]); np.save(E / f'align_{MS}_ms_from_{ref}.npy', Tms)
print(f'\n{MS}→v0.5 (기준 {ref}) 저장. 각 회차 출발점(v0.5 좌표):')
for r in RUNS:
    if r not in A: continue
    Tr = Tms @ A[r]; d = E / f'align_{r}_ms'; d.mkdir(exist_ok=True); np.save(d / 'T_src_to_ref.npy', Tr)
    _, p = load(r); st = Tr[:3, :3] @ p[0] + Tr[:3, 3]; en = Tr[:3, :3] @ p[-1] + Tr[:3, 3]
    xf = E / f'align_{r}_x/T_src_to_ref.npy'; cmp = ''
    if xf.exists():
        Tx = np.load(xf); sx = Tx[:3, :3] @ p[0] + Tx[:3, 3]; cmp = f' | 사진 매칭(align_x) 출발점과 {np.linalg.norm(st[:2] - sx[:2]):.1f}m 차이'
    print(f'  {r:7s} 출발 ({st[0]:+6.1f},{st[1]:+6.1f}) 끝 ({en[0]:+6.1f},{en[1]:+6.1f}){cmp}')
