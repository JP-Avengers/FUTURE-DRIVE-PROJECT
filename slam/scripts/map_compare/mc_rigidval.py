"""독립 검증 자세: 검증 회차의 RTAB-Map 원래 자세(조각 1개, 광학 좌표, IMU 중력)를 강체 변환 '하나'로 건물 좌표에 놓기.
 in1rL 은 키프레임마다 기준 맵 벽에 맞춰 놓았으므로 맵과 같이 휜 부분은 depth 일치로 드러나지 않음 → 여기서는 키프레임별 맞춤 없이
 원래 자세 위치를 in1rL 위치에 강건하게(이상치 잘라 가며) 한 번만 맞춤.
  rigid4  : 요 + 3D 이동 (원래 자세의 IMU 중력을 믿음)
  rigid4zL: rigid4 에서 카메라 높이 z 만 in1rL 값으로 바꿈(원래 RTAB-Map 높이가 흐르는 것을 뺀 판 — 높이는 독립이 아님)
  rigid6  : 회전 3축(롤 · 피치 포함, 키프레임별 자세 차이의 평균 회전) + 이동
 출력: <out>/<raw>_{rigid4,rigid4zL,rigid6}.txt (in1rL 과 같은 형식), rigidval.json(변환 · 잔차 · x 5 m 칸별 위치 차), rigidval_pos.png
사용: mc_rigidval.py <raw_run in1r> <placed_run in1rL> <out_dir>"""
import sys, os, json, numpy as np
from pathlib import Path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mc_lib import load_poses, EXPORTS
from scipy.spatial.transform import Rotation as Rot
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt

raw_run, pl_run, O = sys.argv[1], sys.argv[2], Path(sys.argv[3]); O.mkdir(parents=True, exist_ok=True)
A = load_poses(EXPORTS / raw_run / 'traj_camera_id_camera_poses.txt'); B = load_poses(EXPORTS / pl_run / 'traj_camera_id_camera_poses.txt')
ib = {int(r[8]): r for r in B}; A = np.array([r for r in A if int(r[8]) in ib]); B = np.array([ib[int(r[8])] for r in A])
Pa, Pb = A[:, 1:4], B[:, 1:4]; Ra = Rot.from_quat(A[:, 4:8]).as_matrix(); Rb = Rot.from_quat(B[:, 4:8]).as_matrix()


def trimmed(fit, n_it=10):
    m = np.ones(len(Pa), bool)
    for _ in range(n_it):
        R, t = fit(m); r = np.linalg.norm(Pb - (Pa @ R.T + t), axis=1); m2 = r < max(2.5 * np.median(r[m]), 0.05)
        if (m2 == m).all(): break
        m = m2
    return R, t, m, r


def fit4(m):
    a = Pa[m] - Pa[m].mean(0); b = Pb[m] - Pb[m].mean(0); yaw = np.arctan2((a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum(), (a[:, :2] * b[:, :2]).sum())
    R = Rot.from_euler('z', yaw).as_matrix(); return R, (Pb[m] - Pa[m] @ R.T).mean(0)


def fit6(m):
    U, _, Vt = np.linalg.svd((Rb[m] @ np.transpose(Ra[m], (0, 2, 1))).sum(0)); R = U @ np.diag([1, 1, np.linalg.det(U @ Vt)]) @ Vt   # 자세 차이의 평균 회전(chordal)
    return R, (Pb[m] - Pa[m] @ R.T).mean(0)


out = {}; xs = Pb[:, 0]; bins = np.arange(np.floor(xs.min() / 5) * 5, xs.max() + 5, 5)
for name, fit in (('rigid4', fit4), ('rigid6', fit6)):
    R, t, m, r = trimmed(fit); P = Pa @ R.T + t; Q = Rot.from_matrix(R @ Ra).as_quat(); eul = Rot.from_matrix(R).as_euler('ZYX', degrees=True)
    variants = {name: P} if name == 'rigid6' else {name: P, 'rigid4zL': np.c_[P[:, :2], Pb[:, 2]]}
    for vn, PP in variants.items():
        np.savetxt(O / f'{raw_run}_{vn}.txt', np.c_[A[:, 0], PP, Q, A[:, 8]], fmt='%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d',
                   header=f'timestamp x y z qx qy qz qw id ({raw_run} RTAB-Map 원래 자세를 강체 변환 하나로 건물 좌표에 놓음: {vn}, 맞춘 대상 {pl_run} 위치)')
        d = PP - Pb; rows = []
        for x0 in bins[:-1]:
            k = (xs >= x0) & (xs < x0 + 5)
            if k.any(): rows.append(dict(x0=float(x0), n=int(k.sum()), dx_cm=float(np.median(d[k, 0]) * 100), dy_cm=float(np.median(d[k, 1]) * 100), dz_cm=float(np.median(d[k, 2]) * 100),
                                         dxy_cm=float(np.median(np.linalg.norm(d[k, :2], axis=1)) * 100), dyaw_deg=float(np.median(Rot.from_matrix(np.transpose(Rb[k], (0, 2, 1)) @ R @ Ra[k]).as_euler('ZYX', degrees=True)[:, 0]))))
        out[vn] = dict(yaw_pitch_roll_deg=eul.tolist(), t=t.tolist(), inliers=int(m.sum()), n=len(m), pos_resid_med_cm=float(np.median(np.linalg.norm(d, axis=1)) * 100),
                       pos_resid_max_cm=float(np.linalg.norm(d, axis=1).max() * 100), xy_resid_med_cm=float(np.median(np.linalg.norm(d[:, :2], axis=1)) * 100), bins=rows)
        print(f'{vn}: 요·피치·롤 {np.round(eul, 2)}°, 이상치 빼고 {m.sum()}/{len(m)}장, 위치 차(in1rL 대비) 중앙 {out[vn]["pos_resid_med_cm"]:.1f} cm · 최대 {out[vn]["pos_resid_max_cm"]:.0f} cm (수평 중앙 {out[vn]["xy_resid_med_cm"]:.1f} cm)')
# 참고: 원래 궤적과 in1rL 궤적의 길이 비(수평, 닮음 맞춤)
a = Pa[:, :2] - Pa[:, :2].mean(0); b = Pb[:, :2] - Pb[:, :2].mean(0); U, S, Vt = np.linalg.svd(b.T @ a); D = np.diag([1, np.sign(np.linalg.det(U @ Vt))])
s = float((S * np.diag(D)).sum() / (a ** 2).sum()); out['scale_raw_to_placed_xy'] = s
out['raw_z_range'] = [float(Pa[:, 2].min()), float(Pa[:, 2].max())]; out['placed_z_range'] = [float(Pb[:, 2].min()), float(Pb[:, 2].max())]
print(f'수평 길이 비(in1rL / 원래) {s:.4f} | 원래 z 범위 {Pa[:, 2].min():+.2f}~{Pa[:, 2].max():+.2f} m, in1rL z {Pb[:, 2].min():.2f}~{Pb[:, 2].max():.2f} m')
(O / 'rigidval.json').write_text(json.dumps(out, indent=1, ensure_ascii=False))
fig, ax = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
for vn, c in (('rigid4', 'C0'), ('rigid6', 'C1')):
    P = np.loadtxt(O / f'{raw_run}_{vn}.txt', ndmin=2)[:, 1:4]; d = (P - Pb) * 100
    for k, lab in enumerate(['dx (along) cm', 'dy (across) cm', 'dz (height) cm']): ax[k].plot(xs, d[:, k], '.', color=c, ms=4, label=vn); ax[k].set_ylabel(lab); ax[k].grid(alpha=.3)
ax[0].legend(); ax[2].set_xlabel(f'x of {pl_run} camera (m)'); fig.suptitle(f'{raw_run} raw RTAB-Map poses after one rigid transform minus {pl_run} positions'); fig.tight_layout(); fig.savefig(O / 'rigidval_pos.png', dpi=90)
