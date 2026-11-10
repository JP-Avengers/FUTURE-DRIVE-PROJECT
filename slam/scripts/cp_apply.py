"""cp_local.py 의 2D 강체(세계 xy 회전 Rm·이동 t)를 4x4 정합 T 에 덧씌움: T' = S @ T
사용: cp_apply.py <T_in.npy> <rigid2d.npy> <T_out.npy>"""
import sys, numpy as np
T = np.load(sys.argv[1]); v = np.load(sys.argv[2]); Rm = v[:4].reshape(2, 2); t = v[4:6]
S = np.eye(4); S[:2, :2] = Rm; S[:2, 3] = t; np.save(sys.argv[3], S @ T)
print(f'적용: 요 {np.degrees(np.arctan2(Rm[1, 0], Rm[0, 0])):+.2f}°, 이동 ({t[0]:+.2f},{t[1]:+.2f}) → {sys.argv[3]}')
