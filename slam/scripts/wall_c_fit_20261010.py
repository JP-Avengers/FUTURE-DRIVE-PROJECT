import numpy as np
from scipy.optimize import least_squares
# (줄자 m, 가장 큰 조각 m, 면적 가중 m) — wall_planes2.py 결과에서 정면 조각(기울기≤12°, 수풀·바닥 제외)
S1 = [(2, 1.772, 1.772), (3, 3.041, 3.041), (5, 5.003, 4.988), (8, 8.596, 8.573)]          # 1차 손, 화강암 벽
S2 = [(3, 3.051, 3.024), (5, 5.062, 4.950), (8, 8.478, 8.536), (10, 10.338, 10.333)]       # 2차 차, 창문 건물
def fit(T, M):
    T, M = np.array(T, float), np.array(M, float)
    c0 = np.sum((M / T - 1) * T) / np.sum(T * T); r0 = M - T * (1 + c0 * T)
    f = lambda p: (T + p[1]) * (1 + p[0] * (T + p[1])) - M
    s = least_squares(f, [0.005, 0.0]); r1 = f(s.x)
    return c0, np.sqrt(np.mean(r0 ** 2)), s.x, np.sqrt(np.mean(r1 ** 2))
def pred(c, T): return T * (1 + c * T)
for nm, S in (('1차(손) 2·3·5·8m', S1), ('1차 2m 제외', S1[1:]), ('2차(차) 3·5·8·10m', S2), ('1차(2m 제외)+2차', S1[1:] + S2)):
    for k, lab in ((1, '가장 큰 조각'), (2, '면적 가중')):
        T = [s[0] for s in S]; M = [s[k] for s in S]; c0, r0, (c1, d1), r1 = fit(T, M)
        print(f'{nm:18s} {lab:7s}: c={c0:+.4f} (잔차 {r0*100:4.1f}cm) | 기준점 오프셋 함께: c={c1:+.4f}, 오프셋 {d1*100:+5.1f}cm (잔차 {r1*100:4.1f}cm)')
print('\n참고: c=0.007이면 원본 depth = 줄자 ×', ', '.join(f'{T}m {1+0.007*T:.3f}' for T in (3, 5, 8, 10)))
for nm, S in (('1차', S1), ('2차', S2)):
    print(nm, ' | '.join(f'{s[0]}m: 측정 {s[2]:.3f} (c=0 {s[0]:.2f}, c=0.007 {pred(0.007, s[0]):.3f})' for s in S))
