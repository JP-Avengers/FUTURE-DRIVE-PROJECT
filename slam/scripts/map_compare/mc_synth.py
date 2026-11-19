"""맞춤(mc_register.py) 시험용 가짜 '남의 메시' 만들기와 정답 대조.
 make : 우리 메시에 알려진 변환(크기 · 요 · 이동 · 약간 기울기 → Y 위로 돌리기)과 꼭짓점 잡음을 넣어 PLY(색 유지) · GLB(위치만, 노드 이동 포함)로 저장
 check: 정답 변환과 mc_register.py 결과를 합성해 회전 · 위치 오차와 크기 추정을 비교
사용: mc_synth.py make <src.ply> <out_dir> [--scale 1.01] [--yaw 137.3] [--t -12.4,7.9,3.1] [--tilt 0.4] [--noise 0.003] [--decimate 300000 (삼각형 수로 줄임, Polycam 흉내)] [--tag synth]
      mc_synth.py check <out_dir>/synth_truth.json <reg.json>"""
import sys, os, json, numpy as np
from pathlib import Path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from scipy.spatial.transform import Rotation as Rot

a = sys.argv; opt = lambda k, d: a[a.index(k) + 1] if k in a else d
if a[1] == 'make':
    from mc_lib import load_mesh, save_ply, write_glb
    src, out = a[2], Path(a[3]); out.mkdir(parents=True, exist_ok=True)
    s, yaw, tilt, noise = float(opt('--scale', 1.01)), float(opt('--yaw', 137.3)), float(opt('--tilt', 0.4)), float(opt('--noise', 0.003)); t = np.array([float(x) for x in opt('--t', '-12.4,7.9,3.1').split(',')])
    M = np.eye(4); M[:3, :3] = s * Rot.from_euler('zx', [yaw, tilt], degrees=True).as_matrix(); M[:3, 3] = t     # Z 위 좌표에서 크기 · 요 · 기울기 · 이동
    YUP = np.array([[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0], [0, 0, 0, 1.0]]); Ttrue = YUP @ M                  # (x, y, z) → (x, z, -y): glTF 처럼 Y 위
    V, T, C = load_mesh(src); TAG = opt('--tag', 'synth')
    if '--decimate' in a:
        import open3d as o3d; m = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(T.astype(np.int32)))
        if C is not None: m.vertex_colors = o3d.utility.Vector3dVector(C)
        m = m.simplify_quadric_decimation(int(opt('--decimate', 300000))); V, T = np.asarray(m.vertices).copy(), np.asarray(m.triangles).astype(np.int64); C = np.asarray(m.vertex_colors).copy() if m.has_vertex_colors() else None
    Vf = V @ Ttrue[:3, :3].T + Ttrue[:3, 3] + np.random.default_rng(7).normal(0, noise, V.shape)
    save_ply(out / f'{TAG}_foreign.ply', Vf, T, C); off = Vf.mean(0); write_glb(out / f'{TAG}_foreign.glb', Vf - off, T, node_translation=off)
    (out / f'{TAG}_truth.json').write_text(json.dumps(dict(src=src, scale=s, yaw_deg=yaw, tilt_deg=tilt, t=t.tolist(), noise_m=noise, up='file +Y', T_building_to_foreign=Ttrue.tolist()), indent=1))
    print(f'가짜 메시: 꼭짓점 {len(V):,}, 삼각형 {len(T):,} → {out}/{TAG}_foreign.{{ply,glb}} (크기 {s}, 요 {yaw}°, 기울기 {tilt}°, 이동 {t}, 잡음 {noise * 1000:.0f} mm, Y 위)')
else:
    tr = json.loads(Path(a[2]).read_text()); rg = json.loads(Path(a[3]).read_text())
    M = np.array(rg['T_foreign_to_building']) @ np.array(tr['T_building_to_foreign'])        # 완벽하면 단위 행렬(크기 미적용이면 크기 1.01 닮음 변환)
    sc = np.cbrt(np.linalg.det(M[:3, :3])); R = M[:3, :3] / sc; ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))); yaw = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    print(f'[{Path(a[3]).name}] 위 축 선택 {rg["chosen_up_file"]} (정답 파일 +Y) | 남은 회전 {ang:.3f}° (요 {yaw:+.3f}°) | 남은 크기 {sc:.4f} (크기 적용 {rg["scale_applied"]})')
    for x in (0, 11.5, 23, 34.5, 46):
        p = np.array([[x, -1.1, 0.0], [x, 1.43, 0.0], [x, -1.1, 1.5], [x, 1.43, 1.5]]); d = p @ M[:3, :3].T + M[:3, 3] - p
        print(f'   x={x:5.1f} m 벽 · 바닥 점 위치 오차: 평균 {np.linalg.norm(d, axis=1).mean() * 100:5.1f} cm  (dx {d[:, 0].mean() * 100:+.1f}, dy {d[:, 1].mean() * 100:+.1f}, dz {d[:, 2].mean() * 100:+.1f} cm)')
    ck = rg['scale_check']; print(f'   크기 비 정답 {tr["scale"]} | 추정: x 창 기울기 {ck["scale_x"]}, 크기 허용 ICP {ck["scale_icp"]:.4f} | 벽 잔차 중앙 {rg["residual"]["wall_med_cm"]:.1f} cm')
