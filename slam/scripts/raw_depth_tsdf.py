#!/usr/bin/env python3
"""원본(RAW) depth로 기하 전용 TSDF. 키프레임(컬러 정렬 depth)은 컬러 시야로 잘려 차 근처 바닥(약 1.4~5m)이 빠지므로,
같은 키프레임 시각의 원본 depth(시야가 더 넓음)를 depth 카메라 내부 파라미터로 통합한다. 원본 bag은 읽기만 한다.
메모리: bag을 읽는 동안 주기적으로 해당 파일 페이지 캐시를 비운다(posix_fadvise DONTNEED, sudo 불필요).
사용: raw_depth_tsdf.py <run> <bag_dir>... [--voxel 0.05] [--trunc 8] [--min 1.0]  (--min 미만 depth는 차 지붕 등 근접 노이즈로 보고 제외)"""
import sys, os, argparse
from pathlib import Path
import numpy as np, open3d as o3d
from rosbags.highlevel import AnyReader
ap = argparse.ArgumentParser(); ap.add_argument('run'); ap.add_argument('bags', nargs='+')
ap.add_argument('--voxel', type=float, default=0.05); ap.add_argument('--trunc', type=float, default=8.0); ap.add_argument('--min', type=float, default=1.0); a = ap.parse_args()
E = Path.home() / f'b_slam/exports/{a.run}'; NS = '/camera/camera'
def qR(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
tr = np.loadtxt(E / 'traj_optimized_poses.txt', comments='#', ndmin=2)  # camera_link 자세, stamp x y z qx qy qz qw
want = {round(r[0], 3): r for r in tr}; stamps = np.array(sorted(want))
vol = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=a.voxel, sdf_trunc=4 * a.voxel, color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)
used = 0; T_cl_do = None; K = None
def drop_cache(bagdir):
    for f in Path(bagdir).glob('*.mcap'):
        fd = os.open(f, os.O_RDONLY); os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED); os.close(fd)
for bag in a.bags:
    with AnyReader([Path(bag)]) as r:
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic == '/tf_static']):
            m = r.deserialize(raw, c.msgtype); tf = {(x.header.frame_id, x.child_frame_id): x.transform for x in m.transforms}
            def T(p, ch):
                x = tf[(p, ch)]; M = np.eye(4); q = x.rotation; M[:3, :3] = qR(q.x, q.y, q.z, q.w); M[:3, 3] = [x.translation.x, x.translation.y, x.translation.z]; return M
            T_cl_do = T('camera_link', 'camera_depth_frame') @ T('camera_depth_frame', 'camera_depth_optical_frame')
        for c, t, raw in r.messages(connections=[c for c in r.connections if c.topic == f'{NS}/depth/camera_info']):
            m = r.deserialize(raw, c.msgtype); k = m.k; K = o3d.camera.PinholeCameraIntrinsic(m.width, m.height, k[0], k[4], k[2], k[5]); break
        conns = [c for c in r.connections if c.topic == f'{NS}/depth/image_rect_raw']; n = 0; got = 0
        for c, t, raw in r.messages(connections=conns):
            n += 1
            if n % 300 == 0: drop_cache(bag)
            m = r.deserialize(raw, c.msgtype); ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            j = np.searchsorted(stamps, ts); best = None
            for jj in (j - 1, j):
                if 0 <= jj < len(stamps) and abs(stamps[jj] - ts) < 0.02: best = stamps[jj]
            if best is None: continue
            p = want[best]; Tw = np.eye(4); Tw[:3, :3] = qR(*p[4:8]); Tw[:3, 3] = p[1:4]
            dz = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy(); dz[dz < a.min * 1000] = 0
            depth = o3d.geometry.Image(dz)
            dummy = o3d.geometry.Image(np.zeros((m.height, m.width, 3), np.uint8))
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(dummy, depth, depth_scale=1000.0, depth_trunc=a.trunc, convert_rgb_to_intensity=False)
            vol.integrate(rgbd, K, np.linalg.inv(Tw @ T_cl_do)); used += 1; got += 1
        drop_cache(bag)
    print(f'{Path(bag).parent.name}: depth {n}장 중 키프레임 시각 일치 {got}장 통합', flush=True)
mesh = vol.extract_triangle_mesh(); mesh.compute_vertex_normals()
out = E / f'tsdf_rawdepth_{round(a.voxel*100)}cm_min{a.min:g}m.ply'; o3d.io.write_triangle_mesh(str(out), mesh)
V = np.asarray(mesh.vertices); T3 = tr[:, 1:4]
dist = np.concatenate([np.sqrt(((V[j:j+20000, None, :2] - T3[None, :, :2]) ** 2).sum(-1)).min(1) for j in range(0, len(V), 20000)])
print(f'키프레임 {used}/{len(tr)} 통합 → 꼭짓점 {len(V)}, 삼각형 {len(mesh.triangles)} → {out}')
for lo, hi in ((0, 1.5), (1.5, 3), (3, 5), (5, 8), (8, 99)):
    print(f'  궤적에서 수평 {lo}-{hi}m: 꼭짓점 {((dist >= lo) & (dist < hi)).sum()}')
