#!/usr/bin/env python3
"""원본(RAW) depth 전체 프레임으로 기하 전용 TSDF (raw_depth_tsdf.py의 전체 프레임판).
키프레임(약 1초 간격)만 쓰면 키프레임 사이 바닥이 비므로, STEP 프레임마다 앞뒤 키프레임 자세를 보간(위치 선형, 회전 slerp)해 통합한다.
보간 오차(키프레임 하나 빼고 2초 간격으로 잰 값): 위치 중앙 7cm·회전 0.26° → 1초 간격에선 약 1/4.
앞뒤 키프레임 간격이 --maxgap 초를 넘는 프레임(세션 경계, 키프레임 공백)은 쓰지 않는다. 원본 bag은 읽기만 한다.
사용: raw_depth_tsdf_full.py <run> <bag_dir>... [--step 3] [--voxel 0.05] [--trunc 8] [--min 1.0] [--maxgap 1.5]"""
import os, argparse
from pathlib import Path
import numpy as np, open3d as o3d
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation as Rot, Slerp
ap = argparse.ArgumentParser(); ap.add_argument('run'); ap.add_argument('bags', nargs='+')
ap.add_argument('--step', type=int, default=3); ap.add_argument('--voxel', type=float, default=0.05)
ap.add_argument('--trunc', type=float, default=8.0); ap.add_argument('--min', type=float, default=1.0)
ap.add_argument('--maxgap', type=float, default=1.5); a = ap.parse_args()
E = Path.home() / f'b_slam/exports/{a.run}'; NS = '/camera/camera'
def qR(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)], [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
tr = np.loadtxt(E / 'traj_optimized_poses.txt', comments='#', ndmin=2); tr = tr[np.argsort(tr[:, 0])]  # camera_link 자세, 시간순
st = tr[:, 0]
def pose_at(ts):
    j = np.searchsorted(st, ts)
    if j == 0 or j >= len(st) or st[j] - st[j - 1] > a.maxgap: return None
    p0, p1 = tr[j - 1], tr[j]; w = (ts - p0[0]) / (p1[0] - p0[0])
    T = np.eye(4); T[:3, 3] = p0[1:4] + w * (p1[1:4] - p0[1:4])
    T[:3, :3] = Slerp([0, 1], Rot.from_quat([p0[4:8], p1[4:8]]))([w])[0].as_matrix(); return T
vol = o3d.pipelines.integration.ScalableTSDFVolume(voxel_length=a.voxel, sdf_trunc=4 * a.voxel, color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)
used = 0
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
        conns = [c for c in r.connections if c.topic == f'{NS}/depth/image_rect_raw']; n = got = 0
        for c, t, raw in r.messages(connections=conns):
            n += 1
            if n % 300 == 0: drop_cache(bag)
            if n % a.step: continue
            m = r.deserialize(raw, c.msgtype); Tw = pose_at(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
            if Tw is None: continue
            dz = np.frombuffer(m.data, np.uint16).reshape(m.height, m.width).copy(); dz[dz < a.min * 1000] = 0
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(o3d.geometry.Image(np.zeros((m.height, m.width, 3), np.uint8)), o3d.geometry.Image(dz),
                                                                      depth_scale=1000.0, depth_trunc=a.trunc, convert_rgb_to_intensity=False)
            vol.integrate(rgbd, K, np.linalg.inv(Tw @ T_cl_do)); used += 1; got += 1
        drop_cache(bag)
    print(f'{Path(bag).parent.name}: depth {n}장 중 {got}장 통합 (STEP {a.step})', flush=True)
mesh = vol.extract_triangle_mesh(); mesh.compute_vertex_normals()
out = E / f'tsdf_rawdepth_full{a.step}_{round(a.voxel*100)}cm_min{a.min:g}m.ply'; o3d.io.write_triangle_mesh(str(out), mesh)
print(f'총 {used}장 통합 → 꼭짓점 {len(mesh.vertices)}, 삼각형 {len(mesh.triangles)} → {out}')
