"""먼 구역 3D — 스캔(B 형상 메쉬) 밖을 키프레임 RGB-D 로 직접 쌓은 메쉬. build_usd.py 의 far_tsdf 단계.

10.7 미팅 '배경은 보이는 대로 · 외곽은 점군 보강으로': 벽이나 상자를 세우지 않고, 카메라 depth 가 닿은 곳(최대 약 45 m)까지만
스캔과 같은 방식(TSDF → 메쉬 → 키프레임 사진 투영)으로 3D 를 만든다. 닿지 않은 곳은 비워 둔다.
- 먼 depth 는 잡음이 커서(D435 는 30 m 에서 수 m) 복셀·절단 거리를 크게 (0.4 m · 2.4 m) — 작으면 면이 서로 상쇄돼 조각만 남음
- 하늘 화소의 depth 는 뺌 (역광·노을 쪽 하늘에 잡음 depth 가 찍혀 흰 덩어리가 생김)
- 색: TSDF 평균색은 흐려서, 한 번 잘게 나눈 뒤 키프레임 사진을 다시 투영 (스캔 색과 같은 color_utils.project_colors)
스캔 범위 안은 build_usd.zones 가 잘라내고(VisualFar = 스캔 밖만, 충돌 없음), 결과는 USD 의 /World/Parking/VisualFar.
"""
import numpy as np
import open3d as o3d
from scipy import ndimage

from color_utils import project_colors, raycasting_scene


def sky_mask(img, c):
    """하늘 화소: 매끈하고(7×7 밝기 표준편차 작음) 밝음, 또는 아주 밝음 (흰 구름 · 햇빛)."""
    g = img.mean(2).astype(np.float32) / 255.0
    m1 = ndimage.uniform_filter(g, 7)
    std = np.sqrt(np.maximum(ndimage.uniform_filter(g * g, 7) - m1 * m1, 0))
    return ((std < c["sky_smooth_std"]) & (g > c["sky_smooth_bright"])) | (g > c["sky_white_bright"])


def far_volume(sets, c):
    """sets = [Frames, …] (B 맵 좌표 pose). 반환: (메쉬 — B 맵 좌표, 쌓은 키프레임 수)."""
    vox = c["voxel_m"]
    vol = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=vox, sdf_trunc=c["sdf_trunc_voxels"] * vox, color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    n = 0
    for fr in sets:
        for i in range(0, len(fr.poses), c.get("frame_step", 1)):
            color = o3d.io.read_image(fr.color[i])
            d = np.asarray(o3d.io.read_image(fr.depth[i])).copy()
            d[d < c["min_depth_m"] * fr.depth_scale] = 0
            d[sky_mask(np.asarray(color), c)] = 0
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color, o3d.geometry.Image(d), depth_scale=fr.depth_scale, depth_trunc=c["max_depth_m"], convert_rgb_to_intensity=False)
            vol.integrate(rgbd, fr.intrinsic, np.linalg.inv(fr.poses[i]))
            n += 1
    return vol.extract_triangle_mesh(), n


def recolor(far, scan, sets, c):
    """먼 구역 메쉬를 subdivide 번 잘게 나누고 키프레임 사진을 투영. sets = [(Frames, 씬 좌표 poses)] — 적힌 순서대로,
    앞 묶음이 못 칠한 점에만. 어느 키프레임에도 안 보인 점은 TSDF 평균색 그대로. 반환: (메쉬, 사진으로 칠한 비율)."""
    if c.get("subdivide", 0):
        far = far.subdivide_midpoint(number_of_iterations=c["subdivide"])
    far.compute_vertex_normals()
    occ = raycasting_scene(scan + far)
    v, n = np.asarray(far.vertices), np.asarray(far.vertex_normals)
    colors = np.asarray(far.vertex_colors).copy()
    seen = np.zeros(len(v), bool)
    for fr, poses in sets:
        idx = np.nonzero(~seen)[0]
        if not len(idx):
            break
        col, s, _ = project_colors(v[idx], n[idx], fr, poses, occ, c["color"])
        colors[idx[s]] = col[s]
        seen[idx[s]] = True
    far.vertex_colors = o3d.utility.Vector3dVector(colors)
    return far, float(seen.mean())
