#!/usr/bin/env python3
"""점군·메시 뷰어 (WSLg 창). 사용: view_cloud.py <ply> [<ply> ...]
조작: 왼쪽 드래그 회전, 휠 확대, Ctrl+왼쪽 드래그 이동, +/- 점 크기, R 시점 초기화, Q 닫기"""
import sys
import open3d as o3d
geoms = []
for p in sys.argv[1:]:
    m = o3d.io.read_triangle_mesh(p)
    if len(m.triangles) > 0:
        m.compute_vertex_normals(); geoms.append(m)
    else:
        geoms.append(o3d.io.read_point_cloud(p))
    print(p, '로드')
o3d.visualization.draw_geometries(geoms, window_name=' + '.join(x.split('/')[-1] for x in sys.argv[1:]), width=1600, height=900)
