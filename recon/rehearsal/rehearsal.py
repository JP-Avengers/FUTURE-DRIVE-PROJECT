"""샘플 점군으로 Mesh 파이프라인 리허설: 정제 → Poisson → 경량화 → Collider → OBJ.

Collider 는 점군 전체의 convex hull 하나라 실제 캠퍼스 씬에는 쓰지 않는다 (README 알려진 한계).
"""
import argparse, time, os
import open3d as o3d
import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--out", default="~/jinwoo/out", help="OBJ 출력 폴더 (레포 밖)")
args = parser.parse_args()

out_dir = os.path.expanduser(args.out)
os.makedirs(out_dir, exist_ok=True)

t0 = time.time()

print("[1] 샘플 Point Cloud 다운로드/로드")
sample = o3d.data.PLYPointCloud()
pcd = o3d.io.read_point_cloud(sample.path)
print(f"    원본 포인트 수: {len(pcd.points)}")

print("[2] 다운샘플링 (voxel_down_sample)")
pcd_ds = pcd.voxel_down_sample(voxel_size=0.02)
print(f"    다운샘플 후: {len(pcd_ds.points)}")

print("[3] 이상치 제거 (statistical outlier removal)")
pcd_clean, ind = pcd_ds.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
print(f"    정제 후: {len(pcd_clean.points)} (제거됨: {len(pcd_ds.points)-len(pcd_clean.points)})")

print("[4] 법선 추정 + 방향 정렬")
pcd_clean.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
pcd_clean.orient_normals_consistent_tangent_plane(k=15)

print("[5] Poisson Surface Reconstruction (Mesh v0)")
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_clean, depth=9)
print(f"    Mesh v0: 정점 {len(mesh.vertices)}, 삼각형 {len(mesh.triangles)}")

print("[6] 저밀도 영역 제거 (경계 노이즈 정리)")
densities = np.asarray(densities)
thresh = np.quantile(densities, 0.02)
mesh.remove_vertices_by_mask(densities < thresh)
print(f"    정리 후: 정점 {len(mesh.vertices)}, 삼각형 {len(mesh.triangles)}")

print("[7] 경량화 (quadric decimation, 목표 삼각형 5000개)")
mesh_light = mesh.simplify_quadric_decimation(target_number_of_triangles=5000)
mesh_light.compute_vertex_normals()
print(f"    경량화 후: 정점 {len(mesh_light.vertices)}, 삼각형 {len(mesh_light.triangles)}")

print("[8] Collider용 단순 형상 (convex hull)")
hull, _ = pcd_clean.compute_convex_hull()
print(f"    Collider hull: 정점 {len(hull.vertices)}, 삼각형 {len(hull.triangles)}")

print("[9] OBJ로 내보내기 (시각용 / 충돌용 분리)")
o3d.io.write_triangle_mesh(os.path.join(out_dir, "visual_mesh.obj"), mesh_light)
o3d.io.write_triangle_mesh(os.path.join(out_dir, "collider_hull.obj"), hull)

print(f"\n총 소요 시간: {time.time()-t0:.1f}s")
print("REHEARSAL_OK ->", out_dir)
