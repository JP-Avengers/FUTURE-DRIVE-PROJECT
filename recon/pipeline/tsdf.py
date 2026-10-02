"""RGB-D 프레임 + 카메라 pose → TSDF 메쉬.

    python recon/pipeline/tsdf.py --config recon/configs/lounge.yaml

출력 (<out>/): mesh_raw.ply, tsdf_meta.json (설정·프레임 수·시간·up_hint)
"""
import argparse, json, os, time

import numpy as np
import open3d as o3d
import yaml

import io_rgbd


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", help="출력 폴더 (기본: config 의 out)")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config, encoding="utf-8"))
    out = os.path.expanduser(args.out or cfg["out"])
    os.makedirs(out, exist_ok=True)
    p = cfg["tsdf"]

    fr = io_rgbd.load(cfg["input"])
    used = list(range(0, len(fr.poses), p["frame_step"]))
    print(f"[tsdf] 프레임 {len(fr.poses)}장 중 {len(used)}장 사용 | voxel {p['voxel'] * 100:g} cm | 깊이 {p['depth_trunc']} m 까지")

    vol = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=p["voxel"], sdf_trunc=p["sdf_trunc_voxels"] * p["voxel"],
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    t = time.time()
    for k, i in enumerate(used):
        color, depth = o3d.io.read_image(fr.color[i]), o3d.io.read_image(fr.depth[i])
        if k == 0:  # 해상도가 카메라 파라미터와 다르면 조용히 틀린 메쉬가 나오므로 먼저 막는다
            want = (fr.intrinsic.height, fr.intrinsic.width)
            got_c, got_d = np.asarray(color).shape[:2], np.asarray(depth).shape[:2]
            if got_c != want or got_d != want:
                raise SystemExit(f"해상도 불일치: intrinsic {want}, 컬러 {got_c}, 깊이 {got_d} — 깊이를 컬러에 정렬(align)했는지 확인")
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth, depth_scale=fr.depth_scale, depth_trunc=p["depth_trunc"], convert_rgb_to_intensity=False)
        vol.integrate(rgbd, fr.intrinsic, np.linalg.inv(fr.poses[i]))   # pose(카메라→world)의 역 = extrinsic
        if (k + 1) % 100 == 0:
            print(f"  {k + 1}/{len(used)} ({time.time() - t:.0f}s)", flush=True)

    mesh = vol.extract_triangle_mesh()
    sec = time.time() - t
    o3d.io.write_triangle_mesh(os.path.join(out, "mesh_raw.ply"), mesh)

    # 카메라 화면 아래(+y) 방향 평균의 반대 = 대략적인 위쪽. build_usd.py 가 바닥을 찾을 때 씀
    up = -np.mean([P[:3, 1] for P in fr.poses], axis=0)
    meta = dict(config=os.path.abspath(args.config), tsdf=p, frames_total=len(fr.poses), frames_used=len(used),
                seconds=round(sec, 1), vertices=len(mesh.vertices), triangles=len(mesh.triangles),
                up_hint_camera=(up / np.linalg.norm(up)).tolist())
    json.dump(meta, open(os.path.join(out, "tsdf_meta.json"), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[tsdf] 삼각형 {len(mesh.triangles):,} | {sec:.0f}s → {os.path.join(out, 'mesh_raw.ply')}")


if __name__ == "__main__":
    main()
