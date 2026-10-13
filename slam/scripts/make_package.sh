#!/bin/bash
# C 전달용 맵 패키지 생성 (원본 exports는 복사만 함)
# 사용: make_package.sh <run> <package_name> [extra_run]
set -eu
RUN=$1; PKG=$2; EXTRA=${3:-}
E=~/b_slam/exports/$RUN; OUT=~/b_slam/deliver/$PKG
[ -e "$OUT" ] && { echo "이미 존재: $OUT (덮어쓰지 않음)"; exit 1; }
mkdir -p "$OUT"
cp "$E/cloud_cloud.ply" "$OUT/cloud_3cm.ply"
cp "$E/traj_camera_id_camera_poses.txt" "$OUT/trajectory_camera_optical.txt"
cp "$E/traj_optimized_poses.txt" "$OUT/trajectory_camera_link.txt"
cp -r "$E/keyframes" "$OUT/keyframes"
mv "$OUT/keyframes/kf_rgb" "$OUT/keyframes/rgb"; mv "$OUT/keyframes/kf_depth" "$OUT/keyframes/depth"; mv "$OUT/keyframes/kf_calib" "$OUT/keyframes/calib"
[ -f "$E/tsdf_check_5cm.ply" ] && cp "$E/tsdf_check_5cm.ply" "$OUT/preview_tsdf_5cm.ply"
cp "$E/topview.png" "$OUT/topview.png"
cp ~/b_slam/scripts/tsdf_check.py "$OUT/example_tsdf.py"
if [ -n "$EXTRA" ]; then
  X=~/b_slam/exports/$EXTRA; D="$OUT/extra_$EXTRA"; mkdir -p "$D"
  cp "$X/cloud_cloud.ply" "$D/cloud_3cm.ply"; cp "$X/traj_optimized_poses.txt" "$D/trajectory_camera_link.txt"; cp "$X/topview.png" "$D/topview.png"
fi
cd "$OUT" && find . -type f ! -name checksums.sha256 -print0 | sort -z | xargs -0 sha256sum > checksums.sha256
du -sh "$OUT"; ls "$OUT"
