#!/bin/bash
# 실행 결과 추출: 최적화/오도메트리 궤적(TUM), 포인트 클라우드 PLY(복셀 다운샘플), 키프레임 RGB·depth+보정값
# 사용: export_run.sh <run_name> [voxel=0.03]
source /opt/ros/jazzy/setup.bash
set -eu
RUN=$1; VOX=${2:-0.03}; DB=~/maps/$RUN.db; OUT=~/b_slam/exports/$RUN
[ -f "$DB" ] || { echo "DB 없음: $DB"; exit 1; }
[ -e "$OUT" ] && { echo "이미 존재: $OUT (덮어쓰지 않음)"; exit 1; }
mkdir -p "$OUT"
rtabmap-export --poses --poses_format 10 --output_dir "$OUT" --output traj_optimized "$DB" > "$OUT/export.log" 2>&1
rtabmap-export --poses --poses_format 10 --opt 3 --output_dir "$OUT" --output traj_odom "$DB" >> "$OUT/export.log" 2>&1
rtabmap-export --poses_camera --poses_format 10 --output_dir "$OUT" --output traj_camera "$DB" >> "$OUT/export.log" 2>&1
rtabmap-export --poses_camera --poses_format 11 --output_dir "$OUT" --output traj_camera_id "$DB" >> "$OUT/export.log" 2>&1  # 키프레임 ID 포함 (TSDF·패키지용)
rtabmap-export --cloud --voxel "$VOX" --max_range 6 --decimation 2 --output_dir "$OUT" --output cloud "$DB" >> "$OUT/export.log" 2>&1
rtabmap-export --images_id --output_dir "$OUT/keyframes" --output kf "$DB" >> "$OUT/export.log" 2>&1
ls -la "$OUT" "$OUT/keyframes" | head -30
