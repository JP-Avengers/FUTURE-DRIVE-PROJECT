#!/bin/bash
# 촬영 폴더를 ~/data로 복사(원본 무변경). mcap 이름은 metadata 기준 이름으로 맞추고 체크섬 검증.
# 사용: import_capture.sh <capture_dir> [mcap_path]
#   mcap_path: 다운로드 중 폴더 밖으로 빠진 mcap 파일 경로 (없으면 <capture_dir>/bag/*.mcap 사용)
set -eu
SRC=$1; NAME=$(basename "$SRC"); DST=~/data/$NAME
[ -e "$DST" ] && { echo "이미 존재: $DST"; exit 1; }
want=$(grep -A1 relative_file_paths "$SRC/bag/metadata.yaml" | tail -1 | sed 's/.*- //')
if [ $# -ge 2 ]; then src_mcap=$2; else src_mcap=$(ls "$SRC"/bag/*.mcap); fi
[ $(echo "$src_mcap" | wc -l) -eq 1 ] && [ -f "$src_mcap" ] || { echo "mcap 파일을 하나로 특정할 수 없음: $src_mcap"; exit 1; }
mkdir -p "$DST/bag"
rsync -a --exclude 'bag/*.mcap' "$SRC/" "$DST/"
echo "복사: $src_mcap -> bag/$want"
cp "$src_mcap" "$DST/bag/$want"
cd "$DST" && sha256sum -c checksums.sha256 && echo "IMPORT OK: $DST"
