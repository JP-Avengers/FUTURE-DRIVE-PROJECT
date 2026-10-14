#!/bin/bash
# Nav2 장애물 회피 시연 영상 — sim/nav2/record_demo.py 를 두 단계로 부른다(tmux 안에서).
#   sim/nav2/record_demo.sh <시나리오.yaml> [cw|ccw] [출력.mp4]
#   예) sim/nav2/record_demo.sh eval/scenarios/parking_r3_obs_demo.yaml cw
# 1) 장애물 레이어(sim/make_obstacles.py — 구간이면 출발 자세도) → 2) drive: 영상 렌더 없이 수집 때와 같은 조건으로 주행 · 자세 기록
# (준비 실패 DEMO_ABORT 나 판정 실패면 다시 — 최대 3번, 시도마다 route_try<n>.log 를 남김. FD_DEMO_NEED_PASS=0 이면 판정 실패도 그대로)
# → 3) replay: 기록한 자세로 다시 놓으며 렌더 · 합성 → 4) imageio-ffmpeg 정적 바이너리로 H.264 1080p · 720p(서버에 ffmpeg 명령이 없음).
# FD_DEMO_SKIP_DRIVE=1 = 주행 없이 다시 렌더만(앞 기록 재사용). 결과: FD_DEMO_DIR(기본 logs/nav2/demo)/<이름>_<방향>.mp4 · _1080p · _720p · _logs/ · _drive.log · _replay.log
CD1=$HOME/Capstone_Design_1; cd $CD1
SC=${1:-}; DIR=${2:-cw}
[ -f "$SC" ] || { sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
N=$(basename $SC .yaml); OUT=${3:-$CD1/${FD_DEMO_DIR:-logs/nav2/demo}/${N}_${DIR}.mp4}
mkdir -p $(dirname $OUT)
busy=$(nvidia-smi --query-compute-apps=pid,process_name,used_memory --format=csv,noheader)
[ -n "$busy" ] && { echo "GPU 사용 중 — 녹화 안 함: $busy"; exit 3; }
LAYER=sim/scenes/parking/obstacles/$N.usda
LOUT=$(envs/isaacsim_env/bin/python sim/make_obstacles.py $SC $LAYER 2>&1)
echo "$LOUT" | grep -E "START|SAVED" | sed "s/^/  /"
POSE=$(echo "$LOUT" | sed -nE 's/^START .*\(([-0-9.]+), ([-0-9.]+), [-0-9.]+\) [^-0-9]*([-0-9.]+).*/x0:=\1 y0:=\2 yaw0_deg:=\3/p')
set +u
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 FD_SCENARIO=$SC FD_DIR=$DIR FD_LAYER=$LAYER FD_POSE="$POSE"
LD=${OUT%.mp4}_logs; mkdir -p $LD
# FD_DEMO_SKIP_DRIVE=1 이면 주행은 건너뛰고(앞서 남긴 _logs 의 기록으로) 다시 렌더만
for try in 1 2 3; do
  [ "${FD_DEMO_SKIP_DRIVE:-0}" = 1 ] && break
  echo "DEMO_DRIVE $N $DIR 시도 $try $(date +%T)"
  FD_DEMO_MODE=drive envs/isaacsim_env/bin/python sim/nav2/record_demo.py $OUT > ${OUT%.mp4}_drive.log 2>&1
  grep -E "WARMUP|ROUTE_START|RESULT|DEMO_ABORT|DEMO_DRIVE_DONE|TIME_CAP|PROG" ${OUT%.mp4}_drive.log | sed "s/^/  /" | cut -c1-330
  # 남은 하위 프로세스(Nav2 · 다리 · 탭 · 주행) 정리 — 녹화기가 비정상으로 끝나도
  for g in $(cat $LD/pgids.txt 2>/dev/null); do kill -INT -$g 2>/dev/null; done; sleep 5
  for g in $(cat $LD/pgids.txt 2>/dev/null); do kill -KILL -$g 2>/dev/null; done
  fastdds shm clean > /dev/null 2>&1
  cp $LD/route.log $LD/route_try$try.log 2>/dev/null
  grep -q "DEMO_ABORT" ${OUT%.mp4}_drive.log && { sleep 5; continue; }
  PASS=$(grep -o "pass=[A-Za-z]*" $LD/route.log | tail -1)
  echo "  판정 ${PASS:-없음}"
  [ "$PASS" = "pass=True" ] && break
  [ "${FD_DEMO_NEED_PASS:-1}" = 1 ] || break
  sleep 5
done
grep -q "NAV2_ROUTE_DONE" $LD/route.log 2>/dev/null || { echo "DEMO_FAIL — 주행 결과 없음"; exit 4; }
echo "DEMO_REPLAY $(date +%T)"
FD_DEMO_MODE=replay envs/isaacsim_env/bin/python sim/nav2/record_demo.py $OUT > ${OUT%.mp4}_replay.log 2>&1
grep -E "REPLAY |TEXT_WIDTH.*WARN|DEMO_DONE|Traceback|Error" ${OUT%.mp4}_replay.log | grep -v Warning | sed "s/^/  /" | cut -c1-200
FF=$(ls $HOME/.cache/uv/archive-v0/*/imageio_ffmpeg/binaries/ffmpeg-linux-x86_64-* 2>/dev/null | head -1)
if [ -n "$FF" ] && [ -s $OUT ]; then
  $FF -y -loglevel error -i $OUT -c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p -movflags +faststart ${OUT%.mp4}_1080p.mp4
  $FF -y -loglevel error -i $OUT -vf scale=1280:720 -c:v libx264 -preset slow -crf 23 -pix_fmt yuv420p -movflags +faststart ${OUT%.mp4}_720p.mp4
else
  echo "ffmpeg(imageio-ffmpeg) 없음 또는 영상 없음 — 변환 안 함"
fi
ls -la ${OUT%.mp4}*.mp4 2>/dev/null | awk '{print "  " $5, $9}'
echo "DEMO_FILES_DONE $(date +%T)"
