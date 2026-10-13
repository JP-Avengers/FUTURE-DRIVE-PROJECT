#!/bin/bash
# RTAB-Map 헤드리스 실행 (단일 컨테이너 intra-process: 재생→정렬→오도메트리→SLAM)
# 사용: run_rtabmap.sh <bag_dir> <run_name> [rate=0.5] [rtabmap_args...]
#   예) run_rtabmap.sh ~/data/car_test_3_20261005_172323_657787/bag run01
#       run_rtabmap.sh ~/data/.../bag run02 0.5 --Vis/MaxDepth 6
# 환경변수 DURATION=초 로 앞부분만 재생 (스모크 테스트용), IMU=true 로 IMU 중력 제약 사용,
#   INIT_DB=<기존.db> 로 그 DB의 복사본에 이어서 매핑(multi-session; 원본 DB는 건드리지 않음),
#   BAG_ALIGNED=true 로 bag의 정렬 depth를 그대로 사용(10/9 이후 촬영본; depth 정렬 노드 생략)
source /opt/ros/jazzy/setup.bash
fastdds shm clean > /dev/null 2>&1  # 이전 실행이 남긴 공유 메모리 잔여물 정리
# WSL mirrored 네트워크에서 서브넷 디스커버리가 같은 PC 프로세스를 못 찾음 → localhost로 한정
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS=127.0.0.1  # LOCALHOST만으로는 디스커버리가 느리거나 실패할 때가 있음
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$(( RANDOM % 100 + 1 ))}  # 실행마다 다른 도메인: 동시에 두 실행이 돌아도 서로의 토픽·/clock이 섞이지 않게
set -u
set -m  # 잡 컨트롤: 백그라운드 프로세스가 SIGINT를 무시하지 않게
BAG=$(realpath "$1"); RUN=$2; RATE=${3:-0.5}; shift 3 2>/dev/null || shift $#
EXTRA="$*"; DUR=${DURATION:-0}; USE_IMU=${IMU:-false}; USE_BA=${BAG_ALIGNED:-false}
DB=~/maps/$RUN.db; LOG=~/b_slam/logs/$RUN; mkdir -p "$LOG"
[ -e "$DB" ] && { echo "이미 존재: $DB (덮어쓰지 않음)"; exit 1; }
if [ -n "${INIT_DB:-}" ]; then cp "$INIT_DB" "$DB" && echo "multi-session: $INIT_DB 복사본에서 시작"; fi
BAGLEN=$(python3 -c "import yaml;print(yaml.safe_load(open('$BAG/metadata.yaml'))['rosbag2_bagfile_information']['duration']['nanoseconds']/1e9)")
PLAYLEN=$(python3 -c "d=$DUR;print(d if d>0 else $BAGLEN)")
WAIT=$(python3 -c "print(int($PLAYLEN/$RATE + 20))")
{ echo "bag=$BAG"; echo "rate=$RATE"; echo "rtabmap_args=$EXTRA"; echo "duration=$DUR"; echo "imu=$USE_IMU"; echo "ros_domain_id=$ROS_DOMAIN_ID"; echo "bag_aligned=$USE_BA"; echo "init_db=${INIT_DB:-}"; date -Is; } > "$LOG/params.txt"
ros2 launch ~/b_slam/scripts/rtabmap_intra.launch.py bag:="$BAG" db:="$DB" rate:="$RATE" duration:="$DUR" imu:="$USE_IMU" bag_aligned:="$USE_BA" \
  ${EXTRA:+"rtabmap_args:=$EXTRA"} > "$LOG/rtabmap.log" 2>&1 &
L=$!
# 컴포넌트 로드 확인: 60초 안에 필요한 노드(재생·오도메트리·SLAM + IMU 필터 + depth 정렬)가 로드되지 않으면 중단
NEED=$(( 3 + $( [ "$USE_IMU" = true ] && echo 1 || echo 0 ) + $( [ "$USE_BA" = true ] && echo 0 || echo 1 ) ))
for i in $(seq 60); do [ $(grep -c "Loaded node" "$LOG/rtabmap.log" 2>/dev/null) -ge $NEED ] && break; sleep 1; done
if [ $(grep -c "Loaded node" "$LOG/rtabmap.log") -lt $NEED ]; then echo "오류: 컴포넌트 로드 실패 → 중단 (fastdds shm clean 후 재시도)"; kill -INT $L; sleep 5; kill -TERM $L 2>/dev/null; exit 2; fi
echo "실행 중: $RUN (재생 ${PLAYLEN%.*}s / rate $RATE → 약 ${WAIT}s 후 종료)"
sleep "$WAIT"
kill -INT $L
for i in $(seq 60); do kill -0 $L 2>/dev/null || break; sleep 1; done
kill -0 $L 2>/dev/null && { echo "경고: 60초 내 종료 안 됨 → TERM"; kill -TERM $L; sleep 5; }
wait $L 2>/dev/null
echo "완료: $DB  로그: $LOG/rtabmap.log"
echo "odom 업데이트: $(grep -c 'Odom: quality' "$LOG/rtabmap.log")  실패(quality=0): $(grep -c 'quality=0,' "$LOG/rtabmap.log")"
