#!/bin/bash
# run_rtabmap.sh 를 최대 3번 시도 (컴포넌트 로드 실패 = 종료 코드 2 일 때만 재시도; WSL에서 디스커버리가 간헐적으로 실패함)
# 사용법·환경변수는 run_rtabmap.sh 와 같음
RUN=$2
for k in 1 2 3; do
  ~/b_slam/scripts/run_rtabmap.sh "$@"; rc=$?
  [ $rc -ne 2 ] && exit $rc
  echo "재시도 $k/3: 로드 실패 정리 후 다시 시작"
  mv ~/b_slam/logs/$RUN ~/b_slam/logs/${RUN}_loadfail$k 2>/dev/null; rm -f ~/maps/$RUN.db
  source /opt/ros/jazzy/setup.bash; ros2 daemon stop >/dev/null 2>&1; fastdds shm clean >/dev/null 2>&1; sleep 5
done
exit 2
