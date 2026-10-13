#!/usr/bin/env bash
# 환경 점검. 실행: bash ~/b_slam/setup/99_check.sh
set -euo pipefail

LOG_DIR=$HOME/b_slam/logs
mkdir -p "$LOG_DIR"
LOG=$LOG_DIR/99_check_$(date +%Y%m%d_%H%M%S).log
exec > >(tee -a "$LOG") 2>&1

ROWS=()
FAILS=0
row() {  # row <OK|--> <항목> <비고>
  ROWS+=("$(printf '%-4s | %-38s | %s' "$1" "$2" "$3")")
  [[ $1 == OK ]] || FAILS=$((FAILS + 1))
}

# ROS_DISTRO (새 대화형 셸)
distro=$(bash -ic 'printenv ROS_DISTRO' 2>/dev/null | tail -n1 || true)
[[ $distro == jazzy ]] && row OK "ROS_DISTRO (bash -ic)" "$distro" || row -- "ROS_DISTRO (bash -ic)" "'${distro}'"

# ROS 패키지
for p in ros-jazzy-desktop ros-dev-tools ros-jazzy-rtabmap-ros ros-jazzy-rtabmap-viz \
         ros-jazzy-rosbag2-storage-mcap ros-jazzy-image-transport-plugins \
         ros-jazzy-tf2-tools ros-jazzy-pcl-ros ros-jazzy-depth-image-proc; do
  if dpkg -s "$p" 2>/dev/null | grep -q '^Status: install ok installed'; then
    row OK "$p" "$(dpkg-query -W -f='${Version}' "$p")"
  else
    row -- "$p" "미설치"
  fi
done

# slam_env
if o3d=$(~/slam_env/bin/python -c 'import open3d, rosbags; print(open3d.__version__)' 2>&1 | tail -n1) \
   && [[ -n $o3d ]] && ~/slam_env/bin/python -c 'import open3d, rosbags' >/dev/null 2>&1; then
  row OK "slam_env: import open3d, rosbags" "open3d $o3d"
else
  row -- "slam_env: import open3d, rosbags" "${o3d:-venv 없음}"
fi

# evo
EVO=$(command -v evo_ape || echo "$HOME/.local/bin/evo_ape")
if [[ -x $EVO ]] && "$EVO" --help >/dev/null 2>&1; then
  row OK "evo_ape" "$EVO"
else
  row -- "evo_ape" "실행 불가"
fi

# talker 5초
if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u; source /opt/ros/jazzy/setup.bash; set -u
  export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  rc=0
  out=$(timeout -s INT 5 ros2 run demo_nodes_cpp talker 2>&1) || rc=$?
  n=$(grep -c 'Publishing' <<<"$out" || true)
  if [[ $rc -eq 124 && $n -gt 0 ]]; then
    row OK "talker 5초 실행" "메시지 ${n}건, SIGINT로 종료"
  else
    row -- "talker 5초 실행" "rc=$rc, 메시지 ${n}건"
  fi
else
  row -- "talker 5초 실행" "/opt/ros/jazzy 없음"
fi

# 디스크
home_avail=$(df -h --output=avail ~ | tail -n1 | tr -d ' ')
row OK "df -h ~" "여유 $home_avail"
c_gb=$(df -BG --output=avail /mnt/c | tail -n1 | tr -dc '0-9')
if [[ $c_gb -ge 100 ]]; then
  row OK "df -h /mnt/c (실제 한계)" "여유 ${c_gb}G"
else
  row -- "df -h /mnt/c (실제 한계)" "경고: 여유 ${c_gb}G < 100G"
fi

# GitHub SSH
gh=$(timeout 20 ssh -o StrictHostKeyChecking=accept-new -o BatchMode=yes -T git@github.com 2>&1 || true)
if grep -q 'successfully authenticated' <<<"$gh"; then
  row OK "ssh git@github.com" "$(grep -o 'Hi [^!]*' <<<"$gh")"
else
  row -- "ssh git@github.com" "$(tail -n1 <<<"$gh")"
fi

echo
printf '%-4s | %-38s | %s\n' "결과" "항목" "비고"
echo "-----+----------------------------------------+-----------------------------"
printf '%s\n' "${ROWS[@]}"
echo
echo "실패 ${FAILS}건 — 로그: $LOG"
