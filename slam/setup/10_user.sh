#!/usr/bin/env bash
# sudo 없는 사용자 설정. 실행: bash ~/b_slam/setup/10_user.sh
set -euo pipefail

LOG_DIR=$HOME/b_slam/logs
mkdir -p "$LOG_DIR"
LOG=$LOG_DIR/10_user_$(date +%Y%m%d_%H%M%S).log
exec > >(tee -a "$LOG") 2>&1

step() { echo; echo "=== $* === ($(date +%T))"; }
add_line() { grep -qxF "$1" "$2" 2>/dev/null || echo "$1" >> "$2"; }

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "/opt/ros/jazzy 없음 → 00_system.sh 먼저 실행" >&2
  exit 1
fi

step "1. ~/.bashrc ROS 설정"
add_line 'source /opt/ros/jazzy/setup.bash' ~/.bashrc
add_line 'export ROS_DOMAIN_ID=42' ~/.bashrc
add_line 'export RMW_IMPLEMENTATION=rmw_fastrtps_cpp' ~/.bashrc

# ROS setup 스크립트는 미정의 변수를 참조하므로 source 동안만 -u 해제
set +u
source /opt/ros/jazzy/setup.bash
set -u
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp

step "2. rosdep update"
rosdep update

step "3. ~/ros2_ws 빈 워크스페이스 빌드"
mkdir -p ~/ros2_ws/src
(cd ~/ros2_ws && colcon build)

step "4. Python venv ~/slam_env"
[[ -x ~/slam_env/bin/python ]] || python3 -m venv ~/slam_env
~/slam_env/bin/pip install --retries 5 --timeout 30 -U pip
~/slam_env/bin/pip install --retries 5 --timeout 30 \
  open3d numpy scipy pandas matplotlib opencv-python rosbags

step "5. evo (pipx)"
if [[ -x ~/.local/bin/evo_ape ]]; then
  echo "evo 이미 설치됨 → 건너뜀"
else
  # 중단된 설치로 venv만 남은 경우도 다시 설치
  pipx install --force evo
fi
pipx ensurepath

step "6. 별칭 slam"
add_line "alias slam='source ~/slam_env/bin/activate'" ~/.bashrc

step "7. 작업 폴더"
mkdir -p ~/b_slam/{scripts,runs,exports,logs,setup} ~/data ~/maps

step "8. Git / SSH 키"
git config --global user.name ynso-a8
git config --global user.email ynso1248@naver.com
if [[ -f ~/.ssh/id_ed25519 ]]; then
  echo "~/.ssh/id_ed25519 이미 있음 → 생성 건너뜀"
else
  mkdir -p ~/.ssh && chmod 700 ~/.ssh
  ssh-keygen -t ed25519 -N "" -C "ynso1248@naver.com" -f ~/.ssh/id_ed25519
fi
echo "--- GitHub에 등록할 공개키 ---"
cat ~/.ssh/id_ed25519.pub

step "완료 — 로그: $LOG"
