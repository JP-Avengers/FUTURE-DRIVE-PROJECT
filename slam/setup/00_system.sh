#!/usr/bin/env bash
# sudo가 필요한 시스템 설정. 실행: sudo bash ~/b_slam/setup/00_system.sh
set -euo pipefail

if [[ $EUID -ne 0 ]]; then
  echo "root 권한 필요: sudo bash $0" >&2
  exit 1
fi

TARGET_USER=${SUDO_USER:-$(logname 2>/dev/null || echo root)}
TARGET_HOME=$(getent passwd "$TARGET_USER" | cut -d: -f6)
LOG_DIR=$TARGET_HOME/b_slam/logs
mkdir -p "$LOG_DIR"
LOG=$LOG_DIR/00_system_$(date +%Y%m%d_%H%M%S).log
exec > >(tee -a "$LOG") 2>&1
trap 'chown -R "$TARGET_USER:" "$LOG_DIR" 2>/dev/null || true' EXIT

export DEBIAN_FRONTEND=noninteractive
APT_OPTS=(-o Acquire::Retries=5 -o Acquire::http::Timeout=30)
apt_get() { apt-get "${APT_OPTS[@]}" "$@"; }
step() { echo; echo "=== $* === ($(date +%T))"; }

SRC=/etc/apt/sources.list.d/ubuntu.sources

step "1. Ubuntu 미러 → 카카오"
[[ -f $SRC.orig ]] || cp -a "$SRC" "$SRC.orig"
sed -i -E 's#https?://(kr\.)?archive\.ubuntu\.com/ubuntu/?#https://mirror.kakao.com/ubuntu/#g' "$SRC"
grep -E '^URIs:' "$SRC"

step "2. universe, multiverse 활성화"
for comp in universe multiverse; do
  sed -i -E "/^Components:/{/\b${comp}\b/! s/\$/ ${comp}/}" "$SRC"
done
grep -E '^Components:' "$SRC"

step "3. 릴리스 업그레이드 차단 (Prompt=never)"
RU=/etc/update-manager/release-upgrades
if [[ -f $RU ]] && grep -qE '^Prompt=' "$RU"; then
  sed -i -E 's/^Prompt=.*/Prompt=never/' "$RU"
else
  mkdir -p "$(dirname "$RU")"
  [[ -f $RU ]] || echo "[DEFAULT]" > "$RU"
  echo "Prompt=never" >> "$RU"
fi
grep -E '^Prompt=' "$RU"

step "4. 로케일 en_US.UTF-8"
apt_get update
apt_get install -y locales
locale-gen en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

step "5. 기본 도구"
# 이미 설치된 패키지의 보안·버그 업데이트만 (릴리스는 바뀌지 않음)
apt_get upgrade -y
apt_get install -y build-essential cmake git curl wget vim htop tmux \
  python3-venv python3-pip pipx net-tools file ca-certificates

step "6. ROS 2 Jazzy 저장소"
if dpkg -s ros2-apt-source >/dev/null 2>&1; then
  echo "ros2-apt-source 이미 설치됨 → 건너뜀"
else
  TAG=$(curl -fsSL --retry 5 --max-time 30 \
          https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
        | grep -F '"tag_name"' | awk -F'"' '{print $4}' || true)
  if [[ -z ${TAG:-} ]]; then
    echo "ros-apt-source 최신 태그를 읽지 못함 (GitHub API 제한/네트워크 확인) → 중단" >&2
    exit 1
  fi
  CODENAME=$(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
  DEB=/tmp/ros2-apt-source_${TAG}.${CODENAME}_all.deb
  curl -fL --retry 5 --max-time 120 -o "$DEB" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${TAG}/ros2-apt-source_${TAG}.${CODENAME}_all.deb"
  if ! file "$DEB" | grep -q 'Debian binary package'; then
    echo "다운로드한 파일이 Debian 패키지가 아님: $(file "$DEB")" >&2
    exit 1
  fi
  dpkg -i "$DEB"
fi
# packages.ros.org는 *.osuosl.org 인증서를 내보내 https 호스트명 불일치 → http 사용 (무결성은 GPG 서명으로 검증)
grep -rlF 'https://packages.ros.org' /etc/apt/sources.list.d/ 2>/dev/null \
  | xargs -r sed -i 's#https://packages\.ros\.org#http://packages.ros.org#g' || true
grep -rhE '^(URIs:|deb ).*packages\.ros\.org' /etc/apt/sources.list.d/ || true

step "7-8. ROS 패키지 설치"
apt_get update
apt_get install -y \
  ros-jazzy-desktop ros-dev-tools \
  ros-jazzy-rtabmap-ros ros-jazzy-rtabmap-viz \
  ros-jazzy-rosbag2-storage-mcap ros-jazzy-image-transport-plugins \
  ros-jazzy-tf2-tools ros-jazzy-pcl-ros ros-jazzy-depth-image-proc

step "9. rosdep init"
if [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  echo "rosdep 이미 초기화됨 → 건너뜀"
else
  rosdep init
fi

step "완료 — 로그: $LOG"
