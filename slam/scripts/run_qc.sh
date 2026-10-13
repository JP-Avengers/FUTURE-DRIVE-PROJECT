#!/bin/bash
source /opt/ros/jazzy/setup.bash
cd ~/data
for b in route_119m_10fps_20261005_143357 route_2_10fps_20261005_144303; do
  echo "===== $b"
  sha256sum "$b/bag/bag_0.mcap" | cut -c1-16
  ros2 bag info "$b/bag" > ~/b_slam/logs/${b}_baginfo.txt 2>&1 && echo "bag info OK"
  ~/slam_env/bin/python ~/b_slam/scripts/bag_qc.py "$b/bag" --out ~/b_slam/logs/${b}_qc.md 2>/dev/null
done
