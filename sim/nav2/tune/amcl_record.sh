#!/bin/bash
# sim/nav2/tune/amcl_record.sh — AMCL 조정용 녹화(10.14) — 실제 맵 r3, 자동 교사가 정답 위치로 시계 1바퀴(잡음 없음) + 반시계 1바퀴(잡음 기본값).
# LiDAR 점군 · /scan · /tf · odom · 정답 위치를 녹화해 두고, AMCL 설정을 바꿔 가며 재생으로 시험한다.

cd ~/Capstone_Design_1
OUT=logs/nav2/amcl_tune/bag_r3_cwccw
E="source /opt/ros/jazzy/setup.bash && export ROS_DOMAIN_ID=43 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST"
eval "$E"
rm -rf "$OUT"
tmux new-session -d -s fdsim "cd ~/Capstone_Design_1 && FD_LIDAR=1 FD_DOMAIN=43 FD_SCENE=sim/scenes/parking/parking_v0.4b_r3.usda sim/run_proxy.sh headless"
sleep 5
for i in $(seq 1 40); do sleep 3; L=$(ls -t logs/run_proxy/headless_*.log | head -1); grep -q "PROXY_TIMELINE playing" "$L" 2>/dev/null && break; done
echo "SCENE_PLAYING $(date +%T)"
tmux new-session -d -s fdp2l "cd ~/Capstone_Design_1 && $E && ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r cloud_in:=/front_3d_lidar/lidar_points -r scan:=/scan -p target_frame:=front_3d_lidar -p transform_tolerance:=0.01 -p min_height:=-0.30 -p max_height:=0.47 -p angle_min:=-1.5708 -p angle_max:=1.5708 -p angle_increment:=0.0087 -p scan_time:=0.1 -p range_min:=0.05 -p range_max:=25.0 -p use_inf:=true -p inf_epsilon:=1.0 -p use_sim_time:=true"
sleep 5
tmux new-session -d -s fdrec "cd ~/Capstone_Design_1 && $E && ros2 bag record -s mcap --use-sim-time -o $OUT --topics /front_3d_lidar/lidar_points /scan /tf /chassis/odom /ground_truth/tf /cmd_vel"
sleep 6
echo "REC_STARTED $(date +%T)"
python3 sim/teacher.py --route csv:eval/routes/parking_r3_cw.csv --dir cw --laps 1 --noise-duty 0 --start=-1.756,-7.621,-96.0 2>&1 | grep -E "LAP_END|TEACHER_EXIT" 
python3 sim/teacher.py --route csv:eval/routes/parking_r3_cw.csv --dir ccw --laps 1 --seed 7 --start=-1.756,-7.621,-96.0 2>&1 | grep -E "LAP_END|TEACHER_EXIT"
sleep 3
tmux send-keys -t "=fdrec:" C-c; sleep 8
tmux send-keys -t "=fdp2l:" C-c; sleep 2
tmux send-keys -t "=fdsim:" C-c; sleep 12
for s in fdrec fdp2l fdsim; do tmux kill-session -t "=$s" 2>/dev/null; done
fastdds shm clean > /dev/null 2>&1
ros2 bag info "$OUT" 2>/dev/null | grep -E "Duration|Size|Messages|Topic:" | sed -E "s/ +/ /g; s/Serialization Format: cdr//"
echo "RECORD_DONE $(date +%T)"
