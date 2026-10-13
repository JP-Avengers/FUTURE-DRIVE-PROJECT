#!/bin/bash
# amcl_replay.sh <설정 yaml> <ROS 도메인> <이름> [재생 배속] — amcl_record.sh 녹화를 재생하며 AMCL 만 돌려 정답과 비교.
# 결과 logs/nav2/amcl_tune/res_<이름>.json · eval_<이름>.txt. 도메인을 달리하면 여러 설정을 동시에 돌릴 수 있음(10.14 8개 동시).
# 지도는 MAPF(기본 화단 안 채운 위치 추정용 map.yaml). 시뮬은 안 띄움.
CFG=$1; DOM=$2; NAME=$3; RATE=${4:-2}
cd ~/Capstone_Design_1
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=$DOM RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
D=logs/nav2/amcl_tune; BAG=$D/bag_r3_cwccw; LOG=$D/log_$NAME.txt
setsid ros2 launch sim/nav2/nav2.launch.py map:=$PWD/${MAPF:-data/nav2/parking_v0.4b_r3_loc/map.yaml} localization:=amcl navigation:=false params_file:=$PWD/$CFG > $LOG 2>&1 &
LP=$!
for i in $(seq 1 40); do grep -q "Managed nodes are active" $LOG && break; sleep 1; done
python3 sim/nav2/tune/amcl_eval.py $NAME $D/res_$NAME.json > $D/eval_$NAME.txt 2>&1 &
EP=$!
sleep 2
ros2 bag play $BAG --clock 100 --rate $RATE --disable-keyboard-controls --topics /front_3d_lidar/lidar_points /tf /chassis/odom /ground_truth/tf > $D/play_$NAME.txt 2>&1
wait $EP
kill -INT -$LP 2>/dev/null; sleep 6; kill -TERM -$LP 2>/dev/null; sleep 2
grep EVAL $D/eval_$NAME.txt
