#!/usr/bin/env python3
"""파이프라인 각 단계 토픽 수신 개수 집계 (raw 구독, 역직렬화 없음). 사용: topic_counter.py <seconds>"""
import sys, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
NS = '/camera/camera'
T = {f'{NS}/color/image_raw': Image, f'{NS}/depth/image_rect_raw': Image, f'{NS}/color/camera_info': CameraInfo,
     f'{NS}/aligned_depth_to_color/image_raw': Image, '/odom': Odometry}
rclpy.init(); n = Node('topic_counter'); cnt = {t: 0 for t in T}
qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
for t, ty in T.items():
    n.create_subscription(ty, t, lambda m, t=t: cnt.__setitem__(t, cnt[t] + 1), qos, raw=True)
end = time.time() + float(sys.argv[1])
while time.time() < end: rclpy.spin_once(n, timeout_sec=0.1)
for t, c in cnt.items(): print(f'{c:6d}  {t}')
