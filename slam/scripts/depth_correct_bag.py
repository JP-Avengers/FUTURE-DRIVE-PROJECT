#!/usr/bin/env python3
"""D435i depth 거리 비례 오차 보정 bag 생성: Z' = Z / (1 + c·Z)   (시차 오프셋 모델, Z는 m)
원본 depth(/camera/camera/depth/image_rect_raw)와, 있으면 정렬 depth(/camera/camera/aligned_depth_to_color/image_raw)도
같은 식으로 바꾸고(정렬 depth도 픽셀 값이 같은 거리 Z), 나머지 메시지·QoS·타임스탬프는 그대로 복사. 원본은 읽기만 함.
메모리: 쓰는 동안 원본·출력 파일 페이지 캐시를 주기적으로 비움(posix_fadvise).
사용: depth_correct_bag.py <src_bag_dir> <dst_bag_dir> <c>     (시스템 python3 + ROS 2 source 상태에서)"""
import sys
from pathlib import Path
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import Image

src, dst, c = sys.argv[1], sys.argv[2], float(sys.argv[3])
DEPTH = ('/camera/camera/depth/image_rect_raw', '/camera/camera/aligned_depth_to_color/image_raw')
import os
def drop(*dirs):
    for d in dirs:
        for f in Path(d).glob('*.mcap'):
            fd = os.open(f, os.O_RDONLY); os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED); os.close(fd)
if Path(dst).exists(): sys.exit(f'이미 존재: {dst} (덮어쓰지 않음)')

r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=src, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=dst, storage_id='mcap'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
for t in r.get_all_topics_and_types(): w.create_topic(t)

lut = np.arange(65536, dtype=np.float64)
lut = np.where(lut > 0, np.clip(np.round(lut / (1 + c * lut / 1000.0)), 1, 65535), 0).astype(np.uint16)  # mm → mm
n = 0; cnt = {}; k = 0
while r.has_next():
    topic, data, t = r.read_next()
    k += 1
    if k % 5000 == 0: drop(src, dst)
    if topic in DEPTH:
        m = deserialize_message(data, Image)
        assert m.encoding in ('16UC1', 'mono16') and not m.is_bigendian
        d = np.frombuffer(bytes(m.data), np.uint16)
        m.data = lut[d].tobytes(); data = serialize_message(m); n += 1; cnt[topic] = cnt.get(topic, 0) + 1
    w.write(topic, data, t)
del w
drop(src, dst)
print(f'depth {n}장 보정 ' + ', '.join(f'{t.split("/")[3]} {v}' for t, v in cnt.items()) + f' (c={c}/m: 5m {1/(1+5*c):.3f}배, 10m {1/(1+10*c):.3f}배, 20m {1/(1+20*c):.3f}배) → {dst}')
