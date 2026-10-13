#!/usr/bin/env python3
"""Bag QC: topics, rates, gaps, image formats, camera_info, tf_static.
Usage: ~/slam_env/bin/python bag_qc.py <bag_dir> [--out report.md]"""
import sys, argparse
from collections import defaultdict
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader

ap = argparse.ArgumentParser()
ap.add_argument('bag')
ap.add_argument('--out')
a = ap.parse_args()

stamps = defaultdict(list)
first = {}
tf_static = []
with AnyReader([Path(a.bag)]) as r:
    for c in r.connections:
        print(f'conn {c.topic} {c.msgtype} qos={c.ext.offered_qos_profiles[:80] if hasattr(c.ext,"offered_qos_profiles") else ""}', file=sys.stderr)
    for c, t, raw in r.messages():
        msg = None
        if c.topic not in first or c.topic == '/tf_static':
            msg = r.deserialize(raw, c.msgtype)
            first.setdefault(c.topic, msg)
            if c.topic == '/tf_static':
                tf_static.extend(msg.transforms)
        if c.msgtype.startswith('sensor_msgs/msg/') and c.msgtype != 'sensor_msgs/msg/CameraInfo' or 'Imu' in c.msgtype:
            if msg is None:
                msg = r.deserialize(raw, c.msgtype)
            h = msg.header.stamp
            stamps[c.topic].append(h.sec + h.nanosec * 1e-9)

L = []
p = L.append
p(f'# Bag QC: `{a.bag}`\n')
p('| topic | count | Hz(mean) | dt median ms | dt max ms | gaps>3x median |')
p('|---|---:|---:|---:|---:|---:|')
for tp, s in sorted(stamps.items()):
    s = np.array(s); d = np.diff(s)
    med = np.median(d)
    p(f'| {tp} | {len(s)} | {(len(s)-1)/(s[-1]-s[0]):.3f} | {med*1e3:.2f} | {d.max()*1e3:.1f} | {(d > 3*med).sum()} |')
p('\n## Images')
for tp, m in first.items():
    if hasattr(m, 'encoding'):
        p(f'- {tp}: {m.width}x{m.height} enc={m.encoding} frame={m.header.frame_id} step={m.step}')
p('\n## CameraInfo')
for tp, m in first.items():
    if hasattr(m, 'k') and hasattr(m, 'distortion_model'):
        k = m.k
        p(f'- {tp}: {m.width}x{m.height} frame={m.header.frame_id} fx={k[0]:.2f} fy={k[4]:.2f} cx={k[2]:.2f} cy={k[5]:.2f} model={m.distortion_model} D={[round(x,4) for x in m.d]}')
p('\n## IMU frames')
for tp, m in first.items():
    if 'imu' in tp or 'accel' in tp or 'gyro' in tp:
        p(f'- {tp}: frame={m.header.frame_id} orient_cov0={m.orientation_covariance[0]}')
p('\n## /tf_static')
for t in tf_static:
    tr, q = t.transform.translation, t.transform.rotation
    p(f'- {t.header.frame_id} -> {t.child_frame_id}: t=({tr.x:.4f},{tr.y:.4f},{tr.z:.4f}) q=({q.x:.4f},{q.y:.4f},{q.z:.4f},{q.w:.4f})')
txt = '\n'.join(L)
print(txt)
if a.out:
    Path(a.out).write_text(txt + '\n')
