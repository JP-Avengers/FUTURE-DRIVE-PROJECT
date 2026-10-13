"""재생 중 /amcl_pose 를 정답(/ground_truth/tf, 같은 시뮬 시각)과 비교 — 정답이 6 s(벽시계) 끊기면 끝내고 요약 JSON 한 줄."""
import bisect, json, math, sys, time, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
NAME, OUT = sys.argv[1], sys.argv[2]
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
rclpy.init(); n = Node("amcl_eval"); n.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
gt, am, last = [], [], [None]
def on_gt(m):
    for t in m.transforms:
        if t.header.frame_id == "World" and t.child_frame_id == "nova_carter":
            gt.append((t.header.stamp.sec + t.header.stamp.nanosec * 1e-9, t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)))
            last[0] = time.time()
def on_am(m):
    p = m.pose.pose; am.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.position.x, p.position.y, yaw(p.orientation)))
n.create_subscription(TFMessage, "/ground_truth/tf", on_gt, qos_profile_sensor_data)
n.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_am, QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0 = time.time()
while True:
    rclpy.spin_once(n, timeout_sec=0.05)
    if last[0] and time.time() - last[0] > 6: break
    if not last[0] and time.time() - t0 > 120: break
gt.sort(); ts = [g[0] for g in gt]; rows = []
for s, x, y, th in am:
    i = bisect.bisect_left(ts, s)
    if not ts or i >= len(ts): continue
    g = gt[i]; rows.append((round(s, 2), math.hypot(x - g[1], y - g[2]), math.degrees(abs((th - g[3] + math.pi) % (2 * math.pi) - math.pi))))
e = sorted(r[1] for r in rows); a = sorted(r[2] for r in rows)
q = lambda v, p: v[min(len(v) - 1, int(p * len(v)))] if v else float("nan")
jumps = sum(1 for r0, r1 in zip(rows, rows[1:]) if r1[1] - r0[1] > 1.0)
summ = {"name": NAME, "updates": len(rows), "gt_span": round(ts[-1] - ts[0], 1) if ts else 0,
        "pos_med": round(q(e, .5), 3), "pos_p95": round(q(e, .95), 3), "pos_max": round(max(e), 3) if e else None,
        "yaw_p95": round(q(a, .95), 2), "yaw_max": round(max(a), 2) if a else None,
        "over05": round(sum(v > 0.5 for v in e) / max(len(e), 1), 3), "jumps": jumps, "final": round(rows[-1][1], 3) if rows else None}
json.dump({**summ, "rows": [[r[0], round(r[1], 3), round(r[2], 2)] for r in rows]}, open(OUT, "w"))
print("EVAL " + json.dumps(summ, ensure_ascii=False), flush=True)
