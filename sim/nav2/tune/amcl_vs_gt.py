"""AMCL 추정(/amcl_pose)과 정답 위치(/ground_truth/tf)를 같은 시뮬 시각끼리 비교 — 시간에 따른 위치 · 방향 오차."""
import bisect, json, math, sys, time, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
DUR = float(sys.argv[1]); OUT = sys.argv[2]
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
rclpy.init(); n = Node("amcl_vs_gt"); n.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
gt, am = [], []
def on_gt(m):
    for t in m.transforms:
        if t.header.frame_id == "World" and t.child_frame_id == "nova_carter":
            gt.append((t.header.stamp.sec + t.header.stamp.nanosec * 1e-9, t.transform.translation.x, t.transform.translation.y, yaw(t.transform.rotation)))
def on_am(m):
    p = m.pose.pose
    am.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.position.x, p.position.y, yaw(p.orientation)))
n.create_subscription(TFMessage, "/ground_truth/tf", on_gt, qos_profile_sensor_data)
n.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_am, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
t0 = time.time()
try:
    while time.time() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.1)
except Exception:   # Ctrl+C → rclpy 가 ExternalShutdownException 으로 바꿔 던짐
    pass
gt.sort(); ts = [g[0] for g in gt]; rows = []
for s, x, y, th in am:
    i = bisect.bisect_left(ts, s)
    if not ts or i >= len(ts): continue
    g = gt[i]
    rows.append((round(s, 2), round(math.hypot(x - g[1], y - g[2]), 3), round(math.degrees(abs((th - g[3] + math.pi) % (2 * math.pi) - math.pi)), 2), round(g[1], 2), round(g[2], 2)))
json.dump(rows, open(OUT, "w"))
if rows:
    e = [r[1] for r in rows]; a = [r[2] for r in rows]
    print(f"AMCL {len(rows)}회 갱신 · 위치 오차 중앙 {sorted(e)[len(e)//2]:.2f} · 최대 {max(e):.2f} m · 방향 오차 최대 {max(a):.1f}° · 0.5 m 넘음 {sum(v > 0.5 for v in e)}회")
    for r in rows[:: max(1, len(rows) // 12)]: print("  ", r)
else:
    print("AMCL 갱신 없음")
