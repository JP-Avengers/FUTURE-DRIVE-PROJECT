"""3D LiDAR 로 장애물을 피하며 돌아다니는 워밍업 노드 (Nova Carter · carter_warehouse_navigation.usd).

받는 것: /front_3d_lidar/lidar_points (PointCloud2)   보내는 것: /cmd_vel (Twist)
"""
import math
import signal

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class LidarWander(Node):
    def __init__(self):
        super().__init__("lidar_wander")
        self.declare_parameter("max_speed", 0.5)      # m/s
        self.declare_parameter("stop_dist", 0.9)      # 이보다 가까우면 제자리 회전 (m)
        self.declare_parameter("slow_dist", 1.6)      # 이보다 가까우면 감속 (m)
        self.declare_parameter("z_min", -0.35)        # 라이다 기준 높이 — 바닥(약 -0.53 m) 제외
        self.declare_parameter("z_max", 0.8)
        self.declare_parameter("steer_gain", 0.3)     # 트인 쪽으로 기우는 정도 (0 이면 곧장 직진)
        self.sub = self.create_subscription(
            PointCloud2, "/front_3d_lidar/lidar_points", self.on_cloud, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.last_cloud = self.get_clock().now()
        self.turn_dir = 1.0                           # 회전 중엔 방향 유지 (좌우 떨림 방지)
        self.turning = False
        self.create_timer(0.2, self.watchdog)
        self.create_timer(1.0, self.report)
        self.state = "라이다 기다리는 중"
        self.front = self.left = self.right = float("nan")
        self.cmd = Twist()

    def sector_min(self, ang, dist, lo, hi):
        m = (ang >= math.radians(lo)) & (ang < math.radians(hi))
        return float(dist[m].min()) if m.any() else 99.0

    def on_cloud(self, msg):
        self.last_cloud = self.get_clock().now()
        p = self.get_parameter
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
        pts = pts[np.isfinite(pts).all(axis=1)]
        pts = pts[(pts[:, 2] > p("z_min").value) & (pts[:, 2] < p("z_max").value)]
        dist = np.hypot(pts[:, 0], pts[:, 1])
        keep = dist > 0.6                             # 로봇 자기 몸체 제외
        ang, dist = np.arctan2(pts[keep, 1], pts[keep, 0]), dist[keep]

        front = self.sector_min(ang, dist, -25, 25)
        left = self.sector_min(ang, dist, 25, 80)
        right = self.sector_min(ang, dist, -80, -25)

        cmd = Twist()
        vmax, stop, slow = p("max_speed").value, p("stop_dist").value, p("slow_dist").value
        if self.turning:
            if front > stop + 0.4:                    # 충분히 트였으면 회전 끝
                self.turning = False
        elif front < stop:
            self.turning = True
            self.turn_dir = 1.0 if left >= right else -1.0

        if self.turning:
            cmd.angular.z = 0.8 * self.turn_dir
            self.state = "회전"
        elif front < slow:
            cmd.linear.x = 0.4 * vmax
            cmd.angular.z = 0.6 if left >= right else -0.6
            self.state = "감속"
        else:
            cmd.linear.x = vmax
            gain = p("steer_gain").value
            cmd.angular.z = float(np.clip(gain * (min(left, 4.0) - min(right, 4.0)), -0.4, 0.4))
            self.state = "직진"
        self.pub.publish(cmd)
        self.front, self.left, self.right, self.cmd = front, left, right, cmd

    def watchdog(self):
        # 라이다가 0.5초 넘게 끊기면 정지 — 시뮬을 Pause 했을 때 로봇이 계속 달리지 않게
        if (self.get_clock().now() - self.last_cloud).nanoseconds > 5e8:
            self.cmd = Twist()                        # 로그에도 실제로 보낸 정지 명령이 찍히게
            self.pub.publish(self.cmd)
            self.state = "대기(라이다 없음)"

    def report(self):
        self.get_logger().info(
            f"{self.state:<4} 앞 {self.front:5.2f}  좌 {self.left:5.2f}  우 {self.right:5.2f} m"
            f"  →  v {self.cmd.linear.x:.2f}  w {self.cmd.angular.z:+.2f}")


def _on_term(*_):
    raise KeyboardInterrupt                           # SIGTERM(kill)도 Ctrl+C 와 같은 정리 경로로


def main():
    # rclpy 기본 신호 처리는 Ctrl+C 에서 ROS 연결부터 닫아 마지막 정지 명령을 못 보냄(9.28 확인)
    # → 신호 처리를 끄고 직접 받는다
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGTERM, _on_term)
    node = LidarWander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())                     # 끌 때 반드시 정지 명령
        node.get_logger().info("정지 명령 보냄 — 종료")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
