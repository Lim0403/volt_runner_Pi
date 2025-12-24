#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf_transformations import quaternion_from_euler
import tf2_ros


def forward_kinematics(w1, w2, w3, w4, r, lx, ly):
    """
    메카넘 휠 전진 기구학 (FK)

    인자 순서:
      w1: front-left  (FL)
      w2: front-right (FR)
      w3: rear-left   (RL)
      w4: rear-right  (RR)
    """
    l_sum = lx + ly

    # x축 속도
    vx = (r / 4.0) * (w1 + w2 + w3 + w4)

    # y축 속도 (기존처럼 부호 한 번 더 뒤집은 버전 유지)
    vy = -(r / 4.0) * (-w1 + w2 + w3 - w4)

    # 각속도 (회전 방향 보정 추가)
    omega = (r / 4.0) * (-w1 + w2 - w3 + w4) / l_sum
    omega = -omega  # 🔥 RViz 회전 방향 실제와 맞추기

    return vx, vy, omega


class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # 바퀴별 실시간 rpm 들어오는 토픽
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/wheel_feedback_rpm',   # 실제 토픽 이름 확인
            self.feedback_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 메카넘 파라미터 (실측값 사용)
        self.r = 0.04
        self.lx = 0.10
        self.ly = 0.088

        # 상태 변수 (odom 좌표계에서의 포즈)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now().nanoseconds / 1e9

    def feedback_callback(self, msg: Float32MultiArray):
        # dt 계산
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time

        # 이상한 dt 방어 (처음 콜백이나 시간 점프 등)
        if dt <= 0.0 or dt > 1.0:
            self.last_time = now
            return

        self.last_time = now

        rpm = list(msg.data)
        if len(rpm) != 4:
            self.get_logger().warn(
                f"Received {len(rpm)} wheel values! Expected 4. data={rpm}"
            )
            return

        # 🔥 실제 들어오는 순서 [FL, FR, RR, RL] → FK용 [FL, FR, RL, RR]으로 재배열
        rpm_fl = rpm[0]
        rpm_fr = rpm[1]
        rpm_rr = rpm[2]
        rpm_rl = rpm[3]

        rpm_for_fk = [rpm_fl, rpm_fr, rpm_rl, rpm_rr]

        # rpm → rad/s 변환
        ws = [v * 2.0 * math.pi / 60.0 for v in rpm_for_fk]
        w1, w2, w3, w4 = ws  # FL, FR, RL, RR

        # FK: base_link 기준 속도
        vx, vy, omega = forward_kinematics(
            w1, w2, w3, w4,
            self.r, self.lx, self.ly
        )

        # ---- odom 좌표계에서 적분 ----
        dx = vx * math.cos(self.th) - vy * math.sin(self.th)
        dy = vx * math.sin(self.th) + vy * math.cos(self.th)

        self.x += dx * dt
        self.y += dy * dt
        self.th += omega * dt

        # 쿼터니언 변환
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.th)

        # Odometry 메시지 생성
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(
            x=qx, y=qy, z=qz, w=qw
        )

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        self.odom_pub.publish(odom)

        # TF: odom → base_link
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
