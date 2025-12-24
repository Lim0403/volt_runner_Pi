#!/usr/bin/env python3
# mecanum_fk_node.py

import math
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped

import tf2_ros
from tf_transformations import quaternion_from_euler


def forward_kinematics(w_fl, w_fr, w_rl, w_rr, r, lx, ly):
    """
    메카넘 휠 전진 기구학 (FK)

    인자 순서:
      w_fl : front-left
      w_fr : front-right
      w_rl : rear-left
      w_rr : rear-right
    """
    l_sum = lx + ly

    vx = (r / 4.0) * (w_fl + w_fr + w_rl + w_rr)
    vy = (r / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
    omega = (r / 4.0) * (-w_fl + w_fr - w_rl + w_rr) / l_sum

    return vx, vy, omega


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        # 실제 데이터가 나오는 토픽: /wheel_feedback_rpm
        # (mecanum_serial_bridge 가 퍼블리시하는 값)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/wheel_feedback_rpm',
            self.feedback_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 메카넘 파라미터(실측값)
        self.r = 0.04   # 바퀴 반지름 (m)
        self.lx = 0.10  # x축(중심~바퀴) m
        self.ly = 0.088 # y축(중심~바퀴) m

        # 상태 변수 (odom 좌표계에서의 위치/자세)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now().nanoseconds / 1e9

        self.get_logger().info("OdometryPublisher started. Subscribing /wheel_feedback_rpm")

    def feedback_callback(self, msg: Float32MultiArray):
        now = self.get_clock().now().nanoseconds / 1e9
        dt = now - self.last_time
        self.last_time = now

        rpm_raw = list(msg.data)
        if len(rpm_raw) != 4:
            self.get_logger().warn(f"wheel_feedback_rpm length != 4: {rpm_raw}")
            return

        # =======================================
        # Teensy → 시리얼 → mecanum_serial_bridge 순서:
        #   [FL, FR, RR, RL]
        # 우리는 FK에 넣기 전에 [FL, FR, RL, RR] 로 재배열.
        # =======================================
        rpm_fl = rpm_raw[0]  # FL
        rpm_fr = rpm_raw[1]  # FR
        rpm_rr = rpm_raw[2]  # RR
        rpm_rl = rpm_raw[3]  # RL

        # FK 함수 입력용 순서: [FL, FR, RL, RR]
        rpm_for_fk = [rpm_fl, rpm_fr, rpm_rl, rpm_rr]

        # rpm → rad/s
        ws = [v * 2.0 * math.pi / 60.0 for v in rpm_for_fk]
        w_fl, w_fr, w_rl, w_rr = ws

        # FK 계산 (base_link 기준 속도)
        vx, vy, omega = forward_kinematics(
            w_fl, w_fr, w_rl, w_rr,
            self.r, self.lx, self.ly
        )

        # 적분 (odom 좌표계)
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

        # TF 브로드캐스트 (odom → base_link)
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
    node = OdometryPublisher()
    try:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
