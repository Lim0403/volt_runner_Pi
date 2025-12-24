#!/usr/bin/env python3
# mecanum_ik_node.py

import math
import functools

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray


def inverse_kinematics(vx, vy, omega, r, lx, ly):
    """
    메카넘 휠 역기구학
    return: [FL, FR, RL, RR] (rad/s)
    """
    l_sum = lx + ly

    w_fl = (1.0 / r) * (vx - vy - l_sum * omega)  # FL
    w_fr = (1.0 / r) * (vx + vy + l_sum * omega)  # FR
    w_rl = (1.0 / r) * (vx + vy - l_sum * omega)  # RL
    w_rr = (1.0 / r) * (vx - vy + l_sum * omega)  # RR

    return [w_fl, w_fr, w_rl, w_rr]


class MecanumIKNode(Node):
    def __init__(self):
        super().__init__('mecanum_ik_node')

        # === 메카넘 파라미터 ===
        self.r = 0.04
        self.lx = 0.10
        self.ly = 0.088

        # === 최소 클램핑 ===
        self.min_linear_x = 0.3
        self.min_angular_z = 0.3
        self.enable_clamp = True

        # === 마지막 명령(Last one wins) 상태 저장 ===
        self.last_cmd: Twist = None
        self.last_source: str = None

        # === 구독자: 3개의 속도 명령 모두 받기 ===
        self.sub_manual = self.create_subscription(
            Twist,
            '/cmd_vel',
            functools.partial(self.cmd_vel_callback, source="manual"),
            10
        )

        self.sub_nav2 = self.create_subscription(
            Twist,
            '/cmd_vel_nav2',
            functools.partial(self.cmd_vel_callback, source="nav2"),
            10
        )

        self.sub_rl = self.create_subscription(
            Twist,
            '/cmd_rl',
            functools.partial(self.cmd_vel_callback, source="rl"),
            10
        )

        # === Teensy로 보낼 휠 RPM ===
        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/wheel_target_rpm',
            10
        )

        self.get_logger().info(
            "Mecanum IK Node Started | "
            f"r={self.r}, lx={self.lx}, ly={self.ly}, "
            f"clamp={self.enable_clamp}"
        )

    # ============================================================
    # 🔥 공통 콜백: "어떤 cmd_vel이든 들어오면 적용"
    # ============================================================
    def cmd_vel_callback(self, msg: Twist, source: str):
        self.last_cmd = msg
        self.last_source = source

        # 가장 최신으로 들어온 명령을 사용 → last one wins
        self.apply_cmd(msg, source)

    # ============================================================
    # 🔥 cmd_vel → inverse kinematics → rpm publish
    # ============================================================
    def apply_cmd(self, msg: Twist, source: str):
        raw_vx = msg.linear.x
        raw_vy = msg.linear.y
        raw_omega = msg.angular.z

        # === 부호 보정 ===
        vx = raw_vx
        vy = -raw_vy
        omega = -raw_omega

        # === 최소 속도 클램핑 ===
        if self.enable_clamp:

            # 전진/후진
            if abs(vx) > 0 and abs(vx) < self.min_linear_x:
                vx = math.copysign(self.min_linear_x, vx)

            # 회전
            if abs(omega) > 0 and abs(omega) < self.min_angular_z:
                omega = math.copysign(self.min_angular_z, omega)

        # === 역기구학 ===
        w_list = inverse_kinematics(vx, vy, omega, self.r, self.lx, self.ly)

        # === rad/s → rpm ===
        rpm_list = [w * 60.0 / (2.0 * math.pi) for w in w_list]

        msg_out = Float32MultiArray()
        msg_out.data = rpm_list
        self.publisher.publish(msg_out)

        # === 디버그 출력 ===
        self.get_logger().info(
            f"[{source}] vx={raw_vx:.3f}, vy={raw_vy:.3f}, w={raw_omega:.3f} | "
            f"after fix: vx={vx:.3f}, vy={vy:.3f}, w={omega:.3f} | "
            f"rpm={ [round(r,1) for r in rpm_list] }"
        )


def main(args=None):
    rclpy.init(args=args)
    node = MecanumIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
