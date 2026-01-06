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

        # === 🔥 방향별 속도 보정 (Gain Tuning) ===
        # 1.0이 기본값.
        # - 전진(X): 너무 빠르니까 줄임 (0.3 ~ 0.4 추천)
        # - 횡이동(Y): 마찰이 심해서 힘이 많이 필요함 (1.0 ~ 1.5 추천)
        # - 회전(Z): 너무 빠르면 어지러우니까 적당히 줄임 (0.3 ~ 0.5 추천)
        
        self.gain_x = 0.32   # 전진 힘조절 (아까 30cm 맞춘 값)
        self.gain_y = 0.87    # 횡이동 힘조절 (옆으로 갈 땐 힘을 3배 이상 더 줌!)
        self.gain_z = 0.89    # 회전 힘조절

        # === 최소 클램핑 ===
        self.min_linear_x = 0.3
        self.min_angular_z = 0.3
        self.enable_clamp = True

        # === 마지막 명령 상태 저장 ===
        self.last_cmd: Twist = None
        self.last_source: str = None

        self.sub_manual = self.create_subscription(
            Twist, '/cmd_vel', functools.partial(self.cmd_vel_callback, source="manual"), 10)
        self.sub_nav2 = self.create_subscription(
            Twist, '/cmd_vel_nav2', functools.partial(self.cmd_vel_callback, source="nav2"), 10)
        self.sub_rl = self.create_subscription(
            Twist, '/cmd_rl', functools.partial(self.cmd_vel_callback, source="rl"), 10)

        self.publisher = self.create_publisher(Float32MultiArray, '/wheel_target_rpm', 10)

        self.get_logger().info(
            f"Mecanum IK Node Started | Gains -> X:{self.gain_x}, Y:{self.gain_y}, Z:{self.gain_z}"
        )

    def cmd_vel_callback(self, msg: Twist, source: str):
        self.last_cmd = msg
        self.last_source = source
        self.apply_cmd(msg, source)

    def apply_cmd(self, msg: Twist, source: str):
        raw_vx = msg.linear.x
        raw_vy = msg.linear.y
        raw_omega = msg.angular.z

        # === 1. 최소 입력 처리 (Deadzone) ===
        # 입력값이 너무 작으면 0으로 처리
        vx = raw_vx if abs(raw_vx) > 0.01 else 0.0
        vy = raw_vy if abs(raw_vy) > 0.01 else 0.0
        omega = -raw_omega if abs(raw_omega) > 0.01 else 0.0 # 부호 반전

        # === 2. 최소 구동 속도 클램핑 (Clamp) ===
        if self.enable_clamp:
            if abs(vx) > 0 and abs(vx) < self.min_linear_x:
                vx = math.copysign(self.min_linear_x, vx)
            if abs(omega) > 0 and abs(omega) < self.min_angular_z:
                omega = math.copysign(self.min_angular_z, omega)

        # === 3. 🔥 핵심: 방향별 게인 적용 ===
        # IK 공식에 넣기 전에, 로봇 특성에 맞춰 속도 명령을 튜닝합니다.
        tuned_vx = vx * self.gain_x
        tuned_vy = vy * self.gain_y
        tuned_omega = omega * self.gain_z

        # === 4. 역기구학 계산 ===
        w_list = inverse_kinematics(tuned_vx, tuned_vy, tuned_omega, self.r, self.lx, self.ly)

        # === 5. rad/s → rpm 변환 ===
        rpm_list = [w * 60.0 / (2.0 * math.pi) for w in w_list]

        msg_out = Float32MultiArray()
        msg_out.data = rpm_list
        self.publisher.publish(msg_out)

        # 디버깅 로그 (가끔 확인용)
        # self.get_logger().info(f"In: {raw_vx:.2f}, {raw_vy:.2f} | OutRPM: {[int(r) for r in rpm_list]}")

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
