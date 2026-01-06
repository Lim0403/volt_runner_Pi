import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math

class OdomPublisher(Node):

    def __init__(self):
        super().__init__('odom_publisher')
        
        # 1. 구독 (틴지 -> 라즈베리파이)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_feedback_rpm',  # 틴지가 보내주는 토픽 이름
            self.feedback_callback,
            10)
            
        # 2. 발행 (라즈베리파이 -> Nav2/Rviz)
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 3. 로봇 파라미터 (단위: m)
        self.wheel_radius = 0.0485   # 수정 필요 (사용자 로봇에 맞게)
        self.wheel_separation_width = 0.17  # 수정 필요
        self.wheel_separation_length = 0.20 # 수정 필요
        
        # 4. 위치 초기화
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # 5. 보정 계수 (순정 상태는 1.0)
        self.correction_factor_linear = 0.788
        self.correction_factor_angular = 1.68

    def feedback_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        rpm = msg.data  # [FL, FR, RL, RR] 순서로 들어온다고 가정

        if len(rpm) < 4:
            return

        # =========================================================
        # [수정 전] 일반적인 매핑 (FL, FR, RL, RR 순서 그대로)
        # =========================================================
        rpm_fl = rpm[0]
        rpm_fr = rpm[1]
        rpm_rl = rpm[3]
        rpm_rr = rpm[2]

        # RPM -> m/s 변환
        v_fl = (rpm_fl * 2 * math.pi * self.wheel_radius) / 60.0
        v_fr = (rpm_fr * 2 * math.pi * self.wheel_radius) / 60.0
        v_rl = (rpm_rl * 2 * math.pi * self.wheel_radius) / 60.0
        v_rr = (rpm_rr * 2 * math.pi * self.wheel_radius) / 60.0

        # =========================================================
        # [수정 전] 메카넘 휠 표준 기구학 공식
        # =========================================================
        # 만약 Feedback 데이터가 뒤섞여 있다면, 이 공식에 넣었을 때
        # 직진이 회전으로 계산되거나 그 반대가 됩니다.
        
        # X: 직진 (모두 합)
        linear_x = (v_fl + v_fr + v_rl + v_rr) / 4.0
        
        # Y: 횡이동 (보통 FL, RR이 양수일 때 +Y가 되는 식)
        # 식: (-FL + FR + RL - RR) / 4.0  <-- 이 식은 모터 방향 정의마다 다를 수 있음
        linear_y = (-v_fl + v_fr + v_rl - v_rr) / 4.0 

        # Z: 회전 (오른쪽 - 왼쪽)
        # 식: (-FL + FR - RL + RR) / (4 * k)
        k = self.wheel_separation_width + self.wheel_separation_length
        angular_z = -(-v_fl + v_fr - v_rl + v_rr) / (4.0 * k)

        # 보정 계수 적용
        linear_x *= self.correction_factor_linear
        linear_y *= self.correction_factor_linear
        angular_z *= self.correction_factor_angular

        # ---------------------------------------------------------
        # 여기서부터는 Odom 적분 (위치 계산) - 건드릴 필요 없음
        # ---------------------------------------------------------
        delta_x = (linear_x * math.cos(self.th) - linear_y * math.sin(self.th)) * dt
        delta_y = (linear_x * math.sin(self.th) + linear_y * math.cos(self.th)) * dt
        delta_th = angular_z * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # 오일러 -> 쿼터니언 변환
        q = self.euler_to_quaternion(0, 0, self.th)

        # Odom 메시지 발행
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        odom.twist.twist.linear.x = linear_x
        odom.twist.twist.linear.y = linear_y
        odom.twist.twist.angular.z = angular_z

        self.odom_publisher.publish(odom)

        # TF 발행
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q

        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
