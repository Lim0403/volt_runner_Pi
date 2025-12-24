import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial

class WheelRpmReceiver(Node):
    def __init__(self):
        super().__init__('wheel_rpm_receiver')
        
        # [중요 수정] 토픽 이름을 사용자 환경에 맞게 변경
        self.publisher_ = self.create_publisher(Float32MultiArray, '/wheel_feedback_rpm', 10)
        
        # 시리얼 설정
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        self.timer = self.create_timer(0.05, self.read_serial)

    def read_serial(self):
        line = self.ser.readline().decode(errors='ignore').strip()
        if not line:
            return
        try:
            # Teensy 원본 데이터: [A, B, C, D]
            raw = [float(x) for x in line.split(',')]
            
            if len(raw) == 4:
                # 현재 직진 시: A(-), B(+), C(+), D(-) 로 들어오는 상황
                
                # 1. 데이터 분리
                val_A = raw[0] # FL (현재 음수)
                val_B = raw[1] # FR (현재 양수)
                val_C = raw[2] # RR (현재 양수)
                val_D = raw[3] # RL (현재 음수)

                # 2. 부호 뒤집기 & 위치 매핑
                # 목표: 직진 시 모두 양수(+)가 되어야 함
                
                fl_rpm = val_A * -1.0   # A 뒤집기
                fr_rpm = val_B * 1.0    # B 유지
                
                # D가 RL(왼쪽 뒤)임. 이것도 음수니까 뒤집어야 함.
                rl_rpm = val_D * -1.0   # D 뒤집기
                
                # C가 RR(오른쪽 뒤)임. 이건 양수니까 유지.
                rr_rpm = val_C * 1.0    # C 유지

                # 최종 배열 [FL, FR, RL, RR]
                final_data = [fl_rpm, fr_rpm, rl_rpm, rr_rpm]
                
                msg = Float32MultiArray()
                msg.data = final_data
                self.publisher_.publish(msg)
                
                # (선택) 디버깅용 로그: 실제 나가는 값 확인
                # self.get_logger().info(f"Outgoing: {final_data}")
                
            else:
                self.get_logger().warn(f"Wrong format: {line}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = WheelRpmReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
