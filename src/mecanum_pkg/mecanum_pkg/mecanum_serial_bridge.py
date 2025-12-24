import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import threading
import time

class MecanumSerialBridge(Node):
    def __init__(self):
        super().__init__('mecanum_serial_bridge')
        
        # --- 시리얼 포트 설정 ---
        # TTY 이름과 Baudrate는 환경에 맞게 수정하세요.
        # (RPi 5의 기본 UART는 /dev/ttyAMA0 일 수 있습니다)
        # (RPi 3/4는 /dev/serial0 또는 /dev/ttyS0 일 수 있습니다)
        port_name = '/dev/ttyAMA0'
        baud_rate = 115200
        
        try:
            # 1. 하나의 시리얼 객체만 생성
            self.ser = serial.Serial(port_name, baud_rate, timeout=1)
            self.get_logger().info(f"Successfully opened serial port: {port_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {port_name}: {e}")
            self.get_logger().error("Shutting down node.")
            # rclpy.shutdown() # 노드 생성자에서 종료하면 문제가 생길 수 있음
            # 대신 아무것도 하지 않도록 플래그 설정
            self.ser = None
            return

        # 2. 틴지로 '명령'을 보내는 구독자 (WheelRpmUartSender의 역할)
        self.target_sub = self.create_subscription(
            Float32MultiArray,
            '/wheel_target_rpm',
            self.target_rpm_callback,
            10)

        # 3. 틴지로부터 '피드백'을 받는 발행자 (WheelRpmReceiver의 역할)
        self.feedback_pub = self.create_publisher(
            Float32MultiArray,
            '/wheel_feedback_rpm',
            10)

        # 4. 시리얼 '읽기'를 위한 별도 스레드 또는 타이머
        # rclpy.spin()이 메인 스레드를 막으므로, 블로킹 I/O(readline)는 타이머나 스레드에서 처리
        
        # 방법 1: rclpy 타이머 사용 (권장)
        self.read_timer = self.create_timer(0.01, self.read_serial_callback) # 100Hz
        
        # 방법 2: 별도 스레드 사용 (이 경우 스레드 종료 처리 필요)
        # self.running = True
        # self.read_thread = threading.Thread(target=self.serial_read_loop)
        # self.read_thread.daemon = True
        # self.read_thread.start()

        self.get_logger().info("Mecanum Serial Bridge node started.")

    def target_rpm_callback(self, msg):
        """ /wheel_target_rpm 토픽을 받으면 시리얼로 '쓰기' """
        if self.ser is None or not self.ser.is_open:
            return
            
        if len(msg.data) != 4:
            self.get_logger().warn(f"Received {len(msg.data)} rpm values (expected 4)")
            return
        
        # 틴지가 받을 형식: "10.00,-20.50,30.00,-40.00\n"
        rpm_str = ','.join(f"{v:.2f}" for v in msg.data) + '\n'
        
        try:
            self.ser.write(rpm_str.encode('utf-8'))
            # self.get_logger().info(f"Sent: {rpm_str.strip()}") # 너무 자주 로깅되므로 주석 처리
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def read_serial_callback(self):
        """ 타이머에 의해 주기적으로 호출되어 시리얼 '읽기' """
        if self.ser is None or not self.ser.is_open or not self.ser.in_waiting:
            return

        try:
            # 틴지가 보낸 피드백 라인 읽기 (예: "1.23,4.56,7.89,0.12\n")
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            
            if not line:
                return

            data = [float(x) for x in line.split(',')]
            
            if len(data) == 4:
                msg_out = Float32MultiArray()
                msg_out.data = data
                self.feedback_pub.publish(msg_out)
                # self.get_logger().info(f"Feedback RPM: {data}") # 너무 자주 로깅되므로 주석 처리
            else:
                self.get_logger().warn(f"Received malformed feedback: {line}")

        except Exception as e:
            # self.get_logger().warn(f"Error parsing feedback line: {line} ({e})")
            pass # 파싱 오류는 자주 발생할 수 있으므로 조용히 무시

    # # (스레드 방식 사용 시)
    # def serial_read_loop(self):
    #     while self.running and rclpy.ok() and self.ser:
    #         if not self.ser.is_open:
    #             time.sleep(0.1)
    #             continue
    #         try:
    #             line = self.ser.readline().decode('utf-8', errors='ignore').strip()
    #             if line:
    #                 # (이후 로직은 read_serial_callback과 동일)
    #                 # ... (파싱 및 발행) ...
    #                 # ROS 발행은 스레드 안전하지 않을 수 있으므로, 
    #                 # 타이머 방식이 더 간단하고 안전합니다.
    #                 pass
    #         except Exception as e:
    #             self.get_logger().warn(f"Serial read loop error: {e}")
    #             time.sleep(0.01)

    def destroy_node(self):
        # self.running = False # (스레드 방식 사용 시)
        # if hasattr(self, 'read_thread'): # (스레드 방식 사용 시)
        #     self.read_thread.join(timeout=1)
            
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MecanumSerialBridge()
    if node.ser is not None: # 포트 열기에 성공한 경우에만 spin
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
