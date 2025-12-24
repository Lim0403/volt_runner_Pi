import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time

class WheelRpmUartSender(Node):
    def __init__(self):
        super().__init__('wheel_rpm_uart_sender')
        # Teensy가 연결된 UART 포트와 속도로 수정 (아래는 Raspberry Pi 5 예시)
        self.ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
        # ROS2 토픽 구독
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/wheel_target_rpm',
            self.rpm_callback,
            10)
        self.get_logger().info('Wheel RPM UART Sender Node Started')

    def rpm_callback(self, msg):
        if len(msg.data) != 4:
            self.get_logger().warn(f"Received {len(msg.data)} rpm values (expected 4): {msg.data}")
            return
        # rpm 4개를 소수점 2자리로 포매팅해서 쉼표로 연결
        rpm_str = ','.join(f"{v:.2f}" for v in msg.data) + '\n'
        try:
            self.ser.write(rpm_str.encode())
            self.get_logger().info(f"Sent to Teensy: {rpm_str.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def destroy_node(self):
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WheelRpmUartSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
