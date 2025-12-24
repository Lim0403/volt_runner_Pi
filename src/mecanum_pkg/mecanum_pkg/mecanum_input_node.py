import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleInputPublisher(Node):
    def __init__(self):
        super().__init__('simple_input_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        # 기본값(원하는 속도로 바꾸세요)
        self.vx = 0.2      # 앞/뒤(m/s)
        self.vy = 0.0      # 좌/우(m/s)
        self.omega = 0.3   # 회전(rad/s)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.omega
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publish vx: {self.vx:.2f}, vy: {self.vy:.2f}, omega: {self.omega:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SimpleInputPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
