#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from volt_runner_performance.msg import Perf2Command


class Perf2EchoNode(Node):
    """
    RPI에서 실행.
    - PC가 보낸 /perf2를 수신
    - 수신 즉시 t_recv_rpi(Trecv) 기록
    - 다시 PC로 publish 직전 t_send_rpi(Tsend2) 기록
    - /perf2_echo로 그대로 반사(Echo)
    """
    def __init__(self):
        super().__init__('perf2_echo_node')

        self.sub = self.create_subscription(
            Perf2Command, '/perf2', self.on_perf2, 10
        )
        self.pub = self.create_publisher(
            Perf2Command, '/perf2_echo', 10
        )

        self.get_logger().info("Perf2EchoNode Started. /perf2 -> /perf2_echo (stamp Trecv, Tsend2)")

    def on_perf2(self, msg: Perf2Command):
        # 1) RPI 수신 시각(Trecv) - 콜백 진입 즉시
        msg.t_recv_rpi = self.get_clock().now().to_msg()

        # 2) RPI 재송신 시각(Tsend2) - publish 직전
        msg.t_send_rpi = self.get_clock().now().to_msg()

        # 3) 그대로 반사
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Perf2EchoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
