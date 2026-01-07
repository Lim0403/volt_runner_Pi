#!/usr/bin/env python3
import rclpy
import csv
import os
import datetime
from rclpy.node import Node
from volt_runner_performance.msg import PerfCommand

class PerfRecorder(Node):
    def __init__(self):
        super().__init__('perf_recorder')
        
        # 1. 토픽 구독
        self.create_subscription(PerfCommand, '/perf', self.listener_callback, 10)
        
        # 2. 저장 경로 설정
        self.data_dir = os.path.expanduser('~/latency_data')
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
            
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.filename = os.path.join(self.data_dir, f'latency_log_{timestamp}.csv')
        
        # 3. CSV 헤더 작성
        with open(self.filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Source', 'Time_Sent_Sec', 'Time_Sent_Nano', 
                'Time_Recv_Sec', 'Time_Recv_Nano', 'Latency_ms', 
                #'Linear_X', 
                # 'Linear_Y', # [메카넘] 좌우 이동 (필요시 주석 해제)
                # 'Linear_Z', # [3D] 상하 이동
                # 'Angular_X', # [3D] 롤(Roll)
                # 'Angular_Y', # [3D] 피치(Pitch)
                #'Angular_Z'
            ])
            
        self.get_logger().info(f"Recorder Started! Saving to: {self.filename}")

    def listener_callback(self, msg):
        # 4. 시간 측정 및 계산
        now = self.get_clock().now()
        recv_sec, recv_nano = now.seconds_nanoseconds()
        
        sent_sec = msg.time_sent.sec
        sent_nano = msg.time_sent.nanosec
        
        diff_sec = recv_sec - sent_sec
        diff_nano = recv_nano - sent_nano
        
        latency_ms = (diff_sec * 1000.0) + (diff_nano / 1000000.0)
        
        # 5. 파일 저장
        with open(self.filename, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                msg.source,
                sent_sec, sent_nano,
                recv_sec, recv_nano,
                f"{latency_ms:.4f}",
                #msg.cmd_vel.linear.x,   # 전진/후진
                
                # --- [메카넘/확장용] 필요하면 주석(#) 지우세요 ---
                # msg.cmd_vel.linear.y,   # 게걸음 (Left/Right)
                # msg.cmd_vel.linear.z,
                # msg.cmd_vel.angular.x,
                # msg.cmd_vel.angular.y,
                # ---------------------------------------------

                #msg.cmd_vel.angular.z   # 회전 (Yaw)
            ])

def main(args=None):
    rclpy.init(args=args)
    node = PerfRecorder()
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
