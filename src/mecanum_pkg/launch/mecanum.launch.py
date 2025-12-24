from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    return LaunchDescription([
        # ===== 환경 변수 설정 =====
        SetEnvironmentVariable(
            name='SDL_JOYSTICK_DEVICE',
            value='/dev/input/js0'
        ),

        # ===== Joy Node =====
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # ===== Teleop Twist Joy Node =====
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy',
            output='screen',
            parameters=['/home/jm/ros2_ws/config/my_joy.yaml']
            # ↑ 경로는 실제 사용자 홈 디렉토리 기준으로 수정
        ),

        # ===== Mecanum IK Node =====
        Node(
            package='mecanum_pkg',
            executable='ik_node',
            name='ik_node',
            output='screen'
        ),

        # ===== RPM Receiver Node =====
        Node(
            package='mecanum_pkg',
            executable='rpm_receiver_node',
            name='rpm_receiver_node',
            output='screen'
        ),

        # ===== Wheel RPM Sender Node =====
        Node(
            package='mecanum_pkg',
            executable='wheel_rpm_sender',
            name='wheel_rpm_sender',
            output='screen'
        ),

        # ===== Odom Publisher Node =====
        Node(
            package='mecanum_pkg',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen'
        ),
    ])
