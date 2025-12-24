import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    # 1. 환경 변수 설정
    set_env_var = SetEnvironmentVariable(
        'SDL_JOYSTICK_DEVICE', '/dev/input/js0'
    )

    # 2. 파라미터 파일 경로 찾기
    pkg_share = get_package_share_directory('mecanum_pkg')
    param_file = os.path.join(pkg_share, 'config', 'my_joy.yaml')

    # 3. 각 노드 정의
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node'
    )

    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[param_file]
    )

    ik_node = Node(
        package='mecanum_pkg',
        executable='ik_node', # setup.py에 등록된 실행 파일 이름
        name='ik_node'
    )
    
    # --- 현재 방식 (Sender/Receiver 분리) ---
    rpm_receiver_node = Node(
        package='mecanum_pkg',
        executable='rpm_receiver_node', # setup.py에 등록된 이름
        name='rpm_receiver_node'
    )

    wheel_rpm_sender_node = Node(
        package='mecanum_pkg',
        executable='wheel_rpm_sender', # setup.py에 등록된 이름
        name='wheel_rpm_sender'
    )
    # ----------------------------------------

    odom_publisher_node = Node(
        package='mecanum_pkg',
        executable='odom_publisher', # setup.py에 등록된 이름
        name='odom_publisher'
    )

    # 4. 런치 스크립트 반환
    return LaunchDescription([
        set_env_var,
        joy_node,
        teleop_node,
        ik_node,
        rpm_receiver_node,         # 현재 방식
        wheel_rpm_sender_node,     # 현재 방식
        odom_publisher_node
    ])
