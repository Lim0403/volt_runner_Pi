import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    joy_config_path = os.path.expanduser('~/ros2_ws/config/my_joy.yaml')

    return LaunchDescription([

        # 0. PS2 joystick 입력 -> /joy 토픽 퍼블리시
        SetEnvironmentVariable(
            name='SDL_JOYSTICK_DEVICE',
            value='/dev/input/js0'
        ),

        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # 1. /joy -> /cmd_vel 변환
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            output='screen',
            parameters=[joy_config_path]
        ),

        # 2. /cmd_vel 구독 -> 역기구학 -> /wheel_target_rpm 퍼블리시
        Node(
            package='mecanum_pkg',
            executable='ik_node',
            name='ik_node',
            output='screen'
        ),

        # 3. /wheel_target_rpm 구독 -> Teensy로 목표 rpm 전달
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='mecanum_pkg',
                    executable='rpm_receiver_node',
                    name='rpm_receiver_node',
                    output='screen'
                )
            ]
        ),

        # 4. Pi: 실제 rpm 수신 -> /wheel_feedback 퍼블리시
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='mecanum_pkg',
                    executable='wheel_rpm_sender',
                    name='wheel_rpm_sender',
                    output='screen'
                )
            ]
        ),

        # 5. /wheel_feedback 구독 -> 실제 위치/속도 변환 -> /odom, /tf 퍼블리시
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='mecanum_pkg',
                    executable='odom_publisher',
                    name='odom_publisher',
                    output='screen'
                )
            ]
        ),
    ])
