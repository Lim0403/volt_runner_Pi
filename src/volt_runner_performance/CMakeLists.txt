cmake_minimum_required(VERSION 3.8)
project(volt_runner_performance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 의존성 패키지 확인
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)  # msg 생성에 필요

# 1. 메시지 생성 설정 (msg 파일 등록)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PerfCommand.msg"
  "msg/Perf2Command.msg"
  DEPENDENCIES geometry_msgs builtin_interfaces std_msgs
)

# 2. 파이썬 스크립트 설치 (ros2 run 가능)
install(PROGRAMS
  scripts/recorder_node.py
  scripts/recorder2_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
