#!/bin/bash

WS=~/ros2_ws

cleanup() {
    echo ""
    echo "[STOP] Stopping all mecanum nodes..."

    pkill -INT -f "ros2 run joy joy_node"
    pkill -INT -f "ros2 run teleop_twist_joy teleop_node"
    pkill -INT -f "ros2 run mecanum_pkg ik_node"
    pkill -INT -f "ros2 run mecanum_pkg rpm_receiver_node"
    pkill -INT -f "ros2 run mecanum_pkg wheel_rpm_sender"
    pkill -INT -f "ros2 run mecanum_pkg odom_publisher"

    sleep 1

    pkill -TERM -f "ros2 run joy joy_node"
    pkill -TERM -f "ros2 run teleop_twist_joy teleop_node"
    pkill -TERM -f "ros2 run mecanum_pkg ik_node"
    pkill -TERM -f "ros2 run mecanum_pkg rpm_receiver_node"
    pkill -TERM -f "ros2 run mecanum_pkg wheel_rpm_sender"
    pkill -TERM -f "ros2 run mecanum_pkg odom_publisher"

    echo "[STOP] All nodes stopped."
    exit 0
}

trap cleanup SIGINT SIGTERM

run_terminal() {
    TITLE="$1"
    CMD="$2"

    gnome-terminal --title="$TITLE" -- bash -lc "
        cd $WS
        source install/setup.bash
        $CMD
    "

    sleep 1
}

echo "[START] Starting mecanum full control system..."
echo "[INFO] Press Ctrl+C in this main terminal to stop all nodes."
echo ""

run_terminal "0 joy_node" "
export SDL_JOYSTICK_DEVICE=/dev/input/js0
ros2 run joy joy_node
"

run_terminal "1 teleop_twist_joy" "
ros2 run teleop_twist_joy teleop_node --ros-args --params-file ~/ros2_ws/config/my_joy.yaml
"

run_terminal "2 ik_node" "
ros2 run mecanum_pkg ik_node
"

run_terminal "3 rpm_receiver_node" "
ros2 run mecanum_pkg rpm_receiver_node
"

run_terminal "4 wheel_rpm_sender" "
ros2 run mecanum_pkg wheel_rpm_sender
"

run_terminal "5 odom_publisher" "
ros2 run mecanum_pkg odom_publisher
"

echo "[START] All terminals started."
echo "[INFO] Keep this main terminal open."
echo "[INFO] Press Ctrl+C here to stop everything."

while true; do
    sleep 1
done
