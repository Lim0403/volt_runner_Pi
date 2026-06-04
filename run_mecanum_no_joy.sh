
#!/bin/bash

WS=~/ros2_ws

cleanup() {
    echo ""
    echo "[STOP] Stopping mecanum nodes without joy..."

    pkill -INT -f "ros2 run mecanum_pkg ik_node"
    pkill -INT -f "ros2 run mecanum_pkg rpm_receiver_node"
    pkill -INT -f "ros2 run mecanum_pkg wheel_rpm_sender"
    pkill -INT -f "ros2 run mecanum_pkg odom_publisher"

    sleep 1

    pkill -TERM -f "ros2 run mecanum_pkg ik_node"
    pkill -TERM -f "ros2 run mecanum_pkg rpm_receiver_node"
    pkill -TERM -f "ros2 run mecanum_pkg wheel_rpm_sender"
    pkill -TERM -f "ros2 run mecanum_pkg odom_publisher"

    echo "[STOP] All mecanum nodes stopped."
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

echo "[START] Starting mecanum control system without joy..."
echo "[INFO] This script does NOT run joy_node and teleop_twist_joy."
echo "[INFO] You must provide /cmd_vel from another node or manual command."
echo ""

run_terminal "0 ik_node" "
ros2 run mecanum_pkg ik_node
"

run_terminal "1 rpm_receiver_node" "
ros2 run mecanum_pkg rpm_receiver_node
"

run_terminal "2 wheel_rpm_sender" "
ros2 run mecanum_pkg wheel_rpm_sender
"

run_terminal "3 odom_publisher" "
ros2 run mecanum_pkg odom_publisher
"

echo "[START] All no-joy terminals started."
echo "[INFO] Keep this main terminal open."
echo "[INFO] Press Ctrl+C here to stop everything."

while true; do
    sleep 1
done
