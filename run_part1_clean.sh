#!/bin/bash
set -e

echo "[1/4] Terminating any conflicting background simulator processes..."
pkill -9 -f "gzserver|gzclient|gazebo|px4|MicroXRCEAgent|offboard_position|vision_bridge" || true
sleep 1

echo "[2/4] Starting MicroXRCEAgent on port 8888..."
MicroXRCEAgent udp4 -p 8888 > /dev/null 2>&1 &
AGENT_PID=$!
sleep 1

echo "[3/4] Starting PX4 SITL (Headless simulation mode for clean lockstep)..."
cd /home/priverse/px4/PX4-Autopilot
./build/px4_sitl_default/bin/px4 ./ROMFS/px4fmu_common -s etc/init.d-posix/rcS -t /home/priverse/px4/PX4-Autopilot/test_run_rootfs > /dev/null 2>&1 &
PX4_PID=$!
sleep 2

echo "[4/4] Configuring EKF2 parameters for Vision Navigation..."
python3 /home/priverse/Documents/Aerial_Prep/PART_1/gps_denied_navigation/px4_ekf2_param_configurator.py udp:127.0.0.1:14550

echo ""
echo "[+] Starting Autonomous Position Hold & Vision Health Stack in ROS 2..."
source /opt/ros/humble/setup.bash
source /home/priverse/px4_ros_ws/install/setup.bash

# Run the state machine live with real-time terminal output
python3 -u /home/priverse/px4_ros_ws/src/gps_denied_navigation/gps_denied_navigation/offboard_position_hold_node.py &
OFFBOARD_PID=$!

python3 -u /home/priverse/px4_ros_ws/src/gps_denied_navigation/gps_denied_navigation/px4_sitl_odometry_groundtruth_bridge.py &
BRIDGE_PID=$!

echo "[*] Live Test is Running. Press Ctrl+C at any time to stop and cleanup."
trap 'kill $AGENT_PID $PX4_PID $OFFBOARD_PID $BRIDGE_PID 2>/dev/null || true; exit' INT TERM

wait $OFFBOARD_PID
