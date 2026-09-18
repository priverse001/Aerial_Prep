#!/bin/bash
set -o pipefail

PX4_DIR="/home/priverse/px4/PX4-Autopilot"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
MODELS_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
WORLDS_DIR="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"
LOG_DIR="/tmp/px4_gazebo_logs"

export DISPLAY="${DISPLAY:-:1}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

mkdir -p "$LOG_DIR"
mkdir -p "$BUILD_DIR/rootfs"

echo "Starting GPS-Denied Autonomous Navigation Simulation..."

# 1. Cleanup stale processes
killall -9 gzserver gzclient MicroXRCEAgent px4 python3 2>/dev/null || true
rm -f "$BUILD_DIR/rootfs/parameters.bson" "$BUILD_DIR/rootfs/parameters_backup.bson"
sleep 2

# 2. Source environment
source /opt/ros/humble/setup.bash
source /home/priverse/px4_ros_ws/install/setup.bash
source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$BUILD_DIR" 2>/dev/null

export GAZEBO_MODEL_PATH="$MODELS_DIR:/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH"
export GAZEBO_PLUGIN_PATH="$BUILD_DIR/build_gazebo-classic:/opt/ros/humble/lib:$GAZEBO_PLUGIN_PATH"
export LD_LIBRARY_PATH="$BUILD_DIR/build_gazebo-classic:$LD_LIBRARY_PATH"
export PX4_SIM_MODEL="gazebo-classic_iris"
export PX4_SIM_WORLD="none"
export NO_PXH=1

# 3. Start Gazebo server & spawn model
gzserver --verbose "$WORLDS_DIR/empty.world" > "$LOG_DIR/gzserver.log" 2>&1 &
GZ_SERVER_PID=$!

for i in $(seq 1 20); do
  if gz topic -l > /dev/null 2>&1; then
    break
  fi
  sleep 1
done

gz model --spawn-file="$MODELS_DIR/iris/iris.sdf" --model-name="iris" -x 0.0 -y 0.0 -z 0.1 >> "$LOG_DIR/gzserver.log" 2>&1
sleep 1

# 4. Start PX4 SITL
cd "$BUILD_DIR/rootfs" || exit 1
"$BUILD_DIR/bin/px4" -d "$BUILD_DIR/etc" > "$LOG_DIR/px4.log" 2>&1 &
PX4_PID=$!

for i in $(seq 1 30); do
  if grep -q "Simulator connected on TCP port 4560" "$LOG_DIR/px4.log" 2>/dev/null; then
    break
  fi
  sleep 1
done

# 5. Start MicroXRCEAgent & configure parameters
MicroXRCEAgent udp4 -p 8888 > "$LOG_DIR/agent.log" 2>&1 &
AGENT_PID=$!
sleep 2

python3 /home/priverse/Documents/Aerial_Prep/PART_1/gps_denied_navigation/px4_ekf2_param_configurator.py udp:127.0.0.1:14550

# 6. Start Visual Odometry Bridge
python3 -u /home/priverse/px4_ros_ws/src/gps_denied_navigation/gps_denied_navigation/px4_sitl_odometry_groundtruth_bridge.py > "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 8

python3 /home/priverse/Documents/Aerial_Prep/PART_1/gps_denied_navigation/px4_ekf2_param_configurator.py udp:127.0.0.1:14550
sleep 2

# 7. Start Gazebo GUI & Offboard Controller
DISPLAY=$DISPLAY nice -n 20 gzclient --verbose > "$LOG_DIR/gzclient.log" 2>&1 &
GZ_CLIENT_PID=$!
sleep 2

cleanup() {
  kill $OFFBOARD_PID $BRIDGE_PID $GZ_CLIENT_PID $GZ_SERVER_PID $PX4_PID $AGENT_PID 2>/dev/null || true
  sleep 1
  killall -9 gzserver gzclient px4 MicroXRCEAgent python3 2>/dev/null || true
  exit 0
}
trap cleanup INT TERM

python3 -u /home/priverse/px4_ros_ws/src/gps_denied_navigation/gps_denied_navigation/offboard_position_hold_node.py 2>&1 | tee "$LOG_DIR/offboard.log" &
OFFBOARD_PID=$!

wait $OFFBOARD_PID
echo "Mission execution finished."
cleanup
