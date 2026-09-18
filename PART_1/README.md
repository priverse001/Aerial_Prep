# Part 1: Vision-Based Autonomous Navigation for GPS-Denied UAV Operation

## 1. Overview
This package delivers a robust, GPS-denied autonomous navigation pipeline for PX4 using ROS 2 Humble and external vision estimation (OpenVINS / VINS-Fusion). 

### Key Capabilities:
1. **Frame Transformation & Coordinate Alignment:** Converts camera frame / ROS ENU coordinates to PX4 Local NED and Body FRD.
2. **Dynamic Covariance & Quality Estimation:** Dynamically calculates measurement uncertainty based on real-time feature tracking count, scaling covariance up exponentially during feature degradation to prevent EKF2 filter divergence.
3. **Clean Arming Without Force Bypass:** Meets all EKF2 preflight and variance checks, allowing standard `commander arm` execution (`COM_ARM_WO_GPS 1`, `SYS_HAS_GPS 0`).
4. **Autonomous Position Hold & Failsafe Recovery:** Maintains a stable hover within a 1.5 m radius at 10 m altitude for 90 seconds. Demonstrates graceful failsafe handling during vision dropouts and automatic recovery upon vision resumption.

---

## 2. Parameter Configuration (`config/ekf2_gps_denied.params`)
The following parameters configure PX4 to operate in GPS-denied mode with External Vision:
- `SYS_HAS_GPS`: `0` (Disables GPS requirement)
- `EKF2_GPS_CTRL`: `0` (Disables GPS fusion)
- `EKF2_EV_CTRL`: `11` (Fuses EV Horizontal Position + EV Vertical Position + EV Yaw)
- `EKF2_HGT_REF`: `3` (Sets External Vision as primary height reference)
- `EKF2_EV_DELAY`: `10.0` (Compensates for vision pipeline transmission delay)
- `COM_ARM_WO_GPS`: `1` (Allows normal arming when EKF2 is healthy without GPS)

---

## 3. Node Architecture
- `vision_bridge_health_node`: Subscribes to VIO odometry and feature streams, applies coordinate transforms, dynamically computes covariance, and publishes to `/fmu/in/vehicle_visual_odometry` at 30 Hz.
- `offboard_position_hold_node`: State machine commanding autonomous takeoff, 90-second hover at 10 m altitude, drift tracking, automated vision loss injection, and recovery verification.
- `px4_ekf2_configurator`: Python utility using `pymavlink` to programmatically configure and verify PX4 EKF2 parameters.

---

## 4. How to Run
```bash
# Terminal 1: Launch Gazebo SITL with vision airframe
cd /home/priverse/px4/PX4-Autopilot
make px4_sitl gazebo-classic_iris_vision

# Terminal 2: Configure EKF2 Parameters
python3 /home/priverse/Documents/Aerial_Prep/PART_1/gps_denied_navigation/px4_ekf2_param_configurator.py

# Terminal 3: Launch MicroXRCEAgent & Navigation Stack
source /opt/ros/humble/setup.bash
source /home/priverse/px4_ros_ws/install/setup.bash
ros2 launch gps_denied_navigation gps_denied_mission.launch.py
```
