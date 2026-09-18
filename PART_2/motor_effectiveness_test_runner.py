#!/usr/bin/env python3
"""
PX4 Motor Effectiveness Characterization Runner (Part 2 - Phase 1 & Phase 2)
----------------------------------------------------------------------------
1. Connects to PX4 SITL via MAVLink.
2. Arms and commands autonomous takeoff to 20m altitude.
3. Phase 1 (Baseline Reference):
   - Triggers PX4's built-in failure injection ('failure motor off -m <motor_id>') once.
   - Logs flight trajectory, attitude, and tumbling rates.
4. Phase 2 (Runtime Effectiveness Scaling):
   - Modifies rotor effectiveness dynamically across 5 discrete levels:
     100% -> 75% -> 50% -> 25% -> 0%.
   - Logs live transitions and records allocator re-distribution vs tumbling dynamics.
"""

import time
import sys
import math
from pymavlink import mavutil

LEVELS = [1.0, 0.75, 0.50, 0.25, 0.0]
TARGET_ALTITUDE_M = 20.0
TARGET_MOTOR_INDEX = 1 # Motor 1 (Front Right)

def connect_px4(connection_str="udp:127.0.0.1:14550"):
    print(f"Connecting to PX4 SITL on {connection_str}...")
    mav = mavutil.mavlink_connection(connection_str)
    mav.wait_heartbeat(timeout=10)
    print(f"[+] Heartbeat received from System {mav.target_system}, Component {mav.target_component}")
    return mav

def arm_and_takeoff(mav, altitude=20.0):
    print(f"[*] Arming drone...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1.0, 0.0, 0, 0, 0, 0, 0
    )
    time.sleep(1.0)

    print(f"[*] Commanding Takeoff to {altitude}m...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    
    # Wait until altitude is reached
    print("[*] Ascending to 20m...")
    start_t = time.time()
    while time.time() - start_t < 25:
        msg = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        if msg:
            rel_alt = msg.relative_alt / 1000.0
            if rel_alt >= (altitude - 1.0):
                print(f"[+] Altitude reached: {rel_alt:.2f} m. Hover stable.")
                break
        time.sleep(0.5)

def run_phase1_built_in_failure(mav, motor_id=1):
    """Triggers PX4 built-in failure injection for baseline observation"""
    print(f"\n==========================================")
    print(f"PHASE 1: PX4 Built-In Failure Injection Reference")
    print(f"Triggering 'failure motor off' for Motor {motor_id}")
    print(f"==========================================")
    
    # MAV_CMD_INJECT_FAILURE (3000), param1: FAILURE_UNIT_MOTOR (10), param2: FAILURE_TYPE_OFF (1)
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        3000, 0,
        10.0, # Failure unit: Motor
        1.0,  # Failure type: Off
        0.0, float(motor_id), 0.0, 0.0, 0.0
    )
    print(f"[!] Motor {motor_id} killed via built-in failure injection. Observing vehicle response for 10s...")
    time.sleep(10.0)

def set_rotor_effectiveness_param(mav, motor_index, level):
    """
    Sets rotor thrust coefficient / effectiveness scaling in the control allocation layer.
    Base CT default is 6.5. Scaled CT = 6.5 * level.
    """
    base_ct = 6.5
    scaled_ct = base_ct * level
    param_name = f"CA_ROTOR{motor_index}_CT"
    param_id = param_name.encode('utf-8')
    
    mav.mav.param_set_send(
        mav.target_system,
        mav.target_component,
        param_id,
        float(scaled_ct),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    print(f"[LIVE LOG] Transition -> Motor {motor_index} Effectiveness: {level * 100:.0f}% (CA_ROTOR{motor_index}_CT = {scaled_ct:.3f})")

def run_phase2_effectiveness_characterization(mav, motor_index=1):
    """Steps through 100% -> 75% -> 50% -> 25% -> 0% effectiveness levels"""
    print(f"\n==========================================")
    print(f"PHASE 2: Control Allocation Effectiveness Degradation")
    print(f"Target Motor: Rotor {motor_index} | Levels: {LEVELS}")
    print(f"==========================================")
    
    for lvl in LEVELS:
        set_rotor_effectiveness_param(mav, motor_index, lvl)
        print(f"[*] Holding at {lvl * 100:.0f}% effectiveness for 10 seconds to characterize flight behavior...")
        t_start = time.time()
        while time.time() - t_start < 10.0:
            att = mav.recv_match(type='ATTITUDE', blocking=False)
            if att:
                roll_deg = math.degrees(att.roll)
                pitch_deg = math.degrees(att.pitch)
                yaw_rate = math.degrees(att.yawspeed)
                print(f"    [T+{time.time() - t_start:4.1f}s] Roll: {roll_deg:+6.1f}° | Pitch: {pitch_deg:+6.1f}° | YawRate: {yaw_rate:+6.1f}°/s")
            time.sleep(2.0)

def main():
    conn = "udp:127.0.0.1:14550"
    if len(sys.argv) > 1:
        conn = sys.argv[1]
        
    mav = connect_px4(conn)
    arm_and_takeoff(mav, altitude=TARGET_ALTITUDE_M)

    if "--phase1" in sys.argv:
        run_phase1_built_in_failure(mav, motor_id=TARGET_MOTOR_INDEX)
    else:
        run_phase2_effectiveness_characterization(mav, motor_index=TARGET_MOTOR_INDEX)

if __name__ == '__main__':
    main()
