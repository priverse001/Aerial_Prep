#!/usr/bin/env python3
import time
import sys
import struct
from pymavlink import mavutil

PARAMS_TO_SET = {
    "SYS_HAS_GPS": 0,
    "EKF2_GPS_CTRL": 0,
    "EKF2_EV_CTRL": 15,
    "EKF2_HGT_REF": 3,
    "EKF2_EV_DELAY": 10.0,
    "EKF2_MAG_TYPE": 5,
    "COM_ARM_WO_GPS": 1,
    "COM_DISARM_LAND": 2.0,
    "NAV_RCL_ACT": 0,
    "NAV_DLL_ACT": 0,
    "SENS_IMU_MODE": 1,
    "CBRK_SUPPLY_CHK": 894281,
    "CBRK_VELPOSERR": 201607551,
}

def set_param(mav, name, value):
    param_id = name.encode('utf-8')
    if isinstance(value, float):
        mav.mav.param_set_send(mav.target_system, mav.target_component, param_id, value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    else:
        val_float = struct.unpack('<f', struct.pack('<i', value))[0]
        mav.mav.param_set_send(mav.target_system, mav.target_component, param_id, val_float, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(0.04)

def main():
    target = sys.argv[1] if len(sys.argv) > 1 else "udp:127.0.0.1:14550"
    mav = mavutil.mavlink_connection(target)
    mav.wait_heartbeat(timeout=10)
    for name, val in PARAMS_TO_SET.items():
        set_param(mav, name, val)
    print("EKF2 GPS-denied parameters applied successfully.")

if __name__ == '__main__':
    main()
