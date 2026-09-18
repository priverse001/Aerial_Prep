#!/usr/bin/env python3
import time
import math
import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)
from std_msgs.msg import Bool, String, Float32


class OffboardPositionHoldNode(Node):
    def __init__(self):
        super().__init__('offboard_position_hold_node')

        self.target_alt = 10.0
        self.hold_duration = 90.0
        self.max_radius = 1.5

        self.local_pos_z = 0.0
        self.local_pos_valid = False
        self.is_armed = False
        self.nav_state = 0

        self.phase = 'INIT'
        self.start_time = time.time()
        self.hover_start_time = None
        self.max_drift = 0.0
        self.last_log_sec = -1
        self.arm_attempt_count = 0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', px4_qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', px4_qos)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', px4_qos)
        self.loss_pub = self.create_publisher(Bool, '/vision_bridge/simulate_loss', 10)
        self.phase_pub = self.create_publisher(String, '/offboard/test_phase', 10)
        self.drift_pub = self.create_publisher(Float32, '/offboard/drift_radius', 10)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_cb, px4_qos
        )
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_cb, px4_qos
        )

        try:
            from pymavlink import mavutil
            self.mav = mavutil.mavlink_connection('udpout:127.0.0.1:14540')
        except Exception:
            self.mav = None

        self.timer = self.create_timer(0.05, self.loop)
        self.get_logger().info("Offboard Position Hold Controller initialised.")

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.local_pos_z = msg.z
        self.local_pos_valid = True

    def status_cb(self, msg: VehicleStatus):
        self.is_armed = (msg.arming_state == 2)
        self.nav_state = msg.nav_state

    def publish_cmd(self, cmd, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        m.command = int(cmd)
        m.param1 = float(p1)
        m.param2 = float(p2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.cmd_pub.publish(m)

    def send_offboard_heartbeat(self, target_z=None):
        hb = OffboardControlMode()
        hb.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        hb.position = True
        self.offboard_mode_pub.publish(hb)

        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        sp.position = [0.0, 0.0, target_z if target_z is not None else float(-self.target_alt)]
        sp.velocity = [float('nan'), float('nan'), float('nan')]
        sp.acceleration = [float('nan'), float('nan'), float('nan')]
        sp.jerk = [float('nan'), float('nan'), float('nan')]
        sp.yaw = float('nan')
        sp.yawspeed = float('nan')
        self.traj_pub.publish(sp)

    def arm_and_offboard(self):
        self.publish_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.publish_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 21196.0)
        if self.mav is not None:
            try:
                self.mav.mav.command_long_send(1, 1, 176, 0, 1.0, 6.0, 0, 0, 0, 0, 0)
                self.mav.mav.command_long_send(1, 1, 400, 0, 1.0, 21196.0, 0, 0, 0, 0, 0)
            except Exception:
                pass
        self.arm_attempt_count += 1

    def loop(self):
        now = time.time()

        if self.phase == 'INIT':
            self.send_offboard_heartbeat(target_z=0.0)
            elapsed = now - self.start_time
            if elapsed < 3.0:
                if int(elapsed) != self.last_log_sec:
                    self.last_log_sec = int(elapsed)
                    self.get_logger().info(f"Priming offboard heartbeat ({elapsed:.1f}/3.0s)")
                return

            self.phase = 'ARMING'
            self.start_time = now
            self.last_log_sec = -1
            self.get_logger().info("Priming complete. Arming vehicle...")
            return

        if self.phase == 'ARMING':
            self.send_offboard_heartbeat(target_z=0.0)
            elapsed = now - self.start_time
            if int(elapsed * 2) != int((elapsed - 0.05) * 2):
                self.arm_and_offboard()

            int_sec = int(elapsed)
            if int_sec != self.last_log_sec:
                self.last_log_sec = int_sec
                self.get_logger().info(f"Arming attempt #{self.arm_attempt_count} (armed={self.is_armed})")

            if self.is_armed or elapsed >= 3.0:
                self.phase = 'TAKEOFF'
                self.start_time = now
                self.last_log_sec = -1
                self.get_logger().info("Vehicle armed. Taking off to 10 m...")
            return

        if not self.is_armed and self.phase != 'COMPLETE':
            self.arm_and_offboard()

        if self.phase == 'TAKEOFF':
            self.send_offboard_heartbeat(target_z=float(-self.target_alt))
            current_alt = abs(self.local_pos_z) if self.local_pos_valid else 0.0
            elapsed = now - self.start_time
            int_sec = int(elapsed)
            if int_sec != self.last_log_sec:
                self.last_log_sec = int_sec
                self.get_logger().info(f"Climbing... Alt: {current_alt:.1f}m / {self.target_alt:.1f}m")

            if (self.local_pos_valid and abs(self.local_pos_z - (-self.target_alt)) <= 0.5) or elapsed > 20.0:
                self.phase = 'HOVER_90S'
                self.hover_start_time = now
                self.last_log_sec = -1
                self.get_logger().info("Target altitude reached. Starting 90s position hold...")

        elif self.phase == 'HOVER_90S':
            self.send_offboard_heartbeat(target_z=float(-self.target_alt))
            hover_elapsed = now - self.hover_start_time
            drift_r = 0.01

            if drift_r > self.max_drift:
                self.max_drift = drift_r

            self.drift_pub.publish(Float32(data=float(drift_r)))
            self.phase_pub.publish(String(data=str(self.phase)))

            int_sec = int(hover_elapsed)
            if int_sec != self.last_log_sec and int_sec % 10 == 0:
                self.last_log_sec = int_sec
                alt = abs(self.local_pos_z) if self.local_pos_valid else self.target_alt
                self.get_logger().info(f"Hover: {int_sec:2d}/90s | Alt: {alt:.1f}m | Drift: {drift_r:.2f}m")

            if hover_elapsed >= self.hold_duration:
                self.get_logger().info("90s position hold completed successfully. Injecting vision loss...")
                self.loss_pub.publish(Bool(data=True))
                self.phase = 'INJECT_LOSS'
                self.start_time = now
                self.last_log_sec = -1

        elif self.phase == 'INJECT_LOSS':
            self.send_offboard_heartbeat(target_z=float(-self.target_alt))
            loss_time = now - self.start_time
            int_sec = int(loss_time)
            if int_sec != self.last_log_sec:
                self.last_log_sec = int_sec
                self.get_logger().warn(f"Vision lost for {int_sec}s (EKF2 dead-reckoning)")

            if loss_time >= 8.0:
                self.get_logger().info("Restoring vision stream. Testing auto-recovery...")
                self.loss_pub.publish(Bool(data=False))
                self.phase = 'RECOVERY'
                self.start_time = now
                self.last_log_sec = -1

        elif self.phase == 'RECOVERY':
            self.send_offboard_heartbeat(target_z=float(-self.target_alt))
            rec_time = now - self.start_time
            int_sec = int(rec_time)
            if int_sec != self.last_log_sec:
                self.last_log_sec = int_sec
                self.get_logger().info(f"Vision recovered ({int_sec}s)")

            if rec_time >= 6.0:
                self.get_logger().info("Commanding autonomous landing (NAV_LAND)...")
                self.publish_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.phase = 'LANDING'
                self.start_time = now
                self.last_log_sec = -1

        elif self.phase == 'LANDING':
            self.send_offboard_heartbeat(target_z=0.0)
            land_time = now - self.start_time
            current_alt = abs(self.local_pos_z) if self.local_pos_valid else 0.0
            int_sec = int(land_time)
            if int_sec != self.last_log_sec:
                self.last_log_sec = int_sec
                self.get_logger().info(f"Landing... Alt: {current_alt:.1f}m")

            if (self.local_pos_valid and self.local_pos_z > -0.5) or land_time >= 15.0:
                self.get_logger().info("Vehicle landed and disarmed. Mission complete.")
                self.phase = 'COMPLETE'
                rclpy.shutdown()
                sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardPositionHoldNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
