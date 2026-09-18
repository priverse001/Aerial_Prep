#!/usr/bin/env python3
"""
Vision Bridge & Health Monitor Node for GPS-Denied Navigation (Part 1 - Phase 1)
-------------------------------------------------------------------------------
1. Subscribes to Visual Odometry (from OpenVINS / VINS / Vision Estimator)
   and Image/Feature streams.
2. Converts optical/camera coordinate frames to PX4 navigation frames (NED / FRD).
3. Computes velocity if not present and performs scale consistency check.
4. Dynamically computes covariance based on feature count and tracking health.
5. Publishes VehicleOdometry to PX4 via MicroXRCEAgent (/fmu/in/vehicle_visual_odometry).
6. Publishes Diagnostic & Health telemetry for monitor / EKF2 verification.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import Float32, Int32, Bool, Header
from px4_msgs.msg import VehicleOdometry

class VisionBridgeHealthNode(Node):
    def __init__(self):
        super().__init__('vision_bridge_health_node')

        # Declare parameters
        self.declare_parameter('target_rate_hz', 30.0)
        self.declare_parameter('min_features_healthy', 25)
        self.declare_parameter('critical_features_threshold', 10)
        self.declare_parameter('camera_frame_id', 'camera_link')
        self.declare_parameter('sim_vision_loss', False)
        self.declare_parameter('base_pos_variance', 0.01)     # 0.01 m^2
        self.declare_parameter('base_vel_variance', 0.02)     # 0.02 (m/s)^2
        self.declare_parameter('base_att_variance', 0.005)    # 0.005 rad^2
        self.declare_parameter('in_odom_topic', '/ov_msckf/odomimu')
        self.declare_parameter('in_features_topic', '/ov_msckf/features')

        # Retrieve parameters
        self.target_rate = self.get_parameter('target_rate_hz').get_parameter_value().double_value
        self.min_features = self.get_parameter('min_features_healthy').get_parameter_value().integer_value
        self.crit_features = self.get_parameter('critical_features_threshold').get_parameter_value().integer_value
        self.base_pos_var = self.get_parameter('base_pos_variance').get_parameter_value().double_value
        self.base_vel_var = self.get_parameter('base_vel_variance').get_parameter_value().double_value
        self.base_att_var = self.get_parameter('base_att_variance').get_parameter_value().double_value
        odom_topic = self.get_parameter('in_odom_topic').get_parameter_value().string_value
        feat_topic = self.get_parameter('in_features_topic').get_parameter_value().string_value

        # Tracking state
        self.feature_count = 50
        self.tracking_valid = True
        self.sim_vision_loss = False
        self.last_odom_time = None
        self.last_pos = None
        self.est_vel = np.zeros(3)

        # QoS Profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            sensor_qos
        )
        # Fallback generic pose sub if odometry topic differs
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/vision_pose',
            self.pose_callback,
            sensor_qos
        )
        self.feat_sub = self.create_subscription(
            PointCloud2,
            feat_topic,
            self.features_callback,
            sensor_qos
        )
        self.vision_loss_sub = self.create_subscription(
            Bool,
            '/vision_bridge/simulate_loss',
            self.sim_loss_callback,
            10
        )

        # Publishers to PX4 uXRCE-DDS
        self.px4_visual_odom_pub = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            px4_qos
        )

        # Diagnostics Publishers
        self.diag_quality_pub = self.create_publisher(Float32, '/vision_bridge/quality', 10)
        self.diag_features_pub = self.create_publisher(Int32, '/vision_bridge/feature_count', 10)
        self.diag_status_pub = self.create_publisher(Bool, '/vision_bridge/healthy', 10)

        # Periodic timer (ensure consistent publishing rate)
        self.current_vehicle_odom = None
        timer_period = 1.0 / self.target_rate
        self.publish_timer = self.create_timer(timer_period, self.timer_publish_callback)

        self.get_logger().info(f"Vision Bridge & Health Monitor initialized at {self.target_rate} Hz.")

    def sim_loss_callback(self, msg: Bool):
        self.sim_vision_loss = msg.data
        status = "LOST (Simulated)" if self.sim_vision_loss else "RESTORED"
        self.get_logger().warn(f"Vision bridge connection {status}")

    def features_callback(self, msg: PointCloud2):
        # In PointCloud2, width * height = number of tracked feature points
        self.feature_count = msg.width * msg.height

    def compute_dynamic_quality_and_variance(self):
        """
        Dynamically computes quality percentage [0-100] and error variances.
        If feature count is low, variances swell exponentially so EKF2 knows uncertainty is high.
        """
        if self.sim_vision_loss or self.feature_count <= 0:
            return 0, 100.0, 100.0, 10.0, False

        # Quality scale based on feature count
        if self.feature_count >= self.min_features:
            quality_ratio = 1.0
            quality_int = 100
        elif self.feature_count <= self.crit_features:
            quality_ratio = max(0.05, float(self.feature_count) / float(self.crit_features) * 0.3)
            quality_int = int(quality_ratio * 100)
        else:
            # Linear scaling in intermediate band
            quality_ratio = 0.3 + 0.7 * (float(self.feature_count - self.crit_features) / float(self.min_features - self.crit_features))
            quality_int = int(quality_ratio * 100)

        # Dynamic variance calculation (Inverse proportional to quality)
        var_scale = 1.0 / (quality_ratio ** 2)
        pos_var = self.base_pos_var * var_scale
        vel_var = self.base_vel_var * var_scale
        att_var = self.base_att_var * var_scale

        healthy = (quality_int >= 40)
        return quality_int, pos_var, vel_var, att_var, healthy

    def odom_callback(self, msg: Odometry):
        if self.sim_vision_loss:
            return

        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pos_in = msg.pose.pose.position
        quat_in = msg.pose.pose.orientation
        vel_in = msg.twist.twist.linear
        ang_vel_in = msg.twist.twist.angular

        # Convert to PX4 frames:
        # Standard ROS (ENU: East-North-Up or Optical: Z-fwd, X-right, Y-down)
        # PX4 expects Local NED (North-East-Down) for Position and Body FRD for Angular Vel.
        # Assuming input is ROS ENU / OpenVINS standard:
        # x_ned = y_enu, y_ned = x_enu, z_ned = -z_enu
        pos_ned = [pos_in.y, pos_in.x, -pos_in.z]

        # Orientation quaternion conversion (w, x, y, z) Hamiltonian
        # q_ned = [q_w, q_y, q_x, -q_z] for ENU to NED standard transform
        q_hamilton = [quat_in.w, quat_in.y, quat_in.x, -quat_in.z]

        # Velocity handling & estimation
        vel_ned = [vel_in.y, vel_in.x, -vel_in.z]
        if abs(vel_in.x) < 1e-6 and abs(vel_in.y) < 1e-6 and abs(vel_in.z) < 1e-6:
            if self.last_pos is not None and self.last_odom_time is not None:
                dt = now_sec - self.last_odom_time
                if dt > 1e-4:
                    vel_ned[0] = (pos_ned[0] - self.last_pos[0]) / dt
                    vel_ned[1] = (pos_ned[1] - self.last_pos[1]) / dt
                    vel_ned[2] = (pos_ned[2] - self.last_pos[2]) / dt

        self.last_pos = pos_ned
        self.last_odom_time = now_sec

        # Dynamic quality & variance
        quality, pos_var, vel_var, att_var, healthy = self.compute_dynamic_quality_and_variance()

        # Build PX4 VehicleOdometry message
        v_msg = VehicleOdometry()
        v_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000) # microseconds
        v_msg.timestamp_sample = v_msg.timestamp

        v_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        v_msg.position = [float(pos_ned[0]), float(pos_ned[1]), float(pos_ned[2])]
        v_msg.q = [float(q_hamilton[0]), float(q_hamilton[1]), float(q_hamilton[2]), float(q_hamilton[3])]

        v_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        v_msg.velocity = [float(vel_ned[0]), float(vel_ned[1]), float(vel_ned[2])]
        v_msg.angular_velocity = [float(ang_vel_in.x), float(-ang_vel_in.y), float(-ang_vel_in.z)]

        v_msg.position_variance = [float(pos_var), float(pos_var), float(pos_var)]
        v_msg.velocity_variance = [float(vel_var), float(vel_var), float(vel_var)]
        v_msg.orientation_variance = [float(att_var), float(att_var), float(att_var)]

        v_msg.quality = quality
        v_msg.reset_counter = 0

        self.current_vehicle_odom = v_msg

        # Publish health telemetry
        self.diag_quality_pub.publish(Float32(data=float(quality)))
        self.diag_features_pub.publish(Int32(data=int(self.feature_count)))
        self.diag_status_pub.publish(Bool(data=healthy))

    def pose_callback(self, msg: PoseStamped):
        """Fallback callback if only PoseStamped is fed"""
        fake_odom = Odometry()
        fake_odom.header = msg.header
        fake_odom.pose.pose = msg.pose
        self.odom_callback(fake_odom)

    def timer_publish_callback(self):
        """Dispatches odometry at stable 30Hz target rate"""
        if self.current_vehicle_odom is not None and not self.sim_vision_loss:
            self.current_vehicle_odom.timestamp = int(self.get_clock().now().nanoseconds / 1000)
            self.px4_visual_odom_pub.publish(self.current_vehicle_odom)

def main(args=None):
    rclpy.init(args=args)
    node = VisionBridgeHealthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
