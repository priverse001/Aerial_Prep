#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition, VehicleAttitude
from std_msgs.msg import Bool, Float32, Int32


class SITLOdometryBridge(Node):
    def __init__(self):
        super().__init__('px4_sitl_odometry_bridge')

        self.sim_loss = False
        self.current_z = 0.0
        self.current_vz = 0.0
        self.current_q = [1.0, 0.0, 0.0, 0.0]

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.px4_pub = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos)
        self.quality_pub = self.create_publisher(Float32, '/vision_bridge/quality', 10)
        self.features_pub = self.create_publisher(Int32, '/vision_bridge/feature_count', 10)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.local_pos_cb, px4_qos
        )
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.attitude_cb, px4_qos
        )
        self.sim_loss_sub = self.create_subscription(
            Bool, '/vision_bridge/simulate_loss', self.loss_callback, 10
        )

        self.timer = self.create_timer(1.0 / 30.0, self.publish_odometry)
        self.get_logger().info("Visual odometry bridge initialised at 30 Hz.")

    def loss_callback(self, msg: Bool):
        self.sim_loss = msg.data
        if self.sim_loss:
            self.get_logger().warn("Vision stream dropped (simulated loss).")
        else:
            self.get_logger().info("Vision stream restored.")

    def local_pos_cb(self, msg: VehicleLocalPosition):
        self.current_z = msg.z
        self.current_vz = msg.vz

    def attitude_cb(self, msg: VehicleAttitude):
        self.current_q = [float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])]

    def publish_odometry(self):
        if self.sim_loss:
            self.quality_pub.publish(Float32(data=0.0))
            self.features_pub.publish(Int32(data=0))
            return

        now_us = int(self.get_clock().now().nanoseconds / 1000)
        v_msg = VehicleOdometry()
        v_msg.timestamp = now_us
        v_msg.timestamp_sample = now_us
        v_msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        v_msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        v_msg.position = [0.0, 0.0, float(self.current_z)]
        v_msg.q = [float(x) for x in self.current_q]
        v_msg.velocity = [0.0, 0.0, float(self.current_vz)]
        v_msg.angular_velocity = [float('nan'), float('nan'), float('nan')]

        v_msg.position_variance = [0.001, 0.001, 0.001]
        v_msg.velocity_variance = [0.001, 0.001, 0.001]
        v_msg.orientation_variance = [0.001, 0.001, 0.001]

        self.px4_pub.publish(v_msg)
        self.quality_pub.publish(Float32(data=100.0))
        self.features_pub.publish(Int32(data=100))


def main(args=None):
    rclpy.init(args=args)
    node = SITLOdometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
