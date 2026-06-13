#!/usr/bin/env python3
"""
PX4-ROS2 Base Platform Template
==================================
구조:
  - [FIXED]       수정 금지 영역 - Offboard 유지에 필수
  - [USER DEFINE] 유저가 원하는 PX4 토픽 선택해서 추가

사용법:
  PX4Base를 상속받아서 USER DEFINE 영역만 채우면 됩니다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)


# ================================================================
# [FIXED] QoS - PX4 uXRCE-DDS 전용 (수정 금지)
# ================================================================
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


class PX4Base(Node):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        # ================================================================
        # [FIXED] Publishers - Offboard 필수 토픽 (수정 금지)
        # ================================================================
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            PX4_QOS
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            PX4_QOS
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            PX4_QOS
        )

        # ================================================================
        # [FIXED] Subscriber - VehicleStatus (Offboard 상태 확인 필수)
        # ================================================================
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self._status_callback,
            PX4_QOS
        )
        self.vehicle_status = VehicleStatus()

        # ================================================================
        # [FIXED] Offboard heartbeat 타이머 (10Hz 필수, 수정 금지)
        # ================================================================
        self.offboard_counter = 0
        self.heartbeat_timer = self.create_timer(
            0.1, self._offboard_heartbeat
        )

        self.get_logger().info(f'[PX4Base] {node_name} 초기화 완료')

    # ================================================================
    # [FIXED] 내부 콜백 (수정 금지)
    # ================================================================
    def _status_callback(self, msg: VehicleStatus):
        """Vehicle 상태 업데이트 (Offboard/Arm 상태 확인용)"""
        self.vehicle_status = msg

    def _offboard_heartbeat(self):
        """
        Offboard 모드 유지를 위한 heartbeat
        PX4는 10Hz 이상으로 이 메시지를 받아야 Offboard 유지
        """
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)
        self.offboard_counter += 1

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0,
                                param3=0.0, param4=0.0, param5=0.0,
                                param6=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    # ================================================================
    # [FIXED] 기본 명령 함수 (수정 금지)
    # ================================================================
    def arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm 명령 전송')

    def disarm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm 명령 전송')

    def set_offboard_mode(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard 모드 전환')

    def land(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('착륙 명령 전송')

    def takeoff(self, altitude: float = 5.0):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param7=altitude)
        self.get_logger().info(f'이륙 명령 전송 (목표 고도: {altitude}m)')

    def send_position(self, x: float, y: float, z: float, yaw: float = 0.0):
        """위치 명령 (NED 좌표계, z 음수가 위)"""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """속도 명령 (Following에서 주로 사용)"""
        msg = TrajectorySetpoint()
        msg.velocity = [vx, vy, vz]
        msg.yaw_rate = yaw_rate
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    # ================================================================
    # [USER DEFINE] 여기서부터 유저가 채우는 영역
    # ================================================================
    #
    # 아래에서 원하는 PX4 토픽을 골라서 __init__에 추가하세요
    #
    # ── 자주 쓰는 Subscriber 토픽 목록 ──────────────────────────────
    #
    # from px4_msgs.msg import VehicleLocalPosition
    # self.local_pos_sub = self.create_subscription(
    #     VehicleLocalPosition,
    #     '/fmu/out/vehicle_local_position',
    #     self.local_pos_callback, PX4_QOS)
    #
    # from px4_msgs.msg import VehicleOdometry
    # self.odom_sub = self.create_subscription(
    #     VehicleOdometry,
    #     '/fmu/out/vehicle_odometry',
    #     self.odom_callback, PX4_QOS)
    #
    # from px4_msgs.msg import SensorGps
    # self.gps_sub = self.create_subscription(
    #     SensorGps,
    #     '/fmu/out/sensor_gps',
    #     self.gps_callback, PX4_QOS)
    #
    # from px4_msgs.msg import VehicleAttitude
    # self.attitude_sub = self.create_subscription(
    #     VehicleAttitude,
    #     '/fmu/out/vehicle_attitude',
    #     self.attitude_callback, PX4_QOS)
    #
    # from px4_msgs.msg import AirspeedValidated
    # self.airspeed_sub = self.create_subscription(
    #     AirspeedValidated,
    #     '/fmu/out/airspeed_validated',
    #     self.airspeed_callback, PX4_QOS)
    #
    # ── 외부 토픽 (카메라, YOLO 등) ─────────────────────────────────
    #
    # from sensor_msgs.msg import Image
    # self.camera_sub = self.create_subscription(
    #     Image, '/camera/image_raw',
    #     self.camera_callback, 10)
    #
    # from std_msgs.msg import Float32MultiArray
    # self.detection_sub = self.create_subscription(
    #     Float32MultiArray, '/yolo/person_detection',
    #     self.detection_callback, 10)
    #
    # ── 주기 실행 타이머 ─────────────────────────────────────────────
    #
    # self.create_timer(0.1, self.on_update)  # 10Hz
    #
    # ── Offboard 제어 모드 변경 (velocity 제어로 바꿀 때) ────────────
    #
    # heartbeat에서 position=False, velocity=True 로 바꾸면
    # send_velocity() 명령이 적용됩니다
    #
    # ================================================================


# ================================================================
# 상속 사용 예시
# ================================================================
"""
from px4_msgs.msg import VehicleLocalPosition
from std_msgs.msg import Float32MultiArray

class DroneFollower(PX4Base):
    def __init__(self):
        super().__init__('drone_follower')

        # USER DEFINE: 필요한 토픽만 골라서 추가
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_callback, PX4_QOS)

        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/person_detection',
            self.detection_callback, 10)

        self.local_position = VehicleLocalPosition()
        self.detection = None

        self.create_timer(0.1, self.on_update)

    def local_pos_callback(self, msg):
        self.local_position = msg

    def detection_callback(self, msg):
        self.detection = msg.data

    def on_update(self):
        if self.offboard_counter > 10:
            self.set_offboard_mode()
            self.arm()

        if self.detection:
            # YOLO 감지 결과로 속도 명령 계산
            x_center = self.detection[0]
            # ... 제어 로직
            self.send_velocity(vx, vy, vz)
"""