#!/usr/bin/env python3
# ==============================================================================
# File    : px4_base.py
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-06-01
# Version : 1.2.0
#
# Description:
#   PX4-ROS2 Base Platform Template
#   Reusable base class for PX4 Offboard control via uXRCE-DDS
#
#   [FIXED]       수정 금지 영역 - Offboard 유지에 필수
#   [USER DEFINE] 유저가 원하는 PX4 토픽 선택해서 추가
#
#   v1.2.0 변경사항:
#     - send_velocity_body 추가 (body frame → NED frame 자동 변환)
#     - send_velocity_yaw yaw_rate 조건부 처리 (롤링 방지)
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import math
import numpy as np
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
    VehicleAttitude,
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
        # [FIXED] Subscribers - 필수 토픽 (수정 금지)
        # ================================================================

        # VehicleStatus - Offboard/Arm 상태 확인 필수
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self._status_callback,
            PX4_QOS
        )

        # VehicleAttitude - yaw 정보 (send_velocity_yaw, send_velocity_body에 필수)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_callback,
            PX4_QOS
        )

        # ================================================================
        # [FIXED] 상태 변수 (수정 금지)
        # ================================================================
        self.vehicle_status = VehicleStatus()
        self.current_yaw    = 0.0   # [rad] VehicleAttitude에서 자동 업데이트

        # ================================================================
        # [FIXED] Offboard heartbeat 타이머 (10Hz 필수, 수정 금지)
        # ================================================================
        self.offboard_counter = 0
        self.heartbeat_timer  = self.create_timer(
            0.1, self._offboard_heartbeat
        )

        self.get_logger().info(f'[PX4Base] {node_name} 초기화 완료')

    # ================================================================
    # [FIXED] 내부 콜백 (수정 금지)
    # ================================================================
    def _status_callback(self, msg: VehicleStatus):
        """Vehicle 상태 업데이트 (Offboard/Arm 상태 확인용)"""
        self.vehicle_status = msg

    def _attitude_callback(self, msg: VehicleAttitude):
        """
        쿼터니언 → yaw 변환 후 current_yaw 업데이트
        q = [qw, qx, qy, qz] (Hamilton convention)
        """
        q = msg.q
        siny_cosp        = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp        = 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def _offboard_heartbeat(self):
        """
        Offboard 모드 유지를 위한 heartbeat
        PX4는 10Hz 이상으로 이 메시지를 받아야 Offboard 유지
        position=True, velocity=True → 두 모드 모두 허용
        """
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = True
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)
        self.offboard_counter += 1

    def _publish_vehicle_command(
        self, command,
        param1=0.0, param2=0.0, param3=0.0,
        param4=0.0, param5=0.0, param6=0.0, param7=0.0
    ):
        """VehicleCommand 퍼블리시 헬퍼 (param1~7 전체 지원)"""
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = param1
        msg.param2           = param2
        msg.param3           = param3
        msg.param4           = param4
        msg.param5           = param5
        msg.param6           = param6
        msg.param7           = param7
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    # ================================================================
    # [FIXED] 상태 확인 프로퍼티
    # 메서드처럼 ()없이 변수처럼 사용 가능
    # 예: if self.is_offboard:
    # ================================================================

    @property
    def is_offboard(self) -> bool:
        """현재 Offboard 모드인지 확인"""
        return (
            self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    @property
    def is_armed(self) -> bool:
        """현재 Arm 상태인지 확인"""
        return (
            self.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )

    @property
    def is_landed(self) -> bool:
        """착륙 상태인지 확인"""
        return (
            self.vehicle_status.landed_state
            == VehicleStatus.LANDED_STATE_ON_GROUND
        )

    # ================================================================
    # [FIXED] 기본 명령 함수 (수정 금지)
    # ================================================================
    def arm(self):
        """드론 Arm"""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm 명령 전송')

    def disarm(self):
        """드론 Disarm"""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm 명령 전송')

    def set_offboard_mode(self):
        """Offboard 모드 전환"""
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard 모드 전환')

    def land(self):
        """착륙 명령"""
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('착륙 명령 전송')

    def takeoff(self, altitude: float = 5.0):
        """
        Auto 모드 이륙 명령
        altitude: 목표 고도 (양수, 미터 단위)
        """
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param7=altitude
        )
        self.get_logger().info(f'이륙 명령 전송 - 목표 고도: {altitude}m')

    def takeoff_position(self, altitude: float = 5.0):
        """
        Offboard 모드에서 position setpoint로 이륙
        altitude: 목표 고도 (양수, 미터 단위)
        내부적으로 NED z = -altitude 로 변환
        """
        self.send_position(
            float('nan'),
            float('nan'),
            -altitude
        )
        self.get_logger().info(f'Position 이륙 중 - 목표: {altitude}m')

    def send_position(self, x: float, y: float, z: float, yaw: float = float('nan')):
        """
        위치 명령 (NED 좌표계)
        z: 음수가 위 (예: -5.0 = 5m 상승)
        yaw: 목표 yaw [rad], nan이면 현재 yaw 유지
        """
        msg = TrajectorySetpoint()
        msg.position  = [x, y, z]
        msg.velocity  = [float('nan'), float('nan'), float('nan')]
        msg.yaw       = yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """
        NED frame 속도 명령 (yaw 제어 없음)
        """
        msg = TrajectorySetpoint()
        msg.position  = [float('nan'), float('nan'), float('nan')]
        msg.velocity  = [vx, vy, vz]
        msg.yaw       = float('nan')
        msg.yawspeed  = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def send_velocity_yaw(self, vx: float, vy: float, vz: float, yaw_rate: float = 0.0):
        """
        NED frame 속도 + yaw_rate 명령
        yaw_rate가 있을 때만 current_yaw 넣어서 충돌 방지
        롤링 방지: yaw_rate 임계값 이하면 yaw/yawspeed 모두 nan
        """
        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [vx, vy, vz]

        if abs(yaw_rate) > 0.01:   # yaw_rate 있을 때만
            msg.yaw      = self.current_yaw
            msg.yawspeed = yaw_rate
        else:                       # yaw_rate 없으면 둘 다 nan
            msg.yaw      = float('nan')
            msg.yawspeed = float('nan')

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def send_velocity_body(self, vx_body: float, vy_body: float, vz: float, yaw_rate: float = 0.0):
        """
        Body frame 속도 명령 → NED frame으로 자동 변환 후 전송

        Following처럼 드론이 바라보는 방향으로 이동할 때 사용
        vx_body: 드론 전방(+) / 후방(-) 속도 [m/s]
        vy_body: 드론 우측(+) / 좌측(-) 속도 [m/s]
        vz     : NED z축 속도 (양수=하강, 음수=상승) [m/s]
        yaw_rate: yaw 각속도 [rad/s]

        변환 공식 (NED):
          vx_ned =  vx_body * cos(yaw) - vy_body * sin(yaw)
          vy_ned =  vx_body * sin(yaw) + vy_body * cos(yaw)
        """
        vx_ned = vx_body * math.cos(self.current_yaw) - vy_body * math.sin(self.current_yaw)
        vy_ned = vx_body * math.sin(self.current_yaw) + vy_body * math.cos(self.current_yaw)
        self.send_velocity_yaw(vx_ned, vy_ned, vz, yaw_rate)

    # ================================================================
    # [USER DEFINE] 여기서부터 유저가 채우는 영역
    # ================================================================
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

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_pos_callback, PX4_QOS)

        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/person_detection',
            self.detection_callback, 10)

        self.local_position = VehicleLocalPosition()
        self.detection      = None
        self.create_timer(0.1, self.on_update)

    def local_pos_callback(self, msg):
        self.local_position = msg

    def detection_callback(self, msg):
        self.detection = msg.data

    def on_update(self):
        if not self.is_offboard:
            return
        if not self.is_armed:
            return

        if self.detection:
            x_error  = self.detection[0] - 960.0
            yaw_rate = 0.001 * x_error

            # body frame으로 전진 (드론이 바라보는 방향)
            self.send_velocity_body(1.0, 0.0, 0.0, yaw_rate)
"""