#!/usr/bin/env python3
# ==============================================================================
# File    : face_following_raw.py  (12주차 - raw 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   얼굴 인식 + Offboard 연동 - Following 비행 (raw 버전)
#
#   지금까지 배운 것이 전부 합쳐집니다:
#     8주차  카메라 → 10주차 얼굴 인식 → 11주차 P 제어 → 오늘: 실제 비행!
#
#   제어 방식 (검증된 규약):
#     x_error    → yaw_rate : 드론이 회전해서 얼굴을 정면으로
#     y_error    → vz       : 고도 조절로 화면 y 중앙 유지
#     dist_error → vx(body) : 얼굴 크기로 거리 유지
#
#   이번 주 raw 버전이 특히 긴 이유:
#     "드론 기준 앞으로"(body frame)와 "지도 기준 북쪽으로"(NED)가 달라서
#     드론의 현재 yaw로 좌표 변환이 필요합니다. 그 코드를 전부 직접 쓰면
#     이만큼 깁니다 → base 버전과 꼭 비교해보세요!
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w10_face_detector
#     터미널 4: ros2 run drone_ros2_advanced w12_follow_raw
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
    QoSProfile, ReliabilityPolicy,
    HistoryPolicy, DurabilityPolicy
)
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
)


# ================================================================
# [복붙 영역] QoS 설정 — PX4 uXRCE-DDS 전용
# ================================================================
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


# ================================================================
# 제어 파라미터 (11주차에서 실험한 값!)
# ================================================================
KP_YAW = 0.002    # x 오차 → yaw_rate
KP_VZ  = 0.002    # y 오차 → vz
KP_VX  = 0.005    # 얼굴 높이 오차 → vx

TARGET_FACE_H = 120.0   # 목표 얼굴 높이 [px]

MAX_YAW_RATE = 0.5   # [rad/s]
MAX_VZ       = 1.0   # [m/s]
MAX_VX       = 1.5   # [m/s]

TAKEOFF_ALT = 3.0    # 이륙 고도 [m]
TIMER_HZ    = 10


def clamp(value, limit):
    return max(-limit, min(limit, value))


class FaceFollowingRaw(Node):
    def __init__(self):
        super().__init__('face_following_raw')

        # ── Publishers: Offboard 필수 3종 세트 ──────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # ── Subscribers ─────────────────────────────────────────
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_callback, PX4_QOS)

        # 드론의 현재 자세(yaw) — body frame 변환에 필요!
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._attitude_callback, PX4_QOS)

        # 10주차 얼굴 인식 결과
        self.detection_sub = self.create_subscription(
            Float32MultiArray, '/face/detection',
            self._detection_callback, 10)

        # ── 상태 변수 ────────────────────────────────────────────
        self.tick = 0
        self.armed_sent   = False
        self.takeoff_done = False
        self.local_position = VehicleLocalPosition()
        self.current_yaw  = 0.0
        self.detection    = None
        self._last_detect_time = self.get_clock().now()

        self.create_timer(1.0 / TIMER_HZ, self._control_loop)
        self.get_logger().info(
            '12주차 Following 시작! (face_detector를 먼저 실행하세요)')

    # ============================================================
    # 콜백
    # ============================================================
    def _local_pos_callback(self, msg):
        self.local_position = msg

    def _attitude_callback(self, msg: VehicleAttitude):
        # ============================================================
        # [복붙 영역] 쿼터니언 → yaw 각도 변환
        # 원리: PX4는 자세를 쿼터니언(4개 숫자)으로 보내줍니다.
        #       거기서 "드론이 어느 방향을 보는지(yaw)"만 뽑는 공식.
        # ============================================================
        q = msg.q
        siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2)
        self.current_yaw = np.arctan2(siny_cosp, cosy_cosp)

    def _detection_callback(self, msg):
        self.detection = msg.data
        self._last_detect_time = self.get_clock().now()

    # ============================================================
    # 제어 루프 (10Hz)
    # ============================================================
    def _control_loop(self):
        self.tick += 1
        elapsed = self.tick / TIMER_HZ

        # heartbeat: position과 velocity 둘 다 허용
        # (이륙은 position, following은 velocity 제어라서!)
        self._publish_heartbeat()

        if not self.armed_sent and elapsed >= 1.0:
            self._set_offboard_mode()
            self._arm()
            self.armed_sent = True
            return

        if not self.armed_sent:
            return

        # ── 이륙 단계 (position 제어) ────────────────────────────
        current_alt = abs(self.local_position.z)
        if not self.takeoff_done:
            self._publish_position(
                float('nan'), float('nan'), -TAKEOFF_ALT)
            self.get_logger().info(
                f'이륙 중... {current_alt:.1f}m / {TAKEOFF_ALT}m',
                throttle_duration_sec=1.0)

            if current_alt > TAKEOFF_ALT - 0.3:
                self.takeoff_done = True
                self.get_logger().info('이륙 완료! → Following 시작')
            return

        # ── 얼굴이 없으면 호버링 ─────────────────────────────────
        elapsed_detect = (
            self.get_clock().now() - self._last_detect_time
        ).nanoseconds * 1e-9

        if self.detection is None or elapsed_detect > 1.0:
            self._publish_velocity_body(0.0, 0.0, 0.0, 0.0)   # 정지
            self.get_logger().info(
                '얼굴 없음 → 호버링', throttle_duration_sec=2.0)
            return

        # ── Following: 11주차 P 제어 그대로! ─────────────────────
        cx, cy, w, h, img_w, img_h = self.detection

        x_error    = cx - img_w / 2.0
        y_error    = cy - img_h / 2.0
        dist_error = TARGET_FACE_H - h   # 얼굴 작음(멀음) → +  → 전진

        yaw_rate = clamp(KP_YAW * x_error,    MAX_YAW_RATE)
        vz       = clamp(KP_VZ  * y_error,    MAX_VZ)
        vx_body  = clamp(KP_VX  * dist_error, MAX_VX)

        self._publish_velocity_body(vx_body, 0.0, vz, yaw_rate)

        self.get_logger().info(
            f'Following | x_err={x_error:+.0f} y_err={y_error:+.0f} '
            f'| vx={vx_body:+.2f} vz={vz:+.2f} yaw={yaw_rate:+.3f}',
            throttle_duration_sec=1.0
        )

    # ============================================================
    # [복붙 영역] 퍼블리시 헬퍼
    # ============================================================
    def _publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = True    # ← velocity 제어도 허용!
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def _publish_position(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position  = [float(x), float(y), float(z)]
        msg.velocity  = [float('nan'), float('nan'), float('nan')]
        msg.yaw       = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _publish_velocity_body(self, vx_body, vy_body, vz, yaw_rate):
        """
        Body frame 속도 → NED 변환 후 발행

        원리: "드론 기준 앞으로 1m/s"는 드론이 북쪽을 보면 북쪽으로,
              동쪽을 보면 동쪽으로 가야 합니다. 그래서 현재 yaw로 회전 변환!
          vx_ned = vx_body*cos(yaw) - vy_body*sin(yaw)
          vy_ned = vx_body*sin(yaw) + vy_body*cos(yaw)
        """
        vx_ned = (vx_body * math.cos(self.current_yaw)
                  - vy_body * math.sin(self.current_yaw))
        vy_ned = (vx_body * math.sin(self.current_yaw)
                  + vy_body * math.cos(self.current_yaw))

        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float(vx_ned), float(vy_ned), float(vz)]

        # yaw_rate가 있을 때만 yaw 지정 (없으면 nan → 롤링 방지)
        if abs(yaw_rate) > 0.01:
            msg.yaw      = self.current_yaw
            msg.yawspeed = float(yaw_rate)
        else:
            msg.yaw      = float('nan')
            msg.yawspeed = float('nan')

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = params.get('param1', 0.0)
        msg.param2           = params.get('param2', 0.0)
        msg.param7           = params.get('param7', 0.0)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm 명령 전송')

    def _set_offboard_mode(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard 모드 전환')

    def _land(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('착륙 명령 전송')


def main(args=None):
    rclpy.init(args=args)
    node = FaceFollowingRaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 중... 착륙!')
        node._land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
