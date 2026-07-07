#!/usr/bin/env python3
# ==============================================================================
# File    : face_following_stable_raw.py  (13주차 - raw 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   Following 안정화 튜닝 - 12주차 코드에 3가지 안정화 기법 추가
#   (12주차 raw와 diff로 비교해보세요! 바뀐 곳에 [안정화] 표시)
#
#   12주차의 문제점과 해결책:
#     1. 얼굴이 조금만 움직여도 드론이 반응 → 덜덜 떨림
#        → [안정화 1] 데드존: 오차가 작으면 무시
#     2. 얼굴 인식 위치가 프레임마다 튐 (노이즈)
#        → [안정화 2] 스무딩(EMA): 새 값과 이전 값을 섞어서 부드럽게
#     3. 인식이 한 프레임만 끊겨도 즉시 호버링 → 뚝뚝 끊기는 비행
#        → [안정화 3] 타임아웃 여유: 1초까지는 마지막 명령 유지
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w10_face_detector
#     터미널 4: ros2 run drone_ros2_advanced w13_stable_raw
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
# 제어 파라미터 + [안정화] 튜닝 파라미터
# ================================================================
KP_YAW = 0.002
KP_VZ  = 0.002
KP_VX  = 0.005

TARGET_FACE_H = 120.0

MAX_YAW_RATE = 0.5
MAX_VZ       = 1.0
MAX_VX       = 1.5

TAKEOFF_ALT = 3.0
TIMER_HZ    = 10

# [안정화 1] 데드존: 이 값보다 작은 오차는 0으로 취급 [px]
DEADZONE_XY   = 40.0
DEADZONE_DIST = 20.0

# [안정화 2] 스무딩 계수 (0~1): 작을수록 부드럽고 느리게 반응
#   새값 = ALPHA × 측정값 + (1-ALPHA) × 이전값
SMOOTH_ALPHA = 0.3

# [안정화 3] 감지 타임아웃: 이 시간까지는 기다렸다가 호버링 [s]
DETECTION_TIMEOUT = 1.0


def clamp(value, limit):
    return max(-limit, min(limit, value))


def deadzone(error, zone):
    """[안정화 1] 오차가 zone 이하면 0 (미세한 떨림 무시)"""
    return 0.0 if abs(error) < zone else error


class FaceFollowingStableRaw(Node):
    def __init__(self):
        super().__init__('face_following_stable_raw')

        # ── Publishers (12주차와 동일) ───────────────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # ── Subscribers (12주차와 동일) ──────────────────────────
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_callback, PX4_QOS)
        self.attitude_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._attitude_callback, PX4_QOS)
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

        # [안정화 2] 스무딩된 오차 저장용
        self.smooth_x_err    = 0.0
        self.smooth_y_err    = 0.0
        self.smooth_dist_err = 0.0

        self.create_timer(1.0 / TIMER_HZ, self._control_loop)
        self.get_logger().info(
            '13주차 안정화 Following 시작!\n'
            f'  데드존: {DEADZONE_XY}px  스무딩: {SMOOTH_ALPHA}  '
            f'타임아웃: {DETECTION_TIMEOUT}s')

    # ============================================================
    # 콜백 (12주차와 동일)
    # ============================================================
    def _local_pos_callback(self, msg):
        self.local_position = msg

    def _attitude_callback(self, msg: VehicleAttitude):
        # [복붙 영역] 쿼터니언 → yaw 변환
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

        self._publish_heartbeat()

        if not self.armed_sent and elapsed >= 1.0:
            self._set_offboard_mode()
            self._arm()
            self.armed_sent = True
            return
        if not self.armed_sent:
            return

        # ── 이륙 단계 ────────────────────────────────────────────
        current_alt = abs(self.local_position.z)
        if not self.takeoff_done:
            self._publish_position(
                float('nan'), float('nan'), -TAKEOFF_ALT)
            if current_alt > TAKEOFF_ALT - 0.3:
                self.takeoff_done = True
                self.get_logger().info('이륙 완료! → Following 시작')
            return

        # ── [안정화 3] 얼굴 없을 때 처리 ─────────────────────────
        elapsed_detect = (
            self.get_clock().now() - self._last_detect_time
        ).nanoseconds * 1e-9

        if self.detection is None or elapsed_detect > DETECTION_TIMEOUT:
            # 호버링 + 스무딩 값 리셋 (다시 찾았을 때 튀지 않도록)
            self.smooth_x_err = 0.0
            self.smooth_y_err = 0.0
            self.smooth_dist_err = 0.0
            self._publish_velocity_body(0.0, 0.0, 0.0, 0.0)
            self.get_logger().info(
                f'얼굴 없음 ({elapsed_detect:.1f}s) → 호버링',
                throttle_duration_sec=2.0)
            return

        # ── 오차 계산 ────────────────────────────────────────────
        cx, cy, w, h, img_w, img_h = self.detection

        x_error    = cx - img_w / 2.0
        y_error    = cy - img_h / 2.0
        dist_error = TARGET_FACE_H - h

        # ── [안정화 1] 데드존 적용 ───────────────────────────────
        x_error    = deadzone(x_error,    DEADZONE_XY)
        y_error    = deadzone(y_error,    DEADZONE_XY)
        dist_error = deadzone(dist_error, DEADZONE_DIST)

        # ── [안정화 2] 스무딩(EMA) 적용 ──────────────────────────
        self.smooth_x_err = (SMOOTH_ALPHA * x_error
                             + (1 - SMOOTH_ALPHA) * self.smooth_x_err)
        self.smooth_y_err = (SMOOTH_ALPHA * y_error
                             + (1 - SMOOTH_ALPHA) * self.smooth_y_err)
        self.smooth_dist_err = (SMOOTH_ALPHA * dist_error
                                + (1 - SMOOTH_ALPHA) * self.smooth_dist_err)

        # ── P 제어 (스무딩된 오차 사용!) ─────────────────────────
        yaw_rate = clamp(KP_YAW * self.smooth_x_err,    MAX_YAW_RATE)
        vz       = clamp(KP_VZ  * self.smooth_y_err,    MAX_VZ)
        vx_body  = clamp(KP_VX  * self.smooth_dist_err, MAX_VX)

        self._publish_velocity_body(vx_body, 0.0, vz, yaw_rate)

        self.get_logger().info(
            f'Following | 원본 x_err={cx - img_w/2.0:+.0f} '
            f'→ 스무딩 {self.smooth_x_err:+.0f} '
            f'| vx={vx_body:+.2f} vz={vz:+.2f} yaw={yaw_rate:+.3f}',
            throttle_duration_sec=1.0
        )

    # ============================================================
    # [복붙 영역] 퍼블리시 헬퍼 (12주차와 동일)
    # ============================================================
    def _publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = True
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
        """Body frame → NED 변환 후 속도 명령 (12주차와 동일)"""
        vx_ned = (vx_body * math.cos(self.current_yaw)
                  - vy_body * math.sin(self.current_yaw))
        vy_ned = (vx_body * math.sin(self.current_yaw)
                  + vy_body * math.cos(self.current_yaw))

        msg = TrajectorySetpoint()
        msg.position = [float('nan'), float('nan'), float('nan')]
        msg.velocity = [float(vx_ned), float(vy_ned), float(vz)]

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
    node = FaceFollowingStableRaw()
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
