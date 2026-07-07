#!/usr/bin/env python3
# ==============================================================================
# File    : face_following_stable_base.py  (13주차 - px4_base 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   Following 안정화 튜닝 - PX4Base 상속 버전
#
#   12주차(base)에 3가지 안정화 추가:
#     [안정화 1] 데드존   : 작은 오차 무시 → 덜덜 떨림 제거
#     [안정화 2] 스무딩   : EMA 필터로 인식 노이즈 완화
#     [안정화 3] 타임아웃 : 잠깐 놓쳐도 1초는 기다렸다가 호버링
#
#   튜닝 실습:
#     ros2 param set /face_following_stable_base kp_yaw 0.004
#     ros2 param set /face_following_stable_base smooth_alpha 0.1
#     → 값을 바꿔가며 비행이 어떻게 달라지는지 관찰하세요!
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w10_face_detector
#     터미널 4: ros2 run drone_ros2_advanced w13_stable_base
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition

from ..px4_base import PX4Base, PX4_QOS


# ================================================================
# 기본값 (전부 ROS 파라미터로 실행 중 변경 가능!)
# ================================================================
KP_YAW = 0.002
KP_VZ  = 0.002
KP_VX  = 0.005

TARGET_FACE_H = 120.0

MAX_YAW_RATE = 0.5
MAX_VZ       = 1.0
MAX_VX       = 1.5

TAKEOFF_ALT       = 3.0
DETECTION_TIMEOUT = 1.0    # [안정화 3]

DEADZONE_XY   = 40.0       # [안정화 1]
DEADZONE_DIST = 20.0
SMOOTH_ALPHA  = 0.3        # [안정화 2]


def clamp(value, limit):
    return max(-limit, min(limit, value))


def deadzone(error, zone):
    return 0.0 if abs(error) < zone else error


class FaceFollowingStableBase(PX4Base):
    def __init__(self):
        super().__init__('face_following_stable_base')

        # ── 튜닝 파라미터 선언 (실행 중 변경 가능) ───────────────
        self.declare_parameter('kp_yaw',       KP_YAW)
        self.declare_parameter('kp_vz',        KP_VZ)
        self.declare_parameter('kp_vx',        KP_VX)
        self.declare_parameter('deadzone_xy',  DEADZONE_XY)
        self.declare_parameter('smooth_alpha', SMOOTH_ALPHA)

        # ── Subscribers ─────────────────────────────────────────
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_callback, PX4_QOS)
        self.detection_sub = self.create_subscription(
            Float32MultiArray, '/face/detection',
            self._detection_callback, 10)

        # ── 상태 변수 ────────────────────────────────────────────
        self.local_position = VehicleLocalPosition()
        self.detection = None
        self._last_detect_time = self.get_clock().now()
        self.armed_sent   = False
        self.takeoff_done = False

        # [안정화 2] 스무딩된 오차
        self.smooth_x_err    = 0.0
        self.smooth_y_err    = 0.0
        self.smooth_dist_err = 0.0

        self.create_timer(0.1, self.on_update)
        self.get_logger().info(
            '13주차(base) 안정화 Following 시작!\n'
            '  튜닝: ros2 param set /face_following_stable_base kp_yaw 0.004')

    def _local_pos_callback(self, msg):
        self.local_position = msg

    def _detection_callback(self, msg):
        self.detection = msg.data
        self._last_detect_time = self.get_clock().now()

    # ============================================================
    # 메인 제어 루프 (10Hz)
    # ============================================================
    def on_update(self):
        if not self.armed_sent:
            if self.offboard_counter >= 10:
                self.set_offboard_mode()
                self.arm()
                self.armed_sent = True
            return

        current_alt = abs(self.local_position.z)
        if not self.takeoff_done:
            self.takeoff_position(TAKEOFF_ALT)
            if current_alt > TAKEOFF_ALT - 0.3:
                self.takeoff_done = True
                self.get_logger().info('이륙 완료! → Following 시작')
            return

        # ── [안정화 3] 타임아웃 처리 ─────────────────────────────
        elapsed = (
            self.get_clock().now() - self._last_detect_time
        ).nanoseconds * 1e-9

        if self.detection is None or elapsed > DETECTION_TIMEOUT:
            self.smooth_x_err = 0.0
            self.smooth_y_err = 0.0
            self.smooth_dist_err = 0.0
            self.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            self.get_logger().info(
                f'얼굴 없음 ({elapsed:.1f}s) → 호버링',
                throttle_duration_sec=2.0)
            return

        # ── 현재 파라미터 값 읽기 ────────────────────────────────
        kp_yaw = self.get_parameter('kp_yaw').value
        kp_vz  = self.get_parameter('kp_vz').value
        kp_vx  = self.get_parameter('kp_vx').value
        dz_xy  = self.get_parameter('deadzone_xy').value
        alpha  = self.get_parameter('smooth_alpha').value

        # ── 오차 계산 ────────────────────────────────────────────
        cx, cy, w, h, img_w, img_h = self.detection
        x_error    = cx - img_w / 2.0
        y_error    = cy - img_h / 2.0
        dist_error = TARGET_FACE_H - h

        # ── [안정화 1] 데드존 ────────────────────────────────────
        x_error    = deadzone(x_error,    dz_xy)
        y_error    = deadzone(y_error,    dz_xy)
        dist_error = deadzone(dist_error, DEADZONE_DIST)

        # ── [안정화 2] 스무딩(EMA) ───────────────────────────────
        self.smooth_x_err    = alpha * x_error    + (1 - alpha) * self.smooth_x_err
        self.smooth_y_err    = alpha * y_error    + (1 - alpha) * self.smooth_y_err
        self.smooth_dist_err = alpha * dist_error + (1 - alpha) * self.smooth_dist_err

        # ── P 제어 ───────────────────────────────────────────────
        yaw_rate = clamp(kp_yaw * self.smooth_x_err,    MAX_YAW_RATE)
        vz       = clamp(kp_vz  * self.smooth_y_err,    MAX_VZ)
        vx_body  = clamp(kp_vx  * self.smooth_dist_err, MAX_VX)

        self.send_velocity_body(vx_body, 0.0, vz, yaw_rate)

        self.get_logger().info(
            f'Following | 스무딩 x={self.smooth_x_err:+.0f} '
            f'y={self.smooth_y_err:+.0f} '
            f'| vx={vx_body:+.2f} vz={vz:+.2f} yaw={yaw_rate:+.3f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = FaceFollowingStableBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 중... 착륙!')
        node.land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
