#!/usr/bin/env python3
# ==============================================================================
# File    : face_following_base.py  (12주차 - px4_base 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   얼굴 인식 Following 비행 - PX4Base 상속 버전
#
#   raw 버전에서 길었던 것들이 PX4Base 안으로 들어갔습니다:
#     - 쿼터니언 → yaw 변환         → self.current_yaw (자동)
#     - body → NED 좌표 변환        → send_velocity_body()
#     - heartbeat, 명령 조립        → 전부 자동
#   → 남는 것은 "P 제어 로직"뿐!
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w10_face_detector
#     터미널 4: ros2 run drone_ros2_advanced w12_follow_base
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
# 제어 파라미터 (11주차에서 실험한 값!)
# ================================================================
KP_YAW = 0.002
KP_VZ  = 0.002
KP_VX  = 0.005

TARGET_FACE_H = 120.0   # 목표 얼굴 높이 [px]

MAX_YAW_RATE = 0.5
MAX_VZ       = 1.0
MAX_VX       = 1.5

TAKEOFF_ALT       = 3.0   # [m]
DETECTION_TIMEOUT = 1.0   # [s]


def clamp(value, limit):
    return max(-limit, min(limit, value))


class FaceFollowingBase(PX4Base):
    def __init__(self):
        super().__init__('face_following_base')

        # ============================================================
        # [USER DEFINE] Subscribers
        # ============================================================
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._local_pos_callback, PX4_QOS)

        self.detection_sub = self.create_subscription(
            Float32MultiArray, '/face/detection',
            self._detection_callback, 10)

        # ============================================================
        # [USER DEFINE] 상태 변수
        # ============================================================
        self.local_position = VehicleLocalPosition()
        self.detection = None
        self._last_detect_time = self.get_clock().now()
        self.armed_sent   = False
        self.takeoff_done = False

        self.create_timer(0.1, self.on_update)   # 10Hz
        self.get_logger().info(
            '12주차(base) Following 시작! (face_detector를 먼저 실행하세요)')

    def _local_pos_callback(self, msg):
        self.local_position = msg

    def _detection_callback(self, msg):
        self.detection = msg.data
        self._last_detect_time = self.get_clock().now()

    # ============================================================
    # [USER DEFINE] 메인 제어 루프 (10Hz)
    # ============================================================
    def on_update(self):
        # ── Arm + Offboard (heartbeat 1초 확보 후) ───────────────
        if not self.armed_sent:
            if self.offboard_counter >= 10:
                self.set_offboard_mode()
                self.arm()
                self.armed_sent = True
            return

        # ── 이륙 단계 ────────────────────────────────────────────
        current_alt = abs(self.local_position.z)
        if not self.takeoff_done:
            self.takeoff_position(TAKEOFF_ALT)   # PX4Base 제공!
            if current_alt > TAKEOFF_ALT - 0.3:
                self.takeoff_done = True
                self.get_logger().info('이륙 완료! → Following 시작')
            return

        # ── 얼굴이 없으면 호버링 ─────────────────────────────────
        elapsed = (
            self.get_clock().now() - self._last_detect_time
        ).nanoseconds * 1e-9

        if self.detection is None or elapsed > DETECTION_TIMEOUT:
            self.send_velocity_body(0.0, 0.0, 0.0, 0.0)
            self.get_logger().info(
                '얼굴 없음 → 호버링', throttle_duration_sec=2.0)
            return

        # ── Following: P 제어 (11주차) ───────────────────────────
        cx, cy, w, h, img_w, img_h = self.detection

        x_error    = cx - img_w / 2.0
        y_error    = cy - img_h / 2.0
        dist_error = TARGET_FACE_H - h   # 얼굴 작음(멀음) → + → 전진

        yaw_rate = clamp(KP_YAW * x_error,    MAX_YAW_RATE)
        vz       = clamp(KP_VZ  * y_error,    MAX_VZ)
        vx_body  = clamp(KP_VX  * dist_error, MAX_VX)

        # body → NED 변환은 PX4Base가 알아서!
        self.send_velocity_body(vx_body, 0.0, vz, yaw_rate)

        self.get_logger().info(
            f'Following | x_err={x_error:+.0f} y_err={y_error:+.0f} '
            f'| vx={vx_body:+.2f} vz={vz:+.2f} yaw={yaw_rate:+.3f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = FaceFollowingBase()
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
