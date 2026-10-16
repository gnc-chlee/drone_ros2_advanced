#!/usr/bin/env python3
# ==============================================================================
# File    : face_to_command.py  (10주차)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   얼굴 위치 → 드론 이동 명령 "변환 개념" 확인 (드론 없이!)
#
#   /face/detection을 구독해서, 얼굴 위치가 어떤 드론 명령이 되는지
#   말로 출력해줍니다. 아직 P 제어(11주차)도, 실제 비행(12주차)도 없이
#   "방향 판단"만 합니다.
#
#   변환 규칙 (드론 카메라 기준, 검증된 규약):
#     얼굴이 오른쪽 (x_error +) → 드론 오른쪽 회전 (yaw_rate +)
#     얼굴이 아래   (y_error +) → 드론 하강        (vz +, NED!)
#     얼굴이 크다   (h 큼)      → 너무 가까움 → 후진 (vx -)
#     얼굴이 작다   (h 작음)    → 너무 멀음   → 전진 (vx +)
#
#   실행 방법:
#     터미널 1: ros2 run drone_ros2_advanced w10_face_detector
#     터미널 2: ros2 run drone_ros2_advanced w10_face_command
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


# 방향 판단 기준 (이 픽셀 이상 벗어나면 "움직여야 한다")
DEADZONE_PX    = 50
TARGET_FACE_H  = 120   # 목표 얼굴 높이 [px] — 웹캠 480p 기준
FACE_H_MARGIN  = 30


class FaceToCommand(Node):
    def __init__(self):
        super().__init__('face_to_command')

        self.detection_sub = self.create_subscription(
            Float32MultiArray, '/face/detection',
            self._detection_callback, 10)

        self.get_logger().info(
            '얼굴 → 명령 변환기 시작! (face_detector를 먼저 실행하세요)')

    def _detection_callback(self, msg: Float32MultiArray):
        # face_detector가 보내주는 6개 값 풀기
        cx, cy, w, h, img_w, img_h = msg.data

        # ── 오차 계산: 화면 중앙 기준 ────────────────────────────
        x_error = cx - img_w / 2.0   # +: 얼굴이 오른쪽
        y_error = cy - img_h / 2.0   # +: 얼굴이 아래쪽

        # ── 방향 판단 ────────────────────────────────────────────
        if x_error > DEADZONE_PX:
            yaw_cmd = '오른쪽 회전 (yaw_rate +)'
        elif x_error < -DEADZONE_PX:
            yaw_cmd = '왼쪽 회전 (yaw_rate -)'
        else:
            yaw_cmd = '회전 안함'

        if y_error > DEADZONE_PX:
            vz_cmd = '하강 (vz +)'      # NED: 아래가 +
        elif y_error < -DEADZONE_PX:
            vz_cmd = '상승 (vz -)'
        else:
            vz_cmd = '고도 유지'

        if h > TARGET_FACE_H + FACE_H_MARGIN:
            vx_cmd = '후진 (vx -) : 너무 가까움'
        elif h < TARGET_FACE_H - FACE_H_MARGIN:
            vx_cmd = '전진 (vx +) : 너무 멀음'
        else:
            vx_cmd = '거리 유지'

        self.get_logger().info(
            f'\n얼굴: 중심({cx:.0f},{cy:.0f}) 높이 {h:.0f}px\n'
            f'  x_error={x_error:+.0f}px → {yaw_cmd}\n'
            f'  y_error={y_error:+.0f}px → {vz_cmd}\n'
            f'  얼굴높이 {h:.0f} vs 목표 {TARGET_FACE_H} → {vx_cmd}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = FaceToCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
