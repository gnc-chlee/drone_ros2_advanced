#!/usr/bin/env python3
# ==============================================================================
# File    : p_control_demo.py  (11주차)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   비례(P) 제어 개념 - 10주차의 "방향 판단"을 "속도 계산"으로 업그레이드
#
#   10주차: 얼굴이 오른쪽이다 → "오른쪽으로 회전해!" (방향만)
#   11주차: 얼마나 오른쪽인가? → "오차에 비례해서" 회전 속도 결정!
#
#     명령 = KP × 오차
#     - 오차가 크면 → 빠르게 움직임
#     - 오차가 작으면 → 천천히 (목표에 가까울수록 부드럽게)
#     - KP가 너무 크면 → 목표를 지나쳐서 왔다갔다 (발산!)
#     - KP가 너무 작으면 → 답답하게 느림
#
#   실행 중에 게인을 바꿔가며 실험해보세요:
#     ros2 param set /p_control_demo kp_yaw 0.005
#
#   실행 방법:
#     터미널 1: ros2 run drone_ros2_advanced w10_face_detector
#     터미널 2: ros2 run drone_ros2_advanced w11_p_control
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


# ================================================================
# P 게인 기본값 (웹캠 640x480 기준) — 12주차에서 실제 비행에 사용!
# ================================================================
KP_YAW = 0.002    # x 오차 [px] → yaw_rate [rad/s]
KP_VZ  = 0.002    # y 오차 [px] → vz [m/s]
KP_VX  = 0.005    # 얼굴 높이 오차 [px] → vx [m/s]

TARGET_FACE_H = 120.0   # 목표 얼굴 높이 [px]

# 속도 제한 (안전장치! 실제 드론에서 필수)
MAX_YAW_RATE = 0.5   # [rad/s]
MAX_VZ       = 1.0   # [m/s]
MAX_VX       = 1.5   # [m/s]


def clamp(value, limit):
    """[-limit, +limit] 범위로 자르기"""
    return max(-limit, min(limit, value))


class PControlDemo(Node):
    def __init__(self):
        super().__init__('p_control_demo')

        # 게인을 ROS 파라미터로 → 실행 중에 바꿀 수 있음!
        self.declare_parameter('kp_yaw', KP_YAW)
        self.declare_parameter('kp_vz',  KP_VZ)
        self.declare_parameter('kp_vx',  KP_VX)

        self.detection_sub = self.create_subscription(
            Float32MultiArray, '/face/detection',
            self._detection_callback, 10)

        self.get_logger().info(
            'P 제어 데모 시작!\n'
            '  실행 중 게인 변경: ros2 param set /p_control_demo kp_yaw 0.005')

    def _detection_callback(self, msg: Float32MultiArray):
        cx, cy, w, h, img_w, img_h = msg.data

        # 파라미터에서 현재 게인 읽기 (실행 중 변경 반영)
        kp_yaw = self.get_parameter('kp_yaw').value
        kp_vz  = self.get_parameter('kp_vz').value
        kp_vx  = self.get_parameter('kp_vx').value

        # ── 1. 오차 계산 (10주차와 동일) ─────────────────────────
        x_error    = cx - img_w / 2.0
        y_error    = cy - img_h / 2.0
        dist_error = TARGET_FACE_H - h   # 얼굴이 작으면(멀면) +→ 전진

        # ── 2. P 제어: 명령 = KP × 오차  ← 이번 주 핵심! ─────────
        yaw_rate = kp_yaw * x_error
        vz       = kp_vz  * y_error
        vx       = kp_vx  * dist_error

        # ── 3. 속도 제한 (클램프) ────────────────────────────────
        yaw_rate = clamp(yaw_rate, MAX_YAW_RATE)
        vz       = clamp(vz,       MAX_VZ)
        vx       = clamp(vx,       MAX_VX)

        self.get_logger().info(
            f'\n오차: x={x_error:+6.0f}px  y={y_error:+6.0f}px  '
            f'거리={dist_error:+6.0f}px\n'
            f'명령: yaw_rate={yaw_rate:+.3f}rad/s  '
            f'vz={vz:+.2f}m/s  vx={vx:+.2f}m/s\n'
            f'게인: kp_yaw={kp_yaw}  kp_vz={kp_vz}  kp_vx={kp_vx}',
            throttle_duration_sec=0.5
        )


def main(args=None):
    rclpy.init(args=args)
    node = PControlDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
