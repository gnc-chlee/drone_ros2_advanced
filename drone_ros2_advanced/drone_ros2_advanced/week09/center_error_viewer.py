#!/usr/bin/env python3
# ==============================================================================
# File    : center_error_viewer.py  (9주차)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   카메라 기반 드론 제어 "개념" 시각화 - 아직 드론은 안 움직입니다!
#
#   화면에 보여주는 것:
#     - 화면 중앙 십자선 (드론이 바라보는 정면)
#     - 가장 큰 물체(contour)의 중심점
#     - 중앙 → 물체 화살표 = "오차(error)"
#     - 이 오차가 어떤 드론 명령이 되는지 텍스트로 표시
#
#   핵심 개념 (PX4 드론 적용 차이점):
#     물체가 오른쪽에 있다 (x_error +) → 오른쪽으로 회전 (yaw_rate +)
#     물체가 아래에 있다   (y_error +) → 아래로 이동     (vz +, NED라 +가 하강!)
#     → 12주차에서 이 값을 진짜 드론 명령으로 보냅니다
#
#   실행 방법 (웹캠):
#     ros2 run drone_ros2_advanced w09_center_error
#   실행 방법 (Gazebo 카메라):
#     ros2 run drone_ros2_advanced w09_center_error --ros-args -p use_webcam:=false
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


DEFAULT_IMAGE_TOPIC = (
    '/world/person_follow/model/x500_depth_0/link/camera_link'
    '/sensor/IMX214/image'
)

THRESHOLD = 127
MIN_AREA  = 500


class CenterErrorViewer(Node):
    def __init__(self):
        super().__init__('center_error_viewer')

        self.declare_parameter('use_webcam', True)
        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.use_webcam = self.get_parameter('use_webcam').value

        if self.use_webcam:
            self.cap = cv2.VideoCapture(0)
            self.create_timer(1.0 / 30, self._webcam_loop)   # 30Hz
            self.get_logger().info('웹캠 모드로 시작!')
        else:
            topic = self.get_parameter('image_topic').value
            self.create_subscription(
                Image, topic, self._image_callback, 10)
            self.get_logger().info(f'Gazebo 카메라 모드: {topic}')

    # ── 웹캠 모드 ────────────────────────────────────────────────
    def _webcam_loop(self):
        ret, frame = self.cap.read()
        if ret:
            self._process(frame)

    # ── Gazebo 카메라 모드 ───────────────────────────────────────
    def _image_callback(self, msg: Image):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        frame = frame[:, :, ::-1].copy()   # RGB → BGR
        self._process(frame)

    # ============================================================
    # 오차 계산 + 시각화  ← 이번 주 핵심!
    # ============================================================
    def _process(self, frame):
        h, w = frame.shape[:2]
        center_x, center_y = w // 2, h // 2

        # 화면 중앙 십자선 (드론의 정면)
        cv2.line(frame, (center_x - 30, center_y),
                 (center_x + 30, center_y), (255, 255, 0), 2)
        cv2.line(frame, (center_x, center_y - 30),
                 (center_x, center_y + 30), (255, 255, 0), 2)

        # 8주차 복습: 가장 큰 contour 찾기
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, binary = cv2.threshold(
            blurred, THRESHOLD, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > MIN_AREA:
                x, y, bw, bh = cv2.boundingRect(largest)
                cx = x + bw // 2
                cy = y + bh // 2

                # ── 오차 계산: 물체 중심 - 화면 중앙 ─────────────
                x_error = cx - center_x   # +: 물체가 오른쪽
                y_error = cy - center_y   # +: 물체가 아래쪽

                # 중앙 → 물체 화살표 (= 오차 벡터)
                cv2.arrowedLine(frame, (center_x, center_y), (cx, cy),
                                (0, 0, 255), 3)
                cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)

                # ── 오차 → 드론 명령 해석 ────────────────────────
                yaw_txt = ('오른쪽 회전' if x_error > 50 else
                           '왼쪽 회전' if x_error < -50 else 'yaw 유지')
                vz_txt  = ('하강' if y_error > 50 else
                           '상승' if y_error < -50 else '고도 유지')

                cv2.putText(frame,
                            f'x_err={x_error:+d}px -> {yaw_txt}',
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 0), 2)
                cv2.putText(frame,
                            f'y_err={y_error:+d}px -> {vz_txt}',
                            (20, 75), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 0), 2)

                self.get_logger().info(
                    f'x_err={x_error:+4d} ({yaw_txt}) / '
                    f'y_err={y_error:+4d} ({vz_txt})',
                    throttle_duration_sec=1.0
                )

        cv2.imshow('Center Error Viewer (q: quit)', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CenterErrorViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
