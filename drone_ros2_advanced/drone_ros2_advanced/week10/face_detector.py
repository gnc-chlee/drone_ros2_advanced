#!/usr/bin/env python3
# ==============================================================================
# File    : face_detector.py  (10주차)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   Haar Cascade 얼굴 인식 → 얼굴 위치를 ROS2 토픽으로 발행
#
#   구조 (12주차 Following의 "눈" 역할):
#     [웹캠] → [얼굴 인식] → /face/detection 토픽
#                              [x_center, y_center, w, h, img_w, img_h]
#
#   배우는 개념:
#     - Haar Cascade: 2001년부터 쓰인 고전 얼굴 인식 (가볍고 CPU로 충분)
#     - OpenCV에 학습된 모델이 내장되어 있음 (cv2.data.haarcascades)
#     - 인식 결과를 "토픽"으로 발행 → 제어 노드와 분리!
#       (인식 따로, 제어 따로 = ROS의 핵심 설계 철학)
#
#   실행 방법:
#     ros2 run drone_ros2_advanced w10_face_detector
#   확인:
#     ros2 topic echo /face/detection
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import cv2


CAMERA_INDEX = 0    # 웹캠 번호
DETECT_HZ    = 15   # 초당 인식 횟수 (Haar는 가벼워서 CPU로 충분)


class FaceDetector(Node):
    def __init__(self):
        super().__init__('face_detector')

        # ============================================================
        # [복붙 영역] Haar Cascade 모델 로드
        # 원리: 얼굴의 밝기 패턴(눈은 어둡고 코는 밝고...)을 학습한
        #       XML 파일이 OpenCV에 이미 들어 있습니다.
        # ============================================================
        cascade_path = (cv2.data.haarcascades
                        + 'haarcascade_frontalface_default.xml')
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(f'웹캠 {CAMERA_INDEX}번을 열 수 없습니다!')

        # 얼굴 위치 발행 토픽
        self.detection_pub = self.create_publisher(
            Float32MultiArray, '/face/detection', 10)

        self.create_timer(1.0 / DETECT_HZ, self._detect_loop)
        self.get_logger().info(
            '얼굴 인식 시작! → /face/detection 토픽 발행 중')

    # ============================================================
    # 인식 루프
    # ============================================================
    def _detect_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        img_h, img_w = frame.shape[:2]

        # Haar는 그레이스케일에서 동작
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── 얼굴 인식 ← 이번 주 핵심! ────────────────────────────
        # scaleFactor : 1.1 = 10%씩 크기를 줄여가며 탐색
        # minNeighbors: 클수록 깐깐하게 (오인식↓, 놓침↑)
        faces = self.face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))

        if len(faces) > 0:
            # 가장 큰 얼굴 선택 (가장 가까운 사람)
            x, y, w, h = max(faces, key=lambda f: f[2] * f[3])
            cx = x + w / 2.0
            cy = y + h / 2.0

            # ── 토픽 발행: [중심x, 중심y, 폭, 높이, 이미지폭, 이미지높이]
            msg = Float32MultiArray()
            msg.data = [float(cx), float(cy), float(w), float(h),
                        float(img_w), float(img_h)]
            self.detection_pub.publish(msg)

            # 시각화
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.circle(frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
            cv2.putText(frame, f'face ({int(cx)},{int(cy)}) h={h}px',
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (0, 255, 0), 2)

            self.get_logger().info(
                f'얼굴 감지! 중심=({cx:.0f},{cy:.0f}) 높이={h}px',
                throttle_duration_sec=1.0
            )

        cv2.imshow('Face Detector (q: quit)', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FaceDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
