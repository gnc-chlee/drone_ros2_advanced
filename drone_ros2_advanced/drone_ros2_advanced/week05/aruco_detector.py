#!/usr/bin/env python3
# ==============================================================================
# File    : aruco_detector.py  (5주차 2강 - 수업용 최소판)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-09-21
# Version : 3.0.0
#
# Description:
#   ArUco 마커 감지 노드 — 카메라 화면에서 마커를 찾아
#   "화면 중심에서 얼마나 벗어났는지"를 픽셀 단위로 발행합니다.
#
#   흐름 (콜백 한 번 = 프레임 한 장):
#     /camera/image_raw 구독 → ROS Image → OpenCV 프레임 (5주차 1강 [복붙 영역])
#     → 흑백 변환 → 마커 검출 → 꼭짓점 4개로 중심·크기 계산
#     → 오차 = 마커 중심 − 화면 중심 → /sjcu/error 발행 + 화면 표시
#
#   퍼블리시 토픽:
#     /sjcu/error : Float32MultiArray [x_error, y_error, size_error]
#       x_error    = 마커중심x − 화면중심x [px]  (마커가 화면 오른쪽이면 +)
#       y_error    = 마커중심y − 화면중심y [px]  (마커가 화면 아래쪽이면 +)
#                    ※ 이미지 좌표는 y 가 아래로 갈수록 커집니다 (수학 좌표와 반대!)
#       size_error = 마커 변 길이 평균 − target_marker_size [px]  (가까울수록 +)
#       ※ 마커를 못 찾은 프레임에서는 발행하지 않습니다
#          → 6주차 제어 노드는 "한동안 안 오면 마커 없음"으로 판단합니다
#
#   심화판: aruco_detector_hud.py (w05_aruco_hud) — 같은 검출 코어 + 위치·게이지 HUD
#
#   실행 방법 (터미널 4개):
#     터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=my_custom_world make px4_sitl gz_x500_mono_cam_down   # (또는 aruco)
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w05_camera_bridge     # Gazebo 카메라 → /camera/image_raw
#     터미널 4: ros2 run drone_ros2_advanced w05_aruco
#
#   오차 값 확인: ros2 topic echo /sjcu/error
#
#   OpenCV 4.7 이상 필요 (aruco.ArucoDetector 가 4.7 에서 생김):
#     pip install 'opencv-python>=4.7'
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import cv2
import cv2.aruco as aruco
import numpy as np


# ================================================================
# 파라미터
# ================================================================
ARUCO_DICT         = aruco.DICT_4X4_50   # PX4 내장 aruco 월드의 마커 = 4x4 격자, ID 0
TARGET_MARKER_ID   = 0                   # 이 ID 만 찾는다 (다른 마커는 무시)
TARGET_MARKER_SIZE = 150                 # 이 픽셀 크기를 "목표 거리"로 본다 (6주차에서 사용)

COLOR_GREEN = (0, 255, 0)
COLOR_RED   = (0, 0, 255)
COLOR_BLUE  = (255, 100, 0)


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ── 파라미터 (실행할 때 바꿀 수 있는 값) ─────────────────
        self.declare_parameter('image_topic',        '/camera/image_raw')
        self.declare_parameter('target_marker_id',   TARGET_MARKER_ID)
        self.declare_parameter('target_marker_size', TARGET_MARKER_SIZE)
        self.declare_parameter('display_image',      True)

        self.image_topic = self.get_parameter('image_topic').value
        self.target_id   = self.get_parameter('target_marker_id').value
        self.target_size = self.get_parameter('target_marker_size').value
        self.display     = self.get_parameter('display_image').value

        # ── [복붙 영역] ArUco 검출기 준비 3줄 ───────────────────
        #   사전(dictionary) = 마커 무늬 목록,  검출기 = 그 사전으로 찾는 도구
        self.aruco_dict   = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.aruco_params = aruco.DetectorParameters()
        self.detector     = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # ── 구독 1개: 카메라 (PX4 토픽이 아니라서 PX4_QOS 불필요 — 기본 QoS 10) ──
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, 10)

        # ── 발행 1개: 오차 ───────────────────────────────────────
        self.error_pub = self.create_publisher(Float32MultiArray, '/sjcu/error', 10)

        # ── 창 준비: WINDOW_NORMAL 로 만들면 마우스로 크기 조절 가능 (imshow 만 쓰면 고정) ──
        if self.display:
            cv2.namedWindow('ArUco Detector (q: quit)', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('ArUco Detector (q: quit)', 640, 480)   # 처음엔 절반 크기로

        self.frame_count = 0
        self.get_logger().info(
            f'ArUco 감지 시작!\n'
            f'  구독: {self.image_topic}   발행: /sjcu/error\n'
            f'  찾는 마커: ID {self.target_id}  (사전 4x4_50)')

    # ============================================================
    # 이미지 콜백 — 프레임 한 장마다 실행
    # ============================================================
    def _image_callback(self, msg: Image):
        # ── [복붙 영역] ROS Image → OpenCV 프레임 (5주차 1강과 동일) ──
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            frame = frame[:, :, ::-1]          # RGB → BGR
        frame = frame.copy()               # 쓰기 가능한 복사본

        self.frame_count += 1

        # ── 1. 흑백 변환 — 마커는 흑백 무늬라 색이 필요 없다 ─────
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── 2. 화면 중심 (오차의 기준점) ─────────────────────────
        cx = msg.width  / 2.0
        cy = msg.height / 2.0

        # ── 3. 마커 검출  ← 이번 강 핵심 한 줄 ───────────────────
        #   corners: 찾은 마커마다 꼭짓점 4개의 (x, y)
        #   ids    : 찾은 마커마다 ID  (하나도 없으면 None)
        corners, ids, _ = self.detector.detectMarkers(gray)

        found = False
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id != self.target_id:
                    continue                    # 내가 찾는 ID 가 아니면 건너뜀

                # ── 4. 꼭짓점 4개 → 중심과 크기 ─────────────────
                #   corner 는 4행 2열: [[x0,y0],[x1,y1],[x2,y2],[x3,y3]]
                corner = corners[i][0]
                mx = float(np.mean(corner[:, 0]))          # x 네 개의 평균 = 중심 x
                my = float(np.mean(corner[:, 1]))          # y 네 개의 평균 = 중심 y
                side_a = np.linalg.norm(corner[0] - corner[1])   # 한 변의 길이 [px]
                side_b = np.linalg.norm(corner[1] - corner[2])   # 옆 변의 길이 [px]
                marker_size = float((side_a + side_b) / 2.0)     # 크기 = 두 변의 평균

                # ── 5. 오차 = 마커 중심 − 화면 중심 ──────────────
                #   0 이면 마커가 화면 한가운데 = 드론이 마커 바로 위
                x_error    = mx - cx
                y_error    = my - cy
                size_error = marker_size - self.target_size

                # ── 6. 발행 (찾은 프레임에서만) ───────────────────
                error_msg      = Float32MultiArray()
                error_msg.data = [x_error, y_error, size_error]
                self.error_pub.publish(error_msg)

                # ── 7. 화면에 그리기 ─────────────────────────────
                aruco.drawDetectedMarkers(frame, corners)                     # 마커 테두리 + ID
                cv2.circle(frame, (int(mx), int(my)), 8, COLOR_RED, -1)       # 마커 중심
                cv2.line(frame, (int(mx), int(my)), (int(cx), int(cy)), COLOR_BLUE, 2)  # 중심까지 선
                cv2.putText(frame,
                            f'ID {marker_id}  err x={x_error:+.0f} y={y_error:+.0f}  size={marker_size:.0f}px',
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, COLOR_GREEN, 2)
                found = True
                break                            # 같은 ID 가 여러 개면 첫 번째만

        # ── 화면 중심 십자선 + 미검출 안내 ───────────────────────
        cv2.line(frame, (int(cx)-30, int(cy)), (int(cx)+30, int(cy)), COLOR_GREEN, 2)
        cv2.line(frame, (int(cx), int(cy)-30), (int(cx), int(cy)+30), COLOR_GREEN, 2)
        if not found:
            cv2.putText(frame, 'No Marker', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, COLOR_RED, 2)

        if self.display:
            cv2.imshow('ArUco Detector (q: quit)', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
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
