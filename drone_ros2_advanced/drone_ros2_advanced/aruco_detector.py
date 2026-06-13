#!/usr/bin/env python3
# ==============================================================================
# File    : aruco_detector.py
# Author  : Choonghyeon Lee (gnc-chlee)
# Date    : 2026-06-08
# Version : 2.0.0
#
# Description:
#   ArUco 마커 감지 노드 (HUD 버전)
#   카메라 이미지에서 ArUco 마커를 감지하고
#   이미지 중심 기준 오차값을 퍼블리시
#
#   HUD 표시:
#     좌상단 : GPS 위도/경도, 로컬 X/Y, 고도
#     우상단 : 마커 ID, 고도, Err X/Y
#     하단   : X/Y 오차 게이지 바
#
#   구독 토픽:
#     image_topic (파라미터)
#     /fmu/out/vehicle_local_position
#     /fmu/out/vehicle_global_position
#
#   퍼블리시 토픽:
#     /sjcu/error : [x_error, y_error, z_error]
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy,
    HistoryPolicy, DurabilityPolicy
)
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition, VehicleGlobalPosition, SensorGps

import cv2
import cv2.aruco as aruco
import numpy as np


# ================================================================
# QoS
# ================================================================
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# ================================================================
# 파라미터
# ================================================================
IMAGE_WIDTH  = 1920
IMAGE_HEIGHT = 1080

ARUCO_DICT         = aruco.DICT_4X4_50
TARGET_MARKER_SIZE = 150
TARGET_MARKER_ID   = 0

# HUD 폰트 설정
FONT         = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE   = 0.75
THICKNESS    = 2
COLOR_WHITE  = (255, 255, 255)
COLOR_GREEN  = (0, 255, 0)
COLOR_YELLOW = (0, 255, 255)
COLOR_RED    = (0, 0, 255)
COLOR_CYAN   = (255, 255, 0)
COLOR_BLUE   = (255, 100, 0)

# 정렬 완료 임계값 [pixel]
ALIGN_THRESH = 30


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # ── 파라미터 ─────────────────────────────────────────────
        self.declare_parameter(
            'image_topic',
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image'
        )
        self.declare_parameter('target_marker_id',   TARGET_MARKER_ID)
        self.declare_parameter('target_marker_size', TARGET_MARKER_SIZE)
        self.declare_parameter('display_image',      True)

        self.image_topic = self.get_parameter('image_topic').value
        self.target_id   = self.get_parameter('target_marker_id').value
        self.target_size = self.get_parameter('target_marker_size').value
        self.display     = self.get_parameter('display_image').value

        # ── ArUco 설정 ───────────────────────────────────────────
        self.aruco_dict   = aruco.getPredefinedDictionary(ARUCO_DICT)
        self.aruco_params = aruco.DetectorParameters()
        self.detector     = aruco.ArucoDetector(
            self.aruco_dict, self.aruco_params
        )

        # ── Subscribers ─────────────────────────────────────────
        self.image_sub = self.create_subscription(
            Image, self.image_topic,
            self._image_callback, 10
        )
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self._local_pos_callback, PX4_QOS
        )
        self.gps_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self._gps_callback, PX4_QOS
        )

        # ── Publishers ──────────────────────────────────────────
        self.error_pub = self.create_publisher(
            Float32MultiArray, '/sjcu/error', 10
        )

        # ── 상태 변수 ────────────────────────────────────────────
        self.local_x   = 0.0
        self.local_y   = 0.0
        self.local_z   = 0.0
        self.latitude  = 0.0
        self.longitude = 0.0

        self.last_marker_id = None
        self.last_x_error   = 0.0
        self.last_y_error   = 0.0
        self.detected       = False

        # ── 창 설정 ──────────────────────────────────────────────
        if self.display:
            cv2.namedWindow('ArUco Detector', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('ArUco Detector', IMAGE_WIDTH // 2, IMAGE_HEIGHT // 2)

        self.get_logger().info(
            f'ArucoDetector v2.0 시작!\n'
            f'  topic    : {self.image_topic}\n'
            f'  target_id: {self.target_id}'
        )

    # ============================================================
    # PX4 콜백
    # ============================================================
    def _local_pos_callback(self, msg: VehicleLocalPosition):
        self.local_x = msg.x
        self.local_y = msg.y
        self.local_z = msg.z

    def _gps_callback(self, msg: VehicleGlobalPosition):
        self.latitude  = msg.lat
        self.longitude = msg.lon

    # ============================================================
    # 이미지 콜백
    # ============================================================
    def _image_callback(self, msg: Image):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, -1
        )
        if msg.encoding == 'rgb8':
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        display_frame = frame.copy()
        gray          = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        cx = msg.width  / 2.0
        cy = msg.height / 2.0

        # ── 중심 십자선 ──────────────────────────────────────────
        cv2.line(display_frame,
                 (int(cx)-30, int(cy)), (int(cx)+30, int(cy)),
                 COLOR_GREEN, 2)
        cv2.line(display_frame,
                 (int(cx), int(cy)-30), (int(cx), int(cy)+30),
                 COLOR_GREEN, 2)

        # ── ArUco 감지 ───────────────────────────────────────────
        corners, ids, _ = self.detector.detectMarkers(gray)
        self.detected = False

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if self.target_id is not None and marker_id != self.target_id:
                    continue

                self.detected = True
                corner        = corners[i][0]

                mx = float(np.mean(corner[:, 0]))
                my = float(np.mean(corner[:, 1]))

                w = np.linalg.norm(corner[0] - corner[1])
                h = np.linalg.norm(corner[1] - corner[2])
                marker_size = float((w + h) / 2.0)

                x_error = mx - cx
                y_error = my - cy
                z_error = marker_size - self.target_size

                self.last_marker_id = marker_id
                self.last_x_error   = x_error
                self.last_y_error   = y_error

                # 퍼블리시
                error_msg      = Float32MultiArray()
                error_msg.data = [x_error, y_error, z_error]
                self.error_pub.publish(error_msg)

                # ── 마커 시각화 ──────────────────────────────────
                aruco.drawDetectedMarkers(display_frame, corners)

                # 마커 중심점
                cv2.circle(display_frame,
                           (int(mx), int(my)), 8, COLOR_RED, -1)

                # 마커 → 이미지 중심 선
                cv2.line(display_frame,
                         (int(mx), int(my)), (int(cx), int(cy)),
                         COLOR_BLUE, 2)

                # 정렬 완료 원 표시
                if abs(x_error) < ALIGN_THRESH and abs(y_error) < ALIGN_THRESH:
                    cv2.circle(display_frame,
                               (int(cx), int(cy)), 50,
                               COLOR_GREEN, 3)

        # ── HUD 그리기 ───────────────────────────────────────────
        self._draw_hud(display_frame, msg.width, msg.height)

        if not self.detected:
            cv2.putText(
                display_frame, 'No Marker Detected',
                (int(cx) - 150, int(cy) + 80),
                FONT, 1.0, COLOR_RED, 2
            )

        if self.display:
            cv2.imshow('ArUco Detector', display_frame)
            cv2.waitKey(1)

    # ============================================================
    # HUD 그리기
    # ============================================================
    def _draw_hud(self, frame, width, height):
        altitude = abs(self.local_z)

        # ── 반투명 배경 ──────────────────────────────────────────
        overlay = frame.copy()
        cv2.rectangle(overlay, (0, 0),          (430, 210),      (0,0,0), -1)
        cv2.rectangle(overlay, (width-430, 0),  (width, 210),    (0,0,0), -1)
        cv2.rectangle(overlay, (0, height-70),  (width, height), (0,0,0), -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)

        # ── 좌상단: 위치 정보 ────────────────────────────────────
        y = 35
        cv2.putText(frame, '[ POSITION ]',
                    (10, y), FONT, FONT_SCALE, COLOR_CYAN, THICKNESS)
        y += 35
        cv2.putText(frame, f'Lat  : {self.latitude:.6f}',
                    (10, y), FONT, FONT_SCALE, COLOR_WHITE, THICKNESS)
        y += 35
        cv2.putText(frame, f'Lon  : {self.longitude:.6f}',
                    (10, y), FONT, FONT_SCALE, COLOR_WHITE, THICKNESS)
        y += 35
        cv2.putText(frame, f'X    : {self.local_x:+.2f} m',
                    (10, y), FONT, FONT_SCALE, COLOR_WHITE, THICKNESS)
        y += 35
        cv2.putText(frame, f'Y    : {self.local_y:+.2f} m',
                    (10, y), FONT, FONT_SCALE, COLOR_WHITE, THICKNESS)

        # ── 우상단: 마커 정보 ────────────────────────────────────
        xr = width - 420
        y  = 35
        cv2.putText(frame, '[ MARKER ]',
                    (xr, y), FONT, FONT_SCALE, COLOR_CYAN, THICKNESS)
        y += 35

        if self.detected and self.last_marker_id is not None:
            cv2.putText(frame, f'ID   : {self.last_marker_id}',
                        (xr, y), FONT, FONT_SCALE, COLOR_YELLOW, THICKNESS)
            y += 35
            cv2.putText(frame, f'Alt  : {altitude:.2f} m',
                        (xr, y), FONT, FONT_SCALE, COLOR_WHITE, THICKNESS)
            y += 35

            ex_color = COLOR_GREEN if abs(self.last_x_error) < ALIGN_THRESH else COLOR_RED
            cv2.putText(frame, f'Err X: {self.last_x_error:+.0f} px',
                        (xr, y), FONT, FONT_SCALE, ex_color, THICKNESS)
            y += 35

            ey_color = COLOR_GREEN if abs(self.last_y_error) < ALIGN_THRESH else COLOR_RED
            cv2.putText(frame, f'Err Y: {self.last_y_error:+.0f} px',
                        (xr, y), FONT, FONT_SCALE, ey_color, THICKNESS)
        else:
            cv2.putText(frame, 'No Marker',
                        (xr, y), FONT, FONT_SCALE, COLOR_RED, THICKNESS)
            y += 35
            cv2.putText(frame, f'Alt  : {altitude:.2f} m',
                        (xr, y), FONT, FONT_SCALE, COLOR_WHITE, THICKNESS)

        # ── 하단: X 오차 게이지 ──────────────────────────────────
        if self.detected:
            bar_y    = height - 40
            bar_cx   = width // 2
            bar_half = 300

            # 배경 바
            cv2.rectangle(frame,
                          (bar_cx - bar_half, bar_y - 12),
                          (bar_cx + bar_half, bar_y + 12),
                          (60, 60, 60), -1)

            # 오차 바
            ex_norm  = int(max(-bar_half, min(bar_half,
                          self.last_x_error / (IMAGE_WIDTH/2) * bar_half)))
            ex_color = COLOR_GREEN if abs(self.last_x_error) < ALIGN_THRESH else COLOR_RED
            if ex_norm != 0:
                cv2.rectangle(frame,
                              (bar_cx, bar_y - 12),
                              (bar_cx + ex_norm, bar_y + 12),
                              ex_color, -1)

            # 중심선
            cv2.line(frame,
                     (bar_cx, bar_y - 18), (bar_cx, bar_y + 18),
                     COLOR_WHITE, 2)

            # 레이블
            cv2.putText(frame, 'ERR X',
                        (bar_cx - bar_half - 70, bar_y + 6),
                        FONT, 0.55, COLOR_WHITE, 1)

            # ── Y 오차 게이지 (우측 수직) ────────────────────────
            gy_x    = width - 25
            gy_cy   = height // 2
            gy_half = 150

            cv2.rectangle(frame,
                          (gy_x - 12, gy_cy - gy_half),
                          (gy_x + 12, gy_cy + gy_half),
                          (60, 60, 60), -1)

            ey_norm  = int(max(-gy_half, min(gy_half,
                           self.last_y_error / (IMAGE_HEIGHT/2) * gy_half)))
            ey_color = COLOR_GREEN if abs(self.last_y_error) < ALIGN_THRESH else COLOR_RED
            if ey_norm != 0:
                cv2.rectangle(frame,
                              (gy_x - 12, gy_cy),
                              (gy_x + 12, gy_cy + ey_norm),
                              ey_color, -1)

            cv2.line(frame,
                     (gy_x - 18, gy_cy), (gy_x + 18, gy_cy),
                     COLOR_WHITE, 2)
            cv2.putText(frame, 'Y',
                        (gy_x - 8, gy_cy - gy_half - 10),
                        FONT, 0.55, COLOR_WHITE, 1)

    def destroy_node(self):
        if self.display:
            cv2.destroyWindow('ArUco Detector')
        super().destroy_node()


# ================================================================
# 메인
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()