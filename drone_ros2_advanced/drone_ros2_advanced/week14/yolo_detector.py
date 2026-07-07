#!/usr/bin/env python3
# ==============================================================================
# File    : yolo_detector.py  (14주차)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   YOLOv8n 사람 감지 데모 - Gazebo 카메라에서 사람을 찾아 토픽으로 발행
#
#   Haar Cascade(10주차) vs YOLO(오늘):
#     Haar : 고전 방식. 얼굴 전용, 정면만, 빠름
#     YOLO : 딥러닝. 80종류 물체, 각도/조명에 강함, 그래도 v8n은 CPU 가능!
#
#   발행 토픽: /yolo/person_detection
#     [x_center, y_center, width, height, confidence]
#
#   준비물:
#     pip install ultralytics
#     (첫 실행 시 yolov8n.pt 모델을 자동 다운로드합니다)
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500_depth  (person 월드)
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w14_yolo
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
import numpy as np

# ================================================================
# [복붙 영역] ultralytics(YOLO) 불러오기
# conda 등 다른 환경에 설치했다면 아래 경로를 자기 환경에 맞게 수정!
# 시스템 파이썬에 설치했다면(pip install ultralytics) 두 줄은 삭제 OK
# ================================================================
import sys
sys.path.insert(0, '/home/sentiary/miniconda3/envs/dl_env/lib/python3.10/site-packages')
from ultralytics import YOLO


DEFAULT_IMAGE_TOPIC = (
    '/world/person_follow/model/x500_depth_0/link/camera_link'
    '/sensor/IMX214/image'
)


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # YOLO 모델 로드 (v8n = nano, 가장 작고 빠른 모델)
        self.model = YOLO('yolov8n.pt')

        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.image_topic = self.get_parameter('image_topic').value

        # Gazebo 카메라 구독
        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

        # 감지 결과 발행 [x_center, y_center, width, height, confidence]
        self.detection_pub = self.create_publisher(
            Float32MultiArray, '/yolo/person_detection', 10)

        self.get_logger().info(
            f'YOLO Detector 시작! image_topic={self.image_topic}')

    def image_callback(self, msg):
        # ROS 이미지 → OpenCV (8주차 복습)
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        frame = frame[:, :, ::-1].copy()   # RGB → BGR
        display_frame = frame.copy()

        # ── YOLO 추론  ← 이번 주 핵심! (classes=[0] → person만) ──
        results = self.model(frame, classes=[0], verbose=False)

        if results[0].boxes:
            # 가장 큰 사람 선택 (가장 가까운 사람)
            boxes = results[0].boxes
            areas = [(b.xywh[0][2] * b.xywh[0][3]).item() for b in boxes]
            largest_idx = areas.index(max(areas))
            box = boxes[largest_idx]

            x, y, w, h = box.xywh[0].tolist()
            conf = box.conf[0].item()

            # 모든 감지 결과 그리기 (가장 큰 것은 초록색)
            for idx, detected_box in enumerate(boxes):
                bx, by, bw, bh = detected_box.xywh[0].tolist()
                bconf = detected_box.conf[0].item()
                left, top = int(bx - bw/2), int(by - bh/2)
                right, bottom = int(bx + bw/2), int(by + bh/2)
                color = (0, 255, 0) if idx == largest_idx else (0, 180, 255)
                cv2.rectangle(display_frame, (left, top), (right, bottom),
                              color, 3 if idx == largest_idx else 2)
                cv2.putText(display_frame, f'person {bconf:.2f}',
                            (left, max(top - 8, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            # 발행 → 이 토픽을 구독하면 "사람 Following"도 가능!
            msg_out = Float32MultiArray()
            msg_out.data = [x, y, w, h, conf]
            self.detection_pub.publish(msg_out)

            self.get_logger().info(
                f'사람 감지! x={x:.0f}, y={y:.0f}, conf={conf:.2f}',
                throttle_duration_sec=1.0)

        cv2.imshow('YOLO Person Detection (q: quit)', display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
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
