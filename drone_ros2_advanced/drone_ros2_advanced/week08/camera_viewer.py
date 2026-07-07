#!/usr/bin/env python3
# ==============================================================================
# File    : camera_viewer.py  (8주차)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   Gazebo 카메라 → ROS2 image 토픽 → OpenCV 창으로 보기
#
#   배우는 개념:
#     - Gazebo 카메라 플러그인이 만들어주는 image 토픽 확인:
#       ros2 topic list | grep -i image
#     - sensor_msgs/Image 메시지 구조: height, width, encoding, data(바이트)
#     - ROS Image → numpy 배열 → OpenCV 표시
#       (Gazebo 카메라는 RGB 순서, OpenCV는 BGR 순서 → 뒤집기 필요!)
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500_depth
#               (카메라 달린 기체: x500_depth)
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w08_camera_viewer
#
#   다른 카메라 토픽으로 실행:
#     ros2 run drone_ros2_advanced w08_camera_viewer \
#         --ros-args -p image_topic:=/my/camera/topic
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


# Gazebo x500_depth 기체의 카메라 토픽 (환경에 따라 이름이 다를 수 있어요)
DEFAULT_IMAGE_TOPIC = (
    '/world/person_follow/model/x500_depth_0/link/camera_link'
    '/sensor/IMX214/image'
)


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        self.declare_parameter('image_topic', DEFAULT_IMAGE_TOPIC)
        self.image_topic = self.get_parameter('image_topic').value

        self.subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10)

        self.frame_count = 0
        self.get_logger().info(
            f'카메라 뷰어 시작!\n  구독 토픽: {self.image_topic}\n'
            f'  이미지가 안 나오면: ros2 topic list | grep -i image')

    def image_callback(self, msg: Image):
        # ============================================================
        # ROS Image → OpenCV 변환  ← 이번 주 핵심!
        # ============================================================
        # 1) 바이트 덩어리(msg.data)를 numpy 배열로 해석
        # 2) (높이, 너비, 3채널) 모양으로 재배열
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)

        # 3) RGB → BGR (OpenCV는 BGR 순서를 사용!)
        frame = frame[:, :, ::-1].copy()

        # 화면 정보 표시
        self.frame_count += 1
        cv2.putText(frame,
                    f'{msg.width}x{msg.height}  frame #{self.frame_count}',
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                    1.0, (0, 255, 0), 2, cv2.LINE_AA)

        cv2.imshow('Camera Viewer (q: quit)', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
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
