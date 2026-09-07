#!/usr/bin/env python3
# ==============================================================================
# File    : camera_viewer.py  (5주차 1강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   Gazebo 카메라 → ROS2 image 토픽 → OpenCV 창으로 보기
#
#   브리지가 필요한 이유:
#     Gazebo 카메라는 uXRCE-DDS로 넘어오지 않아 ros_gz_bridge(w05_camera_bridge)가 필요
#
#   배우는 개념:
#     - 브리지가 만들어주는 image 토픽 확인:
#       브리지 실행 후 `ros2 topic list | grep camera`
#     - sensor_msgs/Image 메시지 구조: height, width, encoding, data(바이트)
#     - ROS Image → numpy 배열 → OpenCV 표시
#       (Gazebo 카메라는 RGB 순서, OpenCV는 BGR 순서 → 뒤집기 필요!)
#
#   실행 방법 (터미널 4개):
#     터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w05_camera_bridge     # Gazebo 카메라 → /camera/image_raw
#     터미널 4: ros2 run drone_ros2_advanced w05_camera_viewer
#
#   다른 카메라 토픽으로 실행:
#     ros2 run drone_ros2_advanced w05_camera_viewer \
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


# 브리지(w05_camera_bridge)가 Gazebo 카메라를 이 이름으로 바꿔 줍니다
# (다른 토픽을 보려면 image_topic 파라미터로 바꾸세요)
DEFAULT_IMAGE_TOPIC = '/camera/image_raw'


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
            f'  이미지가 안 나오면: 브리지 실행 후 ros2 topic list | grep camera')

        # 3초마다 프레임이 들어왔는지 확인 — 하나도 없으면 경고
        self.check_timer = self.create_timer(3.0, self._check_frames)

    def _check_frames(self):
        # 프레임이 들어오기 시작했으면 더 이상 확인하지 않음
        if self.frame_count > 0:
            self.check_timer.cancel()
            return
        self.get_logger().warn(
            '아직 이미지 없음 — 터미널 3의 w05_camera_bridge 와 '
            'PX4 카메라 기체 실행을 확인하세요')

    def image_callback(self, msg: Image):
        # ============================================================
        # ROS Image → OpenCV 변환  ← 이번 주 핵심!
        # ============================================================
        # ── [복붙 영역] ROS Image → OpenCV 프레임 ─────────────────
        # bytes 한 줄 → (높이, 너비, 채널) 표로 접기 → OpenCV는 BGR 순서
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            frame = frame[:, :, ::-1]          # RGB → BGR
        frame = frame.copy()               # 쓰기 가능한 복사본 (frombuffer 결과는 읽기 전용)

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
