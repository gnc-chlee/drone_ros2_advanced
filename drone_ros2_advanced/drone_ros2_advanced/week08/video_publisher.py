#!/usr/bin/env python3
# ==============================================================================
# File    : video_publisher.py  (8주차 14강 - 영상 발행 도우미)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-10-09
# Version : 1.0.0
#
# Description:
#   녹화 영상(mp4)이나 웹캠을 /camera/image_raw 토픽으로 발행합니다.
#   드론 카메라(w05_camera_bridge)와 **같은 토픽 이름**으로 내보내므로,
#   사람 감지 노드(w08_person)는 코드를 고치지 않고 그대로 이 영상에서 사람을 찾습니다.
#     → "노드는 토픽 이름만 안다": 사진이 어디서 오는지는 몰라도 된다
#
#   Gazebo 를 돌리지 않아도 되므로 14강 실습 1(녹화 영상)과,
#   Gazebo 가 너무 느린 PC 의 대체 경로로 씁니다.
#
#   실행 방법:
#     ros2 run drone_ros2_advanced w08_video_pub                    # samples/person_walk.mp4 (반복 재생, 교수 촬영본)
#     ros2 run drone_ros2_advanced w08_video_pub gazebo             # samples/gazebo_front.mp4 (Gazebo 녹화본)
#     ros2 run drone_ros2_advanced w08_video_pub ~/Videos/walk.mp4  # 내 영상
#     ros2 run drone_ros2_advanced w08_video_pub cam                # 웹캠 0번 (1번이면 cam1)
#
#   강사용 — 지금 /camera/image_raw 를 mp4 로 녹화 (Gazebo 녹화본 만들기):
#     ros2 run drone_ros2_advanced w08_video_pub --record gazebo_front.mp4      # Ctrl+C 로 끝 (지금 폴더에 저장)
#     ros2 run drone_ros2_advanced w08_video_pub ./gazebo_front.mp4             # 같은 폴더에서 바로 재생
#     (w08_video_pub gazebo 로 쓰려면 src 의 week08/samples/ 로 옮기고 colcon build)
#
#   발행 토픽:
#     /camera/image_raw   : sensor_msgs/Image (bgr8)
#     /camera/camera_info : sensor_msgs/CameraInfo (화면 크기만 채움 — 9주차 시험용)
#
#   큰 영상(예: 휴대폰 1920×1080)은 가로 640 으로 줄여서 보냅니다 (--width 로 변경).
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import argparse
import array
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

import cv2
import numpy as np


# ================================================================
# 파라미터
# ================================================================
HERE       = os.path.dirname(os.path.abspath(__file__))
SAMPLE_DIR = os.path.join(HERE, 'samples')
SOURCES    = {None: 'person_walk.mp4', 'gazebo': 'gazebo_front.mp4'}   # 이름 → samples/ 안의 파일

IMAGE_TOPIC = '/camera/image_raw'
INFO_TOPIC  = '/camera/camera_info'
DEFAULT_FPS = 15.0      # 영상에 fps 정보가 없을 때
WEBCAM_FPS  = 15


def to_image_msg(frame, stamp):
    """OpenCV 프레임(BGR) → ROS Image.  (5주차 1강의 반대 방향)"""
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = 'camera'
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    # [복붙 영역] 바이트를 한 번에 넘긴다 (파이썬 목록으로 바꾸면 한 장에 0.1초 넘게 걸림)
    msg.data = array.array('B', np.ascontiguousarray(frame).tobytes())
    return msg


def resize_to_width(frame, width):
    h, w = frame.shape[:2]
    if width <= 0 or w <= width:
        return frame
    return cv2.resize(frame, (width, int(round(h * width / w))), interpolation=cv2.INTER_AREA)


# ================================================================
# 발행 노드 — 영상·웹캠 → /camera/image_raw
# ================================================================
class VideoPublisher(Node):
    def __init__(self, cap, title, fps, is_file, loop, width):
        super().__init__('video_publisher')
        self.cap, self.title, self.width = cap, title, width
        self.is_file, self.loop = is_file, loop
        self.done = False        # 끝낼 때 표시만 세움 (콜백 안 rclpy.shutdown() 은 Humble 에서 멈춤)
        self.image_pub = self.create_publisher(Image, IMAGE_TOPIC, 10)
        self.info_pub  = self.create_publisher(CameraInfo, INFO_TOPIC, 10)
        self.count = 0
        self.rounds = 1
        self.fail = 0
        self.timer = self.create_timer(1.0 / fps, self._tick)
        self.get_logger().info(f'영상 발행: {title}  {fps:.0f} fps → {IMAGE_TOPIC}'
                               f'{" (반복 재생)" if is_file and loop else ""}   끝내기: Ctrl+C')

    def _tick(self):
        ret, frame = self.cap.read()
        if not ret:
            if self.is_file and self.count > 0:              # 영상 끝
                if not self.loop:
                    self.get_logger().info('영상 끝 → 발행을 멈춥니다.')
                    self.done = True
                    return
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)      # 처음으로 되감기
                self.rounds += 1
                self.get_logger().info(f'영상 끝 → 처음부터 다시 ({self.rounds}번째)')
                return
            self.fail += 1
            if self.fail >= 30:
                self.get_logger().error('프레임을 읽지 못합니다 (영상 파일·웹캠 연결 확인). 종료합니다.')
                self.done = True
            return
        self.fail = 0
        frame = resize_to_width(frame, self.width)
        stamp = self.get_clock().now().to_msg()
        self.image_pub.publish(to_image_msg(frame, stamp))

        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = 'camera'
        info.height, info.width = frame.shape[:2]
        self.info_pub.publish(info)

        self.count += 1
        if self.count == 1:
            self.get_logger().info(f'첫 프레임 발행: {frame.shape[1]}x{frame.shape[0]}  '
                                   f'(확인: ros2 topic hz {IMAGE_TOPIC})')


# ================================================================
# 녹화 노드 (강사용) — /camera/image_raw → mp4
# ================================================================
class VideoRecorder(Node):
    def __init__(self, path, fps):
        super().__init__('video_recorder')
        self.path, self.fps = path, fps
        self.writer = None
        self.count = 0
        self.done = False
        self.create_subscription(Image, IMAGE_TOPIC, self._callback, 10)
        self.get_logger().info(f'녹화 대기: {IMAGE_TOPIC} → {path}   끝내기: Ctrl+C')

    def _callback(self, msg):
        # [복붙 영역] ROS Image → OpenCV 프레임 (5주차 1강과 동일)
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            frame = frame[:, :, ::-1]
        frame = np.ascontiguousarray(frame)
        if self.writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.writer = cv2.VideoWriter(self.path, fourcc, self.fps, (msg.width, msg.height))
            if not self.writer.isOpened():
                self.get_logger().error(f'녹화 파일을 열 수 없습니다: {self.path}')
                self.writer = None
                self.done = True
                return
            self.get_logger().info(f'녹화 시작: {msg.width}x{msg.height} @ {self.fps:.0f} fps')
        self.writer.write(frame)
        self.count += 1
        if self.count % (int(self.fps) * 5) == 0:
            self.get_logger().info(f'녹화 중: {self.count}장')

    def close(self):
        if self.writer is not None:
            self.writer.release()
            print(f'녹화 저장: {self.path} ({self.count}장)')


# ================================================================
# 입력 고르기
# ================================================================
def open_source(src):
    """(VideoCapture, 제목, 영상인가) 반환. 못 열면 안내 후 종료"""
    if src is not None and src.lower().startswith('cam') and (src[3:] == '' or src[3:].isdigit()):
        index = int(src[3:] or 0)
        backend = cv2.CAP_V4L2 if sys.platform.startswith('linux') else cv2.CAP_ANY
        cap = cv2.VideoCapture(index, backend)
        if not cap.isOpened():
            print(f'[오류] 웹캠 {index}번을 열 수 없습니다 (VM 메뉴에서 웹캠을 연결했는지, ls /dev/video* 확인).')
            sys.exit(1)
        # [복붙 영역] VM 웹캠 대응: 압축 포맷 + 낮은 해상도 (13강 데모와 같음)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, WEBCAM_FPS)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap, f'웹캠 {index}', False

    if src in SOURCES:
        path = os.path.join(SAMPLE_DIR, SOURCES[src])
        if not os.path.isfile(path):
            print(f'[안내] 예시 영상이 없습니다: {path}')
            print('       내 영상 경로를 주거나 웹캠을 쓰세요:')
            print('         ros2 run drone_ros2_advanced w08_video_pub ~/Videos/walk.mp4')
            print('         ros2 run drone_ros2_advanced w08_video_pub cam')
            sys.exit(1)
    else:
        path = os.path.expanduser(src)
        if not os.path.isfile(path):
            print(f'[오류] 영상 파일을 찾을 수 없습니다: {path}')
            sys.exit(1)
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f'[오류] 영상 파일을 열 수 없습니다: {path}')
        sys.exit(1)
    return cap, os.path.basename(path), True


def main(args=None):
    parser = argparse.ArgumentParser(description='영상·웹캠 → /camera/image_raw 발행 (14강)')
    parser.add_argument('source', nargs='?', default=None,
                        help='영상 파일 경로, gazebo(Gazebo 녹화본), cam/cam1(웹캠). 없으면 samples/person_walk.mp4')
    parser.add_argument('--fps', type=float, default=0.0, help='발행 속도 (기본: 영상의 fps, 웹캠 15)')
    parser.add_argument('--width', type=int, default=640, help='이보다 넓은 영상은 이 폭으로 줄임 (0 = 그대로)')
    parser.add_argument('--no-loop', action='store_true', help='영상이 끝나면 반복하지 않고 멈춤')
    parser.add_argument('--record', metavar='OUT.mp4', default=None,
                        help='(강사용) 발행 대신 지금 /camera/image_raw 를 mp4 로 녹화')
    argv = sys.argv[1:] if args is None else list(args)
    if '--ros-args' in argv:                    # ros2 run 이 붙이는 --ros-args 뒤는 rclpy 몫
        argv = argv[:argv.index('--ros-args')]
    opts = parser.parse_args(argv)

    if opts.record:
        rclpy.init()
        node = VideoRecorder(os.path.abspath(os.path.expanduser(opts.record)), opts.fps or DEFAULT_FPS)
        try:
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            pass
        finally:
            node.close()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        return

    cap, title, is_file = open_source(opts.source)
    fps = opts.fps
    if fps <= 0:
        fps = cap.get(cv2.CAP_PROP_FPS) if is_file else WEBCAM_FPS
        if not fps or fps != fps or fps > 120:  # 0 · NaN · 엉뚱한 값이면 기본값
            fps = DEFAULT_FPS

    rclpy.init()
    node = VideoPublisher(cap, title, fps, is_file=is_file, loop=not opts.no_loop, width=opts.width)
    try:
        while rclpy.ok() and not node.done:      # rclpy.spin(node) 대신 — done 이면 멈춘다
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
