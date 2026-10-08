#!/usr/bin/env python3
# ==============================================================================
# File    : person_detector.py  (8주차 14강 - 사람 인식 노드)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-10-09
# Version : 1.0.0
#
# Description:
#   사람 감지 노드 — 카메라 화면에서 사람을 찾아
#   "화면 중심에서 얼마나 벗어났는지 + 얼마나 큰지"를 픽셀 단위로 발행합니다.
#   5주차 aruco_detector.py 의 틀을 그대로 쓰고, 검출 자리만 13강 DNN 4단계로 바꿨습니다.
#
#   흐름 (콜백 한 번 = 프레임 한 장):
#     /camera/image_raw 구독 (대기열 1장 = 최신 사진만) → 1초에 최대 5번만 처리
#     → ROS Image → OpenCV 프레임 (5주차와 같은 [복붙 영역])
#     → [13강 ②] 전처리 → [13강 ③] 추론 → [13강 ④] 해석 (사람 · 기준값 · 최소 크기)
#     → 가장 큰 박스(= 가장 가까운 사람) 하나 고르기
#     → 오차 = 박스 중심 − 화면 중심, 크기 오차 = 박스 높이 − 목표 높이
#     → /sjcu/person_error 발행 + 화면 표시
#     ([13강 ①] 모델 읽기는 처음 한 번, __init__ 에서)
#
#   퍼블리시 토픽:
#     /sjcu/person_error : Float32MultiArray [x_err, y_err, size_err, box_h, conf]
#       x_err    = 박스중심x − 화면중심x [px]  (사람이 화면 오른쪽이면 +)
#       y_err    = 박스중심y − 화면중심y [px]  (아래쪽이면 +, 표시용)
#       size_err = 박스 높이 − target_box_ratio × 화면 높이 [px]  (가까우면 +, 멀면 −)
#       box_h    = 박스 높이 [px]   (9주차: 발 잘림 판단 · 거리 짐작용)
#       conf     = 신뢰도 0~1
#       ※ 앞 3칸은 5주차 /sjcu/error 와 같은 약속 (검출기만 바뀌고 약속은 그대로)
#       ※ 사람을 못 찾은 프레임에서는 발행하지 않습니다 (5주차와 같음)
#
#   실행 방법 1 — 녹화 영상 (Gazebo 없이, 터미널 2개):
#     터미널 1: ros2 run drone_ros2_advanced w08_video_pub        # 영상 → /camera/image_raw
#     터미널 2: ros2 run drone_ros2_advanced w08_person
#
#   실행 방법 2 — 드론 정면 카메라 (비행 없음, 터미널 3개):
#     터미널 1: cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=person_world make px4_sitl gz_x500_mono_cam
#     터미널 2: ros2 run drone_ros2_advanced w05_camera_bridge   # pxh> 가 뜬 뒤에!
#     터미널 3: ros2 run drone_ros2_advanced w08_person
#
#   오차 값 확인: ros2 topic echo /sjcu/person_error
#
#   조작 (그림 창을 클릭한 뒤):
#     슬라이더  기준값(%) — 40 이면 0.4. 0.25 아래는 모델이 이미 버려서 25 부터
#     s  원본 사진 + 화면 저장(png 2장)      q  종료
#
#   파라미터 예 (재빌드 없이 바꾸기 — 숫자는 소수점까지):
#     ros2 run drone_ros2_advanced w08_person --ros-args -p conf_threshold:=0.5
#     ros2 run drone_ros2_advanced w08_person --ros-args -p max_rate:=3.0      # VM 이 버거우면
#
#   모델: models/ (chuanqi305/MobileNet-SSD, MIT) — 13강 w08_dnn_demo 와 같은 파일
#   OpenCV 4.x 필요 (5.0 에는 readNetFromCaffe 가 없다):
#     pip install 'opencv-python==4.11.0.86' 'numpy<2'
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import os
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

import cv2
import numpy as np


# ================================================================
# 파라미터
# ================================================================
HERE       = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR  = os.path.join(HERE, 'models')
PROTOTXT   = 'deploy.prototxt'                   # 설계도 (13강)
CAFFEMODEL = 'mobilenet_iter_73000.caffemodel'   # 학습된 숫자 (13강)

TARGET_CLASS_ID  = 15     # 찾을 종류 번호: VOC 15 = 사람  (5주차 TARGET_MARKER_ID 의 짝)
CONF_THRESHOLD   = 0.4    # 기준값 — 14강에서 드론 화면을 보고 다시 정한다
TARGET_BOX_RATIO = 0.25   # 박스 높이가 화면 높이의 25% 면 "목표 거리"(약 4 m)  (5주차 TARGET_MARKER_SIZE 의 짝)
MIN_BOX_RATIO    = 0.08   # 화면 높이의 8% 보다 작은 박스는 버린다 (너무 멀거나 헛잡음)
MAX_RATE         = 5.0    # 1초에 최대 5번만 처리 (남는 힘은 Gazebo 에게)
CONF_MIN         = 0.25   # 모델 안에서 0.25 미만은 이미 버린다 → 슬라이더 하한

WINDOW   = 'Person Detector (q: quit)'
TRACKBAR = 'threshold %'

COLOR_GREEN  = (0, 255, 0)
COLOR_RED    = (0, 0, 255)
COLOR_BLUE   = (255, 100, 0)
COLOR_PERSON = (57, 78, 224)     # BGR — 사람 박스는 붉은 주황 (13강 데모와 같은 색)
FONT         = cv2.FONT_HERSHEY_SIMPLEX


class PersonDetector(Node):
    def __init__(self):
        super().__init__('person_detector')

        # ── 파라미터 (실행할 때 바꿀 수 있는 값) ─────────────────
        self.declare_parameter('image_topic',      '/camera/image_raw')
        self.declare_parameter('target_class_id',  TARGET_CLASS_ID)
        self.declare_parameter('conf_threshold',   CONF_THRESHOLD)
        self.declare_parameter('target_box_ratio', TARGET_BOX_RATIO)
        self.declare_parameter('max_rate',         MAX_RATE)
        self.declare_parameter('model_dir',        MODEL_DIR)
        self.declare_parameter('display_image',    True)

        self.image_topic    = self.get_parameter('image_topic').value
        self.target_id      = self.get_parameter('target_class_id').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.target_ratio   = self.get_parameter('target_box_ratio').value
        self.max_rate       = self.get_parameter('max_rate').value
        self.model_dir      = self.get_parameter('model_dir').value
        self.display        = self.get_parameter('display_image').value

        # q · 창 닫기 → 이 표시만 세우고, 실제 종료는 main 이 한다
        #   (콜백 안에서 rclpy.shutdown() 을 부르면 Humble 에서는 노드가 멈춰 끝나지 않는다)
        self.quit_requested = False

        # ── [13강 ①] 모델 읽기 — 처음 한 번 ──────────────────────
        self.net = self._load_model(self.model_dir)
        # [복붙 영역] DNN 이 CPU 를 다 쓰지 않게 (Gazebo 와 나눠 쓰기)
        cv2.setNumThreads(2)

        # ── 구독 1개: 카메라 — 대기열 1장 (5주차는 10) ─────────────
        #   처리가 느릴 때 10장을 쌓아 두면 가장 오래된 사진부터 보게 된다 → 최신 1장만
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_callback, 1)

        # ── 발행 1개: 오차 ───────────────────────────────────────
        self.error_pub = self.create_publisher(Float32MultiArray, '/sjcu/person_error', 10)

        # ── 창 준비: 기준값 슬라이더 (13강 데모와 같음) ─────────────
        if self.display:
            cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(WINDOW, 640, 480)
            cv2.createTrackbar(TRACKBAR, WINDOW, int(round(self.conf_threshold * 100)), 95, lambda v: None)
            cv2.setTrackbarMin(TRACKBAR, WINDOW, int(CONF_MIN * 100))

        # ── 상태 표시용 기록 ─────────────────────────────────────
        self.frame_count     = 0
        self.start_time      = time.monotonic()
        self.last_infer_time = 0.0            # 마지막으로 처리한 시각 (1초에 max_rate 번 제한)
        self.last_frame_time = None           # 마지막으로 사진이 들어온 시각 (대기 화면용)
        self.pub_times       = deque()        # 최근 10초 동안 발행한 시각들 → 발행 Hz
        self.found_log       = deque()        # 최근 10초 동안 처리한 프레임의 (시각, 찾았나) → 검출률
        self.stats_start     = self.start_time   # 통계를 센 시작 시각 (기준값을 바꾸면 새로 시작)
        self.prev_th         = None

        # 사진이 안 올 때도 창이 멈추지 않게 0.1초마다 화면·키를 챙긴다
        if self.display:
            self.ui_timer = self.create_timer(0.1, self._ui_timer)

        self.get_logger().info(
            f'사람 감지 시작!\n'
            f'  구독: {self.image_topic} (대기열 1장, 1초에 최대 {self.max_rate:g}번)   발행: /sjcu/person_error\n'
            f'  찾는 종류: {self.target_id} (VOC 15 = 사람)   기준값: {self.conf_threshold:.2f}   '
            f'목표 크기: 화면 높이의 {self.target_ratio:.2f}')

    # ============================================================
    # [13강 ①] 모델 읽기 (OpenCV 버전 · 파일 확인 포함)
    # ============================================================
    def _load_model(self, model_dir):
        # ── [복붙 영역] 확인 두 가지 — 안 되면 한국어로 안내하고 끝낸다 ──
        if not hasattr(cv2.dnn, 'readNetFromCaffe'):
            print(f'[오류] 지금 OpenCV({cv2.__version__})에는 readNetFromCaffe 가 없습니다 (OpenCV 5 에서 삭제됨).')
            print("       4.x 로 되돌리세요:  pip install 'opencv-python==4.11.0.86' 'numpy<2'")
            sys.exit(1)
        proto = os.path.join(model_dir, PROTOTXT)
        model = os.path.join(model_dir, CAFFEMODEL)
        for p in (proto, model):
            if not os.path.isfile(p):
                print(f'[오류] 모델 파일이 없습니다: {p}')
                print('       코드 최신화(git) 뒤 colcon build → source install/setup.bash 를 다시 하세요.')
                sys.exit(1)
        self.get_logger().info(f'모델 읽는 중: {PROTOTXT} + {CAFFEMODEL}')
        return cv2.dnn.readNetFromCaffe(proto, model)

    # ============================================================
    # 이미지 콜백 — 프레임 한 장마다 실행
    # ============================================================
    def _image_callback(self, msg: Image):
        now = time.monotonic()
        self.last_frame_time = now

        # ── [복붙 영역] 1초에 max_rate 번만 처리 (그 사이에 온 사진은 그냥 버림) ──
        if now - self.last_infer_time < 1.0 / self.max_rate:
            return
        self.last_infer_time = now

        # ── [복붙 영역] ROS Image → OpenCV 프레임 (5주차 1강과 동일) ──
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        if msg.encoding == 'rgb8':
            frame = frame[:, :, ::-1]          # RGB → BGR
        frame = frame.copy()               # 쓰기 가능한 복사본

        self.frame_count += 1
        raw = frame.copy()                 # s 키로 저장할 원본 (박스 그리기 전)

        # ── 1. [13강 ②] 전처리 — 300×300 으로 줄이고 밝기를 -1~1 로 ──
        #   (5주차는 여기서 흑백 변환. mean 은 반드시 숫자 3개)
        blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300),
                                     (127.5, 127.5, 127.5))

        # ── 2. 화면 중심 (오차의 기준점) — 5주차와 같음 ──────────────
        w, h = msg.width, msg.height
        cx = w / 2.0
        cy = h / 2.0

        # ── 3. [13강 ③] 추론  ← 5주차 detectMarkers 자리 ──────────────
        self.net.setInput(blob)
        t0 = time.perf_counter()
        out = self.net.forward()                     # 늘 (1, 1, 100, 7) 표
        infer_ms = (time.perf_counter() - t0) * 1000.0

        # ── 4. [13강 ④] 해석 — 표를 한 줄씩 읽어 후보 모으기 (5주차 4번 자리) ──
        conf_th = self._current_threshold()
        if conf_th != self.prev_th:                  # 기준값이 바뀌면 통계를 새로 센다 (앞 값이 섞이지 않게)
            self.pub_times.clear()
            self.found_log.clear()
            self.stats_start, self.prev_th = now, conf_th
        candidates = []
        for _, cls, conf, x1, y1, x2, y2 in out[0, 0]:
            if int(cls) != self.target_id:
                continue                             # 사람(15)이 아니면 건너뜀
            if conf < conf_th:
                continue                             # 기준값보다 확신이 낮으면 버림
            # [복붙 영역] 0~1 비율 → 픽셀 (화면 밖으로 살짝 나간 값은 잘라 냄)
            x1, y1, x2, y2 = [min(max(float(v), 0.0), 1.0) for v in (x1, y1, x2, y2)]
            bx1, by1, bx2, by2 = x1 * w, y1 * h, x2 * w, y2 * h
            box_h = by2 - by1
            if box_h < MIN_BOX_RATIO * h:
                continue                             # 너무 작은 박스는 버림
            candidates.append((box_h, float(conf), bx1, by1, bx2, by2))

        # 후보는 모두 얇게 그린다 (고른 것은 아래에서 굵게)
        for _, _, bx1, by1, bx2, by2 in candidates:
            cv2.rectangle(frame, (int(bx1), int(by1)), (int(bx2), int(by2)), COLOR_PERSON, 1)

        found = len(candidates) > 0
        if found:
            # ── 4-2. 고르기: 박스 높이가 가장 큰 사람 = 가장 가까운 사람 ──
            #   (5주차는 '같은 ID 중 첫 번째'. 신뢰도 최고로 고르면 프레임마다 목표가 바뀐다)
            box_h, conf, bx1, by1, bx2, by2 = max(candidates)   # 첫 칸(높이) 기준으로 최대
            px = (bx1 + bx2) / 2.0                              # 박스 중심 x
            py = (by1 + by2) / 2.0                              # 박스 중심 y

            # ── 5. 오차 = 박스 중심 − 화면 중심, 크기 오차 = 높이 − 목표 높이 ──
            x_error    = px - cx
            y_error    = py - cy
            size_error = box_h - self.target_ratio * h          # 화면 높이에 대한 비율로 목표를 정함

            # ── 6. 발행 (찾은 프레임에서만) — 숫자 5개, 모두 float() 로 ──
            error_msg      = Float32MultiArray()
            error_msg.data = [float(x_error), float(y_error), float(size_error),
                              float(box_h), float(conf)]
            self.error_pub.publish(error_msg)
            self.pub_times.append(now)

            # ── 7. 화면에 그리기 ─────────────────────────────────
            cv2.rectangle(frame, (int(bx1), int(by1)), (int(bx2), int(by2)), COLOR_PERSON, 3)   # 고른 사람
            cv2.circle(frame, (int(px), int(py)), 8, COLOR_RED, -1)                           # 박스 중심
            cv2.line(frame, (int(px), int(py)), (int(cx), int(cy)), COLOR_BLUE, 2)            # 중심까지 선
            cv2.putText(frame,
                        f'person {conf:.2f}  x={x_error:+.0f} y={y_error:+.0f}  h={box_h:.0f}px ({box_h / h:.2f})',
                        (20, 40), FONT, 0.7, COLOR_GREEN, 2)

        # ── 화면 중심 십자선 + 목표 높이 막대 + 미검출 안내 ──────────────
        cv2.line(frame, (int(cx) - 30, int(cy)), (int(cx) + 30, int(cy)), COLOR_GREEN, 2)
        cv2.line(frame, (int(cx), int(cy) - 30), (int(cx), int(cy) + 30), COLOR_GREEN, 2)
        target_h = self.target_ratio * h            # 오른쪽 끝 파란 막대 = 목표 박스 높이
        cv2.line(frame, (w - 15, int(cy - target_h / 2)), (w - 15, int(cy + target_h / 2)), COLOR_BLUE, 3)
        #   (사람 박스가 이 막대보다 짧으면 목표보다 멀다 = size_err < 0)
        if not found:
            cv2.putText(frame, 'No Person', (20, 40), FONT, 0.8, COLOR_RED, 2)

        # ── 상태 표시줄 (왼쪽 아래): 기준값 · 처리 시간 · 발행 Hz · 검출률 ──
        pub_hz, found_pct = self._update_stats(now, found)
        self._draw_status(frame, f'threshold {conf_th:.2f} | {infer_ms:.0f} ms | '
                                 f'pub {pub_hz:.1f} Hz | found {found_pct:.0f}%')

        if self.display:
            cv2.imshow(WINDOW, frame)
            self._handle_key(cv2.waitKey(1) & 0xFF, raw, frame)

    # ============================================================
    # 도우미 — 기준값 · 통계 · 화면 · 키
    # ============================================================
    def _current_threshold(self):
        """슬라이더가 있으면 슬라이더 값, 없으면 파라미터 값"""
        if not self.display:
            return self.conf_threshold
        return max(CONF_MIN, cv2.getTrackbarPos(TRACKBAR, WINDOW) / 100.0)

    def _update_stats(self, now, found):
        """발행 Hz 와 검출률(%) — 기준값을 바꾼 뒤부터, 최대 최근 10초"""
        self.found_log.append((now, found))
        while self.pub_times and now - self.pub_times[0] > 10.0:
            self.pub_times.popleft()
        while self.found_log and now - self.found_log[0][0] > 10.0:
            self.found_log.popleft()
        span = min(10.0, max(now - self.stats_start, 1.0))
        pub_hz = len(self.pub_times) / span
        found_pct = 100.0 * sum(1 for _, f in self.found_log if f) / len(self.found_log)
        return pub_hz, found_pct

    @staticmethod
    def _draw_status(frame, text):
        h = frame.shape[0]
        (tw, th), base = cv2.getTextSize(text, FONT, 0.55, 1)
        cv2.rectangle(frame, (0, h - th - base - 12), (tw + 14, h), (0, 0, 0), -1)
        cv2.putText(frame, text, (7, h - base - 6), FONT, 0.55, (255, 255, 255), 1, cv2.LINE_AA)

    def _ui_timer(self):
        """사진이 1초 넘게 안 오면 대기 화면을 띄우고, 창 이벤트(키·슬라이더·닫기)를 처리"""
        now = time.monotonic()
        if self.last_frame_time is None or now - self.last_frame_time > 1.0:
            wait = np.zeros((480, 640, 3), np.uint8)
            cv2.putText(wait, 'Waiting for camera ...', (20, 200), FONT, 1.0, COLOR_GREEN, 2)
            cv2.putText(wait, self.image_topic, (20, 250), FONT, 0.8, (200, 200, 200), 1)
            cv2.putText(wait, 'w08_video_pub  or  w05_camera_bridge', (20, 300), FONT, 0.6, (200, 200, 200), 1)
            cv2.imshow(WINDOW, wait)
        self._handle_key(cv2.waitKey(1) & 0xFF, None, None)

    def _handle_key(self, key, raw, view):
        if key == ord('q') or self._window_closed():
            self.quit_requested = True         # main 의 반복이 멈춘다
        elif key == ord('s') and raw is not None:
            stamp = time.strftime('%H%M%S')
            for name, img in ((f'person_raw_{stamp}.png', raw), (f'person_view_{stamp}.png', view)):
                path = os.path.abspath(name)
                if cv2.imwrite(path, img):
                    self.get_logger().info(f'저장: {path}')

    @staticmethod
    def _window_closed():
        """창의 X 를 눌렀는지 (13강 데모와 같은 방법)"""
        try:
            v = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE)
            if v < 0:                                # 이 빌드는 VISIBLE 을 모름
                return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_AUTOSIZE) < 0
            return v < 1
        except cv2.error:
            return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PersonDetector()
        # rclpy.spin(node) 대신: q 를 누르면(quit_requested) 반복을 멈춘다
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
