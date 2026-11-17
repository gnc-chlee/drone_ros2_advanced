#!/usr/bin/env python3
# ==============================================================================
# File    : state_viewer.py  (13주차 24강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-17
# Version : 1.0.0
#
# Description:
#   통합 미션 상태 뷰어 — 상태도에서 현재 상태 = 빨강, 방금 쓰인 표의 줄 번호 표시
#   12주차 상태 뷰어(week12/state_viewer.py)의 사본 — 상태 9개 · 화살표 14개 (ALIGN · DESCEND 포함)
#   PX4 에 명령을 보내지 않는다 → 언제 켜고 꺼도 미션에 영향이 없다 (드라이 런에서도 동작)
#   그림은 23강 시뮬레이터(w13_fsm_sim)와 같다 (state_view.py — 화살표는 전이 조건표 14줄만)
#   12주차 뷰어와 다른 곳:
#     - 13주차 표의 상태 9개를 모두 받는다 (12주차 뷰어는 ALIGN · DESCEND 를 모르는 상태로 버린다 → 13주차는 이 뷰어)
#     - 스캔 타임아웃은 미션 노드(w13_mission)의 상수 그대로 (끊김 판단 기준이 미션 노드와 같다)
#     - 창 · CSV 이름 w13_ · CSV 전이 칸: 한 주기에 전이가 둘이면 둘 다 ('10 sensor_ok; 3 front_blocked')
#     - 이륙 때 상태 메시지가 전이 메시지보다 먼저 와도 IDLE 띠를 지우지 않음 (두 토픽의 도착 순서는 정해져 있지 않음)
#     - DONE 뒤에 다른 상태가 오면 새 미션으로 봄 (노드를 다시 켰는데 IDLE 메시지를 받기 전에 이륙한 경우)
#
#   창 (글자는 영어·숫자만 — OpenCV 는 한글을 못 쓴다):
#     위     : 상태도. 화살표 옆 숫자 = 표의 줄 번호, 방금 지난 화살표 = 빨강
#     가운데 : 최근 전이 3줄   39.6 s  #11  aligned  ALIGN -> DESCEND
#              표 밖 안전장치는 화살표가 없어 이 줄에만:   45.2 s  safety  px4_takeover  DESCEND -> DONE
#     아래   : 최근 60초 상태 띠 + 앞 거리(±30°) 곡선, 1.5 m 정지 판단 · 2.0 m 해제 선 (히스테리시스)
#              (정렬 · 하강은 카메라만 쓰지만 스캔은 계속 오므로 곡선도 계속 그린다)
#
#   구독 토픽:
#     /sjcu/mission_state      : String 상태 이름 (w13_mission — 나중에 켜도 현재 상태를 받음)
#     /sjcu/mission_transition : String 전이 한 줄 '38.5 | CRUISE → ALIGN | 5 arrived | …' (w13_mission)
#                                안전장치면 '45.2 | DESCEND → DONE | safety px4_takeover | …'
#     /lidar/scan              : LaserScan (w13_bridge) — 앞 거리는 미션 노드와 같은 함수(sector_min)로 계산
#
#   실행 방법 (선택 — 24강 실습 단계에는 없음, 안 켜도 미션은 같다.
#              터미널을 하나 더 열어 미션(터미널 5)보다 먼저 켜면 처음부터 보임):
#     ros2 run drone_ros2_advanced w13_state_viewer
#     ros2 run drone_ros2_advanced w13_state_viewer --ros-args -p record:=true
#         → (교수용) ~/w13_state_<시각>.csv 에 0.1초마다 한 줄 — 전이 시각 · 앞 거리 확인용
#   끝내기: 창에서 q · 창 닫기 · Ctrl+C
#   미션 노드를 다시 켜면(IDLE 이 다시 오면 · DONE 뒤에 다른 상태가 오면) 그림을 처음부터 그린다
#   (PX4 재시작 없이 노드만 다시 켜도 같음)
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import csv
import math
import os
import time
from collections import deque

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ..week11.avoid_base import as_bool, sector_min
# 미션 노드와 같은 기준: 정지 판단 · 해제 거리 · 앞 섹터 · 상태 토픽 QoS · 스캔 타임아웃 (13주차 자체 상수)
from .mission_base import CLEAR_DIST, FRONT_DEG, SCAN_TIMEOUT, STATE_QOS, STOP_DIST
from .mission_table import DONE, IDLE, STATES
from . import state_view as view

WINDOW = 'w13_state_viewer'
SPAN = 60.0           # 상태 띠 · 곡선에 보이는 시간 [s]
DRAW_PERIOD = 0.1     # 창을 다시 그리는 주기 = CSV 한 줄 주기 [s]
CSV_COLUMNS = ['t', 'state', 'front', 'transition']
STATE_NAMES = set(STATES)   # 받는 상태 이름 = 13주차 표의 상태 9개 (이 밖의 이름만 무시)


def parse_transition(text):
    """'38.5 | CRUISE → ALIGN | 5 arrived | 동 …' → (38.5, 'CRUISE', 'ALIGN', 5, 'arrived')
       '45.2 | DESCEND → DONE | safety px4_takeover | …' → (45.2, 'DESCEND', 'DONE', 'safety', 'px4_takeover')
       줄 칸이 숫자면 int, safety 면 글자 그대로 (표 밖 안전장치 — 화살표 없음). 모양이 다르면 None"""
    try:
        t, move, rule = [p.strip() for p in text.split('|')[:3]]
        a, b = [s.strip() for s in move.split('→')]
        no, event = rule.split()
        if a not in STATE_NAMES or b not in STATE_NAMES:
            return None
        if no.isascii() and no.isdigit():
            no = int(no)
        elif no != 'safety':
            return None
        return float(t), a, b, no, event
    except (ValueError, AttributeError):
        return None


def scan_ok(scan):
    """칸 정보가 있는 스캔인지 (미션 노드와 같은 검사 — 없으면 칸 번호 ↔ 각도를 계산할 수 없다)"""
    n = len(scan.ranges)
    ok = (n > 0 and math.isfinite(scan.range_min) and math.isfinite(scan.range_max)
          and scan.range_max > scan.range_min)
    if n > 1:
        ok = ok and math.isfinite(scan.angle_min) and scan.angle_increment > 0.0
    return ok


def csv_num(value, digits=3):
    """CSV 칸 — 값이 없으면 빈칸, inf 는 글자 그대로 (11주차 뷰어와 같음)"""
    if value is None:
        return ''
    return f'{value + 0.0:.{digits}f}' if math.isfinite(value) else f'{value}'


class StateViewer(Node):
    def __init__(self):
        super().__init__('state_viewer')

        # record : True 면 CSV 기록 (교수용). 시작할 때만 읽음
        self.declare_parameter('record', False, ParameterDescriptor(dynamic_typing=True))
        self.record = as_bool(self.get_parameter('record').value)

        self.create_subscription(String, '/sjcu/mission_state', self._state_callback, STATE_QOS)
        self.create_subscription(String, '/sjcu/mission_transition', self._transition_callback, 10)
        self.create_subscription(LaserScan, '/lidar/scan', self._scan_callback, qos_profile_sensor_data)

        self.start = time.monotonic()
        # q · 창 닫기 → 이 표시만 세우고, 실제 종료는 main 이 한다
        #   (콜백 안에서 rclpy.shutdown() 을 부르면 Humble 에서는 노드가 멈춰 끝나지 않는다 — 8주차와 같음)
        self.quit_requested = False
        self.scan = self.scan_at = None
        self.series = deque()          # (시각, 앞 거리 또는 None) — 최근 SPAN 초
        self.bad_logged = False        # '모양이 다른 전이 메시지' 경고를 한 번만
        self.pending = []              # CSV: 이번 줄에 적을 전이 (한 주기에 둘이 올 수 있음 — 10번 줄 다음 틱 3번 줄 등)
        self._new_run(None, self.start)

        self.csv_file = self.csv = None
        self.csv_path, self.csv_rows = '', 0
        if self.record:
            self._open_record()

        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        self.create_timer(DRAW_PERIOD, self.on_draw)
        self.get_logger().info(
            '상태 뷰어 시작 — PX4 에 명령을 보내지 않음 (언제 켜고 꺼도 됨) | 끝내기: 창에서 q\n'
            '  w13_mission 이 켜져 있으면 상태도에 불이 들어옵니다')

    # ============================================================
    # 상태 기록 — 화면에 그릴 값
    # ============================================================
    def _new_run(self, state, now):
        """처음부터 (뷰어 시작 · 미션 노드 다시 시작)"""
        self.state, self.since = state, now
        self.visited = {state} if state else set()
        self.done_rows = set()          # 지나온 줄 {(줄 번호, 출발 상태)}
        self.last = None                # 방금 지난 전이 (출발, 도착, 줄 번호 또는 'safety')
        self.recent = deque(maxlen=3)   # 최근 전이 [(미션 시각, 줄 번호 또는 'safety', 사건, 출발, 도착)]
        self.bands = [(now, state)] if state else []   # 상태 띠: 상태가 바뀐 시각

    def _enter(self, state, now):
        self.state, self.since = state, now
        self.visited.add(state)
        self.bands.append((now, state))
        while len(self.bands) > 1 and self.bands[1][0] < now - SPAN:   # 창 밖으로 나간 띠는 버림
            self.bands.pop(0)

    # ============================================================
    # 콜백 — 두 토픽은 도착 순서가 정해져 있지 않다 (어느 쪽이 먼저 와도 같은 그림)
    # ============================================================
    def _state_callback(self, msg: String):
        name = msg.data.strip()
        if name not in STATE_NAMES:
            return
        now = time.monotonic()
        # 새 미션 = IDLE 이 다시 옴, 또는 DONE(끝 상태 — 나가는 줄이 없음) 뒤에 다른 상태가 옴 (IDLE 을 못 받은 채 다시 켬)
        restart = (name == IDLE and self.state != IDLE) or (self.state == DONE and name != DONE)
        if self.state is None or restart:
            if self.state is not None:
                self.get_logger().info('미션 노드가 다시 시작됨 — 그림을 처음부터')
            self._new_run(name, now)
        elif name != self.state:
            self._enter(name, now)

    def _transition_callback(self, msg: String):
        parsed = parse_transition(msg.data)
        if parsed is None:
            if not self.bad_logged:
                self.bad_logged = True
                self.get_logger().warn(f'모양이 다른 전이 메시지 — 무시: "{msg.data}"')
            return
        t, a, b, no, event = parsed
        now = time.monotonic()
        # IDLE 에서 나가는 전이 = 새 미션 → 처음부터. 단, 같은 미션의 상태 메시지(b)가 먼저 와서 이미 b 를 보이는 중이면
        # (아직 전이를 하나도 못 받음) 같은 미션 — 지우지 않는다 (IDLE 띠가 사라지지 않게)
        same_run = self.state == b and self.last is None
        if self.state is None or (a == IDLE and self.state != IDLE and not same_run):
            self._new_run(a, now)
        if b != self.state:
            self._enter(b, now)
        self.visited.add(a)
        self.done_rows.add((no, a))     # 안전장치('safety')는 맞는 화살표가 없어 그림에는 안 보임
        self.last = (a, b, no)
        self.recent.appendleft((t, no, event, a, b))
        self.pending.append(f'{no} {event}')   # CSV: '3 front_blocked' · 'safety px4_takeover'

    def _scan_callback(self, msg: LaserScan):
        self.scan, self.scan_at = msg, time.monotonic()

    def _front_now(self, now):
        """앞 거리 (±30°) 와 화면 글자 — 미션 노드와 같은 함수 · 같은 스캔 타임아웃"""
        scan = self.scan
        if scan is None or now - self.scan_at > SCAN_TIMEOUT:
            return None, 'no scan'
        if not scan_ok(scan):
            return None, 'bad scan (no angle/range info)'
        if any(r == -math.inf for r in scan.ranges):
            return None, 'too close (-inf)'
        front = sector_min(scan, -FRONT_DEG, FRONT_DEG)
        if front is None:
            return None, 'no reading'
        return front, ('free' if math.isinf(front) else f'{front:.2f} m')

    # ============================================================
    # 0.1초마다: 그리기 · 키 · CSV
    # ============================================================
    def on_draw(self):
        now = time.monotonic()
        front, text = self._front_now(now)
        self.series.append((now, front))
        while self.series and self.series[0][0] < now - SPAN:
            self.series.popleft()
        img = view.viewer_frame(self.state, now - self.since, self.visited, self.done_rows, self.last,
                                list(self.recent), self.bands, list(self.series), now, text,
                                STOP_DIST, CLEAR_DIST, f'front (+-{FRONT_DEG:.0f} deg)', span=SPAN)
        cv2.imshow(WINDOW, img)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord('q'), 27) or self._window_closed():
            self.quit_requested = True              # main 의 반복이 멈춘다
        if self.csv is not None:
            self.csv.writerow([csv_num(now - self.start), self.state or '', csv_num(front), '; '.join(self.pending)])
            self.csv_file.flush()                   # Ctrl+C 로 끝내도 그때까지는 남게
            self.csv_rows += 1
        self.pending = []

    @staticmethod
    def _window_closed():
        """창의 X 를 눌렀는지 (8주차와 같은 방법)"""
        try:
            v = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE)
            if v < 0:                                # 이 빌드는 VISIBLE 을 모름
                return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_AUTOSIZE) < 0
            return v < 1
        except cv2.error:
            return True

    # ============================================================
    # [복붙 영역] CSV 기록 (record:=true, 교수용 — 전이 시각 · 앞 거리 확인용). 0.1초마다 한 줄
    #   t 뷰어 시작 뒤 시간 [s] · state 상태 · front 앞 거리 [m] (자유 공간 = inf, 측정 없음 = 빈칸)
    #   transition 그 사이에 지난 전이 '11 aligned' · 안전장치면 'safety px4_takeover' (없으면 빈칸,
    #              둘 이상이면 '; ' 로 이음 — 예: '10 sensor_ok; 3 front_blocked')
    # ============================================================
    def _open_record(self):
        path = os.path.expanduser(time.strftime('~/w13_state_%Y%m%d_%H%M%S.csv'))
        try:
            self.csv_file = open(path, 'w', newline='', encoding='utf-8')
        except OSError as e:
            self.get_logger().error(f'CSV 파일을 만들지 못함 ({e}) — 기록 없이 계속')
            return
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(CSV_COLUMNS)
        self.csv_path = path
        self.get_logger().info(f'CSV 기록 시작: {path} ({DRAW_PERIOD:g}초마다 한 줄, q 또는 Ctrl+C 로 끝)')

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = self.csv = None
            print(f'CSV 기록 끝: {self.csv_path} ({self.csv_rows}줄)')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StateViewer()
        # rclpy.spin(node) 대신: q 를 누르면(quit_requested) 반복을 멈춘다
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
