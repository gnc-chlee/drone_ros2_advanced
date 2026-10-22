#!/usr/bin/env python3
# ==============================================================================
# File    : range_viewer.py  (10주차 17강 - 측정 뷰어)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-10-22
# Version : 1.0.0
#
# Description:
#   거리 센서 측정 뷰어 — 같은 LiDAR 측정을 두 경로로 받아 0.5초마다 한 줄로 비교한다
#   날지 않는다: PX4Base 를 상속하지 않아 비행 명령을 하나도 보내지 않음
#   (2주차 position_listener 처럼 콜백은 저장만, 출력은 타이머가)
#
#   두 경로 (17강):
#     경로 ① 브리지 길 : Gazebo LiDAR → w10_lidar_bridge → /lidar/down (LaserScan) = 원시값
#     경로 ② PX4 길    : Gazebo LiDAR → PX4 EKF2 지형 추정 → vehicle_local_position 의 dist_bottom = 추정값
#     ※ /fmu/out/distance_sensor 는 PX4 기본 목록에 없어 ROS2 로 나오지 않는다
#       → PX4 를 다시 빌드하지 않고 읽는 길이 위 두 개
#
#   핵심 공식:
#     대지 고도 = LiDAR × cos(기울기) − 장착 오프셋(0.18 m)  ← 다리 끝에서 바로 아래 면까지 = 착지까지 남은 높이
#     상대 고도 = −z                                       ← 이륙 지점 기준 (NED 라 위가 −). 2·6·9주차의 '고도'
#     차이      = 상대 고도 − 대지 고도                    ← 평지면 0, 상자 위면 상자 높이 (0.15 m 를 넘을 때만 표시)
#     dist_bottom 은 PX4 숫자 그대로 보여 준다 (장착 오프셋이 들어 있는 값)
#
#   유효성 검사 (믿기 전에 버리는 값 — 숫자는 보여 주되 '(유효하지 않음)' 을 붙인다):
#     LiDAR      : 범위 밖(inf) · 경계값(range_min·range_max 와 같은 값) · 45° 넘게 기욺
#     dist_bottom: dist_bottom_valid 가 False · dist_bottom_sensor_bitfield 에 거리 센서(1)가 없음
#     경고는 상태가 바뀔 때만 한 줄 (같은 경고를 0.5초마다 다시 찍지 않음)
#
#   구독 토픽:
#     /lidar/down                     : LaserScan — ranges[0], range_min, range_max (w10_lidar_bridge)
#     /fmu/out/vehicle_local_position : 상대 고도(z), 북·동 위치, dist_bottom 과 유효 플래그
#     /fmu/out/vehicle_attitude       : 자세 → cos(기울기) (기울기 보정용)
#
#   실행 방법 (비행 없이 땅에서, 터미널 4개):
#     터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=lidar_world make px4_sitl gz_x500_lidar_down
#     터미널 2: MicroXRCEAgent udp4 -p 8888                    ← 상대 고도·dist_bottom 용 (없으면 그 칸은 '-')
#     터미널 3: ros2 run drone_ros2_advanced w10_lidar_bridge  ← pxh> 가 뜬 뒤에! (없으면 LiDAR 칸은 '-')
#     터미널 4: ros2 run drone_ros2_advanced w10_range_viewer
#     (터미널 5: ros2 run drone_ros2_advanced keyboard_control ← 상자 위 통과, VM 여유가 있을 때만)
#     ※ 위치·자세 메시지만 쓰므로 px4_msgs 가 main 이어도 동작한다 (18강 비행 전에는 release/1.16 필수)
#
#   파라미터 예:
#     ros2 run drone_ros2_advanced w10_range_viewer --ros-args -p detail:=true
#         → 줄 끝에 기울기 · 1초 표본 수 · 최장 간격 · 1초 값 범위 (센서 타임아웃 0.5초의 근거)
#     ros2 run drone_ros2_advanced w10_range_viewer --ros-args -p record:=true
#         → (교수용) ~/w10_range_<시각>.csv 에 0.1초마다 한 줄 — 그래프 데이터
#     ros2 param set /range_viewer offset 0.17
#         → 장착 오프셋 = 땅에 앉아 있을 때의 LiDAR 값 (실측값으로 바로 바꿈)
#
#   모의 측정값으로 유효성 검사 보기 (브리지(3번)를 끄고, 새 터미널에서):
#     ros2 topic pub -r 10 /lidar/down sensor_msgs/msg/LaserScan \
#       "{range_min: 0.1, range_max: 100.0, ranges: [.inf]}"
#     → 'LiDAR 범위 밖 (inf)'   (ranges: [0.1] → 경계값,  ranges: [1.18] → 대지 고도 1.00 m)
#
#   끝내기: Ctrl+C (명령을 보내지 않으므로 비행 중에 꺼도 드론에는 영향 없음)
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

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from sensor_msgs.msg import LaserScan

from ..px4_base import PX4_QOS   # [FIXED] PX4 전용 QoS 만 가져다 쓴다 (PX4Base 상속은 안 함 → 명령을 보내지 않음)


# ================================================================
# [USER DEFINE] 뷰어 설정
# ================================================================
LIDAR_OFFSET = 0.18   # 장착 오프셋 [m]: 땅에 앉아 있을 때의 LiDAR 값 (offset 파라미터 기본값, VM 실측으로 교체)
PRINT_PERIOD = 0.5    # 한 줄 출력 주기 [s] — 1초면 상자 모서리 같은 좁은 구간을 놓칠 수 있다
DIFF_SHOW    = 0.15   # '차이' 칸: 상대 고도 − 대지 고도 가 이 값보다 클 때만 표시 [m] (상자 위)

# ── [복붙 영역] 판정 상수 — 복사해 쓰고 원리만 이해 ──────────────
SCAN_TIMEOUT  = 0.5     # LiDAR 메시지가 이 시간 동안 안 오면 'LiDAR 없음' [s] (18강 노드의 센서 타임아웃과 같은 값)
PX4_TIMEOUT   = 1.0     # PX4 위치·자세가 이 시간 동안 안 오면 '없음' [s]
START_GRACE   = 2.0     # 시작하고 이 시간 동안은 경고를 미룬다 [s] (토픽이 연결되는 데 1~2초 걸림)
COS_MAX_TILT  = 0.7071  # 45° 넘게 기울면 측정 무효 (PX4 EKF 와 같은 기준)
EDGE_EPS      = 1e-4    # 이만큼 가까우면 경계값과 '같은 값' 으로 본다 [m]
OFFSET_MAX    = 1.0     # offset 파라미터로 받는 최대값 [m] (18 처럼 cm 로 잘못 넣는 실수 막기)
DETAIL_WINDOW = 1.0     # detail:=true 일 때 표본을 세는 구간 [s]
RECORD_PERIOD = 0.1     # CSV 기록 주기 [s] (record:=true, 교수용)
DIST_BOTTOM_SENSOR_RANGE = 1   # dist_bottom_sensor_bitfield 의 '거리 센서' 칸 (px4_msgs 상수와 같은 값)
CSV_COLUMNS = ['t', 'north', 'east', 'rel_alt', 'lidar_raw', 'cos_tilt', 'agl_lidar',
               'dist_bottom', 'db_valid', 'db_bitfield', 'db_reset', 'scan_age']


def as_bool(value):
    """파라미터 값 → True/False (true·1·yes 처럼 써도 받아 준다)"""
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1', 'yes', 'on')
    return bool(value)


def m_text(value):
    """[m] 숫자 → '1.00 m'. 반올림한 뒤 + 0.0 — '-0.00' 대신 '0.00' 으로 찍히게"""
    return f'{round(value, 2) + 0.0:.2f} m'


def raw_text(value):
    """측정값 → 글자. inf·nan 은 단위 없이 그대로 (LaserScan 관례: +inf = 최대보다 멂, -inf = 최소보다 가까움)"""
    return m_text(value) if math.isfinite(value) else f'{value}'


def csv_num(value, digits=4):
    """CSV 칸 — 값이 없으면 빈칸, inf·nan 은 글자 그대로"""
    if value is None:
        return ''
    return f'{value + 0.0:.{digits}f}' if math.isfinite(value) else f'{value}'


# ── [복붙 영역] 유효성 검사 — 18강 지형 추종 노드와 같은 규칙 ─────
def lidar_check(r, range_min, range_max, cos_tilt):
    """LiDAR 표본 하나 검사 → (이유 코드, 경고 문구). 쓸 수 있는 값이면 (None, '')
       이유 코드는 '상태가 바뀔 때만 경고' 에 쓴다 (같은 이유가 이어지면 다시 찍지 않음)"""
    if math.isnan(r):
        return 'nan', 'LiDAR 측정 무효 (nan)'
    if math.isinf(r):                                # 범위 밖: 너무 멀거나(+inf) 너무 가까움(-inf)
        return ('inf', 'LiDAR 범위 밖 (inf)') if r > 0 else ('-inf', 'LiDAR 범위 밖 (-inf)')
    if not range_max > range_min:                    # 측정 범위 칸이 비었음 (모의 측정값에서 빠뜨린 경우)
        return 'no_range', (f'LiDAR 측정 범위 없음 (range_min {range_min:g}, range_max {range_max:g})'
                            ' — 모의 측정값이면 range_min·range_max 도 넣기')
    if abs(r - range_min) < EDGE_EPS:                # Gazebo 는 범위 밖 유한값을 최소·최대로 잘라 보낸다
        return 'min', f'LiDAR 경계값 ({range_min:.2f} m = 최소) — 버림'     # → 경계값도 버린다
    if abs(r - range_max) < EDGE_EPS:
        return 'max', f'LiDAR 경계값 ({range_max:.2f} m = 최대) — 버림'
    if r < range_min:
        return 'below', f'LiDAR 범위 밖 ({r:.2f} m < 최소 {range_min:.2f} m) — 버림'
    if r > range_max:
        return 'above', f'LiDAR 범위 밖 ({r:.2f} m > 최대 {range_max:.2f} m) — 버림'
    if cos_tilt < COS_MAX_TILT:                      # 많이 기울면 빔이 엉뚱한 곳을 비스듬히 잰다
        return 'tilt', '기울기 45° 넘음 — 측정 무효'
    return None, ''


class RangeViewer(Node):
    def __init__(self):
        super().__init__('range_viewer')

        # ── 파라미터 ─────────────────────────────────────────────
        #    offset : 장착 오프셋 [m]. 0.5초마다 다시 읽음 → ros2 param set 으로 바로 바꿀 수 있음
        #    detail : True 면 줄 끝에 기울기 · 1초 표본 수 · 최장 간격 · 1초 값 범위. 0.5초마다 다시 읽음
        #    record : True 면 CSV 기록 (교수용). 시작할 때만 읽음
        self.declare_parameter('offset', LIDAR_OFFSET, ParameterDescriptor(dynamic_typing=True))
        self.offset = LIDAR_OFFSET
        self._offset_param(log_change=False)      # -p offset:=0.17 로 시작했으면 그 값 (이상한 값이면 기본값)
        self.detail = self._bool_param('detail', False)
        self.record = self._bool_param('record', False)

        # ============================================================
        # [USER DEFINE] 구독 3개 — 경로 ① 원시값 / 경로 ② 추정값 / 자세 (기울기 보정용)
        # ============================================================
        self.create_subscription(
            LaserScan, '/lidar/down', self._scan_callback,
            qos_profile_sensor_data)   # 센서용 QoS(BEST_EFFORT) — 브리지가 어느 방식으로 보내든 받는다
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._position_callback, PX4_QOS)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._attitude_callback, PX4_QOS)

        # ── 상태 변수 (콜백이 채우고 타이머가 읽는다) ─────────────
        self.scans    = deque(maxlen=500)   # 받은 LiDAR 표본 (받은 시각, r, range_min, range_max) — 최근 몇 초치
        self.position = None                # 마지막 VehicleLocalPosition
        self.pos_at   = None                # 위치를 받은 시각 [s]
        self.cos_tilt = 1.0                 # 마지막 cos(기울기)
        self.att_at   = None                # 자세를 받은 시각 [s] (None = 아직 못 받음)
        self.db_reset = None                # 마지막으로 본 dist_bottom_reset_counter
        self.start    = time.monotonic()    # 노드 시작 시각 [s]
        # 경고 상태 (None = 정상) — 상태가 바뀔 때만 경고 한 줄
        self.warned   = {'scan': None, 'pos': None, 'att': None, 'lidar': None, 'db': None}
        self.csv_file = self.csv = None     # CSV 기록 (record:=true 일 때만)
        self.csv_path, self.csv_rows = '', 0

        self.create_timer(PRINT_PERIOD, self.on_print)   # 0.5초마다 한 줄
        if self.record:
            self._open_record()

        self.get_logger().info(
            '거리 센서 뷰어 시작 — 날지 않음 (명령을 보내지 않고 받아서 찍기만)\n'
            f'  장착 오프셋 {self.offset:g} m, {PRINT_PERIOD:g}초마다 한 줄, '
            f'detail={str(self.detail).lower()}, record={str(self.record).lower()}')

    # ── [복붙 영역] 파라미터 — record:=1 처럼 줘도 받아 준다 (9주차와 같음) ──
    def _bool_param(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        return as_bool(self.get_parameter(name).value)

    # ── [복붙 영역] offset 다시 읽기 — 9주차 _gain 과 같은 검사 (bool·nan·inf 거부) + 0~1 m ──
    def _offset_param(self, log_change=True):
        try:
            raw = self.get_parameter('offset').value
            if isinstance(raw, bool):           # true/false 는 숫자가 아님 (float(True) = 1.0 방지)
                raise TypeError
            value = float(raw)
            if not math.isfinite(value):        # 'nan'·'inf' 도 막기
                raise ValueError
        except (TypeError, ValueError):
            self.get_logger().warn(
                f'offset 값이 숫자가 아님 — 이전 값 {self.offset:g} m 그대로', throttle_duration_sec=2.0)
            return self.offset
        if not 0.0 <= value <= OFFSET_MAX:
            self.get_logger().warn(
                f'offset {value:g} m 는 범위 밖 (0~{OFFSET_MAX:g} m) — 이전 값 {self.offset:g} m 그대로',
                throttle_duration_sec=2.0)
            return self.offset
        if value != self.offset:
            if log_change:
                self.get_logger().info(f'장착 오프셋 변경: {self.offset:g} → {value:g}')
            self.offset = value
        return value

    # ============================================================
    # 콜백 — 받은 값을 저장만 한다 (계산·출력은 타이머가)
    # ============================================================
    def _scan_callback(self, msg: LaserScan):
        if not msg.ranges:                      # 빈 메시지는 무시
            return
        # 하방 라이다는 빔 1개 → ranges[0] 하나만 쓴다
        self.scans.append((time.monotonic(), float(msg.ranges[0]),
                           float(msg.range_min), float(msg.range_max)))

    def _position_callback(self, msg: VehicleLocalPosition):
        self.position, self.pos_at = msg, time.monotonic()

    def _attitude_callback(self, msg: VehicleAttitude):
        # [복붙 영역] cos(기울기) = 회전행렬의 R(2,2) = 1 − 2(qx² + qy²)   (q = [w, x, y, z])
        q = msg.q
        c = float(1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2))   # float(): 파이썬 숫자로 (terrain_follow_base 와 같은 규칙)
        if math.isfinite(c):                    # NaN 자세는 버림
            self.cos_tilt, self.att_at = max(-1.0, min(1.0, c)), time.monotonic()

    # ============================================================
    # 도우미 — 지금 쓸 값 꺼내기 (없는 값은 None)
    # ============================================================
    def _cos_now(self, now):
        """cos(기울기)와 자세를 받고 있는지. 자세가 없으면 1 (= 기울기 0 으로 봄)"""
        if self.att_at is not None and now - self.att_at < PX4_TIMEOUT:
            return self.cos_tilt, True
        return 1.0, False

    def _position_now(self, now):
        """최근 PX4_TIMEOUT 안에 받은 위치 (없으면 None)"""
        if self.pos_at is not None and now - self.pos_at < PX4_TIMEOUT:
            return self.position
        return None

    def _lidar_now(self, now, cos_tilt):
        """마지막 LiDAR 표본 → (r, 받은 지 몇 초, 이유 코드, 경고 문구, 대지 고도). 표본이 없으면 모두 None"""
        if not self.scans:
            return None, None, None, '', None
        t, r, range_min, range_max = self.scans[-1]
        why, why_text = lidar_check(r, range_min, range_max, cos_tilt)
        agl = None
        if why is None:
            # ===== [USER DEFINE] 대지 고도 한 줄 — '다리 끝 기준' =====
            agl = r * cos_tilt - self.offset      # 경사 거리 × cos(기울기) − 장착 오프셋
            # ==========================================================
        return r, now - t, why, why_text, agl

    @staticmethod
    def _db_check(pos, lidar_live):
        """dist_bottom 검사 → (이유 코드, 경고 문구). 쓸 수 있으면 (None, ''). 값보다 유효 플래그를 먼저 본다"""
        bits = int(pos.dist_bottom_sensor_bitfield)
        if not bits & DIST_BOTTOM_SENSOR_RANGE:
            # LiDAR 가 안 들어오면 거리 센서 없는 기체일 가능성이 크다 (gz_x500 등 — 숫자는 나와도 측정이 아님)
            hint = ('LiDAR 는 들어오는 중 → PX4 가 거리 센서를 잠시 쓰지 않음 (부팅 직후·상자 모서리 등)'
                    if lidar_live else 'LiDAR 기체(gz_x500_lidar_down)인가요?')
            return 'not_range', f'dist_bottom 무효 (거리 센서 아님, bitfield {bits}) — {hint}'
        if not pos.dist_bottom_valid:
            return 'invalid', 'dist_bottom 무효 (dist_bottom_valid False) — 부팅 직후면 몇 초 기다리기'
        if not math.isfinite(pos.dist_bottom):
            return 'not_finite', f'dist_bottom 무효 (값 {pos.dist_bottom})'
        return None, ''

    def _warn_on_change(self, key, code, text=''):
        """경고는 상태가 바뀔 때만 한 줄 — code 가 None 이면 정상으로 돌아온 것 (찍지 않음)"""
        if code == self.warned[key]:
            return
        self.warned[key] = code
        if code is not None:
            self.get_logger().warn(text)

    def _check_states(self, now, scan_live, why, why_text, pos, att_live):
        """데이터가 없거나 유효하지 않을 때 경고 (상태가 바뀔 때만)"""
        if pos is not None:
            count = int(pos.dist_bottom_reset_counter)   # PX4 가 지형 추정을 다시 맞출 때마다 1씩 늘어남
            if self.db_reset is not None and count != self.db_reset:
                self.get_logger().warn(f'dist_bottom 재설정 (reset_counter {self.db_reset} → {count})')
            self.db_reset = count
        if now - self.start < START_GRACE:         # 시작하고 2초 동안은 경고를 미룸 (토픽이 연결되는 중)
            return

        self._warn_on_change('scan', None if scan_live else 'missing',
                             'LiDAR 없음 — w10_lidar_bridge(3번) 실행 확인')
        self._warn_on_change('lidar', why if scan_live else None, why_text)

        self._warn_on_change('pos', None if pos is not None else 'missing',
                             'PX4 위치 없음 — Agent(2번) 실행 확인')
        code, text = self._db_check(pos, scan_live) if pos is not None else (None, '')
        self._warn_on_change('db', code, text)

        self._warn_on_change('att', None if att_live else 'missing', '자세 없음 → 기울기 0 으로 봄')

    # ============================================================
    # 0.5초마다 한 줄 — 경로 ① LiDAR → 대지 고도 | 상대 고도 | (차이) | 경로 ② dist_bottom
    # ============================================================
    def on_print(self):
        now = time.monotonic()
        self._offset_param()                                     # ros2 param set 을 바로 반영
        self.detail = as_bool(self.get_parameter('detail').value)
        if as_bool(self.get_parameter('record').value) != self.record:
            self.get_logger().warn(
                'record 는 시작할 때만 읽습니다 — 기록하려면 노드를 다시 실행 (--ros-args -p record:=true)',
                throttle_duration_sec=5.0)

        cos_tilt, att_live = self._cos_now(now)
        pos = self._position_now(now)
        r, age, why, why_text, agl = self._lidar_now(now, cos_tilt)
        scan_live = age is not None and age < SCAN_TIMEOUT
        self._check_states(now, scan_live, why, why_text, pos, att_live)

        # ① 경로 ① 원시값 → 대지 고도 (유효하지 않으면 숫자는 보여 주되 대지 고도는 계산하지 않음)
        if not scan_live:
            lidar_txt = 'LiDAR - → 대지 고도 -'
            agl = None
        elif why is None:
            lidar_txt = f'LiDAR {raw_text(r)} (유효) → 대지 고도 {m_text(agl)}'
        else:
            lidar_txt = f'LiDAR {raw_text(r)} (유효하지 않음) → 대지 고도 -'
        parts = [f'[측정] {lidar_txt}']

        # ② 상대 고도 = −z, 상자 위라면 차이 = 상대 고도 − 대지 고도
        rel = -pos.z if pos is not None and math.isfinite(pos.z) else None
        parts.append(f'상대 고도 {m_text(rel)}' if rel is not None else '상대 고도 -')
        if rel is not None and agl is not None and rel - agl > DIFF_SHOW:
            parts.append(f'차이 {m_text(rel - agl)}')

        # ③ 경로 ② 추정값 dist_bottom (PX4 숫자 그대로)
        if pos is None:
            parts.append('dist_bottom -')
        else:
            state = '유효, 거리 센서' if self._db_check(pos, scan_live)[0] is None else '유효하지 않음'
            parts.append(f'dist_bottom {raw_text(pos.dist_bottom)} ({state})')

        if self.detail:
            parts += self._detail_parts(now, cos_tilt, att_live)
        self.get_logger().info(' | '.join(parts))

    # ── [복붙 영역] detail:=true — 기울기, 최근 1초의 표본 수 · 최장 간격 · 값 범위 ──
    #    1초 표본 수 = 갱신 주기 [Hz]. 50 Hz 면 약 50개 → 센서 타임아웃 0.5초 = 약 25개 (9주차 4.3 Hz → 1초와 같은 원리)
    #    최장 간격 = 메시지와 메시지 사이 가장 긴 시간 (ros2 topic hz 의 max 와 같은 뜻)
    def _detail_parts(self, now, cos_tilt, att_live):
        tilt = f'기울기 {math.degrees(math.acos(cos_tilt)):.0f}°' if att_live else '기울기 -'
        times, values, before = [], [], None
        for t, r, range_min, range_max in self.scans:          # 오래된 것부터 차례로
            if now - t > DETAIL_WINDOW:
                before = t                                       # 1초 구간 바로 앞 표본 (첫 간격 계산용)
                continue
            times.append(t)
            if lidar_check(r, range_min, range_max, 1.0)[0] is None:   # 범위 안의 값만 (기울기는 따로 표시)
                values.append(r)
        stats = f'1초 {len(times)}개'
        seq = ([before] if before is not None else []) + times
        gaps = [b - a for a, b in zip(seq, seq[1:])]
        if gaps:
            stats += f', 최장 간격 {max(gaps):.2f} s'
        if values:
            stats += f', {min(values):.2f}~{max(values):.2f} m'
        return [tilt, stats]

    # ============================================================
    # [복붙 영역] CSV 기록 (record:=true, 교수용 — 그래프 데이터). 0.1초마다 한 줄
    #   t 노드 시작 뒤 시간 [s] · north/east/rel_alt 위치·상대 고도 [m] · lidar_raw 원시값 [m]
    #   cos_tilt · agl_lidar 대지 고도 [m] (유효하지 않으면 빈칸) · dist_bottom [m] · db_valid 1/0
    #   db_bitfield · db_reset · scan_age 마지막 LiDAR 를 받은 지 몇 초 [s] (0.5 넘으면 오래된 값)
    # ============================================================
    def _open_record(self):
        path = os.path.expanduser(time.strftime('~/w10_range_%Y%m%d_%H%M%S.csv'))
        try:
            self.csv_file = open(path, 'w', newline='', encoding='utf-8')
        except OSError as e:
            self.get_logger().error(f'CSV 파일을 만들지 못함 ({e}) — 기록 없이 계속')
            return
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(CSV_COLUMNS)
        self.csv_path = path
        self.create_timer(RECORD_PERIOD, self.on_record)
        self.get_logger().info(f'CSV 기록 시작: {path} ({RECORD_PERIOD:g}초마다 한 줄, Ctrl+C 로 끝)')

    def on_record(self):
        now = time.monotonic()
        cos_tilt, _ = self._cos_now(now)
        pos = self._position_now(now)
        r, age, _, _, agl = self._lidar_now(now, cos_tilt)
        row = [csv_num(now - self.start, 3)]
        if pos is not None:
            row += [csv_num(pos.x), csv_num(pos.y), csv_num(-pos.z)]
        else:
            row += ['', '', '']
        row += [csv_num(r), csv_num(cos_tilt), csv_num(agl)]
        if pos is not None:
            row += [csv_num(pos.dist_bottom), int(bool(pos.dist_bottom_valid)),
                    int(pos.dist_bottom_sensor_bitfield), int(pos.dist_bottom_reset_counter)]
        else:
            row += ['', '', '', '']
        row.append(csv_num(age, 3))
        self.csv.writerow(row)
        self.csv_file.flush()            # Ctrl+C 로 끝내도 그때까지는 남게
        self.csv_rows += 1

    def destroy_node(self):
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            print(f'CSV 기록 끝: {self.csv_path} ({self.csv_rows}줄)')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RangeViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print('사용자 종료 (Ctrl+C)')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
