#!/usr/bin/env python3
# ==============================================================================
# File    : scan_viewer.py  (11주차 19강 - 스캔 뷰어)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-02
# Version : 1.0.0
#
# Description:
#   2D LiDAR 스캔 뷰어 — 1080칸을 '앞 · 왼쪽 · 오른쪽' 섹터 최근접 거리 3개로 줄여 0.5초마다 한 줄로 보여 준다
#   날지 않는다: PX4Base 를 상속하지 않아 비행 명령을 하나도 보내지 않음 (10주차 range_viewer 와 같은 구조)
#
#   핵심 (19강):
#     i번째 칸 각도 = angle_min + i × angle_increment   ← 각도 + = 왼쪽 (반시계, REP 103), 0번 칸 = 오른쪽 뒤 −135°
#     정면 1칸 번호 = (0 − angle_min) ÷ angle_increment ≈ 539.5 → 반올림 (그대로 쓰면 539 또는 540)
#     섹터 최근접 거리 = 섹터 안에서 '범위 안' 값 중 가장 가까운 것 하나
#       앞 ±30° · 왼쪽 +30~+90° · 오른쪽 −90~−30°  (근접 검사는 앞 ±45°)
#     같은 inf, 다른 뜻 (10주차 하방은 +inf = 범위 밖 → 무효):
#       +inf      = 자유 공간 (그 방향 30 m 안에 아무것도 없음)
#       -inf      = 너무 가까움          NaN·경계값 = 무효 (버림)
#       0.4 m 안  = 자기 기체 영역 (프로펠러가 스캔 평면 바로 아래) → 버림
#       섹터에 쓸 값이 하나도 없음 = 측정 없음 → 막힘으로 본다 (측정하지 않은 방향으로는 가지 않는다)
#
#   판단 칸 (20강 회피 노드와 같은 거리 기준):
#     전진       : 앞 3.0 m 이상 (또는 자유 공간)
#     감속       : 앞 3.0 m 안   (20강 노드는 여기서부터 거리에 비례해 속도를 줄임)
#     정지 판단  : 앞 1.5 m 안   (정지 판단 거리 = 안전 거리 1.0 + 정지 거리 0.5)
#     근접       : 앞 ±45° 에 1.0 m 안 (안전 거리)
#
#   정지 거리 재기 (19강 화면 읽기 ①, 위치가 있을 때만):
#     앞 거리가 정지 판단 거리를 지나는 순간 한 줄 → 기체가 멈추면 한 줄
#       [정지 판단] 앞 1.50 m 통과 | 전진 0.48 m/s
#       [멈춤] 앞 1.12 m | 판단 뒤 1.6 s · 0.38 m 이동      ← 사람이 알아채고 멈출 때까지 간 거리 (= 정지 거리)
#
#   구독 토픽:
#     /lidar/scan                     : LaserScan 1080칸 (w11_scan_bridge)
#     /fmu/out/vehicle_local_position : 북·동 위치, 상대 고도(−z), 속도·기수 (전진 속도 = 기수 방향 속도) — 없으면 '위치 없음'
#
#   실행 방법 (비행 없이 땅에서, 터미널 4개 — 위치·전진 속도 칸만 Agent 가 필요):
#     터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500_lidar_2d
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w11_scan_bridge         ← pxh> 가 뜬 뒤에!
#     터미널 4: ros2 run drone_ros2_advanced w11_scan_viewer
#     (터미널 5: ros2 run drone_ros2_advanced keyboard_control ← 기둥 앞까지 수동 조종, VM 여유가 있을 때만)
#     땅 값 (계산): [스캔] 1080칸 | 정면 1칸(540) 5.38 m | 앞 5.38 · 왼쪽 자유 공간 · 오른쪽 3.54 m | 버림 0칸 | 판단: 전진
#
#   파라미터 예:
#     ros2 param set /scan_viewer front_deg 1.0     → 앞 섹터를 ±1° 로 (19강 화면 읽기 ② — 정면 1칸과 비슷해짐)
#     ros2 param set /scan_viewer stop_dist 2.0     → 정지 판단 거리 (판단 칸 기준이 같이 바뀜)
#     ros2 run drone_ros2_advanced w11_scan_viewer --ros-args -p record:=true
#         → (교수용) ~/w11_scan_<시각>.csv 에 0.1초마다 한 줄 — 그래프 데이터
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

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import LaserScan

from ..px4_base import PX4_QOS   # [FIXED] PX4 전용 QoS 만 가져다 쓴다 (PX4Base 상속은 안 함 → 명령을 보내지 않음)


# ================================================================
# [USER DEFINE] 뷰어 설정
# ================================================================
FRONT_DEG    = 30.0   # 앞 섹터 반각 [°] (front_deg 파라미터 기본값 — 20강 노드와 같음)
STOP_DIST    = 1.5    # 정지 판단 거리 [m] (stop_dist 파라미터 기본값 — 20강 노드와 같음)
PRINT_PERIOD = 0.5    # 한 줄 출력 주기 [s]

# ── [복붙 영역] 판정 상수 — 20강 회피 노드와 같은 값 ─────────────
SELF_RANGE   = 0.4    # 이보다 가까운 값은 자기 기체(프로펠러·프레임)로 보고 버림 [m]
SAFE_DIST    = 1.0    # 안전 거리 [m] — 앞 ±45° 에 이보다 가까우면 '근접'
NEAR_DEG     = 45.0   # 근접 검사 반각 [°]
SIDE_DEG     = (30.0, 90.0)   # 옆 섹터 [°]: 왼쪽 +30~+90, 오른쪽은 부호를 바꿔 −90~−30
SLOW_GAP     = 1.5    # 감속 시작 = 정지 판단 거리 + 1.5 m (= 3.0 m, 20강 노드의 0.5 m/s ÷ 0.33)
CLEAR_GAP    = 0.5    # 정지 판단 표시를 다시 켜는 거리 = 정지 판단 거리 + 0.5 (= 20강 해제 거리 2.0 m)
SCAN_TIMEOUT = 0.5    # 스캔이 이 시간 동안 안 오면 '신호 없음' [s] (10주차 센서 타임아웃과 같은 값)
PX4_TIMEOUT  = 1.0    # 위치가 이 시간 동안 안 오면 '위치 없음' [s]
START_GRACE  = 2.0    # 시작하고 이 시간 동안은 경고를 미룬다 [s] (토픽이 연결되는 데 1~2초 걸림)
STOP_SPEED   = 0.05   # 전진 속도가 이보다 작으면 '멈춤' [m/s]
WATCH_PERIOD = 0.05   # 정지 판단·멈춤을 살피는 주기 [s] (0.5 m/s 로 2.5 cm — 0.5초 출력보다 촘촘하게)
RECORD_PERIOD = 0.1   # CSV 기록 주기 [s] (record:=true, 교수용)
CSV_COLUMNS = ['t', 'north', 'east', 'rel_alt', 'front', 'left', 'right', 'near', 'front_1', 'fwd_speed', 'judge']


def as_bool(value):
    """파라미터 값 → True/False (true·1·yes 처럼 써도 받아 준다)"""
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1', 'yes', 'on')
    return bool(value)


# ── [복붙 영역] 섹터 최근접 거리 — 20강 회피 노드(avoid_base.py)와 같은 함수 ─────
def sector_min(scan, lo_deg, hi_deg):
    """lo_deg ~ hi_deg 섹터의 최근접 거리 [m]. 자유 공간이면 inf, 쓸 값이 하나도 없으면 None (= 측정 없음 → 막힘)"""
    n = len(scan.ranges)
    if n == 1:                                   # 1빔 기체: 정면이 든 섹터만
        idx = [0] if lo_deg <= 0.0 <= hi_deg else []
    else:                                        # 경계: 아래 ceil · 위 floor (경계에 걸친 칸을 두 섹터가 나눠 갖지 않게)
        a0, inc = scan.angle_min, scan.angle_increment
        i0 = max(0, math.ceil((math.radians(lo_deg) - a0) / inc - 1e-6))
        i1 = min(n - 1, math.floor((math.radians(hi_deg) - a0) / inc + 1e-6))
        idx = range(i0, i1 + 1)
    seg = [scan.ranges[i] for i in idx]
    lo = max(scan.range_min, SELF_RANGE)         # 0.4 m 안 = 자기 기체 영역
    ok = [r for r in seg if lo <= r < scan.range_max]   # 범위 '안' → NaN·inf·경계값 제외
    if ok:
        return min(ok)
    return math.inf if math.inf in seg else None # None = 측정 없음 → 막힘


def dist_text(value):
    """섹터 값 → 글자: None = '측정 없음', inf = '자유 공간', 숫자 = '5.38'"""
    if value is None:
        return '측정 없음'
    if math.isinf(value):
        return '자유 공간'
    return f'{value:.2f}'


def sectors_text(front, left, right):
    """'앞 5.38 · 왼쪽 자유 공간 · 오른쪽 3.54 m' — 단위 m 는 마지막 숫자 뒤에 한 번"""
    items = [('앞', front), ('왼쪽', left), ('오른쪽', right)]
    last = max((i for i, (_, v) in enumerate(items) if v is not None and math.isfinite(v)), default=None)
    parts = [f'{name} {dist_text(v)}' + (' m' if i == last else '') for i, (name, v) in enumerate(items)]
    return ' · '.join(parts)


def csv_num(value, digits=3):
    """CSV 칸 — 값이 없으면 빈칸, inf·nan 은 글자 그대로"""
    if value is None:
        return ''
    return f'{value + 0.0:.{digits}f}' if math.isfinite(value) else f'{value}'


class ScanViewer(Node):
    def __init__(self):
        super().__init__('scan_viewer')

        # ── 파라미터 ─────────────────────────────────────────────
        #    front_deg : 앞 섹터 반각 [°] (1~45). 0.5초마다 다시 읽음
        #    stop_dist : 정지 판단 거리 [m] (1.2~3.0). 0.5초마다 다시 읽음
        #    record    : True 면 CSV 기록 (교수용). 시작할 때만 읽음
        self.front_deg = self._float_param('front_deg', FRONT_DEG, 1.0, 45.0)
        self.stop_dist = self._float_param('stop_dist', STOP_DIST, 1.2, 3.0)
        self.declare_parameter('record', False, ParameterDescriptor(dynamic_typing=True))
        self.record = as_bool(self.get_parameter('record').value)

        # ============================================================
        # [USER DEFINE] 구독 2개 — 스캔 / 위치 (위치는 '정지 거리 재기'와 표시용)
        # ============================================================
        self.create_subscription(
            LaserScan, '/lidar/scan', self._scan_callback,
            qos_profile_sensor_data)   # 센서용 QoS(BEST_EFFORT) — 브리지가 어느 방식으로 보내든 받는다
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._position_callback, PX4_QOS)

        # ── 상태 변수 (콜백이 채우고 타이머가 읽는다) ─────────────
        self.scan     = None                # 마지막 LaserScan
        self.scan_at  = None                # 스캔을 받은 시각 [s]
        self.position = None                # 마지막 VehicleLocalPosition
        self.pos_at   = None                # 위치를 받은 시각 [s]
        self.start    = time.monotonic()    # 노드 시작 시각 [s]
        self.armed    = True                # 정지 판단 표시를 낼 준비가 됐는지 (한 번 내면 앞이 다시 넓어질 때까지 끔)
        self.judged   = None                # 정지 판단 순간 (시각, 북, 동) — 멈춤 줄 계산용
        # 경고 상태 (None = 정상) — 상태가 바뀔 때만 경고 한 줄
        self.warned   = {'scan': None, 'pos': None, 'beam': None, 'self': None, 'neg': None, 'bad': None}
        self.csv_file = self.csv = None     # CSV 기록 (record:=true 일 때만)
        self.csv_path, self.csv_rows = '', 0

        self.create_timer(PRINT_PERIOD, self.on_print)   # 0.5초마다 한 줄
        self.create_timer(WATCH_PERIOD, self.on_watch)   # 정지 판단 · 멈춤 살피기
        if self.record:
            self._open_record()

        self.get_logger().info(
            '스캔 뷰어 시작 — 날지 않음 (명령을 보내지 않고 받아서 찍기만)\n'
            f'  앞 ±{self.front_deg:g}° · 왼쪽 +{SIDE_DEG[0]:g}~+{SIDE_DEG[1]:g}° · '
            f'오른쪽 -{SIDE_DEG[1]:g}~-{SIDE_DEG[0]:g}° | 정지 판단 {self.stop_dist:g} m · '
            f'감속 {self.stop_dist + SLOW_GAP:g} m · 근접 {SAFE_DIST:g} m | {PRINT_PERIOD:g}초마다 한 줄, '
            f'record={str(self.record).lower()}')

    # ── [복붙 영역] 파라미터 — 숫자 검사 (bool·nan·inf 거부) + 범위 (10주차 _offset_param 과 같은 방식) ──
    def _float_param(self, name, default, low, high):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        value = self._read_float(name, default, low, high, log_change=False)
        return value

    def _read_float(self, name, old, low, high, log_change=True):
        try:
            raw = self.get_parameter(name).value
            if isinstance(raw, bool):           # true/false 는 숫자가 아님 (float(True) = 1.0 방지)
                raise TypeError
            value = float(raw)
            if not math.isfinite(value):        # 'nan'·'inf' 도 막기
                raise ValueError
        except (TypeError, ValueError):
            self.get_logger().warn(f'{name} 값이 숫자가 아님 — {old:g} 그대로', throttle_duration_sec=2.0)
            return old
        if not low <= value <= high:
            self.get_logger().warn(f'{name} {value:g} 는 범위 밖 ({low:g}~{high:g}) — {old:g} 그대로',
                                   throttle_duration_sec=2.0)
            return old
        if log_change and value != old:
            self.get_logger().info(f'{name} 변경: {old:g} → {value:g}')
        return value

    # ============================================================
    # 콜백 — 받은 값을 저장만 한다 (계산·출력은 타이머가)
    # ============================================================
    def _scan_callback(self, msg: LaserScan):
        if not msg.ranges:                      # 빈 메시지는 무시
            return
        self.scan, self.scan_at = msg, time.monotonic()

    def _position_callback(self, msg: VehicleLocalPosition):
        self.position, self.pos_at = msg, time.monotonic()

    # ============================================================
    # 도우미 — 지금 쓸 값 꺼내기 (없는 값은 None)
    # ============================================================
    def _scan_now(self, now):
        """최근 SCAN_TIMEOUT 안에 받은, 칸 정보가 온전한 스캔 (없으면 None)"""
        s = self.scan
        if s is None or now - self.scan_at > SCAN_TIMEOUT:
            return None
        if not (math.isfinite(s.range_min) and math.isfinite(s.range_max) and s.range_max > s.range_min):
            return None
        if len(s.ranges) > 1 and not (math.isfinite(s.angle_min) and s.angle_increment > 0.0):
            return None
        return s

    def _position_now(self, now):
        """최근 PX4_TIMEOUT 안에 받은 위치 (없으면 None)"""
        if self.pos_at is not None and now - self.pos_at < PX4_TIMEOUT:
            return self.position
        return None

    @staticmethod
    def _fwd_speed(pos):
        """전진 속도 [m/s] = 수평 속도를 기수 방향으로 본 값 (NED: vx 북, vy 동, heading 북에서 시계 방향)"""
        if pos is None or not (math.isfinite(pos.vx) and math.isfinite(pos.vy) and math.isfinite(pos.heading)):
            return None
        return pos.vx * math.cos(pos.heading) + pos.vy * math.sin(pos.heading)

    def _sectors(self, scan):
        """섹터 4개 (앞 · 왼쪽 · 오른쪽 · 근접) — 값은 sector_min 그대로 (None = 측정 없음)"""
        f = self.front_deg
        return (sector_min(scan, -f, f),
                sector_min(scan, SIDE_DEG[0], SIDE_DEG[1]),
                sector_min(scan, -SIDE_DEG[1], -SIDE_DEG[0]),
                sector_min(scan, -NEAR_DEG, NEAR_DEG))

    def _judge(self, front, near):
        """판단 칸 — 20강 노드와 같은 거리 기준 (측정 없음은 막힘으로 본다)"""
        if near is not None and near < SAFE_DIST:
            return '근접'
        if front is None:
            return '막힘 (측정 없음)'
        if front < self.stop_dist:
            return '정지 판단'
        if front < self.stop_dist + SLOW_GAP:
            return '감속'
        return '전진'

    def _front_one(self, scan):
        """정면 1칸 (번호, 값) — 10주차 ranges[0] 처럼 한 칸만 보는 방식과 비교용"""
        if len(scan.ranges) == 1:
            return 0, scan.ranges[0]
        # ===== [USER DEFINE] 정면 칸 번호 한 줄 — 19강 9번 =====
        i_front = round((0.0 - scan.angle_min) / scan.angle_increment)
        # ========================================================
        i_front = max(0, min(len(scan.ranges) - 1, i_front))
        return i_front, scan.ranges[i_front]

    def _warn_on_change(self, key, code, text=''):
        """경고는 상태가 바뀔 때만 한 줄 — code 가 None 이면 정상으로 돌아온 것 (찍지 않음)"""
        if code == self.warned[key]:
            return
        self.warned[key] = code
        if code is not None:
            self.get_logger().warn(text)

    def _check_states(self, now, scan, pos):
        """데이터가 없거나 이상할 때 경고 (상태가 바뀔 때만)"""
        if now - self.start < START_GRACE:     # 시작하고 2초 동안은 경고를 미룸 (토픽이 연결되는 중)
            return
        raw = self.scan
        if raw is None or now - self.scan_at > SCAN_TIMEOUT:
            self._warn_on_change('scan', 'missing',
                                 f'[신호 없음] 스캔 {SCAN_TIMEOUT:g}초 없음 — w11_scan_bridge(3번) 실행 확인 (pxh> 뒤에)')
            return
        self._warn_on_change('scan', None)
        self._warn_on_change('bad', None if scan is not None else 'bad',
                             '스캔 칸 정보가 이상함 (range_min·range_max·angle_increment) — 모의 스캔이면 네 칸을 모두 넣기')
        if scan is None:
            return
        n = len(scan.ranges)
        self._warn_on_change('beam', 'one' if n == 1 else None,
                             '1빔 기체 — 좌우 측정 없음 (앞만 봄). 11주차 기체는 gz_x500_lidar_2d')
        near_self = sum(1 for r in scan.ranges if math.isfinite(r) and scan.range_min <= r < SELF_RANGE)
        self._warn_on_change('self', near_self if near_self else None,
                             f'자기 기체 영역 {near_self}칸 버림 ({SELF_RANGE:g} m 안 — 프로펠러·프레임이 잡힘)')
        neg = sum(1 for r in scan.ranges if r == -math.inf)
        self._warn_on_change('neg', neg if neg else None, f'-inf {neg}칸 (너무 가까움) — 20강 노드라면 수평 정지')
        self._warn_on_change('pos', None if pos is not None else 'missing',
                             '위치 없음 — Agent(2번) 실행 확인 (스캔은 그대로 보임, 정지 거리 재기만 안 됨)')

    # ============================================================
    # 0.5초마다 한 줄 — 칸 수 | 정면 1칸 | 섹터 3개 | 버림 | 판단
    # ============================================================
    def on_print(self):
        now = time.monotonic()
        self.front_deg = self._read_float('front_deg', self.front_deg, 1.0, 45.0)   # ros2 param set 을 바로 반영
        self.stop_dist = self._read_float('stop_dist', self.stop_dist, 1.2, 3.0)
        if as_bool(self.get_parameter('record').value) != self.record:
            self.get_logger().warn(
                'record 는 시작할 때만 읽습니다 — 기록하려면 노드를 다시 실행 (--ros-args -p record:=true)',
                throttle_duration_sec=5.0)

        scan, pos = self._scan_now(now), self._position_now(now)
        self._check_states(now, scan, pos)
        if scan is None:
            return                              # 경고는 _check_states 가 (상태가 바뀔 때만)

        front, left, right, near = self._sectors(scan)
        i_front, r_front = self._front_one(scan)
        dropped = sum(1 for r in scan.ranges if math.isfinite(r) and scan.range_min <= r < SELF_RANGE)
        one = dist_text(r_front) + (' m' if math.isfinite(r_front) else '')
        self.get_logger().info(
            f'[스캔] {len(scan.ranges)}칸 | 정면 1칸({i_front}) {one} | {sectors_text(front, left, right)} | '
            f'버림 {dropped}칸 | 판단: {self._judge(front, near)}')

    # ── [복붙 영역] 정지 거리 재기 — 정지 판단 순간과 멈춘 순간 사이의 시간·거리 (19강 13번) ──
    def on_watch(self):
        now = time.monotonic()
        scan, pos = self._scan_now(now), self._position_now(now)
        if scan is None:
            return
        f = self.front_deg
        front = sector_min(scan, -f, f)
        spd = self._fwd_speed(pos)
        if front is not None and front >= self.stop_dist + CLEAR_GAP:
            self.armed, self.judged = True, None          # 앞이 다시 넓어짐 → 다음 정지 판단을 받을 준비
        if self.armed and front is not None and front < self.stop_dist:
            self.armed = False
            speed = f'{spd:.2f} m/s' if spd is not None else '- (위치 없음)'
            self.get_logger().info(f'[정지 판단] 앞 {self.stop_dist:.2f} m 통과 | 전진 {speed}')
            self.judged = (now, pos.x, pos.y) if pos is not None else None
        if self.judged is not None and spd is not None and abs(spd) < STOP_SPEED:
            t0, n0, e0 = self.judged
            moved = math.hypot(pos.x - n0, pos.y - e0)
            self.get_logger().info(
                f'[멈춤] 앞 {dist_text(front)} m | 판단 뒤 {now - t0:.1f} s · {moved:.2f} m 이동')
            self.judged = None

    # ============================================================
    # [복붙 영역] CSV 기록 (record:=true, 교수용 — 그래프 데이터). 0.1초마다 한 줄
    #   t 노드 시작 뒤 시간 [s] · north/east/rel_alt 위치·상대 고도 [m] · front/left/right/near 섹터 값 [m]
    #   (자유 공간 = inf, 측정 없음 = 빈칸) · front_1 정면 1칸 · fwd_speed 전진 속도 [m/s] · judge 판단 칸
    # ============================================================
    def _open_record(self):
        path = os.path.expanduser(time.strftime('~/w11_scan_%Y%m%d_%H%M%S.csv'))
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
        scan, pos = self._scan_now(now), self._position_now(now)
        row = [csv_num(now - self.start)]
        row += [csv_num(pos.x), csv_num(pos.y), csv_num(-pos.z)] if pos is not None else ['', '', '']
        if scan is not None:
            front, left, right, near = self._sectors(scan)
            row += [csv_num(front), csv_num(left), csv_num(right), csv_num(near), csv_num(self._front_one(scan)[1])]
            judge = self._judge(front, near)
        else:
            row += ['', '', '', '', '']
            judge = '신호 없음'
        row += [csv_num(self._fwd_speed(pos)), judge]
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
    node = ScanViewer()
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
