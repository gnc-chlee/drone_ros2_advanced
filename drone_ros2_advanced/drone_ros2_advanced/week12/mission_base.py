#!/usr/bin/env python3
# ==============================================================================
# File    : mission_base.py  (12주차 22강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-06
# Version : 1.0.0
#
# Description:
#   11주차 회피 미션을 상태머신으로 다시 쓴 노드 — 켜면 이륙부터 착륙까지 스스로 끝난다
#   (PX4Base 상속 · 전이 조건표는 mission_table.py · 섹터 계산과 상수는 week11/avoid_base.py 에서 가져옴)
#
#   0.1초마다 세 단계뿐 (on_update):
#     ① sense()      : 입력(PX4 상태 · 사람 명령 · 스캔 · 시간) → 사건 이름들
#     ② next_state() : 전이 조건표를 위 줄부터 찾기 → 다음 상태
#     ③ do[상태]()   : 지금 상태의 일 (명령 1번)
#     상태가 바뀌는 곳은 go() 한 곳뿐: [전이] 로그 → 진입 동작(entry action) → 전이 · 상태 발행
#
#   11주차와 달라진 점:
#     - 막다른 곳 · 도착 · 신호 없음이 호버로 끝나지 않고 나가는 길이 있다 (표 6 · 8 · 11번 줄)
#     - 수동 개입(manual override): 키 M 으로 사람이 넘겨받고, G 로 돌려주고, L 로 착륙
#     - PX4 가 조종권을 가져가면(Offboard 가 아닌 채 1초) 다시 요청하지 않고 손을 뗀다 → DONE
#
#   안전 필터는 상태와 별개 (_send, 명령을 보내기 직전에 매 틱):
#     근접 : 앞 ±45° 1.0 m 미만 → 전진 0        옆 안전 거리 : 가는 쪽 옆 1.0 m 미만 → 좌우 0
#     지오펜스 : 북 ±3 m 바깥쪽 좌우 0 · 상대 고도 5 m 위로 상승 0
#     사각지대 : 뒤 90° 는 측정하지 않으므로 후진 없음 (MANUAL 만 예외 — 사람이 보고 있음)
#
#   구독 토픽:
#     /lidar/scan                     : LaserScan (w11_scan_bridge)
#     /fmu/out/vehicle_local_position : 상대 고도(−z) · 동쪽 위치(y) · 북쪽 위치(x)
#     /sjcu/mission_cmd               : String manual · auto · land (w12_keyboard)
#     /sjcu/manual_cmd                : Float32MultiArray [vx, vy] 몸체 속도, vy + = 오른쪽 (w12_keyboard)
#   발행 토픽:
#     /sjcu/mission_state             : String 상태 이름 (전이 때마다, 나중에 켠 구독자도 마지막 값을 받음)
#     /sjcu/mission_transition        : String 전이 한 줄 '15.4 | CRUISE → AVOID | 12 front_blocked | 앞 1.49 m < 1.5 m'
#                                       (미션 시작 뒤 시각 | 전이 | 표의 줄 번호 사건 | 이유 — w12_state_viewer 가 받음)
#     PX4 명령은 PX4Base 가 맡음
#
#   준비: 11주차 월드 그대로 (obstacle_world.sdf 를 PX4 폴더로 복사했으면 끝)
#
#   실행 방법:
#     0단계 드라이 런 (비행하지 않음, PX4·Agent 없어도 됨 — 브리지 터미널 3 은 끈다):
#       ros2 run drone_ros2_advanced w12_mission
#       (새 터미널) ros2 topic pub -r 10 /lidar/scan sensor_msgs/msg/LaserScan \
#                     "{angle_min: -1.0472, angle_increment: 1.0472, range_min: 0.1, range_max: 30.0, ranges: [2.0, 5.0, .inf]}"
#       → 칸 3개 = −60° · 0° · +60° = 오른쪽 · 앞 · 왼쪽. ranges 를 바꾸면 전이가 일어난다
#         [2.0, 1.2, .inf] → AVOID  /  [2.0, 5.0, .inf] → CRUISE  /  [0.8, 1.2, 0.8] → 막다른 곳 → LAND → DONE
#         [2.0, -.inf, .inf] → HOLD (너무 가까움)
#       ※ ranges 를 바꾸려면 발행을 껐다 켜야 하므로, 드라이 런은 발행이 끊겨도 마지막 스캔을 계속 쓴다
#         (신호 끊김은 비행 중 브리지 끄기로 확인)
#     비행:
#       터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500_lidar_2d
#       터미널 2: MicroXRCEAgent udp4 -p 8888
#       터미널 3: ros2 run drone_ros2_advanced w11_scan_bridge         ← pxh> 가 뜬 뒤에!
#       터미널 4: ros2 run drone_ros2_advanced w12_mission --ros-args -p fly:=true
#       터미널 5: ros2 run drone_ros2_advanced w12_keyboard            ← 수동 개입 (선택)
#       터미널 6: ros2 run drone_ros2_advanced w12_state_viewer        ← 상태도 창 (선택, 드라이 런에서도 됨)
#
#   끝내기: DONE 이 찍히면 Ctrl+C. 비행 중에 끝내려면 키 L (또는 pxh> commander land → 착지 확인 → Ctrl+C)
#     (비행 중에 이 노드를 먼저 끄면 1초 뒤 PX4 가 'Offboard 신호 끊김' 비상 동작으로 넘어간다)
#   다시 비행: PX4(터미널 1)도 다시 시작 — 그대로 켜면 그 자리에서 이륙해 '이미 도착'으로 바로 착륙한다
#
#   결함 주입 (fault injection) — 일부러 문제를 넣고 시스템이 어떻게 반응하는지 본다:
#     브리지(터미널 3) Ctrl+C → HOLD → 다시 실행 → 1.0초 뒤 CRUISE   /   10초 안에 안 켜면 LAND
#     pxh> commander mode auto:loiter → 1초 뒤 DONE, 다시 요청하지 않음 → pxh> commander land
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import math
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String

from ..px4_base import PX4Base, PX4_QOS
from ..week11.avoid_base import (   # 11주차 VM 에서 확인한 함수 · 상수를 그대로 쓴다
    ARRIVE_TOL, CLEAR_GAP, CREEP, GOAL_EAST, K_RET, KZ, LAT_LIMIT, LOG_PERIOD, MAX_CRUISE, MAX_REL_ALT,
    MAX_VZ, NEAR_DEG, REAR_DEG, RECOVER_TIME, RET_DONE, SAFE_DIST, SCAN_TIMEOUT, SIDE_DEG, STEP, TARGET_ALT, TIE_BAND,
    as_bool, clamp, dist_text, floor2, sector_min, sectors_text, side_name, signed,
)
from .mission_table import (
    AVOID, CRUISE, DONE, HOLD, IDLE, LAND, MANUAL, TAKEOFF, next_state, row_number,
)


# ================================================================
# [USER DEFINE] 미션 상수
# ================================================================
STOP_DIST  = 1.5     # 정지 판단 거리 [m] — front_blocked
CLEAR_DIST = STOP_DIST + CLEAR_GAP   # 해제 거리 [m] = 2.0 — front_clear. 둘이 달라서 히스테리시스
V_SIDE     = 0.3     # 좌우 이동 속도 [m/s]
K_SLOW     = 0.33    # 감속 비례 상수 [1/s]: 전진 속도 = K_SLOW × (앞 거리 − 정지 판단 거리)
FRONT_DEG  = 30.0    # 앞 섹터 반각 [°]
AVOID_TIMEOUT = 30.0 # AVOID 타임아웃 [s] — 넘으면 막다른 곳으로 본다

# 파라미터 (기본값, 받는 범위, 바뀌었을 때 찍는 말)
PARAMS = {
    'cruise_speed': (0.5,  0.0, MAX_CRUISE, '전진 속도 변경:'),
    'hold_timeout': (10.0, 1.0, 60.0,       'HOLD 타임아웃 변경:'),
}

# ── [복붙 영역] 시간 상수 ─────────────────────────────────────────
ALT_TOL        = 0.2   # 이륙 완료 고도 오차 [m]
TAKEOVER_TIME  = 1.0   # Offboard 가 아닌 채 이만큼이면 PX4 가 조종권을 가져간 것으로 본다 [s] (상태 메시지 지연 대비)
MANUAL_TIMEOUT = 0.5   # 키보드 값이 이만큼 안 오면 수동 속도 0 [s]
LAND_RETRY     = 2.0   # 착륙 요청 뒤에도 Offboard 면 이 간격으로 다시 요청 [s]
DRY_WAIT       = 1.0   # 드라이 런: PX4 사건(ready · airborne · landed)이 저절로 생길 때까지 [s]

# 키보드 명령 → 사건 이름
CMD_EVENTS = {'manual': 'cmd_manual', 'auto': 'cmd_auto', 'land': 'cmd_land'}

# 상태 토픽 QoS — 나중에 켠 w12_keyboard 도 지금 상태를 바로 받도록 마지막 값을 보관
STATE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def dist_m(value):
    """섹터 값 → 글자 + 단위: '1.20 m' · '자유 공간' · '측정 없음' (숫자일 때만 m)"""
    return dist_text(value) + (' m' if value is not None and math.isfinite(value) else '')


class MissionBase(PX4Base):
    def __init__(self):
        super().__init__('mission_base')

        # ── 파라미터 ─────────────────────────────────────────────
        #    fly          : False 면 드라이 런 (arm·이륙·속도 명령 없음). 시작할 때만 읽음
        #    cruise_speed : 전진 속도 [m/s] 0~0.5
        #    hold_timeout : HOLD 타임아웃 [s] 1~60
        #    fly 를 뺀 둘은 0.1초마다 다시 읽는다 → ros2 param set 으로 비행 중에 바꿀 수 있음
        self.param_warn_at = {}   # 파라미터 경고를 마지막으로 찍은 시각 (이름마다)
        self.declare_parameter('fly', False, ParameterDescriptor(dynamic_typing=True))
        self.fly = as_bool(self.get_parameter('fly').value)
        for name, (default, _, _, _) in PARAMS.items():
            self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
            setattr(self, name, default)
            self._read_param(name, start=True)

        # ============================================================
        # [USER DEFINE] 구독 4개 · 발행 2개
        # ============================================================
        self.create_subscription(
            LaserScan, '/lidar/scan', self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._position_callback, PX4_QOS)
        self.create_subscription(String, '/sjcu/mission_cmd', self._cmd_callback, 10)
        self.create_subscription(Float32MultiArray, '/sjcu/manual_cmd', self._manual_callback, 1)
        self.state_pub = self.create_publisher(String, '/sjcu/mission_state', STATE_QOS)
        self.transition_pub = self.create_publisher(String, '/sjcu/mission_transition', 10)

        # ── 입력 값 ──────────────────────────────────────────────
        self.position   = None    # 마지막 VehicleLocalPosition
        self.scan       = None    # 마지막 유효한 LaserScan
        self.scan_at    = None    # 그 스캔을 받은 시각 [s]
        self.good_since = None    # 유효한 스캔이 끊기지 않고 이어진 첫 시각 [s]
        self.neg        = 0       # 마지막 스캔의 -inf 칸 수 (너무 가까움)
        self.bad_logged = False   # '칸 정보가 이상한 스캔' 경고를 한 번만
        self.cmds       = set()   # 다음 틱에 쓸 사람 명령 사건
        self.manual     = (0.0, 0.0)   # 키보드 몸체 속도 [m/s]
        self.manual_at  = None    # 그 값을 받은 시각 [s]
        self.sec        = {}      # 이번 틱의 섹터 값 (None = 측정 없음, 빈 사전 = 스캔 끊김)
        self.off_since  = None    # Offboard 가 아닌 것을 처음 본 시각 [s]

        # ── 상태 ─────────────────────────────────────────────────
        self.start       = time.monotonic()
        self.state       = IDLE
        self.state_since = self.start
        self.history     = [(0.0, IDLE)]  # [미션 기록]: (시각, 상태)
        self.idle_ticks  = 0      # IDLE 에서 흐른 틱 수 (요청 재시도용)
        self.takeoff_xy  = (math.nan, math.nan)   # 이륙 자리 (NED x, y) [m]
        self.side        = 0      # AVOID 방향: -1 = 왼쪽, +1 = 오른쪽
        self.flipped     = False  # AVOID 에서 재탐색을 이미 했는지
        self.dead_why    = ''     # 막다른 곳 이유 (로그용)
        self.land_at     = 0.0    # 마지막 착륙 요청 시각 [s]
        self.last_vx = self.last_vy = 0.0    # 직전 틱에 보낸 몸체 속도 [m/s] (가속도 제한용)
        self.last_tag    = None   # 직전에 찍은 줄 태그 (바뀌면 바로 한 줄)
        self.last_log_at = -math.inf

        # 진입 동작(들어갈 때 한 번) · 상태의 일(매 틱)
        self.enter = {TAKEOFF: self._enter_takeoff, AVOID: self._enter_avoid, HOLD: self._enter_hold,
                      MANUAL: self._enter_manual, LAND: self._enter_land, DONE: self._enter_done}
        self.do = {IDLE: self._do_idle, TAKEOFF: self._do_takeoff, CRUISE: self._do_cruise, AVOID: self._do_avoid,
                   HOLD: self._do_hold, MANUAL: self._do_manual, LAND: self._do_land, DONE: self._do_done}

        self.state_pub.publish(String(data=self.state))
        self.create_timer(0.1, self.on_update)   # 10Hz

        mode = '비행 (fly:=true)' if self.fly else '드라이 런 (fly:=false)'
        hint = ('끝내기: DONE 뒤 Ctrl+C' if self.fly else
                '비행하지 않음 (비행: --ros-args -p fly:=true) — 모의 스캔은 브리지(터미널 3)를 끄고 ros2 topic pub')
        self.get_logger().info(
            f'미션 노드 시작 — {mode} | 전진 속도 {self.cruise_speed:g} m/s | 목표 동 {GOAL_EAST:.1f} m | '
            f'정지 판단 {STOP_DIST:.1f} m · 해제 {CLEAR_DIST:.1f} m | HOLD 타임아웃 {self.hold_timeout:g} s\n'
            f'  {hint}')

    # ============================================================
    # 메인 루프 (10Hz) — 세 단계뿐
    # ============================================================
    def on_update(self):
        self._read_params()
        events = self.sense()                         # ① 입력 → 사건 이름들
        next_s, event = next_state(self.state, events)   # ② 표에서 위 줄부터 찾기
        if next_s != self.state:
            self.go(next_s, event)                    # 로그 · 진입 동작 · 상태 발행은 여기 한 곳
        self.do[self.state]()                         # ③ 지금 상태의 일 (명령 1번)

    # ============================================================
    # ① 사건 만들기 — 입력을 사건 이름으로 바꿀 뿐, 판단은 표가 한다
    # ============================================================
    def sense(self):
        now = time.monotonic()
        events, self.cmds = self.cmds, set()    # 사람: 키보드 명령 (콜백이 모아 둠)
        self._sense_px4(events, now)            # PX4: ready · airborne · landed · px4_takeover

        # 센서
        live = self._scan_live(now)
        self.sec = self._sectors(self.scan) if live else {}
        if not live or self.neg > 0:
            events.add('sensor_bad')
        else:
            front, left, right = self._front_left_right()
            near = self._sec_value('near')
            if self.scan_at - self.good_since >= RECOVER_TIME:
                events.add('sensor_ok')
            if front < STOP_DIST or near < SAFE_DIST:         # 히스테리시스 — 정지 판단 1.5 m 미만
                events.add('front_blocked')
            if front >= CLEAR_DIST and near >= SAFE_DIST:     # 해제 2.0 m 이상
                events.add('front_clear')
            if self.state == AVOID and self._is_dead_end(now, left, right):
                events.add('dead_end')

        # 미션 (드라이 런은 위치를 쓰지 않아 도착이 없다)
        if self.fly and self.position is not None and self.position.y >= GOAL_EAST - ARRIVE_TOL:
            events.add('arrived')

        # 시간
        if self.state == HOLD and now - self.state_since >= self.hold_timeout:
            events.add('hold_timeout')
        return events

    def _sense_px4(self, events, now):
        if not self.fly:                        # 드라이 런: PX4 없이 잠깐 뒤 저절로 생긴 것으로 본다
            if now - self.state_since >= DRY_WAIT:
                if self.state == IDLE and self._scan_live(now):
                    events.add('ready')
                elif self.state == TAKEOFF:
                    events.add('airborne')
                elif self.state == LAND:
                    events.add('landed')
            return
        if self.is_offboard and self.is_armed:  # PX4Base 가 알려주는 상태
            self.off_since = None
            events.add('ready')
        elif self.off_since is None:
            self.off_since = now
        if self.off_since is not None and now - self.off_since >= TAKEOVER_TIME:
            events.add('px4_takeover')
        if self.position is not None and abs(self.position.z + TARGET_ALT) < ALT_TOL:
            events.add('airborne')
        if self.is_landed:
            events.add('landed')

    def _is_dead_end(self, now, left, right):
        """막다른 곳: 양옆 안전 거리 안 / 재탐색 뒤에도 막힘 / AVOID 타임아웃. 이유는 로그용으로 남긴다"""
        if left < SAFE_DIST and right < SAFE_DIST:
            self.dead_why = '양옆 막힘'
        elif self.flipped and self._side_value(self.side) < SAFE_DIST:
            self.dead_why = '재탐색 뒤에도 막힘'
        elif now - self.state_since >= AVOID_TIMEOUT:
            self.dead_why = f'{AVOID_TIMEOUT:g}초 안에 앞이 열리지 않음'
        else:
            return False
        return True

    # ============================================================
    # ② 전이 — 상태가 바뀌는 곳은 여기 한 곳뿐
    # ============================================================
    def go(self, next_s, event):
        now = time.monotonic()
        prev, why = self.state, self._why(event)    # 왜 바뀌었나 (숫자까지)
        self.get_logger().info(f'[전이] {prev} → {next_s} | {why}')
        self.state, self.state_since = next_s, now
        self.history.append((now - self.start, next_s))
        self.last_tag = None                    # 다음 상태의 첫 줄을 바로 찍음
        if next_s in self.enter:
            self.enter[next_s]()                # 들어갈 때 한 번만 하는 일 (진입 동작)
        # 상태 뷰어용 한 줄: 시각 | 전이 | 표의 줄 번호 + 사건 | 이유
        self.transition_pub.publish(String(
            data=f'{now - self.start:.1f} | {prev} → {next_s} | {row_number(prev, event)} {event} | {why}'))
        self.state_pub.publish(String(data=next_s))

    def _why(self, event):
        """전이 줄 뒷부분 — 로그만 봐도 숫자로 검산할 수 있게"""
        p = self.position
        s = self.sec
        if event == 'ready':
            return 'Offboard + Arm 확인' if self.fly else '스캔 확인 → Offboard + Arm 으로 가정 (드라이 런)'
        if event == 'airborne':
            return f'상대 고도 {-p.z:.2f} m' if self.fly else '이륙한 것으로 가정 (드라이 런)'
        if event == 'landed':
            return '착지 감지' if self.fly else '착지한 것으로 가정 (드라이 런)'
        if event == 'px4_takeover':
            nav = self.vehicle_status.nav_state
            return f'Offboard 가 아닌 채 {TAKEOVER_TIME:g}초 (nav_state {nav}) → 다시 요청하지 않음'
        if event in ('cmd_manual', 'cmd_auto', 'cmd_land'):
            return f'사람 명령 {event[4:]}'
        if event == 'sensor_bad':
            if self.neg > 0 and self.sec:
                return f'-inf {self.neg}칸 (너무 가까움)'
            return f'유효한 스캔 {SCAN_TIMEOUT:g}초 없음'
        if event == 'sensor_ok':
            return f'유효한 스캔 {RECOVER_TIME:.1f}초 이어짐'
        if event == 'hold_timeout':
            return f'{self.hold_timeout:g}초 안에 복구되지 않음'
        if event == 'dead_end':
            return f'{self.dead_why} — {sectors_text(s.get("front"), s.get("left"), s.get("right"))}'
        if event == 'arrived':
            return f'동 {signed(p.y)} m (목표 {GOAL_EAST:.1f})'
        front = self._sec_value('front')
        if event == 'front_blocked':
            if front < STOP_DIST:
                return f'앞 {floor2(front):.2f} m < {STOP_DIST:.1f} m'
            return f'앞쪽 ±{NEAR_DEG:g}° {self._sec_value("near"):.2f} m < 안전 거리 {SAFE_DIST:.1f} m'
        if event == 'front_clear':
            return f'앞 {dist_m(s.get("front"))} ≥ 해제 {CLEAR_DIST:.1f} m'
        return event

    # ── 진입 동작 ────────────────────────────────────────────────
    def _enter_takeoff(self):
        self.takeoff_xy = self._here()          # 지금 자리에서 이륙 (0, 0 으로 쓰면 다시 켰을 때 원점으로 날아간다)

    def _enter_avoid(self):
        left, right = self._sec_value('left'), self._sec_value('right')
        self.side = -1 if left > right + TIE_BAND else 1   # 넓은 쪽. 비슷하면 오른쪽 (9주차 재탐색 방향)
        self.flipped = False
        s = self.sec
        rule = ' → 비슷하면 오른쪽' if self.side > 0 and not right > left + TIE_BAND else ''
        self.get_logger().info(f'[AVOID] {side_name(self.side)} 선택 '
                               f'(왼쪽 {dist_m(s.get("left"))} · 오른쪽 {dist_m(s.get("right"))}{rule})')

    def _enter_hold(self):
        self.last_vx = self.last_vy = 0.0       # 속도는 가속도 제한 없이 바로 0

    def _enter_manual(self):
        self.manual, self.manual_at = (0.0, 0.0), None
        self.get_logger().info('[MANUAL] 사람이 조종 — W·A·S·D 이동 · Space 정지 · G 자율로 돌려주기 · L 착륙')

    def _enter_land(self):
        self.land_at = time.monotonic()
        if self.fly:
            self.land()                         # PX4 착륙 모드로 (한 번. 안 넘어가면 _do_land 가 다시 요청)

    def _enter_done(self):
        record = ' | '.join(f'{t:.1f} {name}' for t, name in self.history)
        self.get_logger().info(f'[미션 기록] {record}')
        self.heartbeat_timer.cancel()           # Offboard 신호도 멈춤 = 완전히 손을 뗌
        if self.fly and not self.is_landed:
            self.get_logger().warn('PX4 가 조종 중 — 이 노드는 더 보내지 않음. 끝내기: pxh> commander land → 착지 확인 → Ctrl+C')
        else:
            self.get_logger().info('미션 끝 — Ctrl+C 로 노드를 끄세요')

    # ============================================================
    # ③ 상태의 일 — 매 틱 명령 1번 (LAND · DONE 은 명령 없음)
    # ============================================================
    def _do_idle(self):
        live = self._scan_live(time.monotonic())
        if not live:
            hint = 'w11_scan_bridge(터미널 3) 실행 확인' if self.fly else '모의 스캔을 발행하세요 (ros2 topic pub … /lidar/scan …)'
            self.get_logger().warn(f'[IDLE] 스캔 대기 — {hint}', throttle_duration_sec=2.0)
        if not self.fly:
            return
        x, y = self._here()
        self.send_position(x, y, -TARGET_ALT)
        self.idle_ticks += 1
        # heartbeat 1초 뒤부터 1초마다. 스캔 없이는 이륙하지 않는다 (눈을 감고 뜨지 않게)
        if live and self.position is not None and self.idle_ticks % 10 == 0:
            self.set_offboard_mode()
            self.arm()

    def _do_takeoff(self):
        if self.fly:
            self.send_position(self.takeoff_xy[0], self.takeoff_xy[1], -TARGET_ALT)

    def _do_cruise(self):
        front, left, right = self._front_left_right()
        vx_t = self.speed_limit(front)          # 전진 (거리 비례 감속)
        vy_t = self.return_vy(left, right)      # 코스 복귀
        vx, vy, note = self._send(vx_t, vy_t)
        s = self.sec
        back = ' (코스 복귀)' if abs(vy_t) > 1e-9 else ''
        self._report('CRUISE', note, f'[CRUISE] {self._where()}{sectors_text(s.get("front"), s.get("left"), s.get("right"))}'
                                     f' | vx {vx:+.2f} · vy {vy:+.2f} m/s{back}')

    def _do_avoid(self):
        chosen, other = self._side_value(self.side), self._side_value(-self.side)
        if chosen < SAFE_DIST <= other and not self.flipped:   # 고른 쪽이 막힘 → 반대쪽으로 한 번 재탐색
            self.flipped, self.side = True, -self.side
            self.get_logger().warn(f'[재탐색] {side_name(-self.side)} {chosen:.2f} m < 안전 거리 {SAFE_DIST:.1f} m '
                                   f'→ {side_name(self.side)} (한 번만)')
        vx, vy, note = self._send(0.0, self.side * V_SIDE)
        self._report('AVOID', note, f'[AVOID] {side_name(self.side)} | 앞 {dist_m(self.sec.get("front"))} '
                                    f'(해제 {CLEAR_DIST:.1f}) | vy {vy:+.2f} m/s{self._north()}')

    def _do_hold(self):
        if self.fly:
            self.last_vx = self.last_vy = 0.0
            self.send_velocity_body(0.0, 0.0, self._hold_vz(), 0.0)
        now = time.monotonic()
        scan = f'스캔 복구 {now - self.good_since:.1f} / {RECOVER_TIME:.1f}초' if self.sec and self.neg == 0 else '스캔 없음'
        self._report('HOLD', None, f'[HOLD] 대기 {now - self.state_since:.1f} / {self.hold_timeout:g}초 | {scan} | 수평 정지 · 고도 유지')

    def _do_manual(self):
        now = time.monotonic()
        fresh = self.manual_at is not None and now - self.manual_at <= MANUAL_TIMEOUT
        vx_t, vy_t = self.manual if fresh else (0.0, 0.0)   # 키보드 입력이 끊기면 0
        vx, vy, note = self._send(vx_t, vy_t)
        wait = '' if fresh else ' | 키보드 입력 없음'
        self._report('MANUAL', note, f'[MANUAL] {self._where()}vx {vx:+.2f} · vy {vy:+.2f} m/s{wait}')

    def _do_land(self):
        if not self.fly or not self.is_offboard:   # PX4 착륙 모드 — 내려가는 일은 PX4 가 한다
            return
        # 아직 Offboard 면 제자리에서 기다리며 다시 요청 (착륙 요청이 거절되거나 상태 메시지가 늦을 때)
        self.send_velocity_body(0.0, 0.0, self._hold_vz(), 0.0)
        now = time.monotonic()
        if now - self.land_at >= LAND_RETRY:
            self.land_at = now
            self.land()

    def _do_done(self):
        pass

    # ── 11주차 판단 두 가지 ──────────────────────────────────────
    # [복붙 영역] 속도 제한: K_SLOW × (앞 거리 − 정지 판단 거리) → 포화 (최소 CREEP, 최대 cruise_speed)
    def speed_limit(self, front):
        if self.cruise_speed <= 0.0:
            return 0.0
        if math.isinf(front):
            return self.cruise_speed
        return min(self.cruise_speed, max(CREEP, K_SLOW * (front - STOP_DIST)))

    # [복붙 영역] 코스 복귀: 돌아갈 쪽의 옆과 뒤 대각선이 해제 거리 넘게 비면 코스(북 0)로
    def return_vy(self, left, right):
        p = self.position if self.fly else None
        if p is None or self.cruise_speed <= 0.0:
            return 0.0
        off = p.x                               # NED x = 북쪽 [m]
        s = math.sin(self.current_yaw)          # 기수 동쪽이면 1 (오른쪽 이동 = 남쪽)
        if abs(off) < RET_DONE or abs(s) < 0.5:
            return 0.0
        direction = 1 if off * s > 0 else -1    # +1 = 오른쪽으로 가야 코스에 가까워짐
        side_val = right if direction > 0 else left
        rear = self._sec_value('rear_r' if direction > 0 else 'rear_l')
        if min(side_val, rear) < CLEAR_DIST:
            return 0.0
        return direction * min(K_RET * abs(off), V_SIDE)

    # ============================================================
    # 명령 보내기 — 가속도 제한 → 안전 필터 → 명령 1번 (드라이 런은 값만 돌려준다)
    # ============================================================
    def _send(self, vx_t, vy_t):
        vx, vy = self._ramp(vx_t, vy_t) if self.fly else (vx_t, vy_t)
        note = None                             # 안전 필터가 바꿨으면 (태그, 글자)
        if not self.sec:                        # 스캔 끊김 (MANUAL, 또는 HOLD 로 가기 전 한 틱): 측정 없음 = 막힘
            vx, vy = min(vx, 0.0), 0.0
            note = ('스캔 없음', '[스캔 없음] 전진 · 좌우 이동 막힘 (수동 조종이면 후진 · L 착륙은 가능)')
        else:
            near = self._sec_value('near')
            if vx > 0.0 and near < SAFE_DIST:   # 근접: 전진만 0 (좌우 이동은 허용 → 빠져나올 수 있게)
                vx = 0.0
                note = ('근접', f'[근접] 앞쪽 {near:.2f} m < 안전 거리 {SAFE_DIST:.1f} m → 전진 정지')
            if abs(vy) > 1e-9:                  # 옆 안전 거리: 실제로 가는 쪽 옆 섹터 (vy 음수 = 왼쪽)
                direction = -1 if vy < 0 else 1
                side_val = self._side_value(direction)
                if side_val < SAFE_DIST:
                    vy = 0.0
                    note = ('안전', f'[안전] {side_name(direction)} {side_val:.2f} m < 안전 거리 {SAFE_DIST:.1f} m → 좌우 이동 정지')
        p = self.position if self.fly else None
        if p is not None:                       # 지오펜스: 북 ±3 m 바깥쪽으로는 좌우 이동하지 않음
            n_dot = -vy * math.sin(self.current_yaw)   # 기수 동쪽: 왼쪽 = 북 +
            if (p.x > LAT_LIMIT and n_dot > 0.0) or (p.x < -LAT_LIMIT and n_dot < 0.0):
                vy = 0.0
                note = ('지오펜스', f'[지오펜스] 북 {signed(p.x)} m → 바깥쪽 이동 정지')
        if self.state != MANUAL:                # 사각지대: 뒤 90° 는 측정하지 않으므로 후진 없음
            vx = max(vx, 0.0)
        if self.fly:
            self.last_vx, self.last_vy = vx, vy
            # PX4Base: 몸체 속도 → 기수 방향으로 NED 로 돌려 보냄 (6주차). 요 각속도 0 → 기수 그대로
            self.send_velocity_body(vx, vy, self._hold_vz(), 0.0)
        return vx + 0.0, vy + 0.0, note         # + 0.0: '-0.00' 대신 '+0.00' 으로 찍히게

    def _ramp(self, vx_t, vy_t):
        """[복붙 영역] 가속도 제한 — 두 축 모두 한 틱에 STEP 까지만 (0.5 m/s², 11주차와 같음)"""
        return (self.last_vx + clamp(vx_t - self.last_vx, STEP),
                self.last_vy + clamp(vy_t - self.last_vy, STEP))

    def _hold_vz(self):
        """[복붙 영역] 고도 유지 — 9주차와 같은 −z 한 줄 (NED: + = 하강), 최대 고도 제한 5 m"""
        if self.position is None:
            return 0.0
        h = -self.position.z                    # 상대 고도 [m]
        vz = clamp(KZ * (h - TARGET_ALT), MAX_VZ)
        if h > MAX_REL_ALT and vz < 0.0:
            vz = 0.0
        return vz

    # ============================================================
    # 콜백 — 받은 값을 저장
    # ============================================================
    def _scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        ok = (n > 0 and math.isfinite(msg.range_min) and math.isfinite(msg.range_max)
              and msg.range_max > msg.range_min)
        if n > 1:                               # 여러 칸이면 각도 정보도 있어야 칸 번호 ↔ 각도를 계산할 수 있다
            ok = ok and math.isfinite(msg.angle_min) and msg.angle_increment > 0.0
        if not ok:
            # 버린 스캔은 '받은 시각'을 갱신하지 않는다 → 계속 버려지면 0.5초 뒤 sensor_bad
            if not self.bad_logged:
                self.bad_logged = True
                self.get_logger().warn('스캔 칸 정보가 이상해 버림 — 모의 스캔이면 '
                                       'angle_min·angle_increment·range_min·range_max 를 모두 넣기')
            return
        now = time.monotonic()
        self.neg = sum(1 for r in msg.ranges if r == -math.inf)
        if self.neg > 0 or self.scan_at is None or now - self.scan_at > SCAN_TIMEOUT:
            self.good_since = now               # 끊겼거나 너무 가까움 → 복구 시간을 처음부터 셈
        self.scan, self.scan_at = msg, now

    def _position_callback(self, msg: VehicleLocalPosition):
        self.position = msg

    def _cmd_callback(self, msg: String):
        word = msg.data.strip().lower()
        event = CMD_EVENTS.get(word)
        if event is None:
            self.get_logger().warn(f'[명령] 모르는 명령 "{msg.data}" — manual · auto · land 중 하나')
        elif next_state(self.state, {event})[1] is None:   # 표에 없는 사건은 무시된다
            self.get_logger().info(f'[명령] {word} — {self.state} 에는 이 사건을 받는 줄이 없어 무시')
        else:
            self.cmds.add(event)

    def _manual_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:
            return
        self.manual = (clamp(float(msg.data[0]), MAX_CRUISE), clamp(float(msg.data[1]), MAX_CRUISE))
        self.manual_at = time.monotonic()

    # ============================================================
    # 도우미
    # ============================================================
    def _scan_live(self, now):
        if self.scan_at is None:
            return False
        # 드라이 런은 마지막 모의 스캔을 계속 쓴다 — ranges 를 바꾸려고 발행을 껐다 켜는 동안 끊기기 때문
        return not self.fly or now - self.scan_at <= SCAN_TIMEOUT

    def _here(self):
        """지금 수평 위치 (NED x, y) [m]. 위치를 아직 모르면 NaN = 수평은 명령하지 않음"""
        p = self.position
        return (math.nan, math.nan) if p is None else (float(p.x), float(p.y))

    def _sectors(self, scan):
        return {
            'front':  sector_min(scan, -FRONT_DEG, FRONT_DEG),
            'left':   sector_min(scan, SIDE_DEG[0], SIDE_DEG[1]),
            'right':  sector_min(scan, -SIDE_DEG[1], -SIDE_DEG[0]),
            'near':   sector_min(scan, -NEAR_DEG, NEAR_DEG),
            'rear_l': sector_min(scan, REAR_DEG[0], REAR_DEG[1]),
            'rear_r': sector_min(scan, -REAR_DEG[1], -REAR_DEG[0]),
        }

    def _sec_value(self, key):
        """섹터 값 — 측정 없음(None)은 0 (막힘)으로 본다"""
        value = self.sec.get(key)
        return 0.0 if value is None else value

    def _front_left_right(self):
        return self._sec_value('front'), self._sec_value('left'), self._sec_value('right')

    def _side_value(self, direction):
        """방향 부호 → 그쪽 옆 섹터 값 (-1 = 왼쪽, +1 = 오른쪽)"""
        return self._sec_value('left' if direction < 0 else 'right')

    def _where(self):
        p = self.position if self.fly else None
        if p is None:
            return ''
        north = f' · 북 {signed(p.x)} m' if abs(p.x) >= RET_DONE else ''
        return f'동 {signed(p.y)} m{north} | '

    def _north(self):
        p = self.position if self.fly else None
        return '' if p is None else f' | 북 {signed(p.x)} m'

    def _report(self, tag, note, text):
        """안전 필터가 바꿨으면 그 말을, 아니면 상태 줄을 찍는다"""
        dry = '' if self.fly else '[드라이 런] '
        if note is not None:
            self._log(note[0], dry + note[1], warn=True)
        else:
            self._log(tag, dry + text)

    def _log(self, tag, text, warn=False):
        """태그가 바뀌면 바로, 같으면 LOG_PERIOD(1초)마다 한 줄"""
        now = time.monotonic()
        if tag == self.last_tag and now - self.last_log_at < LOG_PERIOD:
            return
        self.last_tag, self.last_log_at = tag, now
        # info·warn 을 한 줄에서 번갈아 부르면 rclpy 가 ValueError (같은 호출 자리에서 심각도가 바뀜) → 두 줄로 나눔
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    # ── [복붙 영역] 파라미터 다시 읽기 — 숫자 검사(bool·nan·inf 거부) + 범위, 밖이면 경고 후 이전 값 ──
    #    범위 밖 값도 ros2 param set 은 'Set parameter successful' 이라고 답한다 → 결과는 이 노드 로그로 확인
    def _read_params(self):
        for name in PARAMS:
            self._read_param(name)
        if as_bool(self.get_parameter('fly').value) != self.fly:
            self.get_logger().warn('fly 는 시작할 때만 읽습니다 — 바꾸려면 노드를 다시 실행 (비행 중이면 키 L 먼저)',
                                   throttle_duration_sec=5.0)

    def _read_param(self, name, start=False):
        default, low, high, text = PARAMS[name]
        old = getattr(self, name)
        keep = f'기본값 {default:g} 로 시작' if start else f'이전 값 {old:g} 그대로'
        try:
            raw = self.get_parameter(name).value
            if isinstance(raw, bool):           # true/false 는 숫자가 아님 (float(True) = 1.0 방지)
                raise TypeError
            value = float(raw)
            if not math.isfinite(value):
                raise ValueError
        except (TypeError, ValueError):
            self._param_warn(name, f'{name} 값이 숫자가 아님 — {keep}')
            return
        if not low <= value <= high:
            self._param_warn(name, f'{name} {value:g} 는 범위 밖 ({low:g}~{high:g}) — {keep}')
            return
        if value != old:
            if not start:
                self.get_logger().info(f'{text} {old:g} → {value:g}')
            setattr(self, name, value)

    def _param_warn(self, name, text):
        """파라미터 경고는 이름마다 2초에 한 번"""
        now = time.monotonic()
        if now - self.param_warn_at.get(name, -math.inf) >= 2.0:
            self.param_warn_at[name] = now
            self.get_logger().warn(text)


def main(args=None):
    rclpy.init(args=args)
    node = MissionBase()
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
