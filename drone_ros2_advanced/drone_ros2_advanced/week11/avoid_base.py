#!/usr/bin/env python3
# ==============================================================================
# File    : avoid_base.py  (11주차 20강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-02
# Version : 1.0.0
#
# Description:
#   2D LiDAR 로 앞을 보며 동쪽 12 m 목표까지 가는 반응형 회피(reactive avoidance) 노드
#   — 10주차 terrain_follow_base 뼈대 (PX4Base 상속, IDLE → TAKEOFF → CRUISE)
#
#   19강 규칙을 노드로 (VFH 3섹터판):
#     앞 막힘(1.5 m 미만) → 정지 → 넓은 쪽으로 좌우 이동 → 앞이 2.0 m 넘게 비면 전진 → 옆이 비면 코스 복귀 → 도착
#     - 1080칸을 섹터 최근접 거리 3개로: 앞 ±30° · 왼쪽 +30~+90° · 오른쪽 −90~−30°
#     - 좌우 동점(0.3 m 안)이면 오른쪽 (9주차 재탐색 방향)
#     - 고른 방향은 앞이 해제 거리(정지 판단 + 0.5 = 2.0 m) 넘게 빌 때까지 유지 → 진동(oscillation) 없음
#     - 측정한 방향으로만 이동: 뒤 90° 는 사각지대라 후진 명령 없음, 측정 없음 = 막힘
#     ※ PX4 충돌 방지(Collision Prevention)는 Position 모드에서 사람이 조종할 때만 동작하고 Offboard 에서는
#       불리지 않는다 → 같은 원리를 이 노드가 직접 한다
#     ※ 정지·좌우 이동·복귀·도착은 새 상태가 아니라 CRUISE 안의 if 와 방향 변수 side 로 처리 (모드 전환 설계는 12주차)
#
#   부호 (20강 핵심):
#     스캔 각도 + = 왼쪽 (FLU)   ↔   send_velocity_body 의 vy_body + = 오른쪽 (몸체 좌표계 FRD)
#     → 왼쪽 좌우 이동 = vy_body 음수 (side = -1, vy = side × v_side = -0.3 m/s)
#
#   속도 제한 (19강 19번 — 9주차 kp × 오차 → 포화 와 같은 꼴):
#     전진 속도 = k_slow × (앞 거리 − 정지 판단 거리) → 최소 0.1 · 최대 cruise_speed
#     기본값이면 앞 3.0 m 부터 감속, 1.8 m 아래 0.1 m/s, 1.5 m 미만 정지
#
#   안전장치 ([복붙 영역], 0 은 가속도 제한 없이 바로):
#     신호 없음   : 유효한 스캔 0.5초 없음 → 수평 정지·고도 유지 (1.0초 이어지면 재개)
#     너무 가까움 : -inf 한 칸이라도 → 수평 정지
#     근접        : 앞 ±45° 1.0 m 미만 → 전진 0 (좌우 이동은 허용 — 빠져나올 수 있게)
#     옆 안전 거리 : 가는 쪽 옆 섹터 1.0 m 미만 → 좌우 0 (고른 쪽이면 막다른 곳 → 호버)
#     사각지대·지오펜스 : 후진 없음 (vx ≥ 0) · 북 ±3 m 바깥쪽 좌우 0 · 상대 고도 5 m 위로 상승 0
#
#   구독 토픽:
#     /lidar/scan                     : LaserScan 1080칸 (w11_scan_bridge, sensor_msgs/LaserScan)
#     /fmu/out/vehicle_local_position : 상대 고도(−z) · 동쪽 위치(y = 목표까지) · 북쪽 위치(x = 코스 복귀)
#   발행: PX4Base 가 맡음 (heartbeat 10 Hz + 속도 명령). /fmu/in/obstacle_distance 에는 쓰지 않는다
#
#   준비 (한 번만): 월드를 PX4 폴더로 복사
#     cd ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/worlds
#     cp obstacle_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
#
#   실행 방법:
#     0단계 드라이 런 (비행하지 않음, PX4·Agent 없어도 됨 — 브리지 터미널 3 은 끈다):
#       ros2 run drone_ros2_advanced w11_avoid --ros-args -p cruise_speed:=0.5
#       (새 터미널) ros2 topic pub -r 10 /lidar/scan sensor_msgs/msg/LaserScan \
#                     "{angle_min: -1.0472, angle_increment: 1.0472, range_min: 0.1, range_max: 30.0, ranges: [2.0, 1.2, .inf]}"
#       → 칸 3개 = −60° · 0° · +60° = 오른쪽 · 앞 · 왼쪽. ranges 만 바꿔 가며 전진·감속·정지·방향 유지·막다른 곳 확인
#     비행 (터미널 4 는 뷰어 자리 — 선택):
#       터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500_lidar_2d
#       터미널 2: MicroXRCEAgent udp4 -p 8888
#       터미널 3: ros2 run drone_ros2_advanced w11_scan_bridge         ← pxh> 가 뜬 뒤에!
#       터미널 5: 아래 순서대로
#         1단계 제자리 호버     : ros2 run drone_ros2_advanced w11_avoid --ros-args -p fly:=true
#                                  (전진 속도 기본 0 = 제자리)
#         2단계 비행 중 전진 켜기 : ros2 param set /avoid_base cruise_speed 0.5
#                                  (기둥 앞 정지 → 왼쪽 좌우 이동 → 통과 → 코스 복귀 → 동 12 m 도착)
#
#   끝내기 (비행했다면 이 순서로):
#     PX4 터미널(pxh>)에서 commander land → 착지 확인 → 이 노드 Ctrl+C
#     (비행 중에 이 노드를 먼저 끄면 1초 뒤 PX4 가 'Offboard 신호 끊김' 비상 동작으로 넘어간다)
#
#   일부러 틀리기:
#     브리지(터미널 3) Ctrl+C                         → [신호 없음] 수평 정지 → 다시 실행하면 1.0초 뒤 재개 (학생)
#     드라이 런 ranges [0.8, 1.2, .inf] + -p v_side:=-0.3 → [안전] 오른쪽 0.80 m < 안전 거리 1.0 m (학생)
#     (교수 시연) ros2 param set /avoid_base v_side -0.3   → 로그 '왼쪽', 기체는 오른쪽 (되돌리기 v_side 0.3)
#     (교수 시연, cruise_speed 0.3) ros2 param set /avoid_base front_deg 1.0
#                                                     → 좁은 섹터가 기체 폭을 못 봄 → [근접] (되돌리기 front_deg 30)
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
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import LaserScan

from ..px4_base import PX4Base, PX4_QOS


class State:
    IDLE    = 'IDLE'      # heartbeat 쌓고 Offboard + Arm 요청 (6주차 그대로)
    TAKEOFF = 'TAKEOFF'   # 제자리 이륙 (상대 고도 TARGET_ALT 까지 위치 명령)
    CRUISE  = 'CRUISE'    # 회피 비행: 전진 · 감속 · 정지 · 좌우 이동 · 복귀 · 도착 — 11주차 핵심
    #  ※ 정지·좌우 이동·복귀·도착은 새 상태가 아니라 CRUISE 안의 if 로 처리한다 (모드 전환 설계는 12주차)


# ================================================================
# [USER DEFINE] 미션 파라미터
# ================================================================
TARGET_ALT = 2.0     # 비행 고도 = 상대 고도 [m] (9주차와 같은 −z 한 줄로 지킴)
GOAL_EAST  = 12.0    # 목표 동쪽 위치 [m] (obstacle_world 의 목표 표시)
MAX_CRUISE = 0.5     # cruise_speed 가 받는 최대값 [m/s]

# 파라미터 (기본값, 받는 범위, 바뀌었을 때 찍는 말) — 20강 8번 표
PARAMS = {
    'cruise_speed': (0.0,  0.0,  MAX_CRUISE, '전진 속도 변경:'),        # 0 이면 제자리 (후진 없음)
    'stop_dist':    (1.5,  1.2,  3.0,        '정지 판단 거리 변경:'),
    'k_slow':       (0.33, 0.1,  10.0,       '감속 비례 상수 변경:'),
    'v_side':       (0.3,  -0.5, 0.5,        '좌우 이동 속도 변경:'),    # 음수 = 일부러 틀리기
    'front_deg':    (30.0, 1.0,  45.0,       '앞 섹터 반각 변경:'),
}

# ── [복붙 영역] 거리·각도 상수 — 복사해 쓰고 원리만 이해 ──────────
SAFE_DIST  = 1.0     # 안전 거리 [m] = 센서에서 가장 먼 기체 끝 약 0.5 + 안전 여유 0.5 (근접 · 옆 안전 거리 공통)
NEAR_DEG   = 45.0    # 근접 검사 반각 [°] — 앞 섹터보다 넓게 (기체 폭을 덮도록)
SIDE_DEG   = (30.0, 90.0)    # 옆 섹터 [°]: 왼쪽 +30~+90, 오른쪽은 부호를 바꿔 −90~−30
REAR_DEG   = (90.0, 135.0)   # 뒤 대각선 [°]: 복귀 검사에 옆 섹터와 함께 (기둥을 막 지났을 때 뒤 프로펠러가 아직 옆에 있음)
CLEAR_GAP  = 0.5     # 해제 거리 = 정지 판단 거리 + 0.5 [m] (같게 두면 정지·전진이 번갈아 나옴)
SELF_RANGE = 0.4     # 이보다 가까운 값은 자기 기체(프로펠러·프레임)로 보고 버림 [m]
TIE_BAND   = 0.3     # 좌우가 이만큼 안으로 비슷하면 동점 → 오른쪽 [m]
# ── [복붙 영역] 속도·시간 상수 ─────────────────────────────────
CREEP      = 0.1     # 감속 구간의 최소 전진 속도 [m/s]
K_RET      = 0.5     # 코스 복귀 비례 상수 [1/s]: 북 1.0 m 벗어나면 0.5 m/s (최대 |v_side|)
RET_DONE   = 0.1     # 코스에서 이만큼 안이면 복귀 끝 [m]
STEP       = 0.05    # 가속도 제한: 0.1초마다 0.05 m/s 까지만 (= 0.5 m/s², 10주차와 같음, 19강 감속도)
KZ         = 1.0     # 고도 유지 게인 [1/s] (9주차와 같은 한 줄)
MAX_VZ     = 0.5     # 수직 속도 최대값 [m/s]
MAX_REL_ALT = 5.0    # 최대 고도 제한 (상대 고도) [m]: 넘으면 더 오르지 않음
ARRIVE_TOL = 0.3     # 목표에서 이만큼 앞이면 도착 [m] (감속하며 조금 더 감)
LAT_LIMIT  = 3.0     # 지오펜스: 북 ±3 m 바깥으로는 좌우 이동하지 않음 [m]
SCAN_TIMEOUT = 0.5   # 센서 타임아웃 [s] (10주차와 같은 값, 2D LiDAR 30 Hz → 약 15개)
RECOVER_TIME = 1.0   # 신호 없음 뒤 유효한 스캔이 이만큼 이어지면 재개 [s]
LOG_PERIOD = 1.0     # 같은 행동이 이어지면 이 간격으로 한 줄 [s] (행동이 바뀌면 바로 한 줄)


def clamp(value, limit):
    """value 를 -limit ~ +limit 안으로 자르기 (포화). NaN 이면 0 — 잘못된 값이 최대 명령이 되지 않게"""
    if math.isnan(value):
        return 0.0
    return max(-limit, min(limit, value))


def as_bool(value):
    """파라미터 값 → True/False (true·1·yes 처럼 써도 받아 준다)"""
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1', 'yes', 'on')
    return bool(value)


# ── [복붙 영역] 섹터 최근접 거리 — 19강 뷰어(scan_viewer.py)와 같은 함수 ─────
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
    """'앞 5.38 · 왼쪽 자유 공간 · 오른쪽 3.54 m' — 단위 m 는 마지막 숫자 뒤에 한 번 (뷰어와 같은 모양)"""
    items = [('앞', front), ('왼쪽', left), ('오른쪽', right)]
    last = max((i for i, (_, v) in enumerate(items) if v is not None and math.isfinite(v)), default=None)
    parts = [f'{name} {dist_text(v)}' + (' m' if i == last else '') for i, (name, v) in enumerate(items)]
    return ' · '.join(parts)


def signed(value):
    """위치 [m] → '+1.5' (반올림한 뒤 + 0.0 — '-0.0' 대신 '+0.0' 으로 찍히게)"""
    return f'{round(value, 1) + 0.0:+.1f}'


def floor2(value):
    """거리 [m] → 소수 둘째 자리에서 내림 — '앞 1.497 m < 1.5 m' 가 '1.50 < 1.5' 로 보이지 않게"""
    return math.floor(value * 100.0 + 1e-6) / 100.0


def side_name(direction):
    """방향 부호 → 말 (-1 = 왼쪽, +1 = 오른쪽 — side 변수와 같은 약속)"""
    return '왼쪽' if direction < 0 else '오른쪽'


class AvoidBase(PX4Base):
    def __init__(self):
        super().__init__('avoid_base')

        # ── 파라미터 ─────────────────────────────────────────────
        #    fly          : False 면 드라이 런 (arm·이륙·속도 명령 없음 — 땅에서 판단과 부호 확인). 시작할 때만 읽음
        #    cruise_speed : 전진 속도 [m/s] 0~0.5. 0 이면 제자리 (1단계)
        #    stop_dist    : 정지 판단 거리 [m] 1.2~3.0 (해제 거리 = 이 값 + 0.5)
        #    k_slow       : 감속 비례 상수 [1/s] 0.1~10 (10 이면 감속 구간이 사실상 없음 — 선택 실험)
        #    v_side       : 좌우 이동 속도 [m/s] −0.5~0.5 (음수 = 일부러 틀리기)
        #    front_deg    : 앞 섹터 반각 [°] 1~45
        #    fly 를 뺀 다섯은 0.1초마다 다시 읽는다 → ros2 param set 으로 비행 중에 바꿀 수 있음
        self.param_warn_at = {}   # 파라미터 경고를 마지막으로 찍은 시각 (이름마다)
        self.declare_parameter('fly', False, ParameterDescriptor(dynamic_typing=True))
        self.fly = as_bool(self.get_parameter('fly').value)
        for name, (default, low, high, _) in PARAMS.items():
            self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
            setattr(self, name, default)
            self._read_param(name, start=True)   # -p cruise_speed:=0.5 로 시작했으면 그 값 (이상한 값이면 기본값)
        self.clear_dist = self.stop_dist + CLEAR_GAP

        # ============================================================
        # [USER DEFINE] 구독 2개 — 스캔 / 위치
        # ============================================================
        self.create_subscription(       # 브리지 → 스캔. BEST_EFFORT 로 받으면 발행 쪽 설정과 상관없이 연결된다
            LaserScan, '/lidar/scan', self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._position_callback, PX4_QOS)

        # ── 상태 변수 ────────────────────────────────────────────
        self.position   = None    # 마지막으로 받은 VehicleLocalPosition (None = 아직 못 받음)
        self.scan       = None    # 마지막 유효한 LaserScan
        self.scan_at    = None    # 그 스캔을 받은 시각 [s]
        self.scan_since = None    # 유효한 스캔이 끊기지 않고 이어진 첫 시각 [s] (재개 판정용)
        self.bad_logged = False   # '칸 정보가 이상한 스캔' 경고를 한 번만
        self.beam_logged = False  # '1빔 스캔' 안내를 한 번만
        self.lost       = False   # 신호 없음 상태인지
        self.lost_why   = ''      # 신호 없음 이유 (로그용)
        self.side       = 0       # 좌우 이동 방향: 0 = 없음(전진), -1 = 왼쪽, +1 = 오른쪽 — 처음 멈출 때 한 번만 고름
        self.arrived    = False   # 목표 도착 (그 뒤로는 호버)
        self.sec        = {}      # 이번 틱의 섹터 값 (None = 측정 없음)
        self.ret_note   = None    # 코스 복귀 상태 글자 (로그용: '복귀 보류' / '빔')
        self.last_vx = self.last_vy = 0.0   # 직전 틱에 보낸 몸체 속도 [m/s] (가속도 제한용)
        self.last_tag   = None    # 직전에 찍은 행동 태그 (바뀌면 바로 한 줄)
        self.last_log_at = -math.inf
        self.idle_ticks = 0       # IDLE 에서 흐른 틱 수 (요청 재시도용)
        self.start      = time.monotonic()
        self.state = State.IDLE

        self.create_timer(0.1, self.on_update)   # 10Hz

        mode = '비행 (fly:=true)' if self.fly else '드라이 런 (fly:=false)'
        hint = ('끝낼 때: PX4 터미널(pxh>)에서 commander land → 착지 확인 → Ctrl+C'
                if self.fly else
                '비행하지 않음 (비행: --ros-args -p fly:=true) — 모의 스캔은 브리지(터미널 3)를 끄고 ros2 topic pub')
        self.get_logger().info(
            f'회피 노드 시작 — {mode} | 전진 속도 {self.cruise_speed:g} m/s | '
            f'정지 판단 {self.stop_dist:.1f} m (해제 {self.clear_dist:.1f}) | k_slow {self.k_slow:g} | '
            f'좌우 이동 {self.v_side:g} m/s | 앞 ±{self.front_deg:g}°\n'
            f'  목표 동 {GOAL_EAST:.1f} m, 고도 {TARGET_ALT:.1f} m, 안전 거리 {SAFE_DIST:.1f} m, '
            f'센서 타임아웃 {SCAN_TIMEOUT:g} s | {hint}')

    # ── [복붙 영역] 파라미터 다시 읽기 — 숫자 검사(bool·nan·inf 거부) + 범위, 밖이면 경고 후 이전 값 ──
    #    범위 밖 값도 ros2 param set 은 'Set parameter successful' 이라고 답한다 → 결과는 이 노드 로그로 확인 (18강과 같음)
    def _read_param(self, name, start=False):
        default, low, high, text = PARAMS[name]
        old = getattr(self, name)
        keep = f'기본값 {default:g} 로 시작' if start else f'이전 값 {old:g} 그대로'
        try:
            raw = self.get_parameter(name).value
            if isinstance(raw, bool):           # true/false 는 숫자가 아님 (float(True) = 1.0 방지)
                raise TypeError
            value = float(raw)
            if not math.isfinite(value):        # 'nan'·'inf' 도 막기
                raise ValueError
        except (TypeError, ValueError):
            self._param_warn(name, f'{name} 값이 숫자가 아님 — {keep}')
            return old
        if not low <= value <= high:
            self._param_warn(name, f'{name} {value:g} 는 범위 밖 ({low:g}~{high:g}) — {keep}')
            return old
        if value != old:
            if not start:
                extra = f' (해제 {value + CLEAR_GAP:g})' if name == 'stop_dist' else ''
                self.get_logger().info(f'{text} {old:g} → {value:g}{extra}')
            setattr(self, name, value)
        if name == 'stop_dist':
            self.clear_dist = self.stop_dist + CLEAR_GAP
        return value

    def _param_warn(self, name, text):
        """파라미터 경고는 이름마다 2초에 한 번 (rclpy throttle 은 호출 자리마다 하나라 이름끼리 서로 가린다)"""
        now = time.monotonic()
        if now - self.param_warn_at.get(name, -math.inf) >= 2.0:
            self.param_warn_at[name] = now
            self.get_logger().warn(text)

    # ============================================================
    # 콜백 — 받은 값을 저장 (스캔은 칸 정보가 온전한 것만)
    # ============================================================
    def _scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        ok = (n > 0 and math.isfinite(msg.range_min) and math.isfinite(msg.range_max)
              and msg.range_max > msg.range_min)
        if n > 1:                               # 여러 칸이면 각도 정보도 있어야 칸 번호 ↔ 각도를 계산할 수 있다
            ok = ok and math.isfinite(msg.angle_min) and msg.angle_increment > 0.0
        if not ok:
            # 버린 스캔은 '받은 시각'을 갱신하지 않는다 → 계속 버려지면 0.5초 뒤 신호 없음
            if not self.bad_logged:
                self.bad_logged = True
                self.get_logger().warn(
                    '스캔 칸 정보가 이상해 버림 (ranges·range_min·range_max·angle_increment) '
                    '— 모의 스캔이면 angle_min·angle_increment·range_min·range_max 를 모두 넣기')
            return
        if n == 1 and not self.beam_logged:
            self.beam_logged = True
            self.get_logger().warn('1빔 스캔 — 앞만 측정, 좌우 측정 없음 → 막히면 정지까지만 (11주차 기체: gz_x500_lidar_2d)')
        now = time.monotonic()
        if self.scan_at is None or now - self.scan_at > SCAN_TIMEOUT:
            self.scan_since = now               # 끊겼다가 다시 이어짐 → 처음부터 셈
        self.scan, self.scan_at = msg, now

    def _position_callback(self, msg: VehicleLocalPosition):
        self.position = msg

    # ============================================================
    # [USER DEFINE] 판단 여덟 줄 — 19강 반응형 회피 규칙 (20강 6번)
    #   front · left · right = 섹터 최근접 거리 [m] (자유 공간 = inf, 측정 없음 = 0 → 막힘)
    #   돌려주는 값 = 몸체 속도 (vx 앞 +, vy 오른쪽 +) [m/s] — 가속도 제한·안전장치 전의 목표
    # ============================================================
    def decide(self, front, left, right):
        if front < self.stop_dist and self.side == 0:   # 앞 막힘 → 한 번만 고름
            self.side = -1 if left > right + TIE_BAND else 1  # -1 = 왼쪽
        if self.side != 0 and front >= self.clear_dist: # 2.0 m 넘게 빔 → 해제
            self.side = 0
        if self.side != 0:                              # 좌우 이동
            vx, vy = 0.0, self.side * self.v_side
        else:                                           # 전진 (거리 비례 감속)
            vx, vy = self.speed_limit(front), self.return_vy(left, right)
        return vx, vy

    # ── [복붙 영역] 속도 제한 — 19강 19번 한 줄: k_slow × (앞 거리 − 정지 판단 거리) → 포화 (최소 CREEP, 최대 cruise_speed) ──
    def speed_limit(self, front):
        if self.cruise_speed <= 0.0:            # 제자리 (1단계)
            return 0.0
        if math.isinf(front):                   # 앞이 자유 공간
            return self.cruise_speed
        return min(self.cruise_speed, max(CREEP, self.k_slow * (front - self.stop_dist)))

    # ── [복붙 영역] 코스 복귀 — 기둥을 지나 돌아갈 쪽의 옆과 뒤 대각선이 해제 거리 넘게 비면 코스(북 0)로 ──
    #    기수가 동쪽(스폰 방향)일 때 왼쪽 = 북쪽. 기수 방향은 current_yaw 로 확인해 부호를 정한다 (v_side 부호와 무관)
    def return_vy(self, left, right):
        self.ret_note = None
        p = self.position if self.fly else None # 드라이 런은 위치를 쓰지 않는다 (판단·부호만 확인)
        if p is None or self.cruise_speed <= 0.0:
            return 0.0
        off = p.x                               # NED x = 북쪽 [m] — 코스 선(북 0)에서 벗어난 거리
        s = math.sin(self.current_yaw)          # 기수 동쪽이면 1 (오른쪽 이동 = 남쪽)
        if abs(off) < RET_DONE or abs(s) < 0.5:
            return 0.0                          # 코스 위 / 기수가 동·서쪽이 아니면 몸체 좌우로는 북쪽을 맞출 수 없음
        direction = 1 if off * s > 0 else -1    # +1 = 오른쪽으로 가야 코스에 가까워짐
        side_val = right if direction > 0 else left
        rear = self._sec_value('rear_r' if direction > 0 else 'rear_l')
        room = min(side_val, rear)              # 옆 + 뒤 대각선 (−135~−30° 또는 +30~+135°)
        if room < self.clear_dist:
            self.ret_note = f'{side_name(direction)} {room:.2f} m < {self.clear_dist:.1f} → 복귀 보류'
            return 0.0
        self.ret_note = f'{side_name(direction)} · 뒤 대각선 {self.clear_dist:.1f} m 넘게 빔'
        return direction * min(K_RET * abs(off), abs(self.v_side))

    # ============================================================
    # 도우미
    # ============================================================
    def _sec_value(self, key):
        """섹터 값 — 측정 없음(None)은 0 (막힘)으로 본다"""
        v = self.sec.get(key)
        return 0.0 if v is None else v

    def _sectors(self, scan):
        f = self.front_deg
        self.sec = {
            'front':  sector_min(scan, -f, f),
            'left':   sector_min(scan, SIDE_DEG[0], SIDE_DEG[1]),
            'right':  sector_min(scan, -SIDE_DEG[1], -SIDE_DEG[0]),
            'near':   sector_min(scan, -NEAR_DEG, NEAR_DEG),
            'rear_l': sector_min(scan, REAR_DEG[0], REAR_DEG[1]),
            'rear_r': sector_min(scan, -REAR_DEG[1], -REAR_DEG[0]),
        }

    def _scan_live(self, now):
        return self.scan_at is not None and now - self.scan_at <= SCAN_TIMEOUT

    def _log(self, tag, text, warn=False, period=LOG_PERIOD):
        """행동 태그가 바뀌면 바로, 같으면 period(기본 1초)마다 한 줄"""
        now = time.monotonic()
        if tag == self.last_tag and now - self.last_log_at < period:
            return
        self.last_tag, self.last_log_at = tag, now
        # info·warn 을 한 줄에서 번갈아 부르면 rclpy 가 ValueError (같은 호출 자리에서 심각도가 바뀜) → 두 줄로 나눔
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def _hold_vz(self):
        """[복붙 영역] 고도 유지 — 9주차와 같은 −z 한 줄 (NED: + = 하강), 최대 고도 제한 5 m"""
        h = -self.position.z                    # 상대 고도 [m]
        vz = clamp(KZ * (h - TARGET_ALT), MAX_VZ)
        if h > MAX_REL_ALT and vz < 0.0:
            vz = 0.0
        return vz

    def _ramp(self, vx_t, vy_t):
        """[복붙 영역] 가속도 제한 — 두 축 모두 한 틱에 STEP 까지만 (정상 정지도 0.5 m/s² 로 줄어든다, 19강 표와 같은 감속도)"""
        vx = self.last_vx + clamp(vx_t - self.last_vx, STEP)
        vy = self.last_vy + clamp(vy_t - self.last_vy, STEP)
        return vx, vy

    # ============================================================
    # 회피 한 틱 (20강 핵심) — send=False 면 드라이 런 (명령 없이 판단·명령 값만 로그)
    # ============================================================
    def _avoid_step(self, send=True):
        now = time.monotonic()
        dry = '' if send else '[드라이 런] '

        # ─── ① 신호 없음: 유효한 스캔이 0.5초 없음 → 수평 정지·고도 유지 (1.0초 이어지면 재개) ───
        live = self._scan_live(now)
        if not self.lost and not live:
            self.lost = True
            self.lost_why = ('스캔을 받은 적 없음 — w11_scan_bridge(3번) 실행 확인' if self.scan_at is None
                             else f'유효한 스캔 {SCAN_TIMEOUT:g}초 없음')
        elif self.lost and live and self.scan_at - self.scan_since >= RECOVER_TIME:
            self.lost = False
            self.get_logger().info(f'스캔 신호 복구 ({RECOVER_TIME:.1f}초 연속) → 회피 비행 재개')
        if self.lost:
            if not send:
                hint = ('모의 스캔을 발행하세요 (ros2 topic pub … /lidar/scan …)' if not live
                        else f'복구 대기 {self.scan_at - self.scan_since:.1f}초 / {RECOVER_TIME:.1f}초')
                self.get_logger().warn(f'[드라이 런] 스캔 신호 없음 — {hint} → 비행 중이면 수평 정지',
                                       throttle_duration_sec=2.0)
                return
            self.last_vx = self.last_vy = 0.0   # 안전장치의 0 은 가속도 제한 없이 바로
            self.send_velocity_body(0.0, 0.0, self._hold_vz(), 0.0)
            self._log('신호 없음', f'[신호 없음] {self.lost_why} → 수평 정지·고도 유지', warn=True)
            return

        scan = self.scan
        self._sectors(scan)
        front, left, right = (self._sec_value(k) for k in ('front', 'left', 'right'))
        p = self.position if send else None     # 드라이 런은 위치를 쓰지 않는다

        # ─── ② 너무 가까움: -inf 한 칸이라도 → 수평 정지 ───
        neg = sum(1 for r in scan.ranges if r == -math.inf)
        if neg:
            if send:
                self.last_vx = self.last_vy = 0.0
                self.send_velocity_body(0.0, 0.0, self._hold_vz(), 0.0)
            self._log('너무 가까움', f'{dry}[너무 가까움] -inf {neg}칸 → 수평 정지', warn=True)
            return

        # ─── ③ 도착: 동쪽 12.0 − 0.3 m 를 넘으면 수평 정지 (가속도 제한으로 천천히) → 호버 ───
        if p is not None and not self.arrived and p.y >= GOAL_EAST - ARRIVE_TOL:
            self.arrived = True
        if self.arrived:
            vx, vy = self._ramp(0.0, 0.0)
            self.last_vx, self.last_vy = vx, vy
            self.send_velocity_body(vx, vy, self._hold_vz(), 0.0)
            self._log('도착', f'[도착] 동 {signed(p.y)} m (목표 {GOAL_EAST:.1f}) → 정지·호버 — 끝내기: pxh> commander land',
                      period=10.0)          # 호버 중에는 10초에 한 줄
            return

        # ─── ④ 판단 — [USER DEFINE] 여덟 줄 ───
        was_side = self.side
        vx_t, vy_t = self.decide(front, left, right)
        if self.side != 0 and self.cruise_speed <= 0.0:
            vy_t = 0.0                          # 전진 속도 0 = 제자리 (좌우 이동도 멈춤 — 이상하면 cruise_speed 0.0)
        if was_side == 0 and self.side != 0:
            tag = '정지'
        elif self.side != 0:
            tag = '좌우 이동'
        elif abs(vy_t) > 1e-9:
            tag = '복귀'
        elif 0.0 < vx_t < self.cruise_speed - 1e-9:
            tag = '감속'
        else:
            tag = '전진'
        released = was_side != 0 and self.side == 0

        # ─── ⑤ 가속도 제한 (드라이 런은 목표 값 그대로 보여 준다) ───
        vx, vy = self._ramp(vx_t, vy_t) if send else (vx_t, vy_t)

        # ─── ⑥ 안전장치 — 0 은 가속도 제한 없이 바로 ───
        note = None                             # 안전장치가 바꿨으면 (태그, 글자)
        near = self._sec_value('near')
        if vx > 0.0 and near < SAFE_DIST:       # 근접: 전진만 0 (좌우 이동은 허용 → 빠져나올 수 있게)
            vx = 0.0
            note = ('근접', f'[근접] 앞쪽 {near:.2f} m < 안전 거리 {SAFE_DIST:.1f} m → 전진 정지')
        move = vy_t if abs(vy_t) > 1e-9 else vy  # 지금 가려는 좌우 방향 (목표가 0 이면 아직 남은 속도의 방향)
        if abs(move) > 1e-9:                    # 옆 안전 거리: 실제로 가는 쪽 옆 섹터
            direction = -1 if move < 0 else 1   # vy 음수 = 왼쪽
            side_val = left if direction < 0 else right
            if side_val < SAFE_DIST:
                vy = 0.0
                if direction == self.side:      # 고른 쪽까지 막힘 = 막다른 곳 (19강 고전 알고리즘들의 공통 약점)
                    note = ('막다른 곳', f'[막다른 곳] {sectors_text(self.sec["front"], self.sec["left"], self.sec["right"])}'
                                         ' → 정지·호버')
                else:                           # 고른 쪽과 반대로 가는 중 → 부호가 뒤집혔을 가능성
                    hint = ' (v_side 부호 확인)' if self.side != 0 else ''
                    note = ('안전', f'[안전] {side_name(direction)} {side_val:.2f} m < 안전 거리 {SAFE_DIST:.1f} m '
                                    f'→ 좌우 이동 정지{hint}')
        if p is not None:                       # 지오펜스: 북 ±3 m 바깥쪽으로는 좌우 이동하지 않음
            n_dot = -vy * math.sin(self.current_yaw)   # 몸체 vy 가 북쪽 위치를 바꾸는 빠르기 (기수 동쪽: 왼쪽 = 북 +)
            if (p.x > LAT_LIMIT and n_dot > 0.0) or (p.x < -LAT_LIMIT and n_dot < 0.0):
                vy = 0.0
                note = ('지오펜스', f'[지오펜스] 북 {signed(p.x)} m → 바깥쪽 이동 정지')
        vx = max(vx, 0.0)                       # 사각지대: 뒤 90° 는 측정하지 않으므로 후진 없음

        # ─── ⑦ 명령 보내기 (드라이 런은 로그만) ───
        if send:
            self.last_vx, self.last_vy = vx, vy
            # PX4Base: 몸체 속도 → 기수 방향으로 NED 로 돌려 보냄 (6주차). 요 각속도 0 → 기수 그대로 — 매 틱 반드시 1번
            self.send_velocity_body(vx, vy, self._hold_vz(), 0.0)

        # ─── ⑧ 로그 — 행동이 바뀌면 바로 한 줄 ───
        vx, vy = vx + 0.0, vy + 0.0             # '-0.00' 대신 '+0.00' 으로 찍히게
        if note is not None:
            tag, text = note
            self._log(tag, dry + text + (f' | vx {vx:+.2f} · vy {vy:+.2f} m/s' if not send else ''), warn=True)
            return
        self._log(tag, self._action_text(tag, dry, front, vx_t, vy_t, vx, vy, released, p))

    def _action_text(self, tag, dry, front, vx_t, vy_t, vx, vy, released, p):
        """행동 태그마다 한 줄 — 검산할 수 있게 (20강 14번 로그)"""
        where = ''
        if p is not None:
            where = f'동 {signed(p.y)} m' + (f' · 북 {signed(p.x)} m' if abs(p.x) >= RET_DONE else '') + ' | '
        cmd = f' | vx {vx:+.2f} · vy {vy:+.2f} m/s (− = 왼쪽)' if dry else ''
        vx_t, vy_t = vx_t + 0.0, vy_t + 0.0
        ramp = ''                               # 줄에는 판단이 정한 값, 가속도 제한으로 아직 따라가는 중이면 보낸 값도
        if not dry and (abs(vx - vx_t) > 0.01 or abs(vy - vy_t) > 0.01):
            ramp = f' | 보낸 값 vx {vx:+.2f} · vy {vy:+.2f} (가속도 제한)'
        s = self.sec
        if tag == '정지':
            if dry:
                return f'{dry}[정지] 앞 {floor2(front):.2f} m < {self.stop_dist:.1f} m → {side_name(self.side)} 좌우 이동{cmd}'
            l, r = self._sec_value('left'), self._sec_value('right')
            if self.side < 0:
                why = f'왼쪽 {dist_text(s["left"])} > 오른쪽 {dist_text(s["right"])} m'
            elif r > l + TIE_BAND:
                why = f'오른쪽 {dist_text(s["right"])} > 왼쪽 {dist_text(s["left"])} m'
            else:
                why = f'왼쪽 {dist_text(s["left"])} ≈ 오른쪽 {dist_text(s["right"])} m → 동점이면 오른쪽'
            return f'[정지] 앞 {floor2(front):.2f} m < {self.stop_dist:.1f} m → {side_name(self.side)} 좌우 이동 ({why})'
        if tag == '좌우 이동':
            stay = ' (전진 속도 0 → 제자리)' if self.cruise_speed <= 0.0 else ''
            north = f' | 북 {signed(p.x)} m' if p is not None else ''
            if dry:
                return f'{dry}[좌우 이동] {side_name(self.side)} 유지 | 앞 {front:.2f} m (해제 {self.clear_dist:.1f}){cmd}{stay}'
            return (f'[좌우 이동] {side_name(self.side)} | 앞 {front:.2f} m (해제 {self.clear_dist:.1f}) | '
                    f'vy {vy_t:+.2f} m/s (− = 왼쪽){north}{stay}{ramp}')
        if tag == '복귀':
            return (f'[복귀] {self.ret_note} | {where.rstrip(" |")} → vy {vy_t:+.2f} m/s'
                    f' · 전진 {vx_t:+.2f} m/s{ramp}')
        # 전진 · 감속
        head = f'{dry}[{tag}] {where}'
        if released:                            # 방향을 푼 순간 (판단이 뒤집히지 않게 정지 판단보다 0.5 m 더 비어야 풂)
            ahead = dist_text(front) + (' m' if math.isfinite(front) else '')   # 자유 공간이면 'inf m' 대신 '자유 공간'
            head += f'앞 {ahead} ≥ 해제 {self.clear_dist:.1f} m → '
        if tag == '감속':
            body = '' if released else f'앞 {front:.2f} m | '
            body += f'전진 {vx_t:+.2f} = {self.k_slow:g} × ({front:.2f} - {self.stop_dist:.1f})'
            if vx_t <= CREEP + 1e-9:
                body += f' → 최소 {CREEP:g}'
        elif released:
            body = f'전진 {vx_t:+.2f} m/s'
        else:
            body = f'{sectors_text(s["front"], s["left"], s["right"])} | 전진 {vx_t:+.2f} m/s'
        back = f' | {self.ret_note}' if self.ret_note else ''
        return head + body + ramp + back + cmd

    # ============================================================
    # 메인 제어 루프 (10Hz) — 상태 머신
    # ============================================================
    def on_update(self):
        # 스캔이 없어도 ros2 param set 을 바로 읽고 '변경' 을 찍는다
        for name in PARAMS:
            self._read_param(name)
        if as_bool(self.get_parameter('fly').value) != self.fly:
            self.get_logger().warn(
                'fly 는 시작할 때만 읽습니다 — 바꾸려면 노드를 다시 실행 (비행 중이면 commander land 먼저)',
                throttle_duration_sec=5.0)

        # ─── 드라이 런: arm·이륙·속도 명령 없음 (PX4Base 하트비트만 나감) ─
        if not self.fly:
            self._avoid_step(send=False)
            return

        target_z = -TARGET_ALT

        # ─── IDLE: Offboard + Arm 이 확인될 때까지 1초마다 요청 (6주차 그대로) ─
        if self.state == State.IDLE:
            self.send_position(0.0, 0.0, target_z)
            self.idle_ticks += 1
            if self.is_offboard and self.is_armed:          # PX4Base 가 알려주는 상태
                self.get_logger().info('Offboard + Arm 확인 → 이륙 (TAKEOFF)')
                self.state = State.TAKEOFF
            elif self.idle_ticks % 10 == 0:
                # heartbeat 1초 뒤부터 1초마다 (VM 이 느려 PX4 준비가 늦으면 첫 요청은 거절될 수 있음)
                self.set_offboard_mode()
                self.arm()

        # ─── TAKEOFF: 제자리에서 상대 고도 TARGET_ALT 까지 (위치 명령, 10주차와 같음) ─
        elif self.state == State.TAKEOFF:
            self.send_position(0.0, 0.0, target_z)
            if self.position is not None and abs(self.position.z - target_z) < 0.2:
                self.get_logger().info('이륙 완료 → 순항 (CRUISE)')
                self.state = State.CRUISE
                self.side, self.arrived, self.lost = 0, False, False
                self.last_vx = self.last_vy = 0.0
                self.last_tag = None

        # ─── CRUISE: 회피 비행 (11주차 핵심) ─
        elif self.state == State.CRUISE:
            self._avoid_step()

        if self.state == State.CRUISE and not self.is_offboard:
            self.get_logger().warn(
                'PX4 가 Offboard 가 아님 (착륙 중이거나 비상 동작) — 이 노드의 명령은 무시됨',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = AvoidBase()
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
