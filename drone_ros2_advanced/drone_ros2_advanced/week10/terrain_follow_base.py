#!/usr/bin/env python3
# ==============================================================================
# File    : terrain_follow_base.py  (10주차 18강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-10-22
# Version : 1.0.0
#
# Description:
#   하방 LiDAR 로 잰 대지 고도를 지키며 동쪽으로 직진하는 지형 추종 노드 — 9주차 person_follow_base 뼈대
#
#   "같은 한 줄, 높이만 교체" (9주차 고도 유지 한 줄에서 '높이'만 상대 고도 → 대지 고도로 바꾼다)
#     IDLE → TAKEOFF → CRUISE
#       CRUISE: 대지 고도 유지 + 동쪽 전진 (전진 속도 0 이면 제자리 고도 유지) — 10주차 핵심
#
#   핵심 공식 (NED 라 수직 속도 + = 하강):
#     높이 h       = 대지 고도 = LiDAR × cos(기울기) − 장착 오프셋   ← alt_source:=ned 면 h = -z (9주차와 같음)
#     높이 오차 e  = h − target_agl            ← 목표보다 높으면 +, 낮으면 −
#     수직 속도 vz = kz × e  (최대 0.5 m/s)     ← 높으면 내려가고 낮으면 올라간다
#     검산: 9주차 KZ × (−FOLLOW_ALT − z) = KZ × (상대 고도 − 목표) → 식은 같고 '높이'만 바뀌었다
#
#   고도 기준 alt_source (0.1초마다 다시 읽음):
#     lidar (기본) : 대지 고도 = LiDAR 원시값 × cos(기울기) − 장착 오프셋  (경로 ① 브리지 /lidar/down)
#     ekf          : 대지 고도 = dist_bottom − 장착 오프셋                (경로 ② PX4 추정값)
#     ned          : 상대 고도 = −z  (9주차와 같음 — 상자를 못 본다)
#     ※ 장착 오프셋 = 땅에 앉아 있을 때의 LiDAR 값 (센서가 다리 끝보다 위에 달려 있다, 17강)
#       → 평지에서는 대지 고도 = 상대 고도, 상자 위에서는 둘의 차이 = 상자 높이
#
#   신호 없음 (유효한 측정이 센서 타임아웃 0.5초 동안 없으면 — 9주차 표적 상실과 같은 원리):
#     그 순간의 상대 고도를 지키고 전진 정지 → 유효한 측정이 1.0초 이어지면 지형 추종 재개
#     ※ PX4 는 새 명령이 끊겨도 마지막 속도로 계속 난다 → 멈춤도 명령으로 보낸다
#
#   구독 토픽:
#     /lidar/down                     : LiDAR 원시값 ranges[0] (w10_lidar_bridge, sensor_msgs/LaserScan)
#     /fmu/out/vehicle_local_position : 상대 고도(−z) · 동쪽 위치(y = 코스 위치) · dist_bottom
#     /fmu/out/vehicle_attitude       : 기울기 (PX4Base 도 구독하지만 yaw 만 계산하므로 한 번 더)
#
#   준비 (한 번만): 월드를 PX4 폴더로 복사
#     cd ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/worlds
#     cp lidar_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
#
#   실행 방법:
#     0단계 드라이 런 (비행하지 않음, PX4·Agent 없어도 됨 — 브리지 터미널 3 은 끈다):
#       ros2 run drone_ros2_advanced w10_terrain_follow
#       (새 터미널) ros2 topic pub -r 10 /lidar/down sensor_msgs/msg/LaserScan \
#                     "{range_min: 0.1, range_max: 100.0, ranges: [1.18]}"
#       → 1.18 / 3.18 / 2.18 로 바꿔 가며 '상승 / 하강 / 유지' 확인, -p kz:=-1.0 로 다시 켜면 1.18 에서 '하강'
#     비행 (터미널 4 는 교수 기록용 뷰어 자리 — 학생은 생략):
#       터미널 1: cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=lidar_world make px4_sitl gz_x500_lidar_down
#       터미널 2: MicroXRCEAgent udp4 -p 8888
#       터미널 3: ros2 run drone_ros2_advanced w10_lidar_bridge      ← pxh> 가 뜬 뒤에!
#       터미널 5: 아래 순서대로
#         1단계 제자리 고도 유지 : ros2 run drone_ros2_advanced w10_terrain_follow --ros-args -p fly:=true
#                                  (전진 속도 기본 0 = 제자리)
#         목표 1 m 올리기        : ros2 param set /terrain_follow_base target_agl 3.0   (계단 응답, 되돌리기 2.0)
#         2단계 비행 중 전진 켜기 : ros2 param set /terrain_follow_base cruise_speed 0.3
#                                  (동 9 m 코스 끝에서 정지, -0.3 이면 시작점 쪽으로 되돌아옴)
#
#   끝내기 (비행했다면 이 순서로):
#     PX4 터미널(pxh>)에서 commander land → 착지 확인 → 이 노드 Ctrl+C
#     (비행 중에 이 노드를 먼저 끄면 1초 뒤 PX4 가 'Offboard 신호 끊김' 비상 동작으로 넘어간다)
#
#   일부러 틀리기:
#     ros2 param set /terrain_follow_base alt_source ned  → 상자 위 대지 고도 1 m (되돌리기: alt_source lidar)
#     브리지(터미널 3) Ctrl+C                             → 신호 없음 → 상대 고도 유지·전진 정지 → 다시 실행하면 복구
#     (교수 시연, 평지·전진 0 에서만) ros2 param set /terrain_follow_base kz -1.0   → 되돌리기 kz 1.0
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
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from sensor_msgs.msg import LaserScan

from ..px4_base import PX4Base, PX4_QOS


class State:
    IDLE    = 'IDLE'      # heartbeat 쌓고 Offboard + Arm 요청 (6주차 그대로)
    TAKEOFF = 'TAKEOFF'   # 제자리 이륙 (상대 고도 TARGET_AGL 까지 위치 명령)
    CRUISE  = 'CRUISE'    # 지형 추종: 대지 고도 유지 + 동쪽 전진 (전진 속도 0 이면 제자리) — 10주차 핵심
    #  ※ 신호 없음·코스 끝은 새 상태가 아니라 CRUISE 안의 if 로 처리한다 (모드 전환 설계는 12주차)


# ================================================================
# [USER DEFINE] 미션 파라미터
# ================================================================
TARGET_AGL    = 2.0    # 목표 대지 고도 [m] (target_agl 파라미터 기본값, 이륙 고도)
MAX_CRUISE    = 0.5    # 전진 속도 최대값 [m/s] — 넘는 값은 포화
COURSE_END    = 9.0    # 코스 동쪽 끝 [m] (상자 동 4~7 m 를 지나 2 m 더)
RANGE_TIMEOUT = 0.5    # 센서 타임아웃 [s] (LiDAR 50 Hz → 약 25개)

# ── [복붙 영역] 안전 상수 — 복사해 쓰고 원리만 이해 ──────────────
LIDAR_OFFSET = 0.18      # 장착 오프셋 [m]: 땅에 앉아 있을 때 LiDAR 값 (센서 → 다리 끝, VM 실측으로 교체)
MAX_VZ       = 0.5       # 수직 속도 최대값 [m/s] (9주차와 같음)
KZ_HOLD      = 1.0       # 신호 없음 때 상대 고도 유지 게인 [1/s] — kz 와 따로 (kz 부호 실수에 휘말리지 않게)
MIN_AGL, MIN_CLIMB = 0.8, 0.3      # 최저 고도 제한 [m]: 대지 고도가 이보다 낮으면 MIN_CLIMB [m/s] 이상 상승
MAX_REL_ALT  = 5.0       # 최대 고도 제한 (상대 고도) [m]: 넘으면 더 오르지 않음 (수직 지오펜스)
TARGET_MIN, TARGET_MAX = 1.8, 3.5  # target_agl 이 받는 범위 [m] (아래: 상자 1.0 + 최저 고도 0.8 / 위: 상자 위 상대 고도 4.5 < 5.0)
COURSE_MIN   = 0.0       # 코스 서쪽 끝 = 시작점 (동쪽 좌표) [m]
FWD_STEP     = 0.05      # 가속도 제한: 0.1초마다 0.05 m/s 까지만 (= 0.5 m/s²)
COS_MAX_TILT = 0.7071    # cos 45°: 이보다 더 기울면 측정 무효 (PX4 EKF 와 같은 기준)
RECOVER_TIME = 1.0       # 신호 없음 뒤 유효한 측정이 이만큼 이어지면 복구 [s]
SOURCES      = ('lidar', 'ned', 'ekf')   # 고도 기준 alt_source 로 고를 수 있는 것

# 파라미터를 바꿨을 때 찍는 말 (10주차 로그 문구)
CHANGE_LOG = {
    'kz':           '게인 변경: kz',
    'target_agl':   '목표 대지 고도 변경:',
    'cruise_speed': '전진 속도 변경:',
}


def clamp(value, limit):
    """value 를 -limit ~ +limit 안으로 자르기 (포화). NaN 이면 0 — 잘못된 값이 최대 명령이 되지 않게"""
    if math.isnan(value):
        return 0.0
    return max(-limit, min(limit, value))


def vz_word(vz):
    """수직 속도 → 말 (NED 라 + 가 하강, − 가 상승)"""
    if vz < -0.01:
        return '상승'
    if vz > 0.01:
        return '하강'
    return '유지'


def tilt_deg(cos_t):
    """cos(기울기) → 기울기 [°]"""
    return math.degrees(math.acos(max(-1.0, min(1.0, cos_t))))


def as_bool(value):
    """파라미터 값 → True/False (true·1·yes 처럼 써도 받아 준다)"""
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1', 'yes', 'on')
    return bool(value)


def parse_source(value):
    """alt_source 값 → 'lidar' / 'ned' / 'ekf', 쓸 수 없는 값이면 None (대문자·앞뒤 빈칸은 봐 준다)"""
    if not isinstance(value, str):
        return None
    name = value.strip().lower()
    return name if name in SOURCES else None


class TerrainFollowBase(PX4Base):
    def __init__(self):
        super().__init__('terrain_follow_base')

        # ── 파라미터 ─────────────────────────────────────────────
        #    fly          : False 면 드라이 런 (arm·이륙·속도 명령 없음 — 땅에서 명령 값의 부호 확인). 시작할 때만 읽음
        #    alt_source   : 고도 기준 — lidar (대지 고도, 기본) / ned (상대 고도, 9주차) / ekf (PX4 추정값 dist_bottom)
        #    kz           : 고도 유지 게인 [1/s]    높이 오차 -0.2 m → 수직 속도 -0.2 m/s (상승)
        #    target_agl   : 목표 대지 고도 [m]       1.8 ~ 3.5 만 받는다
        #    cruise_speed : 전진 속도 [m/s] (동쪽 +) 0 이면 제자리 고도 유지 (1단계), 음수면 서쪽(시작점 쪽)으로
        #    fly 를 뺀 넷은 0.1초마다 다시 읽는다 → ros2 param set 으로 비행 중에 바꿀 수 있음
        self.param_warn_at = {}   # 파라미터 경고를 마지막으로 찍은 시각 (이름마다)
        self.fly = self._bool_param('fly', False)
        self.gains = {
            'kz':           self._float_param('kz', 1.0),
            'target_agl':   self._float_param('target_agl', TARGET_AGL),
            'cruise_speed': self._float_param('cruise_speed', 0.0),
        }
        if not TARGET_MIN <= self.gains['target_agl'] <= TARGET_MAX:
            self.get_logger().warn(
                f'target_agl {self.gains["target_agl"]:g} 는 범위 밖 ({TARGET_MIN:g}~{TARGET_MAX:g}) '
                f'— 기본값 {TARGET_AGL:g} 로 시작')
            self.gains['target_agl'] = TARGET_AGL
        self.declare_parameter('alt_source', 'lidar', ParameterDescriptor(dynamic_typing=True))
        raw = self.get_parameter('alt_source').value
        self.source = parse_source(raw)
        if self.source is None:
            self.get_logger().warn(f'alt_source 값 {raw!r} — lidar·ned·ekf 중 하나가 아님, lidar 로 시작')
            self.source = 'lidar'

        # ============================================================
        # [USER DEFINE] 구독 3개 (6·9주차와 같은 자리)
        # ============================================================
        self.create_subscription(       # 브리지 → 원시값. BEST_EFFORT 로 받으면 발행 쪽 설정과 상관없이 연결된다
            LaserScan, '/lidar/down', self._scan_callback, qos_profile_sensor_data)
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._position_callback, PX4_QOS)
        self.create_subscription(       # PX4Base 도 같은 토픽을 받지만 yaw 만 계산 → 기울기용으로 한 번 더
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._tilt_callback, PX4_QOS)

        # ── 상태 변수 ────────────────────────────────────────────
        self.position    = None    # 마지막으로 받은 VehicleLocalPosition (None = 아직 못 받음)
        self.cos_tilt    = None    # cos(기울기) (None = 자세를 아직 못 받음 → 기울기 0 으로 봄)
        self.tilt_warned = False   # '자세 없음' 경고를 한 번만 찍기 위한 표시
        self.r = 0.0               # 마지막 유효한 LiDAR 원시값 [m]
        self.r_cos = 1.0           # 그 측정을 받을 때의 cos(기울기)
        self.scan_ok_at = None     # 마지막 유효한 LiDAR 측정을 받은 시각 [s]
        self.scan_since = None     # 유효한 LiDAR 측정이 끊기지 않고 이어진 첫 시각 [s] (복구 판정용)
        self.invalid_why = None    # 마지막으로 버린 LiDAR 측정의 이유 (로그는 on_update 에서)
        self.db = 0.0              # 마지막 유효한 dist_bottom [m] (PX4 숫자 그대로, 장착 오프셋 포함)
        self.db_ok_at = None       # 그 값을 받은 시각 [s]
        self.db_since = None       # 유효한 dist_bottom 이 끊기지 않고 이어진 첫 시각 [s]
        self.db_reset = None       # dist_bottom_reset_counter (바뀌면 로그 — VM 점검용)
        self.lost = False          # 신호 없음 상태인지
        self.hold_rel = None       # 신호 없음 때 지킬 상대 고도 [m] (끊긴 순간의 −z)
        self.min_agl_off = False   # '최저 고도 제한 꺼짐' 경고를 한 번만 찍기 위한 표시
        self.last_v_east = 0.0     # 직전 틱에 보낸 동쪽 속도 [m/s] (가속도 제한용)
        self.end_logged = None     # '코스 끝/시작점' 로그를 같은 끝에서 한 번만 ('end' / 'start')
        self.idle_ticks = 0        # IDLE 에서 흐른 틱 수 (요청 재시도용)
        self.state = State.IDLE

        self.create_timer(0.1, self.on_update)   # 10Hz

        mode = '비행 (fly:=true)' if self.fly else '드라이 런 (fly:=false)'
        hint = ('끝낼 때: PX4 터미널(pxh>)에서 commander land → 착지 확인 → Ctrl+C'
                if self.fly else
                '비행하지 않음 (비행: --ros-args -p fly:=true) — 모의 측정값은 브리지(터미널 3)를 끄고 ros2 topic pub')
        self.get_logger().info(
            f'지형 추종 노드 시작 — {mode} | 고도 기준 {self.source} | '
            f'목표 대지 고도 {self.gains["target_agl"]:.1f} m | kz {self.gains["kz"]:.1f} | '
            f'전진 속도 {self.gains["cruise_speed"]:.1f} m/s\n'
            f'  장착 오프셋 {LIDAR_OFFSET:.2f} m, 센서 타임아웃 {RANGE_TIMEOUT:.1f} s | {hint}')

    # ── [복붙 영역] 파라미터 — kz:=1 처럼 정수로 줘도, fly:=1 처럼 줘도 받아 준다 ──
    def _float_param(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        value = self.get_parameter(name).value
        try:
            if isinstance(value, bool):         # true/false 는 숫자가 아님
                raise TypeError
            value = float(value)
            if not math.isfinite(value):        # 'nan'·'inf' 도 막기
                raise ValueError
        except (TypeError, ValueError):
            self.get_logger().warn(f'{name} 값이 숫자가 아님 — 기본값 {default:g} 로 시작')
            return float(default)
        return value

    def _bool_param(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        return as_bool(self.get_parameter(name).value)

    # ── [복붙 영역] 0.1초마다 다시 읽기 — ros2 param set 이 바로 반영된다 ──
    def _gain(self, name, low=-math.inf, high=math.inf):
        try:
            raw = self.get_parameter(name).value
            if isinstance(raw, bool):           # true/false 는 숫자가 아님 (float(True) = 1.0 방지)
                raise TypeError
            value = float(raw)
            if not math.isfinite(value):        # 'nan'·'inf' 도 막기
                raise ValueError
        except (TypeError, ValueError):
            self._param_warn(name, f'{name} 값이 숫자가 아님 — 이전 값 {self.gains[name]:g} 그대로')
            return self.gains[name]
        if not low <= value <= high:            # 범위 검사 (target_agl)
            self._param_warn(
                name, f'{name} {value:g} 는 범위 밖 ({low:g}~{high:g}) — 이전 값 {self.gains[name]:g} 그대로')
            return self.gains[name]
        if value != self.gains[name]:
            extra = ''
            if name == 'cruise_speed' and abs(value) > MAX_CRUISE:
                extra = f' (최대 {MAX_CRUISE:g} m/s 로 포화)'
            self.get_logger().info(f'{CHANGE_LOG[name]} {self.gains[name]:g} → {value:g}{extra}')
            self.gains[name] = value
        return value

    def _param_warn(self, name, text):
        """파라미터 경고는 이름마다 2초에 한 번 (rclpy throttle 은 호출 자리마다 하나라 이름끼리 서로 가린다)"""
        now = time.monotonic()
        if now - self.param_warn_at.get(name, -math.inf) >= 2.0:
            self.param_warn_at[name] = now
            self.get_logger().warn(text)

    # ── [복붙 영역] target_agl: 1.8~3.5 m 밖이면 경고 후 이전 값 ──
    #    아래 끝: 상자 1.0 m + 최저 고도 제한 0.8 m (더 낮으면 상자 모서리에 다리가 걸린다)
    #    위 끝  : 상자 위 상대 고도 4.5 m 가 최대 고도 제한 5 m 아래
    def _target(self):
        return self._gain('target_agl', TARGET_MIN, TARGET_MAX)

    # ── [복붙 영역] alt_source: lidar·ned·ekf 밖이면 경고 후 이전 값, 바뀌면 '고도 기준 변경' ──
    def _source(self):
        raw = self.get_parameter('alt_source').value
        name = parse_source(raw)
        if name is None:
            self.get_logger().warn(
                f'alt_source 값 {raw!r} — lidar·ned·ekf 중 하나가 아님, 이전 값 {self.source} 그대로',
                throttle_duration_sec=2.0)
            return self.source
        if name != self.source:
            self.get_logger().info(f'고도 기준 변경: {self.source} → {name}')
            self.source = name
            if name == 'ned':                   # 상대 고도는 늘 있으니 신호 없음도 끝 (바로 정상 제어)
                self.lost, self.hold_rel = False, None
        return self.source

    # ============================================================
    # 콜백 — 받은 값을 저장 (LiDAR 는 유효성 검사를 통과한 것만)
    # ============================================================
    def _scan_callback(self, msg: LaserScan):
        if not msg.ranges:                      # 빈 메시지는 무시
            return
        r = float(msg.ranges[0])                # 빔이 1개라 첫 칸만 쓴다 (경사 거리)
        cos_t = self._cos_tilt()
        why = self._invalid_reason(r, float(msg.range_min), float(msg.range_max), cos_t)
        if why:
            # 버린 측정은 '받은 시각'을 갱신하지 않는다 → 계속 버려지면 0.5초 뒤 신호 없음
            self.invalid_why = why
            return
        now = time.monotonic()
        if self.scan_ok_at is None or now - self.scan_ok_at > RANGE_TIMEOUT:
            self.scan_since = now               # 끊겼다가 다시 이어짐 → 처음부터 셈
        self.r, self.r_cos, self.scan_ok_at = r, cos_t, now

    def _position_callback(self, msg: VehicleLocalPosition):
        now = time.monotonic()
        self.position = msg
        # 경로 ② dist_bottom: 유효 플래그 + 거리 센서(bitfield 1) 일 때만 믿는다
        #   (거리 센서가 없는 기체도 숫자는 나온다 → 숫자만 보면 안 된다, 17강)
        if msg.dist_bottom_valid and (msg.dist_bottom_sensor_bitfield & 1) and math.isfinite(msg.dist_bottom):
            if self.db_ok_at is None or now - self.db_ok_at > RANGE_TIMEOUT:
                self.db_since = now
            self.db, self.db_ok_at = float(msg.dist_bottom), now
        # [VM 점검용] PX4 가 지형 추정을 측정값으로 다시 맞추면 카운터가 오른다 (상자 모서리 등)
        if self.db_reset is not None and msg.dist_bottom_reset_counter != self.db_reset:
            self.get_logger().info(
                f'dist_bottom 재설정 (reset_counter {self.db_reset} → {msg.dist_bottom_reset_counter})')
        self.db_reset = msg.dist_bottom_reset_counter

    def _tilt_callback(self, msg: VehicleAttitude):
        # [복붙 영역] cos(기울기) = 회전행렬의 (3,3) 칸 = 1 − 2(qx² + qy²)   (q = [w, x, y, z])
        q = msg.q
        c = float(1.0 - 2.0 * (q[1] ** 2 + q[2] ** 2))   # float(): 파이썬 숫자로 (numpy 2 PC 에서도 속도 명령 메시지에 들어가게)
        if math.isfinite(c):                    # NaN 자세는 버림 (range_viewer 와 같음)
            self.cos_tilt = max(-1.0, min(1.0, c))

    # ============================================================
    # 도우미 — 측정 확인과 높이 계산
    # ============================================================
    def _cos_tilt(self) -> float:
        """cos(기울기). 자세를 아직 못 받았으면 1 (기울기 0) 로 보고 한 번만 알림 (PX4 없이 하는 드라이 런)"""
        if self.cos_tilt is None:
            if not self.tilt_warned:
                self.tilt_warned = True
                self.get_logger().warn('자세 없음 — 기울기 0 으로 봄 (PX4 없이 하는 드라이 런이면 정상)')
            return 1.0
        return self.cos_tilt

    @staticmethod
    def _invalid_reason(r, r_min, r_max, cos_t):
        """[복붙 영역] 유효성 검사 — 버릴 이유(문자열) 또는 None(믿어도 됨)
             범위 밖(inf) · 경계값(= range_min·range_max, Gazebo 는 범위 밖 값을 경계값으로 잘라 보낸다) · 45° 넘게 기욺"""
        if not (math.isfinite(r_min) and math.isfinite(r_max)) or r_max <= r_min:
            return 'range_min·range_max 가 비어 있음 — LaserScan 에 측정 범위를 넣으세요'
        if math.isinf(r):
            return 'inf: 범위 밖'
        if math.isnan(r):
            return 'nan: 숫자가 아님'
        if r < r_min or r > r_max:
            return f'{r:.2f} m: 범위 밖, 측정 범위 {r_min:.2f}~{r_max:.2f} m'
        if r == r_min:
            return f'경계값 {r:.2f} m = 최소'
        if r == r_max:
            return f'경계값 {r:.2f} m = 최대'
        if not cos_t >= COS_MAX_TILT:           # NaN 도 무효 ('<' 로는 NaN 을 못 거름)
            return f'기울기 {tilt_deg(cos_t):.0f}° — 45° 넘음'
        return None

    def _height(self, source):
        """고도 기준이 고른 '높이' [m] (위가 +) → (h, None), 쓸 수 없으면 (None, 이유)
             lidar : r × cos(기울기) − LIDAR_OFFSET   ← 마지막 유효한 LiDAR 측정 (RANGE_TIMEOUT 안)
             ekf   : dist_bottom − LIDAR_OFFSET       ← PX4 가 기울기 보정까지 한 추정값 (장착 오프셋은 빼지 않고 온다)
             ned   : −z                               ← 상대 고도 (9주차와 같음)
           0.5초 안의 짧은 끊김은 마지막 유효한 값을 그대로 쓴다"""
        if source == 'ned':
            if self.position is None:
                return None, 'PX4 위치 없음'
            return -self.position.z, None
        if source == 'lidar':
            ok_at, value, never = self.scan_ok_at, self.r * self.r_cos, 'LiDAR 측정 없음'
        else:
            ok_at, value, never = self.db_ok_at, self.db, 'dist_bottom 유효한 값 없음'
        if ok_at is None:
            return None, never
        if time.monotonic() - ok_at > RANGE_TIMEOUT:
            return None, f'{RANGE_TIMEOUT:.1f}초'
        return value - LIDAR_OFFSET, None

    def _valid_for(self, source) -> float:
        """유효한 측정이 끊기지 않고 이어진 시간 [s] = 마지막 유효 측정 시각 − 이어지기 시작한 시각 (복구 판정용)
           (지금 시각이 아니라 마지막 측정 시각으로 잰다 — 짧게 끊긴 동안은 '이어진 시간'에 넣지 않게)"""
        if source == 'lidar':
            since, ok_at = self.scan_since, self.scan_ok_at
        else:
            since, ok_at = self.db_since, self.db_ok_at
        return 0.0 if since is None or ok_at is None else ok_at - since

    def _guard_agl(self):
        """최저 고도 제한이 보는 대지 고도: LiDAR → 없으면 dist_bottom → 둘 다 없으면 None (고도 기준과 상관없이)"""
        h, _ = self._height('lidar')
        if h is None:
            h, _ = self._height('ekf')
        return h

    # ── 로그 글자 만들기 ──
    def _lidar_text(self, h):
        if h is None:
            return '대지 고도 - (LiDAR 없음)'
        return f'대지 고도 {h:.2f} m (LiDAR {self.r:.2f} m, 기울기 {tilt_deg(self.r_cos):.0f}°)'

    def _db_text(self):
        p = self.position
        if p is None:
            return 'dist_bottom -'
        ok = '유효' if (p.dist_bottom_valid and (p.dist_bottom_sensor_bitfield & 1)) else '유효하지 않음'
        return f'dist_bottom {p.dist_bottom:.2f} m ({ok})'   # PX4 숫자 그대로 (장착 오프셋 포함)

    def _height_text(self, h):
        """제어에 쓴 높이 (고도 기준에 따라 이름이 다르다)"""
        if self.source == 'lidar':
            return self._lidar_text(h)
        if self.source == 'ekf':
            return f'대지 고도 {h:.2f} m (dist_bottom {self.db:.2f} m)'
        return f'상대 고도 {h:.2f} m'

    def _tail_text(self):
        """비교용으로 함께 찍는 나머지 높이들"""
        rel = f'상대 고도 {-self.position.z:.2f} m' if self.position is not None else '상대 고도 -'
        h_lidar, _ = self._height('lidar')
        if self.source == 'lidar':
            return f'{rel} | {self._db_text()}'
        if self.source == 'ekf':
            lidar = f'LiDAR {self.r:.2f} m' if h_lidar is not None else 'LiDAR -'
            return f'{rel} | {lidar}'
        return f'{self._lidar_text(h_lidar)} | {self._db_text()}'   # ned: 상자 위에서 대지 고도가 얼마인지 본다

    def _command_text(self, h, e, kz, vz):
        """로그 한 줄 — '수직 속도 = kz × 높이 오차' 로 검산할 수 있게 (9주차 _command_text 형식)"""
        p = kz * e + 0.0                        # + 0.0: '-0.00' 대신 '+0.00' 으로 찍히게
        e, vz = e + 0.0, vz + 0.0
        # 포화됐는지는 '크기'로만 본다 — 두 줄의 부호를 바꿔도 '포화'로 잘못 찍히지 않게
        if abs(vz) < MAX_VZ - 1e-9:
            vz_txt = f'수직 속도 {vz:+.2f} m/s = {kz:g} × {e:+.2f}'
        else:
            vz_txt = f'수직 속도 {vz:+.2f} m/s (포화, {kz:g} × {e:+.2f} = {p:+.2f})'
        return (f'고도 기준 {self.source} | {self._height_text(h)} | '
                f'높이 오차 {e:+.2f} m → {vz_txt} ({vz_word(vz)})')

    # ============================================================
    # 신호 없음 판정 — 9주차 '표적 상실'의 거리 센서판 (ned 는 해당 없음)
    # ============================================================
    def _check_signal(self, h, why):
        """[복붙 영역] 센서 타임아웃 동안 유효한 측정이 없으면 신호 없음, RECOVER_TIME 동안 이어지면 복구"""
        if self.source == 'ned':
            self.lost, self.hold_rel = False, None
            return
        if not self.lost:
            if h is None:
                self.lost = True
                # 끊긴 순간의 상대 고도를 지킨다 (위치가 없으면 — PX4 없는 드라이 런 — 비워 둔다)
                self.hold_rel = -self.position.z if self.position is not None else None
                hold = (f'상대 고도 {self.hold_rel:.2f} m 유지' if self.hold_rel is not None
                        else '상대 고도 유지')
                self.get_logger().warn(f'거리 센서 신호 없음 ({why}) → {hold}, 전진 정지')
            return
        if h is not None and self._valid_for(self.source) >= RECOVER_TIME:
            self.lost, self.hold_rel = False, None
            self.get_logger().info(f'거리 센서 신호 복구 ({RECOVER_TIME:.1f}초 연속) → 지형 추종 재개')

    # ── [복붙 영역] 안전장치 ① 최저 고도 제한 ② 최대 고도 제한 → (보낼 수직 속도, 표시) ──
    def _safety_vz(self, vz: float):
        note = ''
        # ① 최저 고도 제한: 대지 고도가 MIN_AGL 보다 낮으면 무조건 MIN_CLIMB 이상 상승
        #    고도 기준과 상관없이 LiDAR 로 본다 (ned 로 날아도 지킨다, LiDAR 가 끊기면 dist_bottom 으로)
        agl = self._guard_agl()
        if agl is None:
            if not self.min_agl_off:
                self.min_agl_off = True
                self.get_logger().warn('최저 고도 제한 꺼짐 — LiDAR·dist_bottom 모두 유효한 값 없음')
        else:
            self.min_agl_off = False
            if agl < MIN_AGL:
                if vz > -MIN_CLIMB:
                    note = '최저 고도'
                vz = min(vz, -MIN_CLIMB)        # 대입이 아니라 min — P 제어가 더 세게 올라가는 중이면 그대로
                if self.is_offboard:            # commander land 중에는 PX4 가 이 명령을 무시 → '상승' 경고를 찍지 않음
                    self.get_logger().warn(
                        f'최저 고도 {MIN_AGL:g} m 아래 (대지 고도 {agl:.2f} m) → 상승 {MIN_CLIMB:.2f} m/s 이상 | '
                        f'보낸 수직 속도 {vz:+.2f}' + (f' ({note})' if note else ''),
                        throttle_duration_sec=2.0)
        # ② 최대 고도 제한 (수직 지오펜스): 상대 고도가 MAX_REL_ALT 를 넘으면 더 오르지 않음 (내려가는 건 막지 않음)
        if -self.position.z > MAX_REL_ALT:
            if vz < 0.0:
                note, vz = '최대 고도 제한', 0.0
            self.get_logger().warn(f'최대 고도 제한 {MAX_REL_ALT:.1f} m → 상승 정지',
                                   throttle_duration_sec=2.0)
        return vz, note

    # ── [복붙 영역] 안전장치 ③ 코스: 끝을 넘어 더 나가는 쪽의 전진 목표만 0 (되돌아오는 쪽은 막지 않음) ──
    def _course_limit(self, v_target: float) -> float:
        y = self.position.y                     # NED 의 y = 동쪽 [m] = 코스 위치
        if v_target > 0.0 and y >= COURSE_END:
            if self.end_logged != 'end':
                self.end_logged = 'end'
                self.get_logger().info(
                    f'코스 끝 (동 {COURSE_END:.1f} m) → 전진 정지 (되돌아가려면 cruise_speed 를 음수로)')
            return 0.0
        if v_target < 0.0 and y <= COURSE_MIN:
            if self.end_logged != 'start':
                self.end_logged = 'start'
                self.get_logger().info(
                    f'코스 시작점 (동 {COURSE_MIN:.1f} m) → 전진 정지 (다시 가려면 cruise_speed 를 양수로)')
            return 0.0
        self.end_logged = None
        return v_target

    # ── [복붙 영역] 안전장치 ④ 가속도 제한 — 한 틱에 FWD_STEP 까지만 바꾼다 ──
    def _ramp(self, target: float) -> float:
        v = self.last_v_east + clamp(target - self.last_v_east, FWD_STEP)
        self.last_v_east = v
        return v

    # ============================================================
    # 지형 추종 한 틱 — 높이 오차 → 수직 속도 (비례 제어, P 제어) + 동쪽 전진
    # ============================================================
    def _cruise_step(self, send: bool = True):
        kz, target_agl = self.gains['kz'], self.gains['target_agl']
        h, why = self._height(self.source)
        self._check_signal(h, why)              # 신호 없음 진입·복구는 send 와 상관없이 판정 (드라이 런도 로그)

        # ─── 드라이 런 + 쓸 측정이 없음: 계산하지 않고 경고만 ───
        if not send and (self.lost or h is None):
            if self.source == 'ned':
                self.get_logger().warn(
                    '[드라이 런] PX4 위치 없음 (고도 기준 ned) → 상대 고도를 알 수 없음 — Agent(터미널 2) 실행 확인',
                    throttle_duration_sec=2.0)
            else:
                reason = why or f'복구 대기 {self._valid_for(self.source):.1f}초 / {RECOVER_TIME:.1f}초'
                self.get_logger().warn(
                    f'[드라이 런] 거리 센서 신호 없음 ({reason}) → 비행 중이면 상대 고도 유지·전진 정지',
                    throttle_duration_sec=2.0)
            return

        if self.lost or h is None:
            # ─── 비행 중 신호 없음: 끊긴 순간의 상대 고도 유지 (게인 KZ_HOLD 고정), 전진 0 ───
            #     마지막 값을 계속 쓰지 않는 이유: 상자를 지나면 틀린 값이 된다
            rel = -self.position.z
            if self.hold_rel is None:           # 끊길 때 위치가 없었으면 지금 채운다
                self.hold_rel = rel
            vz = clamp(KZ_HOLD * (rel - self.hold_rel), MAX_VZ)   # 높으면 + (하강)
            v_target = 0.0
            text = None
        else:
            # ===== [USER DEFINE] 고도 유지 두 줄 — 9주차와 같은 식, 높이만 교체 =====
            e  = h - target_agl           # 높이 오차 [m]: + = 목표보다 높음
            vz = clamp(kz * e, MAX_VZ)    # NED: + = 하강 → 높으면 내려간다
            # =====================================================================
            v_target = clamp(self.gains['cruise_speed'], MAX_CRUISE)   # 동쪽 +, 넘는 값은 포화
            text = self._command_text(h, e, kz, vz)
            if not send:
                # 드라이 런: 안전장치 ①② 를 적용하기 전의 명령(포화까지 적용)으로 검산 로그만
                #   (땅에서는 최저 고도 제한이 늘 걸려 부호 실수를 가리기 때문)
                self.get_logger().info(f'[드라이 런] {text}', throttle_duration_sec=1.0)
                return

        vz_send, note = self._safety_vz(vz)                    # ① 최저 고도 제한 ② 최대 고도 제한
        v_goal = self._course_limit(v_target)                  # ③ 코스 (끝에서는 0)
        v_east = self._ramp(v_goal)                            # ④ 가속도 제한
        # PX4Base: NED 속도 (북 0, 동 v_east, 수직 vz) + 요 각속도 0 → 기수는 그대로 — 매 틱 반드시 1번
        self.send_velocity_yaw(0.0, v_east, vz_send, 0.0)

        vz_send, v_east = vz_send + 0.0, v_east + 0.0
        if text is None:
            mark = f' ({note})' if note else ''
            self.get_logger().info(
                f'[신호 없음] 상대 고도 {self.hold_rel:.2f} m 유지 | 수직 속도 {vz_send:+.2f} m/s{mark} | '
                f'전진 {v_east:+.2f} m/s → 0',
                throttle_duration_sec=1.0)
        else:
            sent = f' | 보낸 수직 속도 {vz_send:+.2f} m/s ({note})' if note else ''
            ramp = f' → {v_goal:g}' if abs(v_east - v_goal) > 1e-3 else ''   # 아직 가속·감속 중이면 목표도
            self.get_logger().info(
                f'[CRUISE] 동 {self.position.y:+.1f} m | {text}{sent} | {self._tail_text()} | '
                f'전진 {v_east:+.2f} m/s{ramp}',
                throttle_duration_sec=1.0)

    def _report_invalid(self):
        """버린 측정을 알린다 (2초에 한 번)"""
        if self.invalid_why:
            self.get_logger().warn(f'LiDAR 측정 무효 ({self.invalid_why})', throttle_duration_sec=2.0)
            self.invalid_why = None
        p = self.position
        if self.source == 'ekf' and p is not None and not (p.dist_bottom_valid and (p.dist_bottom_sensor_bitfield & 1)):
            self.get_logger().warn(
                f'dist_bottom 무효 (dist_bottom_valid {p.dist_bottom_valid}, bitfield {p.dist_bottom_sensor_bitfield}) '
                f'— LiDAR 기체(gz_x500_lidar_down)인가요?',
                throttle_duration_sec=2.0)

    # ============================================================
    # 메인 제어 루프 (10Hz) — 상태 머신
    # ============================================================
    def on_update(self):
        # 측정이 없어도 ros2 param set 을 바로 읽고 '변경' 을 찍는다
        self._gain('kz')
        self._target()
        self._gain('cruise_speed')
        self._source()
        if as_bool(self.get_parameter('fly').value) != self.fly:
            self.get_logger().warn(
                'fly 는 시작할 때만 읽습니다 — 바꾸려면 노드를 다시 실행 (비행 중이면 commander land 먼저)',
                throttle_duration_sec=5.0)
        self._report_invalid()

        # ─── 드라이 런: arm·이륙·속도 명령 없음 (PX4Base 하트비트만 나감) ─
        if not self.fly:
            self._cruise_step(send=False)
            return

        target_z = -TARGET_AGL

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

        # ─── TAKEOFF: 제자리에서 상대 고도 TARGET_AGL 까지 (위치 명령, 9주차와 같음) ─
        elif self.state == State.TAKEOFF:
            self.send_position(0.0, 0.0, target_z)
            if self.position is not None and abs(self.position.z - target_z) < 0.3:
                self.get_logger().info('이륙 완료 → 순항 (CRUISE)')
                self.state = State.CRUISE
                self.lost, self.hold_rel = False, None      # 신호 없음은 순항부터 판정 (이 고도를 지키도록)
                self.last_v_east, self.end_logged = 0.0, None

        # ─── CRUISE: 대지 고도 유지 + 동쪽 전진 (10주차 핵심) ─
        elif self.state == State.CRUISE:
            self._cruise_step()

        if self.state == State.CRUISE and not self.is_offboard:
            self.get_logger().warn(
                'PX4 가 Offboard 가 아님 (착륙 중이거나 비상 동작) — 이 노드의 명령은 무시됨',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainFollowBase()
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
