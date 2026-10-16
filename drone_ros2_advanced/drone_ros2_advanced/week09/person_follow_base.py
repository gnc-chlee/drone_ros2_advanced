#!/usr/bin/env python3
# ==============================================================================
# File    : person_follow_base.py  (9주차 15·16강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-10-16
# Version : 1.0.0
#
# Description:
#   정면 카메라로 걷는 사람을 따라가는 노드 — 6주차 marker_align_base 뼈대
#
#   "화면 가운데, 같은 크기" (촬영 감독이 배우를 따라가듯)
#     IDLE → TAKEOFF → HOLD ⇄ FOLLOW
#       HOLD  : 제자리 (처음 기다릴 때 + 사람을 놓쳤을 때)
#       FOLLOW: 사람 오차로 회전·앞뒤 속도 명령 (9주차 핵심)
#
#   핵심 공식 (정면 카메라 — 같은 오차라도 6주차 하방 카메라와 뜻이 다르다):
#     ex = x_err / (화면 폭 / 2)      ← 사람이 화면 오른쪽이면 +  (-1 ~ +1)
#     es = size_err / 화면 높이        ← 멀면(작게 보이면) -, 가까우면 +
#     회전 yaw_rate = +kp_yaw × ex    (오른쪽에 있으면 오른쪽으로 돈다)
#     앞   vx_body  = -kp_fwd × es    (작게 보이면 앞으로, 크게 보이면 뒤로)
#     고도는 y_err 로 맞추지 않고 FOLLOW_ALT 를 따로 유지 (y_err 로 맞추면 사람 허리 높이까지 내려온다)
#
#   놓쳤을 때 (사람 인식 노드는 못 찾으면 아무것도 보내지 않는다):
#     마지막으로 받은 시각을 재다가 LOST_TIMEOUT(1초)이 지나면 HOLD 로
#       - 화면 가장자리에서 사라졌으면 그쪽으로 천천히 돌며 찾기 (최대 한 바퀴)
#       - 가운데에서 끊겼거나 한 바퀴 돌아도 없으면 천천히 멈춰 제자리에 선다
#     ※ PX4 는 새 명령이 끊겨도 마지막 속도로 계속 난다 → 멈춤도 명령으로 보낸다
#
#   구독 토픽:
#     /sjcu/person_error              : [x_err, y_err, size_err, box_h] px + conf(0~1)  (w08_person)
#     /camera/camera_info             : 화면 크기 (w05_camera_bridge)
#     /fmu/out/vehicle_local_position : 현재 위치·고도·머리 방향
#
#   실행 방법 (터미널 1·3·4 는 14강과 같음, 비행할 때만 터미널 2 추가):
#     터미널 1: cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=person_world make px4_sitl gz_x500_mono_cam
#     터미널 2: MicroXRCEAgent udp4 -p 8888                      ← 비행할 때만
#     터미널 3: ros2 run drone_ros2_advanced w05_camera_bridge  ← pxh> 가 뜬 뒤에!
#     터미널 4: ros2 run drone_ros2_advanced w08_person --ros-args -p conf_threshold:=0.3
#     터미널 5: 아래 중 하나
#       명령만 보기 (날지 않음, 15강) : ros2 run drone_ros2_advanced w09_follow
#       1단계 회전만 (16강)          : ros2 run drone_ros2_advanced w09_follow --ros-args -p fly:=true -p kp_fwd:=0.0
#       2단계 날면서 앞뒤 켜기        : (새 터미널) ros2 param set /person_follow_base kp_fwd 6.0
#
#   끝내기 (비행했다면 이 순서로):
#     PX4 터미널(pxh>)에서 commander land → 착지 확인 → 이 노드 Ctrl+C
#     (비행 중에 이 노드를 먼저 끄면 1초 뒤 PX4 가 'Offboard 신호 끊김' 비상 동작으로 넘어간다)
#     ※ 드론은 원점으로 돌아오지 않고 사람을 따라 자리를 옮긴다 — 한 바퀴(약 76초) 따라간 뒤 착륙 권장
#
#   일부러 틀리기 (앞뒤를 끈 채로): ros2 param set /person_follow_base kp_yaw -0.8
#     → 사람이 오른쪽인데 왼쪽으로 돈다 → 화면 밖 → 1초 뒤 찾기 → 다시 보이면 또 외면
#     되돌리기: ros2 param set /person_follow_base kp_yaw 0.8
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
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray

from ..px4_base import PX4Base, PX4_QOS


class State:
    IDLE    = 'IDLE'      # heartbeat 쌓고 Offboard + Arm 요청 (6주차 그대로)
    TAKEOFF = 'TAKEOFF'   # 제자리 이륙
    HOLD    = 'HOLD'      # 제자리: 처음 기다릴 때 + 놓쳤을 때 (찾기 → 정지)
    FOLLOW  = 'FOLLOW'    # 사람 따라가기 (9주차 핵심)


# ================================================================
# [USER DEFINE] 미션 파라미터
# ================================================================
FOLLOW_ALT   = 2.0    # 비행 고도 [m]
MAX_YAW_RATE = 0.5    # 회전 상한 [rad/s] (약 29°/s ≈ 키보드 E 두 번)
MAX_FWD      = 0.6    # 앞뒤 속도 상한 [m/s] (사람 걸음 평균 약 0.37 m/s 보다 빠르게)
LOST_TIMEOUT = 1.0    # 오차가 이 시간 동안 안 오면 "놓침" [s] (14강 발행 4.3 Hz → 약 4장)

# ── [복붙 영역] 안전 상수 — 복사해 쓰고 원리만 이해 ──────────────
KZ, MAX_VZ  = 1.0, 0.5   # 고도 유지: 속도 = KZ × 고도 차이 [1/s], 상한 [m/s]
FWD_STEP    = 0.05       # 0.1초마다 앞뒤 속도가 바뀔 수 있는 양 [m/s] — 급출발·급정거 금지 (0.5 m/s²)
LOCK_COUNT  = 3          # 연속 몇 번 보여야 따라가기 시작 (헛잡음 방지, 약 0.7초)
EDGE_EX     = 0.5        # 이보다 화면 가장자리에서 사라졌을 때만 그쪽으로 찾기 (화면 끝 = 1)
SEARCH_TURN = 2 * math.pi   # 찾기 최대 회전각 [rad] = 한 바퀴 (머리 방향 변화를 더해서 잰다 — 시뮬레이션이 느려도 한 바퀴)
SEARCH_TIME = 60.0       # 찾기 최대 시간 [s] — 회전이 안 될 때를 막는 안전 한도 (0.3 rad/s 면 한 바퀴 약 21초)
FEET_MARGIN = 3          # 박스 아래끝이 화면 아래 끝 3px 안이면 "발 잘림"
NEAR_RATIO  = 0.40       # 박스 높이가 화면 높이의 40% 를 넘으면 "너무 가까움" (약 2.5 m)
BACK_SPEED  = 0.6        # 너무 가까울 때 최소 후진 속도 [m/s] (걷는 사람은 구간 가운데에서 약 0.55 m/s 까지 빨라진다)
MAX_RADIUS  = 6.0        # 원점에서 이 반경 밖이면 더 멀어지는 쪽으로는 안 감 [m]
DIST_K      = 0.98       # 짐작 거리 ≈ DIST_K ÷ 박스 비율 [m] (로그 전용 — 제어에는 안 씀, 실측으로 보정)


def clamp(value, limit):
    """value 를 -limit ~ +limit 안으로 자르기 (상한). NaN 이면 0 — 잘못된 값이 최대 명령이 되지 않게"""
    if math.isnan(value):
        return 0.0
    return max(-limit, min(limit, value))


def turn_word(rate):
    """회전 빠르기 → 말 (PX4 는 위에서 봐서 시계 방향 = 오른쪽 회전이 +)"""
    if rate > 0.01:
        return '오른쪽'
    if rate < -0.01:
        return '왼쪽'
    return '그대로'


def as_bool(value):
    """파라미터 값 → True/False (true·1·yes 처럼 써도 받아 준다)"""
    if isinstance(value, str):
        return value.strip().lower() in ('true', '1', 'yes', 'on')
    return bool(value)


class PersonFollowBase(PX4Base):
    def __init__(self):
        super().__init__('person_follow_base')

        # ── 파라미터 ─────────────────────────────────────────────
        #    fly    : False 면 명령만 보기 (arm·이륙·속도 명령 없음 — 땅에서 부호 확인). 시작할 때만 읽음
        #    kp_yaw : 회전 게인 [rad/s per ex]   ex 0.5 → 0.4 rad/s
        #    kp_fwd : 앞뒤 게인 [m/s per es]     es -0.05 → +0.3 m/s, 0 이면 회전만 (1단계)
        #    게인 둘은 0.1초마다 다시 읽는다 → ros2 param set 으로 날면서 바꿀 수 있음
        self.fly = self._bool_param('fly', False)
        self.gains = {
            'kp_yaw': self._float_param('kp_yaw', 0.8),
            'kp_fwd': self._float_param('kp_fwd', 6.0),
        }
        self.search_rate = abs(self._float_param('search_rate', 0.3))   # 찾기 회전 [rad/s], 0 이면 찾기 끔

        # ============================================================
        # [USER DEFINE] 구독 3개 (6주차와 같은 자리)
        # ============================================================
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._position_callback, PX4_QOS)
        self.create_subscription(
            Float32MultiArray, '/sjcu/person_error', self._error_callback, 10)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._camera_info_callback, 10)

        # ── 상태 변수 ────────────────────────────────────────────
        self.position   = VehicleLocalPosition()
        self.x_err = self.y_err = self.size_err = self.box_h = self.conf = 0.0   # 마지막으로 받은 5칸
        self.last_seen  = None    # 마지막으로 오차를 받은 시각 [s]
        self.seen_count = 0       # 끊기지 않고 연속으로 받은 횟수
        self.last_ex    = 0.0     # 마지막으로 본 쪽 (찾기 방향, 오른쪽 +)
        self.image_w    = 640     # 화면 크기 — 8주차 표준 (CameraInfo 가 오면 갱신)
        self.image_h    = 480
        self.last_vx    = 0.0     # 지난 틱에 보낸 앞뒤 속도 (급출발·급정거 금지용)
        self.lost_since = None    # 놓친 시각 (None = 이륙 뒤 처음 기다리는 중)
        self.turned     = 0.0     # 놓친 뒤 돈 각도 [rad] (찾기 '한 바퀴' 판정용)
        self.stopped    = False   # HOLD 에서 정지 자리를 잡았는지
        self.search_end_logged = False   # '찾기 끝' 로그를 한 번만 찍기 위한 표시
        self.hold_n = self.hold_e = 0.0  # 정지 자리 (북, 동) [m] — 다 멈춘 순간 잡고 send_position 으로 지킴
        self.idle_ticks = 0       # IDLE 에서 지난 틱 수 (요청 재시도용)
        self.yaw_rate_meas = 0.0  # 실제 회전 빠르기 [rad/s] (명령한 회전과 비교용)
        self.state = State.IDLE

        self.create_timer(0.1, self.on_update)   # 10Hz

        mode = ('비행 (fly:=true) — 끝낼 때는 PX4 터미널에서 commander land 먼저'
                if self.fly else
                '명령만 보기 — 날지 않음 (비행: --ros-args -p fly:=true)')
        self.get_logger().info(
            f'사람 추종 시작! [{mode}]\n'
            f'  고도 {FOLLOW_ALT} m, kp_yaw={self.gains["kp_yaw"]:g}, kp_fwd={self.gains["kp_fwd"]:g}, '
            f'놓침 판단 {LOST_TIMEOUT:g} s, 찾기 {self.search_rate:g} rad/s')

    # ── [복붙 영역] 파라미터 — kp_fwd:=3 처럼 정수로 줘도, fly:=1 처럼 줘도 받아 준다 ──
    def _float_param(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        return float(self.get_parameter(name).value)

    def _bool_param(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        return as_bool(self.get_parameter(name).value)

    # ── [복붙 영역] 0.1초마다 다시 읽기 — ros2 param set 이 바로 반영된다 ──
    def _gain(self, name):
        try:
            raw = self.get_parameter(name).value
            if isinstance(raw, bool):           # true/false 는 숫자가 아님 (float(True) = 1.0 방지)
                raise TypeError
            value = float(raw)
            if not math.isfinite(value):        # 'nan'·'inf' 도 막기
                raise ValueError
        except (TypeError, ValueError):
            self.get_logger().warn(
                f'{name} 값이 숫자가 아님 — 이전 값 {self.gains[name]:g} 그대로',
                throttle_duration_sec=2.0)
            return self.gains[name]
        if value != self.gains[name]:
            self.get_logger().info(f'게인 바뀜: {name} {self.gains[name]:g} → {value:g}')
            self.gains[name] = value
        return value

    # ============================================================
    # 콜백 — 받은 값을 저장 (위치 콜백만 '실제 회전 빠르기' 계산을 덧붙임)
    # ============================================================
    def _position_callback(self, msg: VehicleLocalPosition):
        # [복붙 영역] 실제 회전 = 머리 방향 변화 — 명령한 회전과 로그로 비교 (VM 점검, 16강 관찰)
        #   timestamp 는 uXRCE-DDS 시각 맞추기로 Agent(벽시계) 쪽으로 옮겨져 오고 가끔 건너뛴다
        #   → 간격이 이상한 표본은 빠르기 계산에서 버림. 시뮬레이션이 느리면(RTF<1) 숫자가 작게 보일 수 있다
        prev = self.position
        if prev.timestamp and math.isfinite(msg.heading) and math.isfinite(prev.heading):
            d = math.atan2(math.sin(msg.heading - prev.heading),
                           math.cos(msg.heading - prev.heading))   # -π ~ +π 로 감기
            self.turned += d                                        # 찾기 중 돈 각도 (한 바퀴 판정)
            dt = (msg.timestamp - prev.timestamp) * 1e-6
            if 0.0 < dt < 0.1:
                self.yaw_rate_meas = 0.8 * self.yaw_rate_meas + 0.2 * (d / dt)
        self.position = msg

    def _error_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 5:           # 5칸이 아니면 무시 (3칸 /sjcu/error 를 잘못 연결한 경우 등)
            self.get_logger().warn('5칸이 아닌 오차 메시지 — 무시 (/sjcu/person_error 가 맞나요?)',
                                   throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        if self.last_seen is None or now - self.last_seen > LOST_TIMEOUT:
            self.seen_count = 0         # 끊겼다가 다시 보임 → 처음부터 셈
        self.x_err, self.y_err, self.size_err, self.box_h, self.conf = \
            (float(v) for v in msg.data[:5])
        self.last_ex = self.x_err / (self.image_w / 2.0)   # 마지막으로 본 쪽 (찾기용)
        self.seen_count += 1
        self.last_seen = now

    def _camera_info_callback(self, msg: CameraInfo):
        if msg.width > 0 and msg.height > 0:
            self.image_w, self.image_h = msg.width, msg.height   # 6주차와 달리 높이도 받는다

    # ============================================================
    # 도우미
    # ============================================================
    def _person_seen(self) -> bool:
        """최근 LOST_TIMEOUT 초 안에 오차를 받았으면 사람이 보이는 중"""
        if self.last_seen is None:
            return False
        return time.monotonic() - self.last_seen < LOST_TIMEOUT

    def _should_search(self) -> bool:
        """놓친 뒤 + 화면 가장자리에서 사라짐 + 찾기 켜짐 + 아직 한 바퀴 안 돎"""
        return (self.lost_since is not None
                and abs(self.last_ex) >= EDGE_EX
                and self.search_rate > 0.0
                and abs(self.turned) < SEARCH_TURN
                and time.monotonic() - self.lost_since < SEARCH_TIME)

    def _head_text(self):
        return (f'머리 {math.degrees(self.current_yaw):+.0f}° '
                f'(실제 회전 {self.yaw_rate_meas:+.2f} rad/s)')

    # ── [복붙 영역] 고도 유지 — 세 번째 비례 제어 (NED 라 위가 -) ──
    def _hold_alt_vz(self) -> float:
        return clamp(KZ * (-FOLLOW_ALT - self.position.z), MAX_VZ)

    # ── [복붙 영역] 급출발·급정거 금지 — 한 틱에 FWD_STEP 까지만 바꾼다 ──
    def _ramp(self, target: float) -> float:
        vx = self.last_vx + clamp(target - self.last_vx, FWD_STEP)
        self.last_vx = vx
        return vx

    # ── [복붙 영역] 안전장치: 너무 가까움(발 잘림) → 울타리 → 급출발 금지 ──
    def _safety(self, vx: float):
        note = ''
        h = float(self.image_h)
        # ① 너무 가까움: 발이 화면 아래로 잘리면 박스가 짧아져 "멀다"고 착각 → 더 다가가는 함정
        #    (화면 옆 끝은 원근 때문에 정상 거리에서도 발이 내려가 보여 발 잘림 검사에서 뺀다)
        bottom = h / 2.0 + self.y_err + self.box_h / 2.0      # 박스 아래끝 [px]
        feet_cut = bottom >= h - FEET_MARGIN and abs(self.x_err) < 0.3 * self.image_w
        if feet_cut or self.box_h / h > NEAR_RATIO:
            if vx > -BACK_SPEED:
                note = '너무 가까움'
            vx = min(vx, -BACK_SPEED)        # 대입이 아니라 min — P 제어가 더 세게 물러나는 중이면 그대로
            self.get_logger().warn(f'너무 가까움(발 잘림) → 뒤로 {BACK_SPEED:g} m/s 이상',
                                   throttle_duration_sec=2.0)
        else:
            # ② 울타리: 원점에서 MAX_RADIUS 밖이면 더 멀어지는 앞뒤 목표만 0
            #    단, 사람이 목표보다 가까워 물러나는 중(size_err > 0)이면 막지 않는다
            #    (③ 급정거 금지 때문에 약 0.4 m 더 나간 뒤 선다)
            n, e = self.position.x, self.position.y
            outward = (n * math.cos(self.current_yaw) + e * math.sin(self.current_yaw)) * vx > 0.0
            if math.hypot(n, e) > MAX_RADIUS and outward and self.size_err <= 0.0:
                vx, note = 0.0, '울타리'
                self.get_logger().warn(f'울타리 {MAX_RADIUS:g} m → 더 멀어지지 않게 감속',
                                       throttle_duration_sec=2.0)
        target = vx
        vx = self._ramp(target)              # ③ 급출발·급정거 금지
        if not note and abs(vx - target) > 1e-3:
            note = '급출발 금지'
        return vx, note

    # ============================================================
    # 따라가기 한 틱 — 오차 → 회전·앞뒤 (비례 제어, P 제어)
    # ============================================================
    def _follow_step(self, send: bool = True):
        ex = self.x_err / (self.image_w / 2.0)     # 사람이 오른쪽이면 +
        es = self.size_err / float(self.image_h)   # 멀면 -, 가까우면 +
        ratio = self.box_h / float(self.image_h)   # 박스 비율 (짐작 거리용)
        kp_yaw, kp_fwd = self._gain('kp_yaw'), self._gain('kp_fwd')

        # ===== [USER DEFINE] 손의 규칙표 → 두 줄 =====
        yaw_rate = clamp(+kp_yaw * ex, MAX_YAW_RATE)   # 오른쪽(+) → 오른쪽 회전(+)
        vx_body  = clamp(-kp_fwd * es, MAX_FWD)        # 작게 보임(-) → 앞(+)
        # ============================================

        text = self._command_text(ex, es, ratio, kp_yaw, kp_fwd, yaw_rate, vx_body)
        if not send:                                   # 명령만 보기: 검산 로그만 (위치·자세가 없으니 안전장치·고도는 건너뜀)
            self.get_logger().info(f'[명령만] {text} | conf {self.conf:.2f}',
                                   throttle_duration_sec=1.0)
            return

        vx_send, note = self._safety(vx_body)
        vz = self._hold_alt_vz()
        # PX4Base: 몸 기준 속도를 머리 방향만큼 돌려 NED 로 + 넷째 칸 = 회전 빠르기
        self.send_velocity_body(vx_send, 0.0, vz, yaw_rate)

        sent = f' | 보낸 앞 {vx_send:+.2f} ({note})' if note else ''
        self.get_logger().info(
            f'[FOLLOW] {self._head_text()} | {text}{sent} | '
            f'고도 {-self.position.z:.1f} m | conf {self.conf:.2f}',
            throttle_duration_sec=1.0)

    @staticmethod
    def _command_text(ex, es, ratio, kp_yaw, kp_fwd, yaw_rate, vx_body):
        """로그 한 줄 — '명령 = 게인 × 오차' 로 검산할 수 있게 (6주차 로그에 검산 칸을 붙인 형식)"""
        p_yaw, p_fwd = kp_yaw * ex + 0.0, -kp_fwd * es + 0.0   # + 0.0: '-0.00' 대신 '+0.00' 으로 찍히게
        yaw_rate, vx_body = yaw_rate + 0.0, vx_body + 0.0
        neg_kp_fwd = -kp_fwd + 0.0
        # 상한에 걸렸는지는 '크기'로만 본다 — 두 줄의 부호를 바꿔도 '상한'으로 잘못 찍히지 않게
        if abs(yaw_rate) < MAX_YAW_RATE - 1e-9:
            yaw_txt = f'회전 {yaw_rate:+.2f} rad/s = {kp_yaw:g} × {ex:+.2f}'
        else:
            yaw_txt = f'회전 {yaw_rate:+.2f} rad/s (상한, {kp_yaw:g} × {ex:+.2f} = {p_yaw:+.2f})'
        if abs(vx_body) < MAX_FWD - 1e-9:
            fwd_txt = f'앞 {vx_body:+.2f} m/s = {neg_kp_fwd:g} × {es:+.3f}'
        else:
            fwd_txt = f'앞 {vx_body:+.2f} m/s (상한, {neg_kp_fwd:g} × {es:+.3f} = {p_fwd:+.2f})'
        dist = f'짐작 거리 약 {DIST_K / ratio:.1f} m' if ratio > 0.0 else '짐작 거리 -'
        return (f'ex={ex:+.2f} → {yaw_txt} ({turn_word(yaw_rate)}) | '
                f'es={es:+.3f} → {fwd_txt} | {dist}')

    # ============================================================
    # HOLD 한 틱 — 찾기 또는 정지 (새 상태 없이 if 로)
    # ============================================================
    def _hold_step(self):
        vz = self._hold_alt_vz()

        # ① 찾기: 가장자리에서 사라졌으면 마지막 본 쪽으로 천천히 회전 (앞뒤는 천천히 0 으로)
        if self._should_search():
            rate = math.copysign(self.search_rate, self.last_ex)
            self.send_velocity_body(self._ramp(0.0), 0.0, vz, rate)
            self.get_logger().info(
                f'[HOLD] 찾는 중: {turn_word(rate)} {self.search_rate:g} rad/s | '
                f'놓친 지 {time.monotonic() - self.lost_since:.1f} s, '
                f'돈 각도 {math.degrees(abs(self.turned)):.0f}° / 360° | {self._head_text()}',
                throttle_duration_sec=1.0)
            return

        if (self.lost_since is not None and abs(self.last_ex) >= EDGE_EX
                and self.search_rate > 0.0 and not self.search_end_logged):
            self.search_end_logged = True
            if abs(self.turned) >= SEARCH_TURN:
                self.get_logger().info('한 바퀴 찾아도 없음 → 제자리 정지')
            else:
                self.get_logger().info(
                    f'찾기 {SEARCH_TIME:g}초 지남 (돈 각도 {math.degrees(abs(self.turned)):.0f}°) → 제자리 정지')

        # ② 정지: 움직이던 중이면 먼저 천천히 멈추고(급정거 금지), 다 서면 그 자리를 잡는다
        if not self.stopped:
            if abs(self.last_vx) > 1e-3:
                self.send_velocity_body(self._ramp(0.0), 0.0, vz, 0.0)
                return
            self.last_vx = 0.0
            self.hold_n, self.hold_e = self.position.x, self.position.y
            self.stopped = True
        self.send_position(self.hold_n, self.hold_e, -FOLLOW_ALT)   # 멈춤도 명령 (매 틱 보냄)
        self.get_logger().info('[HOLD] 사람 기다리는 중 (제자리)', throttle_duration_sec=2.0)

    # ============================================================
    # 메인 제어 루프 (10Hz) — 상태 머신
    # ============================================================
    def on_update(self):
        # 사람이 안 보여도 ros2 param set 을 바로 읽고 '게인 바뀜' 을 찍는다
        self._gain('kp_yaw')
        self._gain('kp_fwd')
        if as_bool(self.get_parameter('fly').value) != self.fly:
            self.get_logger().warn(
                'fly 는 시작할 때만 읽습니다 — 바꾸려면 노드를 다시 실행 (비행 중이면 commander land 먼저)',
                throttle_duration_sec=5.0)

        # ─── 명령만 보기: arm·이륙·속도 명령 없음 (PX4Base 하트비트만 나감) ─
        if not self.fly:
            if self._person_seen():
                self._follow_step(send=False)
            else:
                self.get_logger().warn(
                    '[명령만] 사람 없음 → (비행 중이라면) 1초 뒤 멈추고, 가장자리에서 사라졌으면 그쪽을 찾는다',
                    throttle_duration_sec=2.0)
            return

        target_z = -FOLLOW_ALT

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

        # ─── TAKEOFF: 제자리에서 목표 고도까지 ──────────────────
        elif self.state == State.TAKEOFF:
            self.send_position(0.0, 0.0, target_z)
            if abs(self.position.z - target_z) < 0.3:
                self.get_logger().info('이륙 완료 → 사람 기다리기 (HOLD)')
                self.state = State.HOLD
                self.lost_since, self.stopped = None, False

        # ─── HOLD: 3번 연속 보이면 따라가기, 아니면 찾기 또는 정지 ─
        elif self.state == State.HOLD:
            if self._person_seen() and self.seen_count >= LOCK_COUNT:
                self.get_logger().info(f'사람 찾음 ({LOCK_COUNT}번 연속) → 따라가기 (FOLLOW)')
                self.state = State.FOLLOW
                self.lost_since, self.stopped = None, False
                self.search_end_logged = False
                self._follow_step()
            else:
                self._hold_step()

        # ─── FOLLOW: 사람 오차로 회전·앞뒤 (9주차 핵심) ─────────
        elif self.state == State.FOLLOW:
            if not self._person_seen():
                self.state = State.HOLD
                self.lost_since, self.stopped = time.monotonic(), False
                self.turned = 0.0
                self.search_end_logged = False
                what = (f'{turn_word(self.last_ex)}으로 찾기' if self._should_search()
                        else '제자리 기다림')
                self.get_logger().info(f'사람 놓침 {LOST_TIMEOUT:.1f}초 → {what}')
                self._hold_step()
            else:
                self._follow_step()

        if self.state in (State.HOLD, State.FOLLOW) and not self.is_offboard:
            self.get_logger().warn(
                'PX4 가 Offboard 가 아님 (착륙 중이거나 비상 동작) — 이 노드의 명령은 무시됨',
                throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowBase()
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
