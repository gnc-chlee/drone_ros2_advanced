#!/usr/bin/env python3
# ==============================================================================
# File    : precision_land_base.py  (6주차 2강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-09-27
# Version : 1.1.0
#
# Description:
#   마커 위 정밀착륙 노드 — 1강 marker_align_base 에 "하강 + 착륙"을 더한 것
#
#     IDLE → TAKEOFF → GOTO → ALIGN → DESCEND → LAND → DONE
#                              ↑___________|  (하강 중 마커를 놓치면 다시 정렬)
#
#   1강과 달라진 곳:
#     - _align_step: ① 고도 보정 — 내려갈수록 같은 1m 가 화면에서 크게 보이므로
#                      오차에 (고도 / 정렬고도) 를 곱해 낮은 고도에서 과하게 반응하지 않게
#                    ② 하강 속도 조절 — 정렬이 흐트러질수록 천천히, 많이 벗어나면 멈춤
#                      (상태를 ALIGN 으로 뒤집지 않으므로 '정렬↔하강 반복'이 생기지 않는다)
#     - ALIGN: 정렬이 ALIGN_HOLD 초 유지되면 DESCEND 로
#     - DESCEND(새로): 정렬하며 하강 → LAND_ALT 에서 한 번 더 정렬 확인 → PX4 착륙에 맡김
#     - LAND / DONE(새로): PX4 착륙 모드 전환 확인 → 착지 감지
#       ※ PX4 착륙 모드에 들어가면 우리 명령은 무시되고 PX4 가 그 자리에서 수직으로 내려간다
#
#   구독 토픽:
#     /sjcu/error                     : [x_error, y_error, size_error] px  (w05_aruco)
#     /camera/camera_info             : 화면 크기 (w05_camera_bridge)
#     /fmu/out/vehicle_local_position : 현재 위치·고도
#
#   실행 방법 (터미널 5개):
#     터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=my_custom_world make px4_sitl gz_x500_mono_cam_down
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w05_camera_bridge
#     터미널 4: ros2 run drone_ros2_advanced w05_aruco
#     터미널 5: ros2 run drone_ros2_advanced w06_land
#
#   게인 바꿔보기:
#     ros2 run drone_ros2_advanced w06_land --ros-args -p kp:=3.0
#   PX4 내장 aruco 월드(마커가 원점)라면:
#     ros2 run drone_ros2_advanced w06_land --ros-args -p marker_n:=0.0 -p marker_e:=0.0
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
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32MultiArray

from ..px4_base import PX4Base, PX4_QOS


class State:
    IDLE    = 'IDLE'      # heartbeat 쌓고 Offboard + Arm 요청
    TAKEOFF = 'TAKEOFF'   # 제자리 이륙
    GOTO    = 'GOTO'      # 위치 명령으로 마커 근처까지 (GPS 역할)
    ALIGN   = 'ALIGN'     # 카메라 오차로 마커 바로 위에 정렬
    DESCEND = 'DESCEND'   # 정렬하며 하강 (새로)
    LAND    = 'LAND'      # PX4 착륙 모드로 넘기기 (새로)
    DONE    = 'DONE'      # 착지 감지 대기 (새로)


# ================================================================
# [USER DEFINE] 미션 파라미터
# ================================================================
ALIGN_ALT     = 4.0            # 비행·정렬 고도 [m]
GPS_OFFSET    = (-1.0, -1.0)   # GOTO 도착점을 마커에서 일부러 빗나가게 (북, 동) [m] — GPS 오차 흉내
GOTO_TOL      = 0.3            # GOTO 도착 판정 반경 [m]
MAX_SPEED     = 0.5            # 정렬 속도 상한 [m/s]
ALIGN_TOL     = 0.05           # 정렬 완료 판정: 오차가 화면 반폭의 5% 이내
LOST_TIMEOUT  = 0.5            # 이 시간 동안 오차가 안 오면 "마커 놓침" [s]
ALIGN_HOLD    = 1.0            # 정렬을 이 시간 유지하면 하강 시작 [s]         (새로)
DESCEND_SPEED = 0.3            # 정렬이 완벽할 때의 하강 속도 [m/s]            (새로)
STOP_ERR      = 0.10           # 오차가 이만큼이면 하강을 완전히 멈춤          (새로)
LAND_ALT      = 1.0            # 이 고도에서 정렬 확인 후 PX4 착륙에 맡김 [m]  (새로)


def clamp(value, limit):
    """value 를 -limit ~ +limit 안으로 자르기 (속도 상한)"""
    return max(-limit, min(limit, value))


class PrecisionLandBase(PX4Base):
    def __init__(self):
        super().__init__('precision_land_base')

        # ── 파라미터: 마커 위치(4주차 월드, NED)와 게인 ─────────
        #    my_custom_world 의 마커 <pose>5 3 …> (ENU) → NED 로는 (북 3, 동 5)
        self.marker_n = self._float_param('marker_n', 3.0)
        self.marker_e = self._float_param('marker_e', 5.0)
        self.kp       = self._float_param('kp', 1.5)   # 속도 = kp × 오차 (예: ex 0.2 → 0.3 m/s)

        # ============================================================
        # [USER DEFINE] 구독 3개
        # ============================================================
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._position_callback, PX4_QOS)
        self.create_subscription(
            Float32MultiArray, '/sjcu/error', self._error_callback, 10)
        self.create_subscription(
            CameraInfo, '/camera/camera_info', self._camera_info_callback, 10)

        # ── 상태 변수 ────────────────────────────────────────────
        self.position   = VehicleLocalPosition()
        self.x_error    = 0.0     # 마지막으로 받은 픽셀 오차
        self.y_error    = 0.0
        self.last_seen  = None    # 마지막으로 오차를 받은 시각 [s]
        self.image_w    = 1280    # 화면 폭 (첫 CameraInfo 전까지 기본값)
        self.idle_ticks = 0       # IDLE 에서 지난 틱 수 (요청 재시도용)
        self.aligned_since = None # 정렬이 시작된 시각 [s]          (새로)
        self.land_ticks    = 0    # LAND 에서 지난 틱 수 (요청 재시도용) (새로)

        self.goto_n = self.marker_n + GPS_OFFSET[0]
        self.goto_e = self.marker_e + GPS_OFFSET[1]
        self.state  = State.IDLE

        self.create_timer(0.1, self.on_update)   # 10Hz

        self.get_logger().info(
            f'정밀착륙 시작! 마커(NED)=({self.marker_n}, {self.marker_e}) '
            f'→ GOTO=({self.goto_n}, {self.goto_e}), 고도 {ALIGN_ALT}m, kp={self.kp}')

    # ── [복붙 영역] 실수 파라미터 — kp:=3 처럼 정수로 줘도 받아 준다 ──
    def _float_param(self, name, default):
        self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        return float(self.get_parameter(name).value)

    # ============================================================
    # 콜백 — 받은 값을 저장만
    # ============================================================
    def _position_callback(self, msg: VehicleLocalPosition):
        self.position = msg

    def _error_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 2:           # 형식이 다른 메시지는 무시
            return
        self.x_error   = msg.data[0]
        self.y_error   = msg.data[1]
        self.last_seen = time.monotonic()

    def _camera_info_callback(self, msg: CameraInfo):
        if msg.width > 0:
            self.image_w = msg.width

    # ============================================================
    # 도우미
    # ============================================================
    def _marker_seen(self) -> bool:
        """최근 LOST_TIMEOUT 초 안에 오차를 받았으면 마커가 보이는 중"""
        if self.last_seen is None:
            return False
        return time.monotonic() - self.last_seen < LOST_TIMEOUT

    def _align_step(self, descend_speed: float = 0.0) -> float:
        """
        카메라 오차 → 드론 몸 기준 속도 (비례 제어, P 제어)
        descend_speed: 정렬이 완벽할 때의 하강 속도 (NED 라 + 가 아래)
        반환값: 고도 보정한 오차의 큰 쪽
        """
        # ① 고도 보정 (새로): 낮을수록 같은 거리가 화면에서 크게 보이므로 줄여 준다
        #    정렬 고도(4m)에서는 1.0 → 1강과 똑같고, 1m 에서는 0.25
        altitude = max(-self.position.z, LAND_ALT)
        scale    = min(altitude / ALIGN_ALT, 1.0)

        half = self.image_w / 2.0
        ex = self.x_error / half * scale  # 마커가 오른쪽이면 +
        ey = self.y_error / half * scale  # 마커가 아래면 +
        err = max(abs(ex), abs(ey))

        vx_body = clamp(-self.kp * ey, MAX_SPEED)   # 앞(+) / 뒤(-)
        vy_body = clamp(+self.kp * ex, MAX_SPEED)   # 오른쪽(+) / 왼쪽(-)

        # ② 하강 속도 조절 (새로): 정렬이 흐트러질수록 천천히, STOP_ERR 이상이면 0
        vz = descend_speed * max(0.0, 1.0 - err / STOP_ERR)

        # PX4Base: 몸 기준 속도를 드론이 보는 방향(yaw)만큼 돌려 NED 로 보냄
        self.send_velocity_body(vx_body, vy_body, vz)

        self.get_logger().info(
            f'[{self.state}] 고도 {-self.position.z:.1f}m  머리 {math.degrees(self.current_yaw):+.0f}°  '
            f'오차 ex={ex:+.2f} ey={ey:+.2f} → 앞 {vx_body:+.2f} 오른쪽 {vy_body:+.2f} 아래 {vz:+.2f} m/s',
            throttle_duration_sec=1.0)
        return err

    # ============================================================
    # 메인 제어 루프 (10Hz) — 상태 머신
    # ============================================================
    def on_update(self):
        target_z = -ALIGN_ALT

        # ─── IDLE: Offboard + Arm 이 확인될 때까지 1초마다 요청 ─
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
                self.get_logger().info('이륙 완료 → 마커 근처로 이동 (GOTO)')
                self.state = State.GOTO

        # ─── GOTO: 위치 명령으로 마커 근처까지 (3·4주차 방식) ──
        elif self.state == State.GOTO:
            self.send_position(self.goto_n, self.goto_e, target_z)
            dist = math.hypot(self.position.x - self.goto_n,
                              self.position.y - self.goto_e)
            if dist < GOTO_TOL:
                self.get_logger().info('근처 도착 → 카메라로 정렬 시작 (ALIGN)')
                self.state = State.ALIGN

        # ─── ALIGN: 정렬 + "정렬이 1초 유지되면 하강" (바뀜) ───
        elif self.state == State.ALIGN:
            if not self._marker_seen():
                self.send_velocity_body(0.0, 0.0, 0.0)     # 놓치면 제자리에서 기다림
                self.aligned_since = None
                self.get_logger().warn('마커가 안 보임 — 제자리 대기',
                                       throttle_duration_sec=2.0)
                return
            err = self._align_step()
            if err < ALIGN_TOL:
                now = time.monotonic()
                if self.aligned_since is None:
                    self.aligned_since = now
                    self.get_logger().info(f'정렬됨 — {ALIGN_HOLD}초 유지되는지 확인')
                elif now - self.aligned_since >= ALIGN_HOLD:
                    self.get_logger().info('정렬 유지 확인 → 하강 시작 (DESCEND)')
                    self.state = State.DESCEND
            else:
                self.aligned_since = None

        # ─── DESCEND: 정렬하며 하강 (새로) ──────────────────────
        elif self.state == State.DESCEND:
            if not self._marker_seen():
                self.get_logger().warn('하강 중 마커 놓침 → 멈추고 다시 정렬 (ALIGN)')
                self.aligned_since = None
                self.state = State.ALIGN
                return
            if -self.position.z > LAND_ALT:
                # 정렬이 흐트러지면 _align_step 이 알아서 하강을 늦추거나 멈춘다
                self._align_step(descend_speed=DESCEND_SPEED)
                return
            # LAND_ALT 도착: 더 내려가지 않고 정렬만 한 번 더 확인
            err = self._align_step(descend_speed=0.0)
            if err < ALIGN_TOL:
                self.get_logger().info(f'고도 {LAND_ALT}m 에서 정렬 확인 → PX4 착륙에 맡김 (LAND)')
                self.state = State.LAND

        # ─── LAND: PX4 착륙 모드로 넘어갈 때까지 1초마다 요청 (새로) ─
        elif self.state == State.LAND:
            if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
                self.get_logger().info('PX4 착륙 모드 확인 → 착지 대기 (DONE)')
                self.state = State.DONE
                return
            self.send_velocity_body(0.0, 0.0, 0.0)          # 넘어가기 전까지 제자리
            if self.land_ticks % 10 == 0:
                self.land()              # PX4Base 제공 — 마지막 1m는 PX4 가 수직으로 내려감
            self.land_ticks += 1

        # ─── DONE: 착지 감지 (새로) ─────────────────────────────
        elif self.state == State.DONE:
            if self.is_landed:   # PX4Base 제공 — PX4 착지 감지(VehicleLandDetected)
                self.get_logger().info('===== 정밀착륙 완료! =====',
                                       throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandBase()
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
