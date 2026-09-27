#!/usr/bin/env python3
# ==============================================================================
# File    : marker_align_base.py  (6주차 1강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-09-27
# Version : 1.1.0
#
# Description:
#   카메라로 마커 위에 스스로 정렬하는 노드 — PX4Base 첫 사용
#
#   "GPS로 근처까지, 카메라로 마지막 1m"
#     IDLE → TAKEOFF → GOTO (위치 명령으로 마커 근처까지)
#                    → ALIGN (카메라 오차로 속도 명령 → 마커 바로 위에서 호버)
#
#   핵심 공식 (하방 카메라: 화면 위 = 드론 앞, 화면 오른쪽 = 드론 오른쪽):
#     ex = x_error / (화면 폭 / 2)     ← -1 ~ +1 (마커가 오른쪽이면 +)
#     ey = y_error / (화면 폭 / 2)     ← -0.75 ~ +0.75 (마커가 아래면 +, 세로도 폭으로 나눔)
#     앞 속도      vx_body = -KP * ey  (마커가 화면 위에 있으면 앞으로)
#     오른쪽 속도  vy_body = +KP * ex  (마커가 화면 오른쪽에 있으면 오른쪽으로)
#     → send_velocity_body 가 드론이 보는 방향(yaw)만큼 돌려서 PX4(NED)로 보낸다
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
#     터미널 5: ros2 run drone_ros2_advanced w06_align
#
#   게인 바꿔보기:
#     ros2 run drone_ros2_advanced w06_align --ros-args -p kp:=3.0
#   PX4 내장 aruco 월드(마커가 원점)라면:
#     ros2 run drone_ros2_advanced w06_align --ros-args -p marker_n:=0.0 -p marker_e:=0.0
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
    IDLE    = 'IDLE'      # heartbeat 쌓고 Offboard + Arm 요청
    TAKEOFF = 'TAKEOFF'   # 제자리 이륙
    GOTO    = 'GOTO'      # 위치 명령으로 마커 근처까지 (GPS 역할)
    ALIGN   = 'ALIGN'     # 카메라 오차로 마커 바로 위에 정렬 (이번 강 핵심)


# ================================================================
# [USER DEFINE] 미션 파라미터
# ================================================================
ALIGN_ALT    = 4.0            # 비행·정렬 고도 [m]
GPS_OFFSET   = (-1.0, -1.0)   # GOTO 도착점을 마커에서 일부러 빗나가게 (북, 동) [m] — GPS 오차 흉내
GOTO_TOL     = 0.3            # GOTO 도착 판정 반경 [m]
MAX_SPEED    = 0.5            # 정렬 속도 상한 [m/s]
ALIGN_TOL    = 0.05           # 정렬 완료 판정: 오차가 화면 반폭의 5% 이내
LOST_TIMEOUT = 0.5            # 이 시간 동안 오차가 안 오면 "마커 놓침" [s]


def clamp(value, limit):
    """value 를 -limit ~ +limit 안으로 자르기 (속도 상한)"""
    return max(-limit, min(limit, value))


class MarkerAlignBase(PX4Base):
    def __init__(self):
        super().__init__('marker_align_base')

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

        self.goto_n = self.marker_n + GPS_OFFSET[0]
        self.goto_e = self.marker_e + GPS_OFFSET[1]
        self.state  = State.IDLE

        self.create_timer(0.1, self.on_update)   # 10Hz

        self.get_logger().info(
            f'마커 정렬 시작! 마커(NED)=({self.marker_n}, {self.marker_e}) '
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

    def _align_step(self, vz: float = 0.0) -> float:
        """
        카메라 오차 → 드론 몸 기준 속도 (비례 제어, P 제어)
        vz: 아래 방향 속도 (NED 라 + 가 하강). 반환값: 정규화 오차의 큰 쪽
        """
        half = self.image_w / 2.0
        ex = self.x_error / half          # 마커가 오른쪽이면 +
        ey = self.y_error / half          # 마커가 아래면 +

        vx_body = clamp(-self.kp * ey, MAX_SPEED)   # 앞(+) / 뒤(-)
        vy_body = clamp(+self.kp * ex, MAX_SPEED)   # 오른쪽(+) / 왼쪽(-)

        # PX4Base: 몸 기준 속도를 드론이 보는 방향(yaw)만큼 돌려 NED 로 보냄
        self.send_velocity_body(vx_body, vy_body, vz)

        self.get_logger().info(
            f'[{self.state}] 머리 {math.degrees(self.current_yaw):+.0f}°  '
            f'오차 ex={ex:+.2f} ey={ey:+.2f} → 앞 {vx_body:+.2f} 오른쪽 {vy_body:+.2f} m/s',
            throttle_duration_sec=1.0)
        return max(abs(ex), abs(ey))

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

        # ─── ALIGN: 카메라 오차로 마커 바로 위에 (이번 강 핵심) ─
        elif self.state == State.ALIGN:
            if not self._marker_seen():
                self.send_velocity_body(0.0, 0.0, 0.0)     # 놓치면 제자리에서 기다림
                self.get_logger().warn('마커가 안 보임 — 제자리 대기',
                                       throttle_duration_sec=2.0)
                return
            err = self._align_step()
            if err < ALIGN_TOL:
                self.get_logger().info('정렬 완료 — 마커 바로 위!',
                                       throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerAlignBase()
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
