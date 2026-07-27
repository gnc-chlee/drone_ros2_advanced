#!/usr/bin/env python3
# ==============================================================================
# File    : keyboard_control.py
# Author  : Changhyeon Lee (gnc-chlee)
# Date    : 2026-06-01
# Version : 2.0.0
#
# Description:
#   키보드 드론 제어 노드 (V2 버전 - px4_base 미사용)
#   PX4 uXRCE-DDS와 직접 통신하여 Offboard 제어 구현
#
#   모드:
#     manual : 키보드로 직접 드론 제어
#     aruco  : ArUco 자동 착륙 모드 (aruco_controller가 제어)
#
#   키 바인딩:
#     T     : 시동(Arm) + Offboard 모드 + 이륙
#     L     : 안전 착륙
#     M     : ArUco 모드 토글 (manual ↔ aruco)
#     Space : 호버링 + 강제 manual 모드 복귀
#     W/S   : 전진/후진
#     A/D   : 좌/우
#     Q/E   : 좌/우 회전 (Yaw)
#     ↑/↓   : 상승/하강
#     Ctrl+C: 종료
#
# Repository:
#   https://github.com/gnc-chlee/px4-ros2-ai-drone
#
# License : MIT
# ==============================================================================

import math
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)
from std_msgs.msg import String


# ================================================================
# QoS 설정 - PX4 uXRCE-DDS 전용
# ================================================================
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


# ================================================================
# 제어 파라미터
# ================================================================
V_STEP      = 0.5               # 속도 증가 단계 [m/s]
YAW_STEP    = math.radians(15)  # Yaw 증가 단계 [rad/s]
TAKEOFF_ALT = 3.0               # 이륙 고도 [m]
TIMER_HZ    = 20                # 제어 루프 주파수 [Hz]

MSG = """
드론 키보드 제어 (V2 버전)
------------------------------------------------------------
[시스템 제어]
    T     : 시동(Arm) + Offboard 모드 + 이륙
    L     : 안전 착륙
    M     : ArUco 자동 착륙 모드 토글 (manual ↔ aruco)
    Space : 호버링 + 강제 수동 모드 복귀

[비행 컨트롤] (기체 머리 방향 기준)
         W (전진)
    A (좌측)    D (우측)
         S (후진)

[고도 제어]
    ↑ / ↓ : 상승 / 하강

[Heading 제어]
    Q (좌회전) / E (우회전)

종료: Ctrl+C
------------------------------------------------------------
"""


class KeyboardControlV2(Node):
    def __init__(self):
        super().__init__('keyboard_control_v2')

        # ================================================================
        # Publishers
        # ================================================================
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            PX4_QOS
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            PX4_QOS
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            PX4_QOS
        )
        self.mode_pub = self.create_publisher(
            String,
            '/sjcu/mode',
            10
        )

        # ================================================================
        # Subscribers
        # ================================================================
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self._odom_callback,
            PX4_QOS
        )

        # ================================================================
        # 상태 변수
        # ================================================================
        self.vx           = 0.0
        self.vy           = 0.0
        self.vz           = 0.0
        self.yaw          = 0.0
        self.yaw_rate     = 0.0
        self.is_hovering  = False
        self.target_z     = 0.0
        self.current_z    = 0.0
        self.offboard_counter = 0
        self.current_mode = 'manual'   # 'manual' or 'aruco'

        # ================================================================
        # 제어 루프 타이머 (20Hz)
        # ================================================================
        self.create_timer(1.0 / TIMER_HZ, self._control_loop)

        self.get_logger().info('KeyboardControlV2 초기화 완료')

    # ================================================================
    # Subscriber 콜백
    # ================================================================
    def _odom_callback(self, msg: VehicleOdometry):
        q = msg.q
        self.yaw = math.atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] ** 2 + q[3] ** 2)
        )
        self.current_z = msg.position[2]

    # ================================================================
    # 제어 루프 (20Hz)
    # ================================================================
    def _control_loop(self):
        """
        20Hz 제어 루프

        aruco 모드: heartbeat만 발행 (setpoint는 aruco_controller가 담당)
        manual 모드: heartbeat + setpoint 둘 다 발행
        """
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # ── Offboard heartbeat (항상 발행) ───────────────────────
        offboard_msg           = OffboardControlMode()
        offboard_msg.acceleration = False
        offboard_msg.attitude     = False
        offboard_msg.timestamp    = timestamp

        if self.is_hovering:
            offboard_msg.position = True
            offboard_msg.velocity = False
        else:
            offboard_msg.position = False
            offboard_msg.velocity = True

        self.offboard_mode_pub.publish(offboard_msg)
        self.offboard_counter += 1

        # ── aruco 모드면 setpoint 발행 안 함 ────────────────────
        if self.current_mode == 'aruco':
            return

        # ── manual 모드: setpoint 발행 ──────────────────────────
        setpoint           = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.yaw       = float('nan')
        setpoint.yawspeed  = float(self.yaw_rate)

        if self.is_hovering:
            setpoint.position = [float('nan'), float('nan'), float(self.target_z)]
            setpoint.velocity = [0.0, 0.0, 0.0]
            setpoint.yawspeed = 0.0
        else:
            cos_y  = math.cos(self.yaw)
            sin_y  = math.sin(self.yaw)
            ned_vx = self.vx * cos_y - self.vy * sin_y
            ned_vy = self.vx * sin_y + self.vy * cos_y
            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [ned_vx, ned_vy, self.vz]

        self.trajectory_pub.publish(setpoint)

    # ================================================================
    # VehicleCommand 헬퍼
    # ================================================================
    def _publish_vehicle_command(
        self, command,
        param1=0.0, param2=0.0, param3=0.0,
        param4=0.0, param5=0.0, param6=0.0, param7=0.0
    ):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = param1
        msg.param2           = param2
        msg.param3           = param3
        msg.param4           = param4
        msg.param5           = param5
        msg.param6           = param6
        msg.param7           = param7
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    # ================================================================
    # 기본 명령
    # ================================================================
    def arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm 명령 전송')

    def set_offboard_mode(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard 모드 전환')

    def land(self):
        self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('착륙 명령 전송')

    # ================================================================
    # T키 - 이륙 시퀀스 (별도 스레드)
    # ================================================================
    def _arm_and_takeoff(self):
        """Arm → Offboard → 이륙 순차 실행"""
        self.get_logger().info('=== 이륙 시퀀스 시작 ===')

        # heartbeat 충분히 쌓을 때까지 대기
        while self.offboard_counter < 10:
            time.sleep(0.1)

        self.arm()
        time.sleep(1.0)
        self.set_offboard_mode()
        time.sleep(0.5)

        # 상승 시작
        self.is_hovering = False
        self.vz = -1.0
        self.get_logger().info(f'이륙 중... 목표: {TAKEOFF_ALT}m')

    # ================================================================
    # 모드 퍼블리시 헬퍼
    # ================================================================
    def _publish_mode(self, mode: str):
        msg      = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.current_mode = mode

    # ================================================================
    # 키 입력 처리
    # ================================================================
    def process_key(self, key: str):

        # ── 이동 제어 (manual 모드에서만) ───────────────────────
        if self.current_mode == 'manual':
            if   key == 'w': self.vx += V_STEP;         self.is_hovering = False
            elif key == 's': self.vx -= V_STEP;         self.is_hovering = False
            elif key == 'a': self.vy -= V_STEP;         self.is_hovering = False
            elif key == 'd': self.vy += V_STEP;         self.is_hovering = False
            elif key == 'q': self.yaw_rate -= YAW_STEP; self.is_hovering = False
            elif key == 'e': self.yaw_rate += YAW_STEP; self.is_hovering = False
            elif key == '\x1b[A': self.vz -= V_STEP;   self.is_hovering = False
            elif key == '\x1b[B': self.vz += V_STEP;   self.is_hovering = False

        # ── Space: 호버링 + 강제 manual 복귀 ────────────────────
        if key == ' ':
            self.vx, self.vy, self.vz = 0.0, 0.0, 0.0
            self.yaw_rate  = 0.0
            self.target_z  = self.current_z
            self.is_hovering = True

            if self.current_mode == 'aruco':
                self._publish_mode('manual')
                self.get_logger().info('Space → 수동 모드 강제 복귀 + 호버링')
                print('\n[수동 모드] 키보드 제어 복귀!')
            else:
                self.get_logger().info(f'호버링: 고도 유지 ({self.target_z:.2f}m)')

        # ── M: ArUco 모드 토글 ───────────────────────────────────
        elif key == 'm':
            if self.current_mode == 'manual':
                self._publish_mode('aruco')
                self.get_logger().info('=== ArUco 모드 전환 ===')
                print('\n[ArUco 모드] 마커 자동 착륙 시작!')
            else:
                self._publish_mode('manual')
                self.get_logger().info('=== 수동 모드 복귀 ===')
                print('\n[수동 모드] 키보드 제어 복귀!')

        # ── T: 이륙 ─────────────────────────────────────────────
        elif key == 't':
            threading.Thread(
                target=self._arm_and_takeoff, daemon=True
            ).start()

        # ── L: 착륙 ─────────────────────────────────────────────
        elif key == 'l':
            self.vx, self.vy, self.vz = 0.0, 0.0, 0.0
            self.yaw_rate  = 0.0
            self.is_hovering = False
            if self.current_mode == 'aruco':
                self._publish_mode('manual')
            self.land()


# ================================================================
# 키 입력 헬퍼
# ================================================================
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# ================================================================
# 메인
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlV2()
    settings = termios.tcgetattr(sys.stdin)

    print(MSG)

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    try:
        while True:
            key = get_key(settings)
            if not key:
                continue
            if key == '\x03':
                break

            node.process_key(key)

            # 상태 출력
            if node.current_mode == 'aruco':
                print(f'\r[ArUco 모드] 자동 착륙 진행 중... (Space: 수동 복귀)  ', end='')
            elif node.is_hovering:
                print(f'\r[호버링] 고도: {abs(node.current_z):.1f}m  ', end='')
            else:
                print(
                    f'\r[수동] '
                    f'VX:{node.vx:+.1f} '
                    f'VY:{node.vy:+.1f} '
                    f'VZ:{node.vz:+.1f} '
                    f'Yaw:{math.degrees(node.yaw_rate):+.1f}°/s '
                    f'고도:{abs(node.current_z):.1f}m  ',
                    end=''
                )

    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.get_logger().info('종료 중...')
        node.land()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()