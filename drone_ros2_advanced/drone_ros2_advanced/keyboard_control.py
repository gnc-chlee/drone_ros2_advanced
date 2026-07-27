#!/usr/bin/env python3
# ==============================================================================
# File    : keyboard_control.py
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-27
# Version : 2.1.0
#
# Description:
#   키보드 드론 제어 노드 (2주차 메인 실습)
#   PX4 uXRCE-DDS와 직접 통신하여 Offboard 제어 구현
#
#   keyboard_control_v2.py에서 ArUco 모드(6주차 내용)를 제거한 버전.
#   모드 전환(M키)이 포함된 확장판은 6주차 정밀착륙 실습에서 다시 만난다.
#
#   키 바인딩:
#     T     : 시동(Arm) + Offboard 모드 + 이륙
#     L     : 안전 착륙
#     Space : 호버링 (현재 고도 유지)
#     W/S   : 전진/후진
#     A/D   : 좌/우
#     Q/E   : 좌/우 회전 (Yaw)
#     ↑/↓   : 상승/하강
#     Ctrl+C: 종료
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
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
TIMER_HZ    = 20                # 제어 루프 주파수 [Hz]

MSG = """
드론 키보드 제어
------------------------------------------------------------
[시스템 제어]
    T     : 시동(Arm) + Offboard 모드 + 이륙
    L     : 안전 착륙
    Space : 현재 고도 유지 (호버링)

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


class KeyboardControl(Node):
    def __init__(self):
        super().__init__('keyboard_control')

        # ================================================================
        # Publishers - PX4로 보내는 토픽
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

        # ================================================================
        # Subscribers - PX4에서 받는 토픽
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
        self.vx          = 0.0
        self.vy          = 0.0
        self.vz          = 0.0
        self.yaw         = 0.0   # 현재 yaw [rad]
        self.yaw_rate    = 0.0   # yaw 속도 [rad/s]
        self.is_hovering = False
        self.target_z    = 0.0
        self.current_z   = 0.0
        self.offboard_counter = 0

        # ================================================================
        # 제어 루프 타이머 (20Hz)
        # Offboard heartbeat + TrajectorySetpoint 동시 발행
        # ================================================================
        self.create_timer(1.0 / TIMER_HZ, self._control_loop)

        self.get_logger().info('KeyboardControl 초기화 완료')

    # ================================================================
    # Subscriber 콜백
    # ================================================================
    def _odom_callback(self, msg: VehicleOdometry):
        """
        Odometry → yaw, 현재 고도 업데이트
        쿼터니언 q = [w, x, y, z] → yaw 변환
        """
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
        1. Offboard heartbeat 발행 (필수 - 끊기면 Offboard 해제)
        2. TrajectorySetpoint 발행
        """
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # ── 1. Offboard heartbeat ────────────────────────────────
        # PX4는 이 메시지를 10Hz 이상으로 받아야 Offboard 유지
        offboard_msg              = OffboardControlMode()
        offboard_msg.acceleration = False
        offboard_msg.attitude     = False
        offboard_msg.timestamp    = timestamp

        if self.is_hovering:
            # 호버링: position 제어 모드
            offboard_msg.position = True
            offboard_msg.velocity = False
        else:
            # 이동: velocity 제어 모드
            offboard_msg.position = False
            offboard_msg.velocity = True

        self.offboard_mode_pub.publish(offboard_msg)
        self.offboard_counter += 1

        # ── 2. TrajectorySetpoint ────────────────────────────────
        setpoint           = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.yaw       = float('nan')
        setpoint.yawspeed  = float(self.yaw_rate)

        if self.is_hovering:
            # position 제어: x,y nan (현재 위치 유지), z 고정
            setpoint.position = [float('nan'), float('nan'), float(self.target_z)]
            setpoint.velocity = [0.0, 0.0, 0.0]
            setpoint.yawspeed = 0.0
        else:
            # velocity 제어: Body Frame → NED Frame 변환
            # 드론이 바라보는 방향(yaw)을 기준으로 변환
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
        """VehicleCommand 퍼블리시"""
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
        """
        Arm → Offboard → 이륙 순차 실행
        heartbeat를 충분히 쌓은 뒤 시작해야 Offboard 진입이 된다.
        별도 스레드에서 실행해야 제어 루프가 블로킹되지 않는다.
        """
        self.get_logger().info('=== 이륙 시퀀스 시작 ===')

        # heartbeat 충분히 쌓을 때까지 대기
        while self.offboard_counter < 10:
            time.sleep(0.1)

        self.arm()
        time.sleep(1.0)
        self.set_offboard_mode()
        time.sleep(0.5)

        # 상승 시작 (원하는 고도에서 Space를 눌러 호버링)
        self.is_hovering = False
        self.vz = -1.0
        self.get_logger().info('이륙 중... 원하는 고도에서 Space를 누르세요')

    # ================================================================
    # 키 입력 처리
    # ================================================================
    def process_key(self, key: str):
        """키 입력 → 속도/모드 업데이트"""

        # ── 이동 제어 ────────────────────────────────────────────
        if   key == 'w': self.vx += V_STEP;         self.is_hovering = False
        elif key == 's': self.vx -= V_STEP;         self.is_hovering = False
        elif key == 'a': self.vy -= V_STEP;         self.is_hovering = False
        elif key == 'd': self.vy += V_STEP;         self.is_hovering = False
        elif key == 'q': self.yaw_rate -= YAW_STEP; self.is_hovering = False
        elif key == 'e': self.yaw_rate += YAW_STEP; self.is_hovering = False
        elif key == '\x1b[A': self.vz -= V_STEP;    self.is_hovering = False  # ↑ 상승
        elif key == '\x1b[B': self.vz += V_STEP;    self.is_hovering = False  # ↓ 하강

        # ── Space: 호버링 ────────────────────────────────────────
        if key == ' ':
            self.vx, self.vy, self.vz = 0.0, 0.0, 0.0
            self.yaw_rate    = 0.0
            self.target_z    = self.current_z
            self.is_hovering = True
            self.get_logger().info(f'호버링: 고도 유지 ({self.target_z:.2f}m)')

        # ── T: 이륙 (별도 스레드로 순차 실행) ────────────────────
        elif key == 't':
            threading.Thread(
                target=self._arm_and_takeoff, daemon=True
            ).start()

        # ── L: 착륙 ──────────────────────────────────────────────
        elif key == 'l':
            self.vx, self.vy, self.vz = 0.0, 0.0, 0.0
            self.yaw_rate    = 0.0
            self.is_hovering = False
            self.land()


# ================================================================
# 키 입력 헬퍼
# ================================================================
def get_key(settings):
    """단일 키 입력 읽기 (화살표 키 등 멀티바이트 처리)"""
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    if key == '\x1b':  # ESC 시퀀스 (화살표 키 등)
        key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


# ================================================================
# 메인
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControl()
    settings = termios.tcgetattr(sys.stdin)

    print(MSG)

    # spin을 별도 스레드에서 실행 (키 입력 루프와 충돌 방지)
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    )
    spin_thread.start()

    try:
        while True:
            key = get_key(settings)
            if not key:
                continue
            if key == '\x03':  # Ctrl+C
                break

            node.process_key(key)

            # 현재 상태 출력
            if node.is_hovering:
                print(
                    f"\r[호버링] 고도: {abs(node.target_z):.1f}m  ",
                    end=""
                )
            else:
                print(
                    f"\r[수동] "
                    f"VX:{node.vx:+.1f}  "
                    f"VY:{node.vy:+.1f}  "
                    f"VZ:{node.vz:+.1f}  "
                    f"Yaw:{math.degrees(node.yaw_rate):+.1f}°/s  "
                    f"고도:{abs(node.current_z):.1f}m  ",
                    end=""
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
