#!/usr/bin/env python3
# ==============================================================================
# File    : keyboard_control_ab.py
# Author  : Choonghyeon Lee (gnc-chlee)
# Date    : 2026-06-08
# Version : 1.0.0
#
# Description:
#   키보드 드론 제어 노드 (Arbitrator 버전)
#   직접 PX4로 setpoint 보내지 않고
#   /sjcu/cmd, /sjcu/mode 토픽으로 Arbitrator에 전달
#
#   키 바인딩:
#     T     : 시동(Arm) + Offboard 모드 + 이륙
#     L     : 안전 착륙
#     M     : ArUco 정밀 착륙 모드 토글
#     Space : 호버링 + 강제 수동 모드 복귀
#     W/S   : 전진/후진
#     A/D   : 좌/우
#     Q/E   : 좌/우 회전
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
    QoSProfile, ReliabilityPolicy,
    HistoryPolicy, DurabilityPolicy,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)
from std_msgs.msg import String, Float32MultiArray


# ================================================================
# QoS
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
V_STEP      = 0.5
YAW_STEP    = math.radians(15)
TAKEOFF_ALT = 3.0
TIMER_HZ    = 20

MSG = """
드론 키보드 제어 (Arbitrator 버전)
------------------------------------------------------------
[시스템 제어]
    T     : 시동(Arm) + Offboard 모드 + 이륙
    L     : 안전 착륙
    M     : ArUco 정밀 착륙 모드 토글 (manual ↔ land)
    Space : 호버링 + 강제 수동 모드 복귀

[비행 컨트롤]
    W/S   : 전진/후진
    A/D   : 좌/우
    Q/E   : 좌/우 회전
    ↑/↓   : 상승/하강

종료: Ctrl+C
------------------------------------------------------------
"""


class KeyboardControlAB(Node):
    def __init__(self):
        super().__init__('keyboard_control_ab')

        # ================================================================
        # Publishers
        # ================================================================

        # Offboard heartbeat (키보드 노드가 담당)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            PX4_QOS
        )
        # VehicleCommand (arm/disarm/offboard)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            PX4_QOS
        )
        # 키보드 속도 명령 → Arbitrator
        self.cmd_pub = self.create_publisher(
            Float32MultiArray,
            '/sjcu/cmd',
            10
        )
        # 모드 전환 → Arbitrator
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
        self.current_mode = 'manual'

        # ================================================================
        # 제어 루프 타이머 (20Hz)
        # ================================================================
        self.create_timer(1.0 / TIMER_HZ, self._control_loop)

        self.get_logger().info('KeyboardControlAB 초기화 완료')

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
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # ── Offboard heartbeat (항상 발행) ───────────────────────
        offboard_msg              = OffboardControlMode()
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

        # ── /sjcu/cmd 퍼블리시 (Arbitrator로 전달) ──────────────
        # land 모드면 cmd 안 보냄 (Arbitrator가 ArUco 제어)
        if self.current_mode == 'land':
            return

        # Body frame → NED 변환
        cos_y  = math.cos(self.yaw)
        sin_y  = math.sin(self.yaw)
        ned_vx = self.vx * cos_y - self.vy * sin_y
        ned_vy = self.vx * sin_y + self.vy * cos_y

        cmd_msg      = Float32MultiArray()
        cmd_msg.data = [
            float(ned_vx),
            float(ned_vy),
            float(self.vz),
            float(self.yaw_rate),
            float(1.0 if self.is_hovering else 0.0),   # 호버링 플래그
            float(self.target_z),                       # 호버링 목표 고도
        ]
        self.cmd_pub.publish(cmd_msg)

    # ================================================================
    # VehicleCommand 헬퍼
    # ================================================================
    def _publish_vehicle_command(self, command,
                                  param1=0.0, param2=0.0, param7=0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = param1
        msg.param2           = param2
        msg.param7           = param7
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

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
    # T키 - 이륙 시퀀스
    # ================================================================
    def _arm_and_takeoff(self):
        self.get_logger().info('=== 이륙 시퀀스 시작 ===')
        while self.offboard_counter < 10:
            time.sleep(0.1)
        self.arm()
        time.sleep(1.0)
        self.set_offboard_mode()
        time.sleep(0.5)
        self.is_hovering = False
        self.vz = -1.0
        self.get_logger().info(f'이륙 중... 목표: {TAKEOFF_ALT}m')

    # ================================================================
    # 모드 퍼블리시
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

        if self.current_mode == 'manual':
            if   key == 'w': self.vx += V_STEP;         self.is_hovering = False
            elif key == 's': self.vx -= V_STEP;         self.is_hovering = False
            elif key == 'a': self.vy -= V_STEP;         self.is_hovering = False
            elif key == 'd': self.vy += V_STEP;         self.is_hovering = False
            elif key == 'q': self.yaw_rate -= YAW_STEP; self.is_hovering = False
            elif key == 'e': self.yaw_rate += YAW_STEP; self.is_hovering = False
            elif key == '\x1b[A': self.vz -= V_STEP;   self.is_hovering = False
            elif key == '\x1b[B': self.vz += V_STEP;   self.is_hovering = False

        # Space: 호버링 + 강제 manual 복귀
        if key == ' ':
            self.vx, self.vy, self.vz = 0.0, 0.0, 0.0
            self.yaw_rate  = 0.0
            self.target_z  = self.current_z
            self.is_hovering = True
            if self.current_mode == 'land':
                self._publish_mode('manual')
                print('\n[수동 모드] 강제 복귀!')
            else:
                self.get_logger().info(f'호버링 ({self.target_z:.2f}m)')

        # M: 정밀 착륙 모드 토글
        elif key == 'm':
            if self.current_mode == 'manual':
                self._publish_mode('land')
                print('\n[정밀 착륙 모드] ArUco 착륙 시작!')
            else:
                self._publish_mode('manual')
                print('\n[수동 모드] 복귀!')

        # T: 이륙
        elif key == 't':
            threading.Thread(
                target=self._arm_and_takeoff, daemon=True
            ).start()

        # L: 착륙
        elif key == 'l':
            self.vx, self.vy, self.vz = 0.0, 0.0, 0.0
            self.yaw_rate  = 0.0
            self.is_hovering = False
            if self.current_mode == 'land':
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
    node = KeyboardControlAB()
    settings = termios.tcgetattr(sys.stdin)

    print(MSG)

    threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True
    ).start()

    try:
        while True:
            key = get_key(settings)
            if not key:
                continue
            if key == '\x03':
                break
            node.process_key(key)

            if node.current_mode == 'land':
                print(f'\r[정밀 착륙] ArUco 착륙 진행 중... (Space: 수동 복귀)  ', end='')
            elif node.is_hovering:
                print(f'\r[호버링] 고도: {abs(node.current_z):.1f}m  ', end='')
            else:
                print(
                    f'\r[수동] '
                    f'VX:{node.vx:+.1f} VY:{node.vy:+.1f} '
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