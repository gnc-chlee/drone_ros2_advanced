#!/usr/bin/env python3
# ==============================================================================
# File    : mission_keyboard.py  (12주차 22강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-06
# Version : 1.0.0
#
# Description:
#   수동 개입(manual override) 키보드 — 자율 비행 중 사람이 조종을 넘겨받고 돌려준다
#   PX4 로 직접 보내지 않고 미션 노드(w12_mission)에 전달만 한다 (PX4 에 명령하는 노드는 미션 하나)
#   → 2주차 keyboard_control 과 함께 켜지 않습니다 (Offboard 명령이 섞임)
#
#   키 바인딩:
#     M     : 수동으로 넘겨받기 (MANUAL)
#     G     : 자율로 돌려주기 (CRUISE)
#     L     : 착륙 (LAND)
#     W/S   : 전진/후진   A/D : 왼쪽/오른쪽   (MANUAL 에서만, 누르고 있는 동안)
#     Space : 정지
#     Ctrl+C: 종료 (미션은 계속된다. MANUAL 이었다면 0.5초 뒤 제자리)
#
#   발행 토픽:
#     /sjcu/mission_cmd : String manual · auto · land
#     /sjcu/manual_cmd  : Float32MultiArray [vx, vy] 몸체 속도 [m/s], vy + = 오른쪽 (10 Hz)
#   구독 토픽:
#     /sjcu/mission_state : String 상태 이름 (화면 표시용)
#
#   실행 방법:
#     ros2 run drone_ros2_advanced w12_keyboard
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import math
import os
import select
import sys
import termios
import threading
import time
import tty

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from .mission_base import STATE_QOS


# ================================================================
# [USER DEFINE] 키 설정
# ================================================================
KEY_SPEED = 0.5      # 이동 키 속도 [m/s]
KEY_HOLD  = 0.7      # 마지막 키 입력 뒤 이만큼 지나면 속도 0 [s]
#   터미널은 키를 뗀 순간을 알려 주지 않고 누르는 동안 글자만 반복해 보낸다.
#   첫 반복까지 약 0.5초 걸리므로 그보다 길게 잡아야 누르는 동안 끊기지 않는다

MOVE_KEYS = {'w': (KEY_SPEED, 0.0), 's': (-KEY_SPEED, 0.0), 'a': (0.0, -KEY_SPEED), 'd': (0.0, KEY_SPEED)}
CMD_KEYS  = {'m': 'manual', 'g': 'auto', 'l': 'land'}

MSG = """
미션 키보드 (w12_keyboard) — 미션 노드에 전달만 합니다
------------------------------------------------------------
  M : 수동으로 넘겨받기 (MANUAL)
  G : 자율로 돌려주기 (CRUISE)
  L : 착륙 (LAND)

  MANUAL 에서만, 누르고 있는 동안:
       W (전진)
  A (왼쪽)    D (오른쪽)
       S (후진)
  Space : 정지

종료: Ctrl+C (미션은 계속됩니다)
------------------------------------------------------------
"""


class MissionKeyboard(Node):
    def __init__(self):
        super().__init__('mission_keyboard')
        self.cmd_pub = self.create_publisher(String, '/sjcu/mission_cmd', 10)
        self.manual_pub = self.create_publisher(Float32MultiArray, '/sjcu/manual_cmd', 1)
        self.create_subscription(String, '/sjcu/mission_state', self._state_callback, STATE_QOS)

        self.state = None         # 미션 노드의 상태 (None = 아직 못 받음)
        self.vx = self.vy = 0.0   # 지금 키 속도 [m/s]
        self.key_at = -math.inf   # 마지막 이동 키를 받은 시각 [s]
        self.create_timer(0.1, self._publish_manual)   # 10Hz

    def _state_callback(self, msg: String):
        if msg.data != self.state:
            self.state = msg.data
            print(f'[상태] {self.state}', flush=True)

    def _publish_manual(self):
        if time.monotonic() - self.key_at > KEY_HOLD:
            self.vx = self.vy = 0.0
        self.manual_pub.publish(Float32MultiArray(data=[float(self.vx), float(self.vy)]))

    def process_key(self, key):
        """키 하나 → 명령 또는 속도"""
        if key in CMD_KEYS:
            self.vx = self.vy = 0.0
            self.cmd_pub.publish(String(data=CMD_KEYS[key]))
            print(f'[명령] {CMD_KEYS[key]}', flush=True)
        elif key in MOVE_KEYS:
            if self.state == 'MANUAL':
                self.vx, self.vy = MOVE_KEYS[key]
                self.key_at = time.monotonic()
            elif self.state is None:
                print('미션 노드를 기다리는 중 — w12_mission 실행 확인', flush=True)
            else:
                print(f'지금 {self.state} — 이동 키는 MANUAL 에서만 (M 을 먼저)', flush=True)
        elif key == ' ':
            self.vx = self.vy = 0.0


def read_key(fd):
    """키 입력 하나 (없으면 ''). 화살표 같은 ESC 묶음은 버린다 — 끝 글자 'A' 가 왼쪽 이동으로 읽히지 않게"""
    if not select.select([fd], [], [], 0.1)[0]:
        return ''
    data = os.read(fd, 8).decode('utf-8', 'ignore')
    if not data or data.startswith('\x1b'):
        return ''
    key = data[-1].lower()
    if ord(key) > 127:
        print('영문 입력 상태인지 확인하세요 (한/영 키)', flush=True)
        return ''
    return key


def spin(node):
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main(args=None):
    if not sys.stdin.isatty():
        print('w12_keyboard 는 터미널에서 직접 실행해야 합니다 (키 입력을 받음)')
        return
    rclpy.init(args=args)
    node = MissionKeyboard()
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    print(MSG)

    # spin 을 별도 스레드에서 실행 (키 입력 루프와 충돌 방지 — 2주차 keyboard_control 과 같음)
    spin_thread = threading.Thread(target=spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        tty.setcbreak(fd)        # 글자를 Enter 없이 바로 받음 (화면에 찍지 않음, Ctrl+C 는 그대로)
        while rclpy.ok():
            key = read_key(fd)
            if key:
                node.process_key(key)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        print('키보드 종료 — 미션 노드는 계속 동작합니다')
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)
        node.destroy_node()


if __name__ == '__main__':
    main()
