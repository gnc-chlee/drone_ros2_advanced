#!/usr/bin/env python3
# ==============================================================================
# File    : waypoint_yaml_raw.py  (3주차 - raw 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   다중 Waypoint 비행 - waypoint를 코드가 아닌 "YAML 파일"로 관리
#   2주차와 비행 로직은 같고, 목표 위치를 파일에서 읽어옵니다.
#
#   배우는 개념:
#     - 설정과 코드의 분리: 미션 바꿀 때 코드 수정 없이 yaml만 수정
#     - yaml.safe_load()로 파일 읽기 → 파이썬 딕셔너리
#     - ROS2 파라미터로 파일 경로 받기:
#       ros2 run ... w03_yaml_raw --ros-args -p waypoint_file:=/path/to/my.yaml
#
#   YAML 파일 위치 (기본값):
#     drone_ros2_advanced/config/waypoints.yaml
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w03_yaml_raw
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import os
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy,
    HistoryPolicy, DurabilityPolicy
)
from ament_index_python.packages import get_package_share_directory
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
)


# ================================================================
# [복붙 영역] QoS 설정 — PX4 uXRCE-DDS 전용
# ================================================================
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

TIMER_HZ = 20


class WaypointYamlRaw(Node):
    def __init__(self):
        super().__init__('waypoint_yaml_raw')

        # ============================================================
        # YAML 파일에서 미션 읽기  ← 이번 주 학습 포인트!
        # ============================================================
        default_yaml = os.path.join(
            get_package_share_directory('drone_ros2_advanced'),
            'config', 'waypoints.yaml'
        )
        self.declare_parameter('waypoint_file', default_yaml)
        yaml_path = self.get_parameter('waypoint_file').value

        with open(yaml_path, 'r') as f:
            mission = yaml.safe_load(f)   # 파일 → 파이썬 딕셔너리

        self.takeoff_alt = float(mission['takeoff_altitude'])
        self.hold_sec    = float(mission['hold_sec'])
        self.waypoints   = [(float(x), float(y))
                            for x, y in mission['waypoints']]

        # ── Publishers: Offboard 필수 3종 세트 ──────────────────
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        # ── 상태 변수 ────────────────────────────────────────────
        self.tick = 0
        self.armed_sent = False
        self.current_idx = 0

        self.create_timer(1.0 / TIMER_HZ, self._control_loop)

        self.get_logger().info(
            f'3주차 시작! YAML: {yaml_path}\n'
            f'  이륙 고도: {self.takeoff_alt}m, '
            f'Waypoint {len(self.waypoints)}개, 각 {self.hold_sec}초'
        )
        for i, (x, y) in enumerate(self.waypoints):
            self.get_logger().info(f'  WP{i+1}: ({x}, {y})')

    # ============================================================
    # 제어 루프 (20Hz) — 2주차와 동일한 시간 기반 전환
    # ============================================================
    def _control_loop(self):
        self.tick += 1
        elapsed = self.tick / TIMER_HZ

        self._publish_heartbeat()

        if not self.armed_sent and elapsed >= 1.0:
            self._set_offboard_mode()
            self._arm()
            self.armed_sent = True

        # 처음 hold_sec 동안은 이륙 지점 위에서 상승
        if elapsed < self.hold_sec:
            self._publish_position(0.0, 0.0, -self.takeoff_alt)
            return

        idx = min(int(elapsed / self.hold_sec) - 1, len(self.waypoints) - 1)

        if idx != self.current_idx:
            self.current_idx = idx
            x, y = self.waypoints[idx]
            self.get_logger().info(f'→ WP{idx+1} ({x}, {y}) 로 이동 시작!')

        x, y = self.waypoints[self.current_idx]
        self._publish_position(x, y, -self.takeoff_alt)

    # ============================================================
    # [복붙 영역] 퍼블리시 헬퍼 (1~2주차와 동일)
    # ============================================================
    def _publish_heartbeat(self):
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def _publish_position(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position  = [float(x), float(y), float(z)]
        msg.velocity  = [float('nan'), float('nan'), float('nan')]
        msg.yaw       = float('nan')
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = params.get('param1', 0.0)
        msg.param2           = params.get('param2', 0.0)
        msg.param7           = params.get('param7', 0.0)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def _arm(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm 명령 전송')

    def _set_offboard_mode(self):
        self._publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info('Offboard 모드 전환')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointYamlRaw()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
