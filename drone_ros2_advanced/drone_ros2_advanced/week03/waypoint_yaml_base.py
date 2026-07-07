#!/usr/bin/env python3
# ==============================================================================
# File    : waypoint_yaml_base.py  (3주차 - px4_base 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   다중 Waypoint 비행 (YAML 관리) - PX4Base 상속 버전
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w03_yaml_base
#
#   내 미션 파일로 실행:
#     ros2 run drone_ros2_advanced w03_yaml_base \
#         --ros-args -p waypoint_file:=/home/me/my_mission.yaml
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import os
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from ..px4_base import PX4Base


class WaypointYamlBase(PX4Base):
    def __init__(self):
        super().__init__('waypoint_yaml_base')

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
            mission = yaml.safe_load(f)

        self.takeoff_alt = float(mission['takeoff_altitude'])
        self.hold_sec    = float(mission['hold_sec'])
        self.waypoints   = [(float(x), float(y))
                            for x, y in mission['waypoints']]

        self.armed_sent = False
        self.current_idx = 0
        self.create_timer(0.1, self.on_update)   # 10Hz

        self.get_logger().info(
            f'3주차(base) 시작! Waypoint {len(self.waypoints)}개 '
            f'({yaml_path})'
        )

    def on_update(self):
        elapsed = self.offboard_counter * 0.1

        if not self.armed_sent and elapsed >= 1.0:
            self.set_offboard_mode()
            self.arm()
            self.armed_sent = True

        # 처음 hold_sec 동안은 이륙 지점 위에서 상승
        if elapsed < self.hold_sec:
            self.send_position(0.0, 0.0, -self.takeoff_alt)
            return

        idx = min(int(elapsed / self.hold_sec) - 1, len(self.waypoints) - 1)

        if idx != self.current_idx:
            self.current_idx = idx
            x, y = self.waypoints[idx]
            self.get_logger().info(f'→ WP{idx+1} ({x}, {y}) 로 이동 시작!')

        x, y = self.waypoints[self.current_idx]
        self.send_position(x, y, -self.takeoff_alt)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointYamlBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
