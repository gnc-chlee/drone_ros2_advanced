#!/usr/bin/env python3
# ==============================================================================
# File    : goto_gps_base.py  (6주차 - px4_base 버전)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   지도에서 클릭한 GPS waypoint로 드론 이동 - PX4Base 상속 버전
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w06_map_server
#     터미널 4: ros2 run drone_ros2_advanced w06_goto_base
#     브라우저 : http://localhost:5000 → 지도 클릭!
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import math
import rclpy
from sensor_msgs.msg import NavSatFix
from px4_msgs.msg import VehicleLocalPosition

from ..px4_base import PX4Base, PX4_QOS


TAKEOFF_ALT = 5.0

# GPS → NED 변환 (5주차와 동일)
METERS_PER_DEG_LAT = 111320.0


def gps_to_ned(lat, lon, ref_lat, ref_lon):
    north = (lat - ref_lat) * METERS_PER_DEG_LAT
    east  = (lon - ref_lon) * METERS_PER_DEG_LAT * math.cos(
        math.radians(ref_lat))
    return north, east


class GotoGpsBase(PX4Base):
    def __init__(self):
        super().__init__('goto_gps_base')

        # ============================================================
        # [USER DEFINE] Subscribers
        # ============================================================
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_pos_callback, PX4_QOS)

        # 지도 클릭 waypoint  ← 이번 주 학습 포인트!
        self.map_wp_sub = self.create_subscription(
            NavSatFix, '/map_waypoint',
            self._map_wp_callback, 10)

        self.local_position = VehicleLocalPosition()
        self.armed_sent = False
        self.target_x = 0.0
        self.target_y = 0.0

        self.create_timer(0.1, self.on_update)   # 10Hz
        self.get_logger().info(
            '6주차(base) 시작! 이륙 후 지도 클릭을 기다립니다...')

    def _local_pos_callback(self, msg):
        self.local_position = msg

    def _map_wp_callback(self, msg: NavSatFix):
        """지도 클릭 → GPS를 NED로 변환해서 목표 갱신"""
        lp = self.local_position
        north, east = gps_to_ned(
            msg.latitude, msg.longitude, lp.ref_lat, lp.ref_lon)

        self.target_x = north
        self.target_y = east
        self.get_logger().info(
            f'새 목표 수신! GPS({msg.latitude:.6f}, {msg.longitude:.6f}) '
            f'→ NED({north:.1f}, {east:.1f})')

    def on_update(self):
        if not self.armed_sent and self.offboard_counter >= 10:
            self.set_offboard_mode()
            self.arm()
            self.armed_sent = True

        self.send_position(self.target_x, self.target_y, -TAKEOFF_ALT)

        dx = self.local_position.x - self.target_x
        dy = self.local_position.y - self.target_y
        self.get_logger().info(
            f'목표: ({self.target_x:.1f}, {self.target_y:.1f}) '
            f'남은 거리: {math.sqrt(dx**2 + dy**2):.1f}m',
            throttle_duration_sec=2.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = GotoGpsBase()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('사용자 종료 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
