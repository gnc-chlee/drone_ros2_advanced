#!/usr/bin/env python3
# ==============================================================================
# File    : gps_ned_viewer.py  (5주차)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   GPS 좌표와 Local NED 좌표를 동시에 구독해서 비교 출력
#
#   배우는 개념:
#     - GPS(위도/경도)와 NED(미터)의 관계
#       * PX4는 시동 위치를 "원점(0,0,0)"으로 잡고 NED로 비행
#       * ref_lat/ref_lon = 그 원점의 GPS 좌표
#     - 간단 변환 공식 (지구를 평평하다고 근사, 수 km 이내 OK):
#       north [m] = (lat - ref_lat) * 111320
#       east  [m] = (lon - ref_lon) * 111320 * cos(ref_lat)
#       (위도 1도 ≈ 111.32km, 경도는 위도에 따라 간격이 좁아짐)
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w05_gps_viewer
#     (QGC에서 이륙시키고 움직여보면서 값 변화를 관찰하세요)
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy,
    HistoryPolicy, DurabilityPolicy
)
from px4_msgs.msg import VehicleLocalPosition, VehicleGlobalPosition


# ================================================================
# [복붙 영역] QoS 설정 — PX4 uXRCE-DDS 전용
# ================================================================
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)


# ================================================================
# GPS ↔ NED 변환 함수  ← 이번 주 핵심! (6주차에서 재사용)
# ================================================================
METERS_PER_DEG_LAT = 111320.0   # 위도 1도 ≈ 111.32 km


def gps_to_ned(lat, lon, ref_lat, ref_lon):
    """GPS(위도,경도) → NED(north, east) [m]"""
    north = (lat - ref_lat) * METERS_PER_DEG_LAT
    east  = (lon - ref_lon) * METERS_PER_DEG_LAT * math.cos(
        math.radians(ref_lat))
    return north, east


def ned_to_gps(north, east, ref_lat, ref_lon):
    """NED(north, east) [m] → GPS(위도,경도)"""
    lat = ref_lat + north / METERS_PER_DEG_LAT
    lon = ref_lon + east / (METERS_PER_DEG_LAT * math.cos(
        math.radians(ref_lat)))
    return lat, lon


class GpsNedViewer(Node):
    def __init__(self):
        super().__init__('gps_ned_viewer')

        # Local NED 위치 (+ 원점의 GPS 좌표 ref_lat/ref_lon 포함)
        self.local_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_callback, PX4_QOS)

        # GPS 위치 (위도/경도/고도)
        self.global_sub = self.create_subscription(
            VehicleGlobalPosition,
            '/fmu/out/vehicle_global_position',
            self._global_callback, PX4_QOS)

        self.local_pos  = None
        self.global_pos = None

        self.create_timer(1.0, self._print_compare)   # 1초마다 출력
        self.get_logger().info('GPS ↔ NED 비교 뷰어 시작!')

    def _local_callback(self, msg):
        self.local_pos = msg

    def _global_callback(self, msg):
        self.global_pos = msg

    def _print_compare(self):
        if self.local_pos is None or self.global_pos is None:
            self.get_logger().info('데이터 수신 대기 중...')
            return

        lp, gp = self.local_pos, self.global_pos

        # GPS → NED 직접 변환해보고, PX4의 NED 값과 비교!
        calc_n, calc_e = gps_to_ned(
            gp.lat, gp.lon, lp.ref_lat, lp.ref_lon)

        self.get_logger().info(
            f'\n───────────────────────────────────────\n'
            f' GPS   : lat={gp.lat:.7f}, lon={gp.lon:.7f}, alt={gp.alt:.1f}m\n'
            f' 원점   : ref_lat={lp.ref_lat:.7f}, ref_lon={lp.ref_lon:.7f}\n'
            f' NED(PX4)  : x(북)={lp.x:+7.2f}m, y(동)={lp.y:+7.2f}m, z={lp.z:+6.2f}m\n'
            f' NED(계산) : x(북)={calc_n:+7.2f}m, y(동)={calc_e:+7.2f}m  ← 공식으로 직접 계산\n'
            f'───────────────────────────────────────'
        )


def main(args=None):
    rclpy.init(args=args)
    node = GpsNedViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
