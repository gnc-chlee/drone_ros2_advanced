#!/usr/bin/env python3
# ==============================================================================
# File    : folium_map_demo.py  (5주차)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   Folium으로 waypoint 미션을 "실제 지도" 위에 그려보기
#   (ROS2 없이 실행되는 순수 파이썬 스크립트)
#
#   동작:
#     1. config/waypoints.yaml의 NED waypoint를 읽음
#     2. NED [m] → GPS(위도/경도)로 변환 (5주차 공식)
#     3. Folium 지도에 마커 + 경로선을 그려 HTML로 저장
#     4. 웹브라우저로 자동 열기
#
#   준비물:
#     pip install folium
#
#   실행 방법:
#     ros2 run drone_ros2_advanced w05_map_demo
#     (또는 python3 folium_map_demo.py)
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import os
import math
import webbrowser
import yaml
import folium
from ament_index_python.packages import get_package_share_directory


# ================================================================
# 홈(원점) GPS 좌표 — PX4 SITL 기본값 (취리히 근처)
# 실제 비행이라면 5주차 gps_ned_viewer에서 본 ref_lat/ref_lon을 넣으세요!
# ================================================================
HOME_LAT = 47.397971
HOME_LON = 8.546164


# ================================================================
# NED → GPS 변환 (gps_ned_viewer.py와 동일한 공식)
# ================================================================
METERS_PER_DEG_LAT = 111320.0


def ned_to_gps(north, east, ref_lat, ref_lon):
    lat = ref_lat + north / METERS_PER_DEG_LAT
    lon = ref_lon + east / (METERS_PER_DEG_LAT * math.cos(
        math.radians(ref_lat)))
    return lat, lon


def main():
    # ── 1. YAML에서 waypoint 읽기 (3주차 복습) ───────────────────
    yaml_path = os.path.join(
        get_package_share_directory('drone_ros2_advanced'),
        'config', 'waypoints.yaml'
    )
    with open(yaml_path, 'r') as f:
        mission = yaml.safe_load(f)
    waypoints = mission['waypoints']

    # ── 2. 지도 만들기 (홈 중심, 확대 레벨 19) ───────────────────
    m = folium.Map(location=[HOME_LAT, HOME_LON], zoom_start=19)

    # 홈 마커
    folium.Marker(
        [HOME_LAT, HOME_LON],
        popup='HOME (이륙 지점)',
        icon=folium.Icon(color='red', icon='home')
    ).add_to(m)

    # ── 3. NED waypoint → GPS 변환 후 마커 찍기 ──────────────────
    path = [(HOME_LAT, HOME_LON)]
    for i, (x, y) in enumerate(waypoints):
        lat, lon = ned_to_gps(x, y, HOME_LAT, HOME_LON)
        path.append((lat, lon))
        folium.Marker(
            [lat, lon],
            popup=f'WP{i+1}: NED({x}, {y})',
            icon=folium.Icon(color='blue')
        ).add_to(m)
        print(f'WP{i+1}: NED({x:5.1f}, {y:5.1f})  →  '
              f'GPS({lat:.7f}, {lon:.7f})')

    # ── 4. 경로선 그리기 ─────────────────────────────────────────
    folium.PolyLine(path, color='blue', weight=3, opacity=0.7).add_to(m)

    # ── 5. HTML 저장 + 브라우저 열기 ─────────────────────────────
    out_path = os.path.expanduser('~/mission_map.html')
    m.save(out_path)
    print(f'\n지도 저장 완료: {out_path}')
    webbrowser.open('file://' + out_path)


if __name__ == '__main__':
    main()
