#!/usr/bin/env python3
# ==============================================================================
# File    : map_click_server.py  (6주차)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   웹 지도 클릭 → ROS2 토픽으로 waypoint 전송
#
#   구조 (원리만 이해하면 됩니다):
#     [브라우저 지도] --클릭(위도,경도)--> [Flask 웹서버] --publish--> /map_waypoint
#                                                                    (NavSatFix)
#     1. Flask가 Leaflet 지도 HTML을 브라우저에 보여줌
#     2. 지도를 클릭하면 자바스크립트가 위도/경도를 서버로 POST 전송
#     3. 서버 안의 ROS2 노드가 /map_waypoint 토픽으로 publish
#     4. goto_gps 노드(별도 실행)가 이 토픽을 받아 드론을 이동
#
#   준비물:
#     pip install flask
#
#   실행 방법:
#     터미널 1: cd ~/PX4-Autopilot && make px4_sitl gz_x500
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w06_map_server
#     터미널 4: ros2 run drone_ros2_advanced w06_goto_base  (또는 w06_goto_raw)
#     브라우저 : http://localhost:5000 접속 → 지도 클릭!
#
#   확인용:
#     ros2 topic echo /map_waypoint
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from flask import Flask, request, jsonify

# ================================================================
# 홈(원점) GPS — PX4 SITL 기본값. 실제 환경이면 ref_lat/ref_lon으로 교체
# ================================================================
HOME_LAT = 47.397971
HOME_LON = 8.546164
PORT     = 5000


# ================================================================
# ROS2 퍼블리셔 노드 — 클릭된 좌표를 /map_waypoint로 발행
# ================================================================
class MapWaypointPublisher(Node):
    def __init__(self):
        super().__init__('map_click_server')
        self.wp_pub = self.create_publisher(NavSatFix, '/map_waypoint', 10)
        self.get_logger().info(
            f'지도 서버 시작! 브라우저에서 http://localhost:{PORT} 접속')

    def publish_waypoint(self, lat, lon):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.latitude  = lat
        msg.longitude = lon
        self.wp_pub.publish(msg)
        self.get_logger().info(f'클릭 waypoint 발행: ({lat:.7f}, {lon:.7f})')


# ================================================================
# [복붙 영역] 웹 지도 페이지 (Leaflet)
# 원리: 오픈소스 지도 라이브러리 Leaflet으로 지도를 그리고,
#       클릭하면 fetch()로 좌표를 서버에 보냅니다.
# ================================================================
MAP_HTML = f"""
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <title>드론 Waypoint 지도</title>
  <link rel="stylesheet"
        href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    body {{ margin:0; }}
    #map {{ height: 100vh; }}
    #info {{ position:absolute; top:10px; right:10px; z-index:1000;
             background:white; padding:10px 14px; border-radius:8px;
             font-family:sans-serif; box-shadow:0 2px 6px rgba(0,0,0,.3); }}
  </style>
</head>
<body>
  <div id="info">지도를 클릭하면 드론이 이동합니다</div>
  <div id="map"></div>
  <script>
    var map = L.map('map').setView([{HOME_LAT}, {HOME_LON}], 19);
    L.tileLayer('https://tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png',
                {{ maxZoom: 19 }}).addTo(map);

    // 홈 마커
    L.marker([{HOME_LAT}, {HOME_LON}]).addTo(map)
      .bindPopup('HOME (이륙 지점)').openPopup();

    var clickMarker = null;
    map.on('click', function(e) {{
      var lat = e.latlng.lat, lon = e.latlng.lng;

      // 클릭 위치 마커 표시
      if (clickMarker) map.removeLayer(clickMarker);
      clickMarker = L.marker([lat, lon]).addTo(map)
        .bindPopup('목표: ' + lat.toFixed(6) + ', ' + lon.toFixed(6))
        .openPopup();

      // 서버로 좌표 전송
      fetch('/waypoint', {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/json' }},
        body: JSON.stringify({{ lat: lat, lon: lon }})
      }});
      document.getElementById('info').innerText =
        '전송됨: ' + lat.toFixed(6) + ', ' + lon.toFixed(6);
    }});
  </script>
</body>
</html>
"""


# ================================================================
# [복붙 영역] Flask 웹서버
# ================================================================
app = Flask(__name__)
ros_node = None   # main()에서 채워짐


@app.route('/')
def index():
    return MAP_HTML


@app.route('/waypoint', methods=['POST'])
def waypoint():
    data = request.get_json()
    ros_node.publish_waypoint(float(data['lat']), float(data['lon']))
    return jsonify(ok=True)


def main(args=None):
    global ros_node
    rclpy.init(args=args)
    ros_node = MapWaypointPublisher()

    # ROS2 spin은 백그라운드 스레드에서, Flask는 메인 스레드에서
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    try:
        app.run(host='0.0.0.0', port=PORT)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
