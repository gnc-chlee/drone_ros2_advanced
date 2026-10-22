#!/usr/bin/env python3
# ==============================================================================
# File    : lidar_bridge.py  (10주차 17·18강 - 도우미)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-10-22
# Version : 1.0.0
#
# Description:
#   Gazebo 하방 LiDAR → ROS2 브리지 실행 도우미 (5주차 camera_bridge 를 복사해 바꾼 것)
#
#   왜 필요한가:
#     PX4 토픽(/fmu/...)은 uXRCE-DDS Agent가 ROS2로 옮겨주지만,
#     Gazebo LiDAR 측정값은 Gazebo 자체 토픽이라 Agent를 거치지 않는다.
#     (PX4 도 이 측정값을 받아 쓰지만, /fmu/out/distance_sensor 는 Agent 기본 목록에 없어 ROS2 로 안 나온다)
#     → 5주차 카메라처럼 ros_gz_bridge 라는 "두 번째 다리"를 놓아야 ROS2에서 보인다.
#     (2주차: Agent = PX4↔ROS2 다리 / 5주차·10주차: ros_gz_bridge = Gazebo↔ROS2 다리)
#
#   같은 측정을 ROS2 에서 보는 두 경로 (17강):
#     ① 이 브리지 → /lidar/down (sensor_msgs/LaserScan)                     = 원시값 (시뮬레이션 전용)
#     ② PX4 EKF2  → /fmu/out/vehicle_local_position 의 dist_bottom           = 추정값 (실제 드론에서도 같은 코드)
#
#   이 스크립트가 하는 일:
#     1. Gazebo에 떠 있는 하방 LiDAR를 **자동으로 찾는다** (월드·기체 이름을 몰라도 됨)
#        - 토픽 이름만 보지 않고 실제로 측정값을 내는(발행자가 있는) 토픽인지도 확인한다
#          (PX4 는 LiDAR 없는 기체에서도 LiDAR 토픽을 구독만 해 두어 gz topic -l 에 이름이 보인다)
#     2. ros_gz_bridge 를 실행하고, 이름을 /lidar/down 으로 바꿔줌
#        → 10주차 노드(w10_range_viewer, w10_terrain_follow)는 /lidar/down 만 구독하면 된다
#     3. Gazebo가 아직 뜨는 중이면 잠시 기다렸다가 다시 찾는다 (최대 30초)
#
#   LaserScan 한 칸:
#     LaserScan 의 ranges 는 원래 여러 칸(각도마다 한 칸)이다.
#     실습 기체(x500_lidar_down)는 빔이 1개라 ranges[0] 한 칸만 쓴다.
#     땅에 앉아 있어도 0 이 아니다 — 센서가 다리 끝보다 위에 달려 있다 (17강 장착 오프셋)
#
#   사전 설치 (한 번만 — 5주차에 설치했으면 생략):
#     sudo apt install ros-humble-ros-gzharmonic
#
#   실행 방법 (터미널 1~3 은 17·18강 공통):
#     터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=lidar_world make px4_sitl gz_x500_lidar_down
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w10_lidar_bridge          # ← 이 스크립트 (pxh> 가 뜬 뒤에!)
#     확인    : ros2 topic echo --once /lidar/down
#     터미널 4·5: 17강 w10_range_viewer · keyboard_control / 18강 w10_terrain_follow (각 파일 머리말 참고)
#
#   ※ PX4 보다 30초 넘게 먼저 켜면 자동 감지가 끝나 기본값(lidar_world)으로 연결된다
#      → pxh> 가 뜬 뒤에 켜고, 감지 줄 '[감지] world=… model=…' 을 꼭 확인한다
#   ※ 18강 드라이 런(ros2 topic pub 으로 모의 측정값 발행)을 할 때는 이 브리지를 끈다
#      (/lidar/down 발행자가 둘이면 땅 값과 모의 측정값이 번갈아 들어온다)
#
#   자동 감지가 틀리면 직접 지정:
#     ros2 run drone_ros2_advanced w10_lidar_bridge --world lidar_world
#     ros2 run drone_ros2_advanced w10_lidar_bridge --world default --model x500_lidar_down_0
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import argparse
import re
import shutil
import subprocess
import sys
import time

# Gazebo 하방 LiDAR 토픽의 생김새:  /world/<월드>/model/<기체>/link/lidar_sensor_link/sensor/lidar/scan
#   - 링크·센서 이름(lidar_sensor_link / lidar)은 PX4 x500_lidar_down 모델에 정해져 있다
#   - 끝의 $ : 같은 센서가 함께 낼 수 있는 …/scan/points (점군, point cloud) 토픽은 빼고 /scan 만 잡는다
#   - 전방 LiDAR 기체(x500_lidar_front)도 이름이 같아서 잡힌다 → 아래 detect_lidar 에서 경고
LIDAR_TOPIC_RE = re.compile(
    r'^/world/([^/]+)/model/([^/]+)/link/lidar_sensor_link/sensor/lidar/scan$')

# 자동 감지에 실패했을 때 쓰는 기본값 (10주차 표준 구성)
DEFAULT_WORLD  = 'lidar_world'
DEFAULT_MODEL  = 'x500_lidar_down_0'
DEFAULT_LINK   = 'lidar_sensor_link'
DEFAULT_SENSOR = 'lidar'

DETECT_TRIES    = 15   # Gazebo 가 뜰 때까지 기다리는 횟수
DETECT_INTERVAL = 2.0  # [s]


def list_gz_topics():
    """gz topic -l 결과를 목록으로 반환. gz 가 없거나 응답이 없으면 None"""
    if shutil.which('gz') is None:
        return None
    try:
        result = subprocess.run(['gz', 'topic', '-l'],
                                capture_output=True, text=True, timeout=10)
    except subprocess.TimeoutExpired:
        return None
    return result.stdout.split()


def has_publisher(topic):
    """실제 발행자가 있는 토픽인지 — Harmonic 의 gz topic -l 은 '구독만 된' 토픽도 보여 준다
       (PX4 GZBridge 는 LiDAR 없는 기체에서도 …/lidar_sensor_link/sensor/lidar/scan 을 구독)"""
    try:
        result = subprocess.run(['gz', 'topic', '-i', '-t', topic],
                                capture_output=True, text=True, timeout=10)
    except subprocess.TimeoutExpired:
        return False
    return 'Publishers [' in result.stdout   # 없으면 'No publishers on topic [...]'


def find_lidars(topics):
    """토픽 목록에서 하방 LiDAR 토픽(발행자가 있는 것만)을 전부 찾아 [(world, model), ...] 반환"""
    found = []
    for t in topics or []:
        m = LIDAR_TOPIC_RE.match(t)
        if m and has_publisher(t):
            found.append(m.groups())
    return found


def detect_lidar():
    """Gazebo LiDAR를 자동 감지. 뜨는 중이면 잠시 기다렸다 재시도. 못 찾으면 None"""
    if shutil.which('gz') is None:
        print('[안내] gz 명령을 찾지 못해 자동 감지를 건너뜁니다.')
        return None

    for attempt in range(1, DETECT_TRIES + 1):
        topics = list_gz_topics()
        lidars = find_lidars(topics)
        if lidars:
            if len(lidars) > 1:
                print('[안내] LiDAR 가 여러 개입니다. 첫 번째를 씁니다 '
                      '(다른 걸 쓰려면 --world/--model 로 지정):')
                for w, mdl in lidars:
                    print(f'       world={w}  model={mdl}')
            world, model = lidars[0]
            # 전방 LiDAR 기체는 앞쪽 거리를 재므로 대지 고도로 쓸 수 없다
            if 'front' in model.lower():
                print('[경고] 전방 LiDAR 기체입니다 — 10주차는 gz_x500_lidar_down')
                print('       다시 실행: PX4_GZ_WORLD=lidar_world make px4_sitl gz_x500_lidar_down')
            return world, model

        if attempt == 1:
            print('[감지] Gazebo에서 LiDAR 를 찾는 중... '
                  f'(최대 {int(DETECT_TRIES * DETECT_INTERVAL)}초 기다립니다)')
        time.sleep(DETECT_INTERVAL)

    return None


def build_topic(world: str, model: str, link: str, sensor: str):
    """Gazebo LiDAR 토픽 이름 조립 (LaserScan 하나 — 카메라처럼 camera_info 짝이 없다)"""
    return f'/world/{world}/model/{model}/link/{link}/sensor/{sensor}/scan'


def print_not_found_hint():
    print('[경고] Gazebo에서 LiDAR 토픽을 찾지 못했습니다. 기본값으로 시도합니다.')
    print('  - PX4 SITL을 **LiDAR 기체**로 실행했나요?')
    print('      PX4_GZ_WORLD=lidar_world make px4_sitl gz_x500_lidar_down')
    print('  - PX4 보다 먼저 켰다면: pxh> 가 뜬 뒤 Ctrl+C 하고 다시 실행하세요')
    topics = list_gz_topics()
    if topics is None:
        print('  - Gazebo 가 응답하지 않습니다 (아직 뜨는 중이거나 실행 안 됨)')
    else:
        scan_topics = [t for t in topics if 'scan' in t and has_publisher(t)]
        if scan_topics:
            print('  - 현재 Gazebo에서 측정값을 내는 scan 토픽 (LiDAR 이름이 다른 경우):')
            for t in scan_topics:
                print('      ', t)
        else:
            print('  - Gazebo에 측정값을 내는 scan 토픽이 하나도 없습니다 '
                  '(LiDAR 없는 기체? 예: gz_x500, gz_x500_mono_cam)')
            print('    ※ gz topic -l 에 …/lidar/scan 이 보여도 PX4 가 구독만 해 둔 빈 토픽일 수 있습니다')
    print('  → 브리지는 일단 실행합니다. 토픽이 생기면 자동으로 연결됩니다.\n')


def main():
    parser = argparse.ArgumentParser(description='Gazebo 하방 LiDAR → ROS2 브리지 도우미')
    parser.add_argument('--world',  default=None, help='Gazebo 월드 이름 (생략하면 자동 감지)')
    parser.add_argument('--model',  default=None, help='기체 모델 이름 (생략하면 자동 감지)')
    parser.add_argument('--link',   default=None, help=f'센서가 달린 링크 이름 (기본 {DEFAULT_LINK} — 다른 LiDAR 를 쓸 때만)')
    parser.add_argument('--sensor', default=None, help=f'LiDAR 센서 이름 (기본 {DEFAULT_SENSOR} — 다른 LiDAR 를 쓸 때만)')
    parser.add_argument('--ros-topic', default='/lidar/down', help='ROS2 쪽 LaserScan 토픽 이름')
    args, _ = parser.parse_known_args()   # ros2 run 이 붙이는 --ros-args 는 무시

    # ros_gz_bridge 설치 확인
    if subprocess.run(['ros2', 'pkg', 'prefix', 'ros_gz_bridge'],
                      capture_output=True).returncode != 0:
        print('[오류] ros_gz_bridge 패키지가 없습니다. 먼저 설치하세요:')
        print('       sudo apt install ros-humble-ros-gzharmonic')
        sys.exit(1)

    # 1) 자동 감지 (인자를 하나도 안 줬을 때만)
    world, model = DEFAULT_WORLD, DEFAULT_MODEL
    if not (args.world or args.model or args.link or args.sensor):
        detected = detect_lidar()
        if detected:
            world, model = detected
            print(f'[감지] world={world} model={model} → {args.ros_topic}')
        else:
            print_not_found_hint()

    # 2) 인자로 준 값이 있으면 그것이 우선
    world  = args.world  or world
    model  = args.model  or model
    link   = args.link   or DEFAULT_LINK
    sensor = args.sensor or DEFAULT_SENSOR

    gz_scan  = build_topic(world, model, link, sensor)
    ros_scan = args.ros_topic

    # '[' = Gazebo → ROS2 한 방향 브리지
    #   타입 표기 gz.msgs.LaserScan 은 5주차 gz.msgs.Image 와 같은 형식
    #   (/lidar/down 이 안 오면 ignition.msgs.LaserScan 표기로 바꿔 시험)
    cmd = [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        f'{gz_scan}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '--ros-args',
        '-r', f'{gz_scan}:={ros_scan}',
    ]
    print(f'[브리지] Gazebo {gz_scan}\n     →  ROS2   {ros_scan}  (sensor_msgs/LaserScan)')
    print('         확인: ros2 topic hz -w 50 ' + ros_scan + '   /   종료: Ctrl+C\n')

    try:
        sys.exit(subprocess.call(cmd))
    except KeyboardInterrupt:
        print('\n[브리지] 종료')


if __name__ == '__main__':
    main()
