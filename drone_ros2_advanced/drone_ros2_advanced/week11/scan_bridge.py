#!/usr/bin/env python3
# ==============================================================================
# File    : scan_bridge.py  (11주차 19·20강 - 도우미)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-02
# Version : 1.0.0
#
# Description:
#   Gazebo 2D LiDAR → ROS2 브리지 실행 도우미 (10주차 lidar_bridge 를 복사해 바꾼 것)
#
#   왜 필요한가 (10주차와 같음):
#     PX4 토픽(/fmu/...)은 uXRCE-DDS Agent가 ROS2로 옮겨주지만,
#     Gazebo LiDAR 측정값은 Gazebo 자체 토픽이라 Agent를 거치지 않는다.
#     (PX4 도 같은 스캔을 받아 5° 섹터 평균 obstacle_distance 로 정리하지만, 그 out 토픽은 Agent 기본 목록에 없다)
#     → ros_gz_bridge 라는 "두 번째 다리"를 놓아야 ROS2에서 보인다.
#
#   10주차와 달라진 점:
#     - 토픽 이름 규칙을 넓혔다: 링크·센서 이름이 기체마다 달라도 …/scan 이면 모두 잡는다
#         하방 1빔 (x500_lidar_down)  : …/link/lidar_sensor_link/sensor/lidar/scan
#         2D 270°  (x500_lidar_2d)    : …/link/link/sensor/lidar_2d_v2/scan        ← 11주차 기본
#     - 여러 개가 보이면 2D > 전방 > 하방 순서로 고른다 (11주차는 앞을 봐야 하므로)
#     - ROS2 쪽 이름은 /lidar/scan (10주차 /lidar/down 과 겹치지 않게)
#
#   이 스크립트가 하는 일:
#     1. Gazebo에 떠 있는 LiDAR 를 **자동으로 찾는다** (월드·기체 이름을 몰라도 됨)
#        - 토픽 이름만 보지 않고 실제로 측정값을 내는(발행자가 있는) 토픽인지도 확인한다
#          (PX4 는 모든 기체에서 하방·2D LiDAR 토픽을 구독만 해 두어 gz topic -l 에 이름이 보인다)
#     2. ros_gz_bridge 를 실행하고, 이름을 /lidar/scan 으로 바꿔줌
#        → 11주차 노드(w11_scan_viewer, w11_avoid)는 /lidar/scan 만 구독하면 된다
#     3. Gazebo가 아직 뜨는 중이면 잠시 기다렸다가 다시 찾는다 (최대 30초)
#
#   LaserScan 여러 칸 (19강):
#     2D LiDAR 는 한 평면 270° 를 1080칸으로 잰다 (각도 분해능 0.25°, 30 Hz, 0.1~30 m)
#     i번째 칸 각도 = angle_min + i × angle_increment  (각도 + = 왼쪽, 0번 칸 = 오른쪽 뒤 −135°, 정면 ≈ 540번)
#     → 10주차처럼 ranges[0] 을 그대로 쓰면 오른쪽 뒤를 본다
#
#   사전 설치 (한 번만 — 5주차에 설치했으면 생략):
#     sudo apt install ros-humble-ros-gzharmonic
#
#   실행 방법 (터미널 1~3 은 19·20강 공통):
#     터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500_lidar_2d
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w11_scan_bridge           # ← 이 스크립트 (pxh> 가 뜬 뒤에!)
#     확인    : ros2 topic echo /lidar/scan --once --no-arr           (ranges: length 1080)
#     터미널 4·5: 19강 w11_scan_viewer · keyboard_control / 20강 w11_avoid (각 파일 머리말 참고)
#
#   ※ PX4 보다 30초 넘게 먼저 켜면 자동 감지가 끝나 기본값(obstacle_world)으로 연결된다
#      → pxh> 가 뜬 뒤에 켜고, 감지 줄 '[감지] world=… model=…' 을 꼭 확인한다
#   ※ 20강 드라이 런(ros2 topic pub 으로 모의 스캔 발행)을 할 때는 이 브리지를 끈다
#      (/lidar/scan 발행자가 둘이면 진짜 스캔과 모의 스캔이 번갈아 들어온다)
#
#   자동 감지가 틀리면 직접 지정 (기본값: 링크 link, 센서 lidar_2d_v2):
#     ros2 run drone_ros2_advanced w11_scan_bridge --world obstacle_world
#     ros2 run drone_ros2_advanced w11_scan_bridge --world walls --model x500_lidar_2d_0
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

# Gazebo LiDAR 토픽의 생김새:  /world/<월드>/model/<기체>/link/<링크>/sensor/<센서>/scan
#   - 10주차는 링크·센서 이름을 하방 LiDAR 것으로 고정했지만, 11주차는 [^/]+ 로 아무 이름이나 받는다
#   - 끝의 $ : 같은 센서가 함께 낼 수 있는 …/scan/points (점군, point cloud) 토픽은 빼고 /scan 만 잡는다
SCAN_TOPIC_RE = re.compile(
    r'^/world/([^/]+)/model/([^/]+)/link/([^/]+)/sensor/([^/]+)/scan$')

# 자동 감지에 실패했을 때 쓰는 기본값 (11주차 표준 구성)
DEFAULT_WORLD  = 'obstacle_world'
DEFAULT_MODEL  = 'x500_lidar_2d_0'
DEFAULT_LINK   = 'link'
DEFAULT_SENSOR = 'lidar_2d_v2'

DETECT_TRIES    = 15   # Gazebo 가 뜰 때까지 기다리는 횟수
DETECT_INTERVAL = 2.0  # [s]

# 기체 고르는 순서 — 앞을 보는 것부터 (모델 이름에 들어 있는 글자로 판단)
#   0: 2D 270° (11주차 기본)  1: 1빔 전방 (앞만, 좌우 없음)  2: 1빔 하방 (앞을 못 봄)  3: 그 밖
RANK_KEYS = ('lidar_2d', 'lidar_front', 'lidar_down')


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
       (PX4 GZBridge 는 모든 기체에서 하방·2D LiDAR scan 토픽을 둘 다 구독 → 이름만으로는 구별할 수 없다)"""
    try:
        result = subprocess.run(['gz', 'topic', '-i', '-t', topic],
                                capture_output=True, text=True, timeout=10)
    except subprocess.TimeoutExpired:
        return False
    return 'Publishers [' in result.stdout   # 없으면 'No publishers on topic [...]'


def rank(model):
    """기체 이름 → 고르는 순서 (작을수록 먼저)"""
    name = model.lower()
    for i, key in enumerate(RANK_KEYS):
        if key in name:
            return i
    return len(RANK_KEYS)


def find_scans(topics):
    """토픽 목록에서 LiDAR scan 토픽(발행자가 있는 것만)을 전부 찾아 [(world, model, link, sensor), ...] 반환"""
    found = []
    for t in topics or []:
        m = SCAN_TOPIC_RE.match(t)
        if m and has_publisher(t):
            found.append(m.groups())
    return sorted(found, key=lambda g: rank(g[1]))   # 2D → 전방 → 하방 순서


def warn_airframe(model):
    """11주차 기체(2D)가 아니면 무엇이 달라지는지 알려 준다"""
    r = rank(model)
    if r == 1:
        print('[안내] 1빔 전방 LiDAR — 앞만 측정, 회피는 정지까지만 (11주차 기본 기체: gz_x500_lidar_2d)')
    elif r == 2:
        print('[경고] 하방 LiDAR 기체 — 11주차는 gz_x500_lidar_2d')
        print('       다시 실행: PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500_lidar_2d')
    elif r == 3:
        print(f'[안내] 이름으로 종류를 알 수 없는 LiDAR 입니다 (model={model}) — '
              '11주차 기본 기체는 gz_x500_lidar_2d')


def detect_scan():
    """Gazebo LiDAR 를 자동 감지. 뜨는 중이면 잠시 기다렸다 재시도. 못 찾으면 None"""
    if shutil.which('gz') is None:
        print('[안내] gz 명령을 찾지 못해 자동 감지를 건너뜁니다.')
        return None

    for attempt in range(1, DETECT_TRIES + 1):
        topics = list_gz_topics()
        scans = find_scans(topics)
        if scans:
            if len(scans) > 1:
                print('[안내] LiDAR 가 여러 개입니다. 앞을 보는 것(2D > 전방 > 하방)을 씁니다 '
                      '(다른 걸 쓰려면 --world/--model/--link/--sensor 로 지정):')
                for w, mdl, lnk, sen in scans:
                    print(f'       world={w}  model={mdl}  link={lnk}  sensor={sen}')
            warn_airframe(scans[0][1])
            return scans[0]

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
    print('  - PX4 SITL을 **2D LiDAR 기체**로 실행했나요?')
    print('      PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500_lidar_2d')
    print('  - PX4 보다 먼저 켰다면: pxh> 가 뜬 뒤 Ctrl+C 하고 다시 실행하세요')
    topics = list_gz_topics()
    if topics is None:
        print('  - Gazebo 가 응답하지 않습니다 (아직 뜨는 중이거나 실행 안 됨)')
    else:
        scan_topics = [t for t in topics if 'scan' in t and has_publisher(t)]
        if scan_topics:
            print('  - 현재 Gazebo에서 측정값을 내는 scan 토픽 (이름 규칙이 다른 경우):')
            for t in scan_topics:
                print('      ', t)
        else:
            print('  - Gazebo에 측정값을 내는 scan 토픽이 하나도 없습니다 '
                  '(LiDAR 없는 기체? 예: gz_x500, gz_x500_mono_cam)')
            print('    ※ gz topic -l 에 …/scan 이 보여도 PX4 가 구독만 해 둔 빈 토픽일 수 있습니다')
    print('  → 브리지는 일단 실행합니다. 토픽이 생기면 자동으로 연결됩니다.\n')


def main():
    parser = argparse.ArgumentParser(description='Gazebo 2D LiDAR → ROS2 브리지 도우미')
    parser.add_argument('--world',  default=None, help='Gazebo 월드 이름 (생략하면 자동 감지)')
    parser.add_argument('--model',  default=None, help='기체 모델 이름 (생략하면 자동 감지)')
    parser.add_argument('--link',   default=None, help=f'센서가 달린 링크 이름 (기본 {DEFAULT_LINK})')
    parser.add_argument('--sensor', default=None, help=f'LiDAR 센서 이름 (기본 {DEFAULT_SENSOR})')
    parser.add_argument('--ros-topic', default='/lidar/scan', help='ROS2 쪽 LaserScan 토픽 이름')
    args, _ = parser.parse_known_args()   # ros2 run 이 붙이는 --ros-args 는 무시

    # ros_gz_bridge 설치 확인
    if subprocess.run(['ros2', 'pkg', 'prefix', 'ros_gz_bridge'],
                      capture_output=True).returncode != 0:
        print('[오류] ros_gz_bridge 패키지가 없습니다. 먼저 설치하세요:')
        print('       sudo apt install ros-humble-ros-gzharmonic')
        sys.exit(1)

    # 1) 자동 감지 (인자를 하나도 안 줬을 때만)
    world, model, link, sensor = DEFAULT_WORLD, DEFAULT_MODEL, DEFAULT_LINK, DEFAULT_SENSOR
    if not (args.world or args.model or args.link or args.sensor):
        detected = detect_scan()
        if detected:
            world, model, link, sensor = detected
            print(f'[감지] world={world} model={model} → {args.ros_topic}')
        else:
            print_not_found_hint()

    # 2) 인자로 준 값이 있으면 그것이 우선
    world  = args.world  or world
    model  = args.model  or model
    link   = args.link   or link
    sensor = args.sensor or sensor

    gz_scan  = build_topic(world, model, link, sensor)
    ros_scan = args.ros_topic

    # '[' = Gazebo → ROS2 한 방향 브리지 (10주차와 같은 형식)
    cmd = [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        f'{gz_scan}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '--ros-args',
        '-r', f'{gz_scan}:={ros_scan}',
    ]
    print(f'[브리지] Gazebo {gz_scan}\n     →  ROS2   {ros_scan}  (sensor_msgs/LaserScan)')
    print('         확인: ros2 topic echo ' + ros_scan + ' --once --no-arr   /   종료: Ctrl+C\n')

    try:
        sys.exit(subprocess.call(cmd))
    except KeyboardInterrupt:
        print('\n[브리지] 종료')


if __name__ == '__main__':
    main()
