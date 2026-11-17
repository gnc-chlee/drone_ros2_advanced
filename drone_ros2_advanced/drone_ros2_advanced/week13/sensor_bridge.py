#!/usr/bin/env python3
# ==============================================================================
# File    : sensor_bridge.py  (13주차 24강 - 도우미)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-17
# Version : 1.0.0
#
# Description:
#   Gazebo 2D LiDAR + 아래 카메라 → ROS2 묶음 브리지 (실행 이름 w13_bridge)
#   11주차 스캔 브리지(w11_scan_bridge)와 5주차 카메라 브리지(w05_camera_bridge)를 터미널 하나로 합친 것
#
#   왜 하나로 묶나:
#     24강 기체(x500_lidar_2d_cam_down)는 센서가 둘이다 — 2D LiDAR(기둥 회피) + 아래 카메라(마커 정렬)
#     브리지를 따로 켜면 터미널이 하나 늘고, 두 브리지가 같은 ROS 노드 이름(/ros_gz_bridge)을 쓴다
#     → ros_gz_bridge(parameter_bridge) 한 프로세스로 세 토픽을 함께 잇는다 (ROS 노드 /ros_gz_bridge 하나)
#
#   이어 주는 토픽 — ROS2 쪽 이름 · 메시지 형식은 기존 두 브리지와 같다 (마커 인식 · 미션 노드는 그대로 구독)
#     스캔        /lidar/scan          sensor_msgs/LaserScan   ← …/link/link/sensor/lidar_2d_v2/scan
#     영상        /camera/image_raw    sensor_msgs/Image       ← …/link/camera_link/sensor/imager/image
#     카메라 정보 /camera/camera_info  sensor_msgs/CameraInfo  ← …/link/camera_link/sensor/imager/camera_info
#     (… = /world/<월드>/model/<기체>. Gazebo 센서 토픽은 Agent 를 거치지 않으므로 이 다리가 따로 필요하다)
#
#   이 스크립트가 하는 일:
#     1. Gazebo에 떠 있는 2D LiDAR · 아래 카메라를 **자동으로 찾는다** (기존 두 브리지의 찾기 함수를 그대로 씀)
#        - 스캔은 측정값을 내는(발행자가 있는) 토픽만 — PX4 는 하방 1빔 · 2D LiDAR scan 토픽을 구독만 해 두어
#          센서가 없어도 gz topic -l 에 이름이 보인다
#        - 두 센서가 한 기체에 함께 달린 것을 고른다
#     2. ros_gz_bridge 를 한 번 실행해 세 토픽의 이름을 ROS2 쪽 이름으로 바꿔 잇는다
#     3. Gazebo가 아직 뜨는 중이면 잠시 기다렸다가 다시 찾는다 (최대 30초)
#        끝내 못 찾으면 원인 · 확인 명령을 안내하고 그대로 실행 — 센서 하나만 찾았으면 그 기체 이름으로,
#        둘 다 못 찾으면 기본값(parking_world · x500_lidar_2d_cam_down_0 = 24강 표준 구성)으로
#
#   사전 설치 (한 번만 — 5주차에 설치했으면 생략):
#     sudo apt install ros-humble-ros-gzharmonic
#
#   실행 방법 (24강 터미널 5개 중 3번):
#     터미널 1: cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_lidar_2d_cam_down PX4_GZ_WORLD=parking_world ./build/px4_sitl_default/bin/px4
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w13_bridge                # ← 이 스크립트 (pxh> 가 뜬 뒤에!)
#     터미널 4: ros2 run drone_ros2_advanced w05_aruco
#     터미널 5: ros2 run drone_ros2_advanced w13_mission --ros-args -p fly:=true
#     확인    : ros2 node list (/ros_gz_bridge) · ros2 topic hz /lidar/scan · ros2 topic hz /camera/image_raw
#
#   ※ 시작 줄 '[w13_bridge] parking_world · x500_lidar_2d_cam_down_0 연결 — …' 을 꼭 확인한다
#      (센서를 못 찾으면 [경고] 안내 뒤 시작 줄이 '… 연결 시도 — 위 [경고] 먼저 확인 (찾지 못함: …)'
#       → 그 안내대로 PX4 쪽을 먼저 확인. '다음: 터미널 4 …' 안내는 두 센서를 모두 찾았을 때만)
#   ※ 이 브리지를 끄면 스캔과 영상이 함께 끊긴다
#   ※ 이 브리지로 막히면 기존 브리지 둘을 따로 켠다 (터미널 하나 더): w11_scan_bridge · w05_camera_bridge
#
#   자동 감지가 틀리면 직접 지정 (센서 이름은 13주차 기체의 값 — 2D LiDAR link/lidar_2d_v2 · 카메라 camera_link/imager):
#     ros2 run drone_ros2_advanced w13_bridge --world parking_world
#     ros2 run drone_ros2_advanced w13_bridge --world obstacle_world --model x500_lidar_2d_cam_down_0
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import argparse
import shutil
import subprocess
import sys
import time

# 기존 두 브리지의 찾기 함수 · 이름 규칙을 그대로 쓴다 (import — 복사하지 않음. 두 파일은 고치지 않는다)
from ..week05.camera_bridge import DEFAULT_SENSOR as CAMERA_SENSOR       # 아래 카메라 센서 이름 imager
from ..week05.camera_bridge import build_topics as build_camera_topics, find_cameras
from ..week11.scan_bridge import DEFAULT_LINK as SCAN_LINK               # 2D LiDAR 링크 이름 link
from ..week11.scan_bridge import DEFAULT_SENSOR as SCAN_SENSOR           # 2D LiDAR 센서 이름 lidar_2d_v2
from ..week11.scan_bridge import DETECT_INTERVAL, DETECT_TRIES           # 2초 × 15번 = 최대 30초
from ..week11.scan_bridge import build_topic as build_scan_topic
from ..week11.scan_bridge import find_scans, has_publisher, list_gz_topics

# 자동 감지에 실패했을 때 쓰는 기본값 (24강 표준 구성)
DEFAULT_WORLD = 'parking_world'
DEFAULT_MODEL = 'x500_lidar_2d_cam_down_0'

# ROS2 쪽 토픽 이름 — 기존 두 브리지와 같다 (미션 노드 · 마커 인식 노드가 구독하는 이름)
ROS_SCAN = '/lidar/scan'
ROS_IMAGE = '/camera/image_raw'      # 카메라 정보는 같은 폴더의 /camera/camera_info (5주차 규칙)

# 안내용 — 13주차 기체를 띄우는 명령 (터미널 1)
PX4_CMD = ('cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_lidar_2d_cam_down '
           'PX4_GZ_WORLD=parking_world ./build/px4_sitl_default/bin/px4')


def find_pairs(scans, cameras):
    """2D LiDAR 와 아래 카메라가 한 기체(같은 world · model)에 함께 달린 짝 [(scan, camera), ...]
       scan = (world, model, link, sensor) · camera = (world, model, sensor)
       고르는 순서: 2D 센서(link/lidar_2d_v2) 먼저 — 한 기체에 1빔 LiDAR 가 함께 있어도 2D 를 고른다
                    나머지는 find_scans 순서 그대로 (기체 이름으로 2D > 전방 > 하방, 11주차 규칙)"""
    pairs = [(s, c) for s in scans for c in cameras if s[:2] == c[:2]]
    return sorted(pairs, key=lambda p: p[0][2:] != (SCAN_LINK, SCAN_SENSOR))


def detect_sensors():
    """Gazebo 의 2D LiDAR · 아래 카메라를 자동 감지 → (scan, camera), 못 찾은 쪽은 None
       한 기체에 둘 다 있으면 바로 반환. 아니면 Gazebo 가 뜨는 중일 수 있어 최대 30초 기다린다"""
    if shutil.which('gz') is None:
        print('[안내] gz 명령을 찾지 못해 자동 감지를 건너뜁니다.')
        return None, None

    scans, cameras = [], []
    for attempt in range(1, DETECT_TRIES + 1):
        topics = list_gz_topics()
        scans, cameras = find_scans(topics), find_cameras(topics)
        pairs = find_pairs(scans, cameras)
        if pairs:
            vehicles = []
            for s, _ in pairs:
                if s[:2] not in vehicles:
                    vehicles.append(s[:2])
            if len(vehicles) > 1:
                print('[안내] 2D LiDAR · 아래 카메라가 함께 달린 기체가 여럿입니다. 첫 번째를 씁니다 '
                      '(다른 걸 쓰려면 --world/--model 로 지정):')
                for w, mdl in vehicles:
                    print(f'       world={w}  model={mdl}')
            return pairs[0]

        if attempt == 1:
            print('[감지] Gazebo에서 2D LiDAR · 아래 카메라를 찾는 중... '
                  f'(최대 {int(DETECT_TRIES * DETECT_INTERVAL)}초 기다립니다)')
        time.sleep(DETECT_INTERVAL)

    # 시간 초과 — 찾은 것만이라도 반환 (둘 다 찾았지만 서로 다른 기체일 수도 있음)
    return (scans[0] if scans else None), (cameras[0] if cameras else None)


def print_missing_hint(scan, camera):
    """못 찾은 센서가 있을 때 — 원인과 확인 명령 (기존 두 브리지의 안내와 같은 꼴). 반환값 = 못 찾은 센서 이름"""
    missing = ' · '.join(name for name, ok in (('2D LiDAR', scan), ('아래 카메라', camera)) if not ok)
    found = scan or camera
    if found:
        # 하나만 찾음 = 센서가 하나뿐인 기체로 실행한 경우
        print(f'[경고] Gazebo에서 {missing} 토픽을 찾지 못했습니다. 찾은 기체 이름(model={found[1]})으로 시도합니다.')
        if camera:
            print('  - 2D LiDAR + 아래 카메라 기체로 실행했나요? gz_x500_mono_cam_down 에는 2D LiDAR 가 없습니다')
        else:
            print('  - 2D LiDAR + 아래 카메라 기체로 실행했나요? gz_x500_lidar_2d 에는 아래 카메라가 없습니다')
        print('    PX4 Ctrl+C → 남은 Gazebo 정리 → 터미널 1 다시 실행:')
        print('      pkill -9 -f "gz sim"')
        print('      ' + PX4_CMD)
    else:
        print(f'[경고] Gazebo에서 {missing} 토픽을 찾지 못했습니다. '
              f'기본값({DEFAULT_WORLD} · {DEFAULT_MODEL})으로 시도합니다.')
        print('  - PX4 SITL을 2D LiDAR + 아래 카메라 기체로 실행했나요? (터미널 1)')
        print('      ' + PX4_CMD)
        print('  - 기체 모델 폴더를 PX4 로 복사했나요? (빠지면 pxh> 는 떠도 Gazebo 에 기체가 없습니다)')
        print('      ls ~/PX4-Autopilot/Tools/simulation/gz/models | grep x500_lidar_2d_cam_down')
        print('  - PX4 보다 먼저 켰다면: pxh> 가 뜬 뒤 Ctrl+C 하고 다시 실행하세요')

    topics = list_gz_topics()
    if not topics:
        print('  - Gazebo 가 응답하지 않습니다 (아직 뜨는 중이거나 실행 안 됨)')
    else:
        live_scans = [t for t in topics if 'scan' in t and has_publisher(t)]
        images = [t for t in topics if t.endswith('image')]      # …/image · …/depth_image (camera_info 는 빼고)
        print('  - 현재 Gazebo에서 측정값을 내는 scan 토픽:' + ('' if live_scans else ' 없음'))
        for t in live_scans:
            print(f'       {t}')
        if not live_scans and any(t.endswith('/scan') for t in topics):
            print('    ※ gz topic -l 에 …/scan 이 보여도 PX4 가 구독만 해 둔 빈 토픽일 수 있습니다')
        print('  - 현재 Gazebo에 있는 image 토픽:' + ('' if images else ' 없음'))
        for t in images:
            print(f'       {t}')
    if found:   # 기체를 바꾸면 기체 이름도 바뀌므로 브리지를 다시 켜야 한다
        print('  → 브리지는 일단 실행합니다. PX4 를 다시 실행했다면 이 브리지도 Ctrl+C 후 다시 실행하세요.\n')
    else:       # 기본값 = 24강 표준 구성의 이름 → PX4 가 뜨면 그대로 이어진다
        print('  → 브리지는 일단 실행합니다. 토픽이 생기면 자동으로 연결됩니다.\n')
    return missing


def main():
    parser = argparse.ArgumentParser(description='Gazebo 2D LiDAR + 아래 카메라 → ROS2 묶음 브리지 도우미 (13주차)')
    parser.add_argument('--world', default=None, help=f'Gazebo 월드 이름 (생략하면 자동 감지, 기본 {DEFAULT_WORLD})')
    parser.add_argument('--model', default=None, help=f'기체 모델 이름 (생략하면 자동 감지, 기본 {DEFAULT_MODEL})')
    args, _ = parser.parse_known_args()   # ros2 run 이 붙이는 --ros-args 는 무시

    # ros_gz_bridge 설치 확인
    if subprocess.run(['ros2', 'pkg', 'prefix', 'ros_gz_bridge'],
                      capture_output=True).returncode != 0:
        print('[오류] ros_gz_bridge 패키지가 없습니다. 먼저 설치하세요:')
        print('       sudo apt install ros-humble-ros-gzharmonic')
        sys.exit(1)

    # 1) 자동 감지 (인자를 하나도 안 줬을 때만)
    scan, camera = None, None
    missing = ''          # 자동 감지에서 못 찾은 센서 (있으면 시작 줄 = '연결 시도')
    if not (args.world or args.model):
        scan, camera = detect_sensors()
        if scan and camera and scan[:2] != camera[:2]:
            print('[경고] 2D LiDAR 와 아래 카메라가 서로 다른 기체에 있습니다 — 찾은 그대로 잇습니다')
            print(f'       LiDAR model={scan[1]}  /  카메라 model={camera[1]}  '
                  '(13주차 기체는 두 센서가 한 기체: x500_lidar_2d_cam_down)')
        elif not (scan and camera):
            missing = print_missing_hint(scan, camera)

    # 2) 못 찾은 쪽은 찾은 쪽의 기체 이름으로, 둘 다 없으면 기본값 — 인자로 준 값이 있으면 그것이 우선
    #    (센서 이름은 13주차 기체의 값: 2D LiDAR link/lidar_2d_v2 · 아래 카메라 camera_link/imager)
    found = scan or camera
    world = args.world or (found[0] if found else DEFAULT_WORLD)
    model = args.model or (found[1] if found else DEFAULT_MODEL)
    scan = scan or (world, model, SCAN_LINK, SCAN_SENSOR)
    camera = camera or (world, model, CAMERA_SENSOR)

    gz_scan = build_scan_topic(*scan)
    gz_img, gz_info, ros_img, ros_info = build_camera_topics(*camera, ROS_IMAGE)

    if world != DEFAULT_WORLD:
        print(f'[안내] 월드 = {world} — 24강 미션 월드는 {DEFAULT_WORLD} (터미널 1 의 PX4_GZ_WORLD 확인)')

    # '[' = Gazebo → ROS2 한 방향 브리지. 기존 두 브리지의 인자를 한 줄로 합친 것 → 프로세스 하나 · 노드 하나
    cmd = [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        f'{gz_scan}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        f'{gz_img}@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{gz_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '--ros-args',
        '-r', f'{gz_scan}:={ROS_SCAN}',
        '-r', f'{gz_img}:={ros_img}',
        '-r', f'{gz_info}:={ros_info}',
    ]

    # 시작 안내 — 찾은 구성과 다음 명령을 한 줄로, 아래에 토픽 대응 (… = /world/<월드>/model/<기체>)
    #   못 찾은 센서가 있으면 '연결 시도' — 다음 터미널 안내 대신 위 [경고] 를 먼저 보게 한다
    if missing:
        print(f'[w13_bridge] {world} · {model} 연결 시도 — 위 [경고] 먼저 확인 (찾지 못함: {missing})')
    else:
        print(f'[w13_bridge] {world} · {model} 연결 — 스캔 · 영상 · 카메라 정보 | '
              '다음: 터미널 4 w05_aruco → 터미널 5 w13_mission')
    prefix = f'/world/{world}/model/{model}/'
    for label, ros, gz in (('스캔       ', ROS_SCAN, gz_scan),
                           ('영상       ', ros_img, gz_img),
                           ('카메라 정보', ros_info, gz_info)):
        gz_short = '…/' + gz[len(prefix):] if gz.startswith(prefix) else gz
        print(f'             {label} {ros:<19} ← Gazebo {gz_short}')
    print(f'             확인: ros2 topic hz {ROS_SCAN} · ros2 topic hz {ros_img}   /   종료: Ctrl+C\n')
    sys.stdout.flush()   # 안내가 브리지 출력보다 먼저 보이게

    try:
        sys.exit(subprocess.call(cmd))
    except KeyboardInterrupt:
        print('\n[w13_bridge] 종료')


if __name__ == '__main__':
    main()
