#!/usr/bin/env python3
# ==============================================================================
# File    : camera_bridge.py  (5주차 1강 - 도우미)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-09-22
# Version : 1.1.0
#
# Description:
#   Gazebo 카메라 → ROS2 브리지 실행 도우미
#
#   왜 필요한가:
#     PX4 토픽(/fmu/...)은 uXRCE-DDS Agent가 ROS2로 옮겨주지만,
#     Gazebo 카메라 이미지는 Gazebo 자체 토픽이라 Agent를 거치지 않는다.
#     → ros_gz_bridge 라는 "두 번째 다리"를 따로 놓아야 ROS2에서 보인다.
#     (2주차: Agent = PX4↔ROS2 다리 / 5주차: ros_gz_bridge = Gazebo↔ROS2 다리)
#
#   이 스크립트가 하는 일:
#     1. Gazebo에 떠 있는 카메라를 **자동으로 찾는다** (월드·기체·센서 이름을 몰라도 됨)
#        → 4주차 my_custom_world 든 PX4 내장 aruco 월드든 그대로 동작
#     2. ros_gz_bridge 를 실행하고, 이름을 /camera/image_raw 로 바꿔줌
#        → 이후 모든 비전 노드는 /camera/image_raw 만 구독하면 된다
#     3. Gazebo가 아직 뜨는 중이면 잠시 기다렸다가 다시 찾는다
#
#   사전 설치 (한 번만):
#     sudo apt install ros-humble-ros-gzharmonic
#
#   실행 방법 (터미널 4개):
#     터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=my_custom_world make px4_sitl gz_x500_mono_cam_down
#               (PX4 내장 마커 월드를 쓰려면 PX4_GZ_WORLD=aruco)
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w05_camera_bridge          # ← 이 스크립트
#     터미널 4: ros2 run drone_ros2_advanced w05_camera_viewer
#
#   카메라가 여러 개거나 자동 감지가 틀리면 직접 지정:
#     ros2 run drone_ros2_advanced w05_camera_bridge --world aruco
#     ros2 run drone_ros2_advanced w05_camera_bridge --model x500_depth_0 --sensor IMX214
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

# Gazebo 카메라 토픽의 생김새:  /world/<월드>/model/<기체>/link/camera_link/sensor/<센서>/image
CAMERA_TOPIC_RE = re.compile(
    r'^/world/([^/]+)/model/([^/]+)/link/camera_link/sensor/([^/]+)/image$')

# 자동 감지에 실패했을 때 쓰는 기본값 (5주차 표준 구성)
DEFAULT_WORLD  = 'my_custom_world'
DEFAULT_MODEL  = 'x500_mono_cam_down_0'
DEFAULT_SENSOR = 'imager'

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


def find_cameras(topics):
    """토픽 목록에서 카메라 이미지 토픽을 전부 찾아 [(world, model, sensor), ...] 반환"""
    found = []
    for t in topics or []:
        m = CAMERA_TOPIC_RE.match(t)
        if m:
            found.append(m.groups())
    return found


def detect_camera():
    """Gazebo 카메라를 자동 감지. 뜨는 중이면 잠시 기다렸다 재시도. 못 찾으면 None"""
    if shutil.which('gz') is None:
        print('[안내] gz 명령을 찾지 못해 자동 감지를 건너뜁니다.')
        return None

    for attempt in range(1, DETECT_TRIES + 1):
        topics  = list_gz_topics()
        cameras = find_cameras(topics)
        if cameras:
            if len(cameras) > 1:
                print('[안내] 카메라가 여러 개입니다. 첫 번째를 씁니다 '
                      '(다른 걸 쓰려면 --world/--model/--sensor 로 지정):')
                for w, mdl, s in cameras:
                    print(f'       world={w}  model={mdl}  sensor={s}')
            return cameras[0]

        if attempt == 1:
            print('[감지] Gazebo에서 카메라를 찾는 중... '
                  f'(최대 {int(DETECT_TRIES * DETECT_INTERVAL)}초 기다립니다)')
        time.sleep(DETECT_INTERVAL)

    return None


def build_topics(world: str, model: str, sensor: str, ros_image: str):
    """Gazebo 토픽 이름 조립 + ROS2 쪽 이름 결정"""
    base     = f'/world/{world}/model/{model}/link/camera_link/sensor/{sensor}'
    gz_img   = base + '/image'
    gz_info  = base + '/camera_info'
    ros_info = ros_image.rsplit('/', 1)[0] + '/camera_info'   # /camera/image_raw → /camera/camera_info
    return gz_img, gz_info, ros_image, ros_info


def print_not_found_hint():
    print('[경고] Gazebo에서 카메라 토픽을 찾지 못했습니다. 기본값으로 시도합니다.')
    print('  - PX4 SITL을 **카메라 기체**로 실행했나요?')
    print('      PX4_GZ_WORLD=my_custom_world make px4_sitl gz_x500_mono_cam_down')
    topics = list_gz_topics()
    if topics is None:
        print('  - Gazebo 가 응답하지 않습니다 (아직 뜨는 중이거나 실행 안 됨)')
    else:
        image_topics = [t for t in topics if 'image' in t]
        if image_topics:
            print('  - 현재 Gazebo에 있는 image 토픽 (카메라 이름이 다른 경우):')
            for t in image_topics:
                print('      ', t)
        else:
            print('  - Gazebo에 image 토픽이 하나도 없습니다 (카메라 없는 기체?)')
    print('  → 브리지는 일단 실행합니다. 토픽이 생기면 자동으로 연결됩니다.\n')


def main():
    parser = argparse.ArgumentParser(description='Gazebo 카메라 → ROS2 브리지 도우미')
    parser.add_argument('--world',  default=None, help='Gazebo 월드 이름 (생략하면 자동 감지)')
    parser.add_argument('--model',  default=None, help='기체 모델 이름 (생략하면 자동 감지)')
    parser.add_argument('--sensor', default=None, help='카메라 센서 이름 (생략하면 자동 감지; mono_cam: imager, x500_depth: IMX214)')
    parser.add_argument('--ros-topic', default='/camera/image_raw', help='ROS2 쪽 이미지 토픽 이름')
    args, _ = parser.parse_known_args()   # ros2 run 이 붙이는 --ros-args 는 무시

    # ros_gz_bridge 설치 확인
    if subprocess.run(['ros2', 'pkg', 'prefix', 'ros_gz_bridge'],
                      capture_output=True).returncode != 0:
        print('[오류] ros_gz_bridge 패키지가 없습니다. 먼저 설치하세요:')
        print('       sudo apt install ros-humble-ros-gzharmonic')
        sys.exit(1)

    # 1) 자동 감지 (인자를 하나도 안 줬을 때만)
    world, model, sensor = DEFAULT_WORLD, DEFAULT_MODEL, DEFAULT_SENSOR
    if not (args.world or args.model or args.sensor):
        detected = detect_camera()
        if detected:
            world, model, sensor = detected
            print(f'[감지] Gazebo 카메라: world={world}  model={model}  sensor={sensor}')
        else:
            print_not_found_hint()

    # 2) 인자로 준 값이 있으면 그것이 우선
    world  = args.world  or world
    model  = args.model  or model
    sensor = args.sensor or sensor

    gz_img, gz_info, ros_img, ros_info = build_topics(world, model, sensor, args.ros_topic)

    # '[' = Gazebo → ROS2 한 방향 브리지
    cmd = [
        'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        f'{gz_img}@sensor_msgs/msg/Image[gz.msgs.Image',
        f'{gz_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        '--ros-args',
        '-r', f'{gz_img}:={ros_img}',
        '-r', f'{gz_info}:={ros_info}',
    ]
    print(f'[브리지] Gazebo {gz_img}\n     →  ROS2   {ros_img}  (+ {ros_info})')
    print('         확인: ros2 topic hz ' + ros_img + '   /   종료: Ctrl+C\n')

    try:
        sys.exit(subprocess.call(cmd))
    except KeyboardInterrupt:
        print('\n[브리지] 종료')


if __name__ == '__main__':
    main()
