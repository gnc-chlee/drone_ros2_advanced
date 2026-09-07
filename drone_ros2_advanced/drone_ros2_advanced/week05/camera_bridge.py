#!/usr/bin/env python3
# ==============================================================================
# File    : camera_bridge.py  (5주차 1강 - 도우미)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-09-07
# Version : 1.0.0
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
#     1. 월드·기체·센서 이름으로 긴 Gazebo 토픽 이름을 대신 조립
#     2. Gazebo에 그 토픽이 실제로 있는지 확인 (없으면 힌트 출력)
#     3. ros_gz_bridge 를 실행하고, 이름을 /camera/image_raw 로 바꿔줌
#        → 이후 모든 비전 노드는 /camera/image_raw 만 구독하면 된다
#
#   사전 설치 (한 번만):
#     sudo apt install ros-humble-ros-gzharmonic
#
#   실행 방법 (터미널 4개):
#     터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w05_camera_bridge          # ← 이 스크립트
#     터미널 4: ros2 run drone_ros2_advanced w05_camera_viewer
#
#   월드나 기체가 다르면 인자로 지정:
#     ros2 run drone_ros2_advanced w05_camera_bridge --world default
#     ros2 run drone_ros2_advanced w05_camera_bridge --model x500_depth_0 --sensor IMX214
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


def build_topics(world: str, model: str, sensor: str, ros_image: str):
    """Gazebo 토픽 이름 조립 + ROS2 쪽 이름 결정"""
    base    = f'/world/{world}/model/{model}/link/camera_link/sensor/{sensor}'
    gz_img  = base + '/image'
    gz_info = base + '/camera_info'
    ros_info = ros_image.rsplit('/', 1)[0] + '/camera_info'   # /camera/image_raw → /camera/camera_info
    return gz_img, gz_info, ros_image, ros_info


def check_gz_topic(gz_img: str) -> None:
    """Gazebo에 토픽이 있는지 확인 — 없으면 원인 힌트 출력 (브리지는 그래도 실행)"""
    if shutil.which('gz') is None:
        print('[안내] gz 명령을 찾지 못해 토픽 확인을 건너뜁니다.')
        return
    try:
        result = subprocess.run(['gz', 'topic', '-l'],
                                capture_output=True, text=True, timeout=10)
    except subprocess.TimeoutExpired:
        print('[안내] Gazebo 응답이 없습니다 — PX4 SITL이 실행 중인지 확인하세요.')
        return

    topics = result.stdout.split()
    if gz_img in topics:
        print(f'[확인] Gazebo 토픽 있음: {gz_img}')
        return

    print(f'[경고] Gazebo에서 다음 토픽을 찾지 못했습니다:\n       {gz_img}')
    print('  - PX4 SITL을 카메라 기체로 실행했나요?')
    print('      PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down')
    image_topics = [t for t in topics if 'image' in t]
    if image_topics:
        print('  - 현재 Gazebo에 있는 image 토픽:')
        for t in image_topics:
            print('      ', t)
        print('  - 월드/기체 이름이 다르면 --world / --model / --sensor 로 맞춰주세요')
    else:
        print('  - Gazebo에 image 토픽이 하나도 없습니다 (아직 뜨는 중이거나 카메라 없는 기체)')
    print('  → 브리지는 일단 실행합니다. 토픽이 생기면 자동으로 연결됩니다.\n')


def main():
    parser = argparse.ArgumentParser(description='Gazebo 카메라 → ROS2 브리지 도우미')
    parser.add_argument('--world',  default='aruco',               help='Gazebo 월드 이름 (기본: aruco)')
    parser.add_argument('--model',  default='x500_mono_cam_down_0', help='기체 모델 이름 (기본: x500_mono_cam_down_0)')
    parser.add_argument('--sensor', default='imager',              help='카메라 센서 이름 (mono_cam: imager, x500_depth: IMX214)')
    parser.add_argument('--ros-topic', default='/camera/image_raw', help='ROS2 쪽 이미지 토픽 이름')
    args, _ = parser.parse_known_args()   # ros2 run 이 붙이는 --ros-args 는 무시

    # ros_gz_bridge 설치 확인
    if subprocess.run(['ros2', 'pkg', 'prefix', 'ros_gz_bridge'],
                      capture_output=True).returncode != 0:
        print('[오류] ros_gz_bridge 패키지가 없습니다. 먼저 설치하세요:')
        print('       sudo apt install ros-humble-ros-gzharmonic')
        sys.exit(1)

    gz_img, gz_info, ros_img, ros_info = build_topics(
        args.world, args.model, args.sensor, args.ros_topic)

    check_gz_topic(gz_img)

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
