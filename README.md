# drone_ros2_advanced

세종사이버대학교 드론로봇융합학과 **로봇운영체제(ROS2)응용** 실습 패키지 (2026-2학기)

## 환경 (버전 고정)
| 구성요소 | 버전 |
|----------|------|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| PX4 | v1.16.0 |
| px4_msgs | v1.16.0 (PX4와 동일 태그) |
| Gazebo | Harmonic |
| QGroundControl | v4.4.5 (v5는 Ubuntu 22.04 미지원) |
| 통신 브리지 | uXRCE-DDS |

## 설치 방법
```bash
cd ~/ros2_ws/src
git clone https://github.com/gnc-chlee/drone_ros2_advanced.git
cd ~/ros2_ws
colcon build --packages-select drone_ros2_advanced
source install/setup.bash
```

추가 설치 (해당 주차에):
```bash
# 5주차~ 비전 실습: Gazebo 카메라 → ROS2 브리지 (Humble + Harmonic 조합용) + 이미지 확인 도구
sudo apt install ros-humble-ros-gzharmonic ros-humble-rqt-image-view
# 5주차~ OpenCV (ArUco 포함 — 4.7 이상 필요)
pip install 'opencv-python>=4.7'
# 14주차 YOLO
pip install ultralytics
```
※ Fortress용 `ros-humble-ros-gz*`가 이미 설치돼 있으면 충돌하므로 먼저 제거: `apt list --installed | grep ros-humble-ros-gz`

## 실습 코드는 두 가지 버전!

| 버전 | 설명 |
|------|------|
| **raw** | PX4 토픽에 직접 접근. QoS, heartbeat, 명령 조립까지 전부 코드에 보임 |
| **base** | `px4_base.py`(PX4Base 클래스)를 상속. 어려운 부분은 Base가 처리하고 로직만 작성 |

전반부에는 raw로 원리를 눈으로 확인하고, 뒤로 갈수록 base를 사용합니다.
코드에서 `[복붙 영역]`이라고 표시된 부분은 **복사해서 쓰고 원리만 이해**하면 됩니다.
비전 노드(감지기 등 드론 제어와 무관한 노드)는 raw/base 구분이 없습니다.

## 커리큘럼 & 실습 코드

| 주차 | 내용 | 실행 명령 (`ros2 run drone_ros2_advanced ...`) |
|------|------|------|
| 1주차 | PX4-ROS2 개요 / PX4 SITL 개발환경 구축 | - |
| 2주차 | PX4-ROS2 연동 / 키보드 제어 노드 실습 | `first_node`, `position_listener`, `keyboard_control` |
| 3주차 | 단일 Waypoint / 다중 Waypoint 비행 설계 | `w03_takeoff_*`, `w03_multi_*`, `w03_yaml_*`, `w03_mission_*` |
| 4주차 | Gazebo World 구조와 SDF / 커스텀 World 실습 | `worlds/my_custom_world.sdf` (+ `w03_mission_raw` 재사용) |
| 5주차 | ROS2 카메라 토픽과 OpenCV / ArUco 마커 인식 | `w05_camera_bridge` + `w05_camera_viewer`, `w05_contour`, `w05_aruco` |
| 6주차 | 마커 기준 오차 계산과 제어 / 정밀착륙 노드 | `w06_center_error` / 정밀착륙 조합: `w05_aruco` + `w06_keyboard_ab` + `w06_precision_land` |
| 7주차 | **중간고사** | - |
| 8주차 | OpenCV DNN 기반 객체 인식 / 사람 인식 노드 | `w08_face_detector` (DNN판 추가 예정) |
| 9주차 | 사람 인식·추종 비행 제어 설계 / 추종 비행 실습 | `w09_face_command`, `w09_p_control`, `w09_follow_*` |
| 10주차 | 거리 센서 개념 / LiDAR 고도 데이터 활용 | (추가 예정) |
| 11주차 | 장애물 감지 원리 / 회피 비행 노드 | (추가 예정) |
| 12주차 | 상태머신 기반 미션 설계 / 모드 전환 로직 | (추가 예정) |
| 13주차 | 통합 시나리오 설계 / 통합 미션 구현·디버깅 | `w13_stable_raw` / `w13_stable_base` |
| 14주차 | 통합 미션 시연 / 심화 주제 소개 (RL, YOLO) | `w14_yolo`, `w14_haar_vs_yolo` |
| 15주차 | **기말고사** | - |

## 기본 실행 순서 (비행 실습 공통)
```bash
# 터미널 1: PX4 SITL + Gazebo
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# 터미널 2: uXRCE-DDS Agent (PX4 ↔ ROS2 다리)
MicroXRCEAgent udp4 -p 8888

# 터미널 3: 실습 노드
source ~/ros2_ws/install/setup.bash
ros2 run drone_ros2_advanced position_listener
```

## 5~6주차 카메라 실습 실행 순서
Gazebo 카메라 이미지는 uXRCE-DDS Agent로 넘어오지 않습니다 — **ros_gz_bridge라는 두 번째 다리**가 필요합니다.
```bash
# 터미널 1: 하방 카메라 기체 + ArUco 마커 월드 (PX4 내장). VM이면 HEADLESS=1 을 앞에 붙여 GUI 부하를 줄일 수 있음
cd ~/PX4-Autopilot && PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down

# 터미널 2: uXRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# 터미널 3: 카메라 브리지 (Gazebo → /camera/image_raw)
ros2 run drone_ros2_advanced w05_camera_bridge

# 터미널 4: 실습 노드
ros2 run drone_ros2_advanced w05_camera_viewer
```
- 확인: `ros2 topic hz /camera/image_raw` 에 수치가 찍히면 성공. `rqt_image_view` 로 화면도 볼 수 있음
- 월드/기체가 다르면 브리지에 인자: `w05_camera_bridge --world default --model x500_depth_0 --sensor IMX214`
- 6주차 접근 실습(마커에서 떨어져 시작): `PX4_GZ_MODEL_POSE="2,1,0,0,0,0" PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down`
- VM에서 카메라가 너무 느리면 `~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf` 의 해상도를 640x480, update_rate 를 15 로 낮춰 보세요

## 4주차 커스텀 World
```bash
# 1) PX4 월드 폴더로 복사 (PX4가 월드를 찾는 경로가 고정돼 있음)
cp drone_ros2_advanced/worlds/my_custom_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
# 2) 월드를 지정해 실행
cd ~/PX4-Autopilot && PX4_GZ_WORLD=my_custom_world make px4_sitl gz_x500
```
- 규칙: **파일 이름과 `<world name>` 이 같아야** 함 (다르면 `Timed out waiting for Gazebo world`)
- 규칙: `<world>` 바로 아래에 `<plugin>` 을 쓰지 말 것 (PX4가 자동으로 붙이는 13개 시스템이 취소됨)
- 자세한 설명과 좌표 변환표는 [`worlds/README.md`](drone_ros2_advanced/worlds/README.md)

## 폴더 구조
```
drone_ros2_advanced/
├── config/waypoints.yaml      # 3주차~ waypoint 미션 설정
└── drone_ros2_advanced/
    ├── px4_base.py            # PX4Base 클래스 (base 버전의 부모)
    ├── week02/                # 노드 기초 + 키보드 제어
    ├── week03/                # 단일·다중 Waypoint 비행
    ├── worlds/                # 4주차 커스텀 Gazebo World (PX4 폴더로 복사해 사용)
    ├── week05/                # 카메라 · OpenCV · ArUco
    ├── week06/                # 오차 제어 · 정밀착륙
    ├── week08/                # 객체 인식
    ├── week09/                # 사람 추종 비행
    ├── week13/                # 추종 안정화 (통합 미션)
    ├── week14/                # 심화 (YOLO)
    └── extras/                # 구 커리큘럼 GPS/지도 자료 (참고용, folium·flask 필요)
```
※ week10~12(LiDAR·회피·상태머신)는 교안 제작 진도에 맞춰 추가됩니다. 4주차는 파이썬 노드가 아니라 `worlds/` 의 SDF 파일이 실습 자산입니다.

## 참고
- 트러블슈팅: Gazebo 화면이 검게 나오면 `export LIBGL_ALWAYS_SOFTWARE=1` (VM 환경)
- 트러블슈팅: `/camera/image_raw` 가 안 보이면 → 터미널 3의 `w05_camera_bridge` 실행 여부, `gz topic -l | grep image` 로 Gazebo 쪽 토픽 확인
- 비전 실습(5주차~)은 카메라 렌더링 부담이 큼 — VM에서는 `HEADLESS=1` + 해상도 축소 권장, GPU가 있는 네이티브 Ubuntu면 더 원활
- YOLOv8n은 CPU 환경에서도 동작 가능 (저사양 PC 대응)
