# drone_ros2_advanced

세종사이버대학교 드론로봇융합학과 **로봇운영체제(ROS2)응용** 실습 패키지 (2026-2학기)

## 환경 (버전 고정)
| 구성요소 | 버전 |
|----------|------|
| OS | Ubuntu 22.04 |
| ROS2 | Humble |
| PX4 | v1.16.0 |
| px4_msgs | `release/1.16` 브랜치 (PX4 v1.16.0 과 짝) — [확인·전환 방법](#6주차-시작-전에-한-번만--px4_msgs-버전-맞추기) |
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
# 5주차~ OpenCV — 반드시 4.7 이상 (시스템에 딸려온 4.5.4 에는 aruco.ArucoDetector 가 없음).
#          numpy 는 1.x 로 고정 (Humble 이 numpy 1 기준이라 2.x 로 올라가면 일부 패키지가 깨질 수 있음)
pip install 'opencv-python>=4.7' 'numpy<2'
# 설치 확인 — 버전이 4.7 이상으로 찍히고 에러가 없으면 OK
python3 -c "import cv2; print(cv2.__version__); cv2.aruco.ArucoDetector"
# 14주차 YOLO
pip install ultralytics
```
※ Fortress용 `ros-humble-ros-gz*`가 이미 설치돼 있으면 충돌하므로 먼저 제거: `apt list --installed | grep ros-humble-ros-gz`

## 6주차 시작 전에 한 번만 — px4_msgs 버전 맞추기

PX4 는 v1.16.0 인데 px4_msgs 를 브랜치 지정 없이 받았다면 최신판(main)이 설치돼 있습니다. 이 경우 드론의 상태 메시지(`VehicleStatus`) 형식이 PX4 와 달라서, 6주차부터 쓰는 PX4Base 노드가 **이륙만 하고 "Offboard 모드 전환 / Arm 명령 전송"을 1초마다 반복**하며 다음 단계로 넘어가지 않습니다.

```bash
# 1) 확인 — "MESSAGE_VERSION = 1" 이면 이미 맞음 (아래는 건너뜀)
ros2 interface show px4_msgs/msg/VehicleStatus | grep MESSAGE_VERSION
```

```bash
# 2) = 1 이 아니면(다른 숫자·빈 출력) PX4 v1.16 짝으로 전환 후 다시 빌드 (몇 분 걸림)
cd ~/ros2_ws/src/px4_msgs && git fetch origin && git checkout release/1.16
cd ~/ros2_ws && colcon build --packages-select px4_msgs && source install/setup.bash
```

- 다시 확인해서 `= 1` 이면 끝입니다. 이미 열어 둔 터미널에는 `source ~/ros2_ws/install/setup.bash` 를 다시 하세요
- 확인 명령에서 `Unknown package` 오류가 나면 먼저 `source ~/ros2_ws/install/setup.bash`
- 처음부터 새로 설치한다면: `cd ~/ros2_ws/src && git clone -b release/1.16 https://github.com/PX4/px4_msgs.git` 후 위 2) 의 둘째 줄로 빌드
- 2~5주차는 상태 메시지를 읽지 않았고, 쓰던 위치·자세·명령 메시지는 두 버전의 형식이 같아서 영향이 없었습니다

## 매주 실습 전에 — 최신 코드 받기

주차마다 코드와 자료가 추가됩니다. **각 주차 실습을 시작하기 전에 아래 두 줄을 그대로 실행**하세요.

```bash
cd ~/ros2_ws/src/drone_ros2_advanced && git fetch origin && git reset --hard origin/main
cd ~/ros2_ws && colcon build --packages-select drone_ros2_advanced && source install/setup.bash
```

- 첫 줄: 내 컴퓨터의 저장소를 **GitHub와 똑같이** 맞춥니다. 실습하면서 제공 파일을 고쳤어도 상관없이 **항상 성공**합니다
- 둘째 줄: 새로 추가된 노드를 `ros2 run` 으로 실행할 수 있게 빌드합니다 (빌드를 빼먹으면 "No executable found")
- 확인: `ros2 pkg executables drone_ros2_advanced` 에 그 주차 노드 이름(예: 8주차면 `w08_…`)이 보이면 끝

### 이 명령이 하는 일 / 안 하는 일

| | 결과 |
|---|---|
| 제공된 파일(`waypoints.yaml`, `keyboard_control.py` 등)을 내가 고친 것 | **원래대로 돌아감** (실험은 다시 하면 됨) |
| 내가 **새로 만든** 파일(`my_node.py`, `my_mission.yaml`, 내 월드 등) | **그대로 남음** |
| `setup.py` 에 내가 추가한 줄(2주차 `my_node` 등록) | 원래대로 돌아감 — 필요하면 한 줄 다시 추가 |

> 고친 내용을 남겨두고 싶으면 첫 줄 실행 **전에** `git diff > ~/내수정_$(date +%m%d).patch` 로 백업해 두세요. 나중에 `cat` 으로 열어볼 수 있습니다.

### 충돌을 아예 피하는 습관

제공된 파일을 직접 고치는 대신 **내 파일을 새로 만들어** 쓰면 위 명령에도 사라지지 않습니다.

```bash
# waypoints.yaml 을 고치지 말고, 복사해서 내 미션을 만든다
cp config/waypoints.yaml config/my_mission.yaml
```

```bash
# 실행할 때 내 파일을 지정 (config/*.yaml 은 빌드하면 자동 설치됨)
ros2 run drone_ros2_advanced w03_mission_raw --ros-args -p waypoint_file:=$(ros2 pkg prefix drone_ros2_advanced)/share/drone_ros2_advanced/config/my_mission.yaml
```

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
| 5주차 | ROS2 카메라 토픽과 OpenCV / ArUco 마커 인식 | `w05_camera_bridge` + `w05_camera_viewer`, `w05_aruco` (심화: `w05_aruco_hud` / 참고: `w05_contour` 웹캠 윤곽선) |
| 6주차 | 마커 기준 오차 계산과 제어 / 정밀착륙 노드 | `w06_align` (마커 위 정렬) / `w06_land` (정밀착륙) — **PX4Base 첫 사용** (참고: 키보드 조합 `w06_keyboard_ab` + `w06_precision_land` — 사람 ↔ 자율 전환 예시) |
| 7주차 | **중간고사** | - |
| 8주차 | OpenCV DNN 기반 객체 인식 / 사람 인식 노드 | `w08_dnn_demo` (13강: 사진·웹캠으로 DNN 4단계) / `w08_person` + `w08_video_pub`, `worlds/person_world.sdf` (14강: 영상 → 드론 정면 카메라에서 사람 찾기 → `/sjcu/person_error`) (참고: `w08_face_detector` Haar 원형) |
| 9주차 | 사람 인식·추종 비행 제어 설계 / 추종 비행 실습 | `w09_follow` (+ 14강 구성 그대로: `w05_camera_bridge`·`w08_person`·`person_world`) — 15강: 드라이 런(비행하지 않고 명령 값만 확인) / 16강: 회전만 → 비행 중에 전진 속도 켜기(단계적 통합, incremental integration) |
| 10주차 | 거리 센서 개념과 ROS2 토픽 연동 / LiDAR 고도 데이터 활용 실습 | `w10_lidar_bridge` + `w10_range_viewer`, `worlds/lidar_world.sdf` (17강: 하방 LiDAR 거리를 두 경로 — 브리지 원시값 `/lidar/down` · PX4 추정값 `dist_bottom` — 로 읽고 대지 고도 확인) / `w10_terrain_follow` (18강: 드라이 런 → 제자리 고도 유지 → 비행 중에 전진 속도 켜기로 상자 넘기 = 지형 추종, terrain following) |
| 11주차 | 장애물 감지 원리와 회피 전략 / 회피 비행 노드 실습 | `w11_scan_bridge` + `w11_scan_viewer`, `worlds/obstacle_world.sdf` (19강: 2D LiDAR 스캔 1080칸 → 앞·왼쪽·오른쪽 섹터 최근접 거리, 정지 거리) / `w11_avoid` (20강: 드라이 런 → 제자리 호버 → 비행 중에 전진 켜기로 기둥 앞 정지 → 넓은 쪽 좌우 이동 → 코스 복귀 → 도착 = 반응형 회피, reactive avoidance) |
| 12주차 | 상태머신 기반 미션 설계 / 모드 전환 로직 | `w12_fsm_sim` (21강: 전이 조건표 시험 · 결함 주입) / `w12_mission` + `w12_state_viewer` (+ 11주차 `w11_scan_bridge`·`obstacle_world`) (22강: 켜면 이륙부터 착륙까지 → 브리지 끄기 → PX4 모드 변경) |
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
# 터미널 1: 하방 카메라 기체 + 4주차 내 월드 (마커는 동쪽 5m·북쪽 3m). VM이면 HEADLESS=1 을 앞에 붙여 GUI 부하를 줄일 수 있음
cd ~/PX4-Autopilot && PX4_GZ_WORLD=my_custom_world make px4_sitl gz_x500_mono_cam_down
#   (Fuel 모델을 못 받았거나 마커를 원점에서 바로 보고 싶으면: PX4_GZ_WORLD=aruco)

# 터미널 2: uXRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888

# 터미널 3: 카메라 브리지 (Gazebo → /camera/image_raw)
ros2 run drone_ros2_advanced w05_camera_bridge

# 터미널 4: 실습 노드
ros2 run drone_ros2_advanced w05_camera_viewer    # 1강: 카메라 보기
ros2 run drone_ros2_advanced w05_aruco            # 2강: 마커 찾기 → /sjcu/error 발행
```
- `w05_aruco` 는 검출 코어만 담은 수업용 최소판. HUD(위치 패널·게이지)가 붙은 심화판은 `w05_aruco_hud`
- **6주차**: 위 4개(PX4·Agent·브리지·`w05_aruco`)를 켠 채 터미널 5에서
  ```bash
  ros2 run drone_ros2_advanced w06_align     # 1강: 마커 근처로 날아가 카메라로 정렬·호버
  ros2 run drone_ros2_advanced w06_land      # 2강: 정렬 → 하강 → 1m 에서 PX4 착륙
  ros2 run drone_ros2_advanced w06_land --ros-args -p kp:=3.0          # 게인 바꿔보기
  ros2 run drone_ros2_advanced w06_align --ros-args -p marker_n:=0.0 -p marker_e:=0.0   # aruco 월드(마커 원점)
  ```
- 오차 확인: `ros2 topic echo /sjcu/error` — 마커를 찾은 프레임에서만 `[x, y, size]` 픽셀이 흐릅니다
- 확인: `ros2 topic hz /camera/image_raw` 에 수치가 찍히면 성공. `rqt_image_view` 로 화면도 볼 수 있음
- 브리지는 Gazebo의 카메라를 **자동 감지**합니다 (월드가 `my_custom_world` 든 `aruco` 든 그대로). 카메라가 여러 개면 `--world`/`--model`/`--sensor` 로 지정
- 마커 위에서 바로 시작하려면 (10강 지름길): `PX4_GZ_MODEL_POSE="5,3,0,0,0,0" PX4_GZ_WORLD=my_custom_world make px4_sitl gz_x500_mono_cam_down` (좌표는 Gazebo ENU)
- VM에서 카메라가 너무 느리면 `~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam/model.sdf` 의 해상도를 640x480, update_rate 를 15 로 낮춰 보세요

## 8주차 OpenCV DNN 데모 (13강)
Gazebo·드론 없이 사진으로 OpenCV DNN(MobileNet-SSD)의 4단계(읽기 → 전처리 → 추론 → 해석)를 봅니다.
ROS 통신은 쓰지 않는 OpenCV 프로그램이라 **PX4·Agent·브리지 없이 터미널 1개**면 됩니다.
```bash
# 먼저 한 번: OpenCV 가 4.x 인지 확인 (5.x 에는 readNetFromCaffe 가 없음)
python3 -c "import cv2; print(cv2.__version__); cv2.dnn.readNetFromCaffe"
#   5.x 가 나오면 → pip install 'opencv-python==4.11.0.86' 'numpy<2'

ros2 run drone_ros2_advanced w08_dnn_demo                          # week08/samples/ 의 예시 사진
ros2 run drone_ros2_advanced w08_dnn_demo ~/Pictures/my_photo.jpg  # 내 사진 (폴더·영상 파일도 됨)
ros2 run drone_ros2_advanced w08_dnn_demo cam                      # 웹캠 0번 (1번이면 cam1)
```
- 조작: 슬라이더 = 신뢰도 기준값(25~95%) · `n`/스페이스 = 다음 사진 · `t` = 결과 표를 터미널에 출력 · `s` = 화면 저장 · `q` = 종료
- 모델 파일은 저장소 `week08/models/` 에 들어 있고(MIT, 출처·체크섬은 그 폴더 README), **빌드해야 설치 경로에 복사**됩니다 — 코드 최신화 뒤 `colcon build` 를 꼭 다시 하세요
- 실습으로 `week08/dnn_demo.py` 를 고친 뒤에도 `colcon build` → `source install/setup.bash` 를 해야 반영됩니다

## 8주차 사람 인식 노드 (14강)
13강의 DNN 4단계를 5주차 `aruco_detector` 틀에 넣은 노드 `w08_person` 이 `/camera/image_raw` 에서 사람을 찾아
`/sjcu/person_error` = `[x_err, y_err, size_err, box_h, conf]` 를 발행합니다 (앞 3칸은 5주차 `/sjcu/error` 와 같은 약속, **찾았을 때만** 발행).
노드는 토픽 이름만 알기 때문에 **입력만 바꿔 끼우면** 녹화 영상에서도, 드론 정면 카메라에서도 그대로 동작합니다. 14강은 비행하지 않습니다.

### 0) 준비 (한 번만)
```bash
# 사람 월드 + 사람 모델(걷는 사람 walk.dae, 사진 입간판)을 PX4 폴더로 복사
cd ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/worlds
cp person_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cp -r models/* ~/PX4-Autopilot/Tools/simulation/gz/models/

# (8주차 표준) 카메라를 640x480 @ 15Hz 로 — 모델 입력은 늘 300x300 이라 검출 손해 없이 부하만 약 1/4
cd ~/PX4-Autopilot/Tools/simulation/gz/models/mono_cam
sed -i 's|<width>1280</width>|<width>640</width>|; s|<height>960</height>|<height>480</height>|; s|<update_rate>30</update_rate>|<update_rate>15</update_rate>|' model.sdf
grep -E "<width>|<height>|<update_rate>" model.sdf     # 640 / 480 / 15 이면 OK (하방 기체도 같이 바뀜 — 6주차 노드는 영향 없음)
```

### 1) 실습 1 — 녹화 영상으로 (Gazebo 없이, 터미널 2개)
```bash
ros2 run drone_ros2_advanced w08_video_pub        # week08/samples/ 의 동봉 영상 → /camera/image_raw (반복)
ros2 run drone_ros2_advanced w08_person           # 사람 박스 + 상태줄
ros2 topic echo /sjcu/person_error                # (확인용) 숫자 5개
```
- 동봉 영상은 Gazebo 정면 카메라 녹화본(`gazebo_front.mp4`). 휴대폰으로 찍은 걷는 영상이 있으면 그걸 써도 됩니다: `w08_video_pub ~/Videos/walk.mp4` / `w08_video_pub cam` (큰 영상은 가로 640 으로 줄여 보냄)
- 일부러 틀리기: `ros2 run drone_ros2_advanced w08_person --ros-args -p target_class_id:=1` → VOC 1번은 비행기라 오류 없이 못 찾음

### 2) 실습 2 — 드론 정면 카메라로 (비행 없음, 터미널 3개)
```bash
# 터미널 1: 정면 카메라 기체 + 사람 월드 (HEADLESS=1 은 명령 앞에만 — export 하지 말 것)
cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=person_world make px4_sitl gz_x500_mono_cam
# 터미널 2: pxh> 가 뜬 뒤에 브리지 — [감지] ... model=x500_mono_cam_0 줄 확인
ros2 run drone_ros2_advanced w05_camera_bridge
# 터미널 3: 실습 1 과 같은 명령
ros2 run drone_ros2_advanced w08_person
```
- 사람은 드론 앞 4 m ↔ 8 m 를 한 바퀴(시뮬레이션 약 76초) 돕니다: 멀어짐(뒷모습) → 8 m 정지(앞모습) → 다가옴(앞모습) → 4 m 가로지름(옆모습) → 4 m 정지(앞모습)
- 창 왼쪽 위: `person 신뢰도  x=… y=…  h=박스 높이px (화면 높이 비율)` / 오른쪽 끝 파란 막대 = 목표 박스 높이(`target_box_ratio` × 화면 높이)
- 상태줄(왼쪽 아래): `threshold 기준값 | 처리 ms | pub 발행 Hz | found 검출률 %` — 발행 Hz·검출률은 **기준값을 바꾼 뒤부터** 최대 10초 평균이라, 슬라이더를 옮기면 2~3초 기다렸다 읽기. `ros2 topic hz -w 10 /sjcu/person_error` 로도 확인
- 조작(창 클릭 후): 슬라이더 = 기준값(25~95%) · `s` = 원본 + 화면 저장 · `q` = 종료
- 재빌드 없이 바꾸기: `--ros-args -p conf_threshold:=0.5` · `-p target_box_ratio:=0.33` · `-p max_rate:=3.0` (VM 이 버거우면)

### 막혔을 때
| 증상 | 원인 → 해결 |
|------|-------------|
| 창에 `Waiting for camera ...` | 영상이 안 옴 → `ros2 topic hz /camera/image_raw` 확인. 실습 2 면 브리지를 `pxh>` 뒤에 켰는지 |
| 브리지가 `[경고] … 기본값으로 시도` + 하방 기체 안내 | Gazebo 가 뜨기 전에 켬 → 그 안내는 무시하고 Ctrl+C → `pxh>` 확인 → 다시. 직접 지정: `--world person_world --model x500_mono_cam_0 --sensor imager` (셋 다) |
| 화면 두 개가 번갈아 보임 | 영상 발행기와 브리지가 동시에 켜짐 → `ros2 topic info /camera/image_raw` 의 Publisher count 가 1 이 되게 하나를 끔 |
| 사람이 안 보임 | 사람 모델 복사 누락 → `ls ~/PX4-Autopilot/Tools/simulation/gz/models \| grep sjcu` (sjcu_walker 가 있어야 함) |
| 사람이 땅에 반쯤 묻힘 | 월드를 고치며 actor waypoint `<pose>` 의 z 를 0 으로 둠 → z = 1.0 (walk.dae 원점이 허리 높이, 4주차 종이상자 z 함정과 같음) |
| `gazebo already running world:` | 이전 Gazebo 서버가 남음 → `pkill -9 -f "gz sim"` → `pgrep -af "gz sim"` 빈 줄 확인 후 다시 |
| `readNetFromCaffe 가 없습니다` / `모델 파일이 없습니다` | 13강 0부와 같음 — OpenCV 4.11 로 되돌리기 / `colcon build` 다시 |

- 강사용: `worlds/person_check_world.sdf` (사람이 3·4·5·6·8 m 에서 약 10초씩 정지 — 거리별 신뢰도 측정)
- 강사용 녹화: `w08_video_pub --record gazebo_front.mp4` (지금 폴더에 저장, Ctrl+C 로 끝) → 바로 재생은 `w08_video_pub ./gazebo_front.mp4`. `w08_video_pub gazebo` 로 쓰려면 src 의 `week08/samples/` 로 옮긴 뒤 `colcon build --packages-select drone_ros2_advanced` → `source install/setup.bash`

## 9주차 사람 추종 비행 (15·16강)
14강의 `/sjcu/person_error` 를 받아 **화면 가운데, 같은 크기**로 사람(이동 표적)을 추종하는 노드 `w09_follow` (6주차 `w06_align` 뼈대, PX4Base).
- 가로 오차 `x_err` → **요 각속도** (오른쪽에 있으면 오른쪽으로 회전) / 크기 오차 `size_err` → **전진 속도** (작게 보이면 전진, 크게 보이면 후진) / 고도 2 m 는 따로 유지
- 오차가 1초(타임아웃) 동안 안 오면 **표적 상실(target loss)** → 화면 가장자리에서 사라졌으면 그쪽으로 천천히 돌며 재탐색(최대 한 바퀴), 아니면 천천히 멈춰 제자리 호버링
- 기본값은 **드라이 런(dry run)** — 비행하지 않고 명령 값만 확인한다. 비행하려면 `-p fly:=true` 를 일부러 붙인다

```bash
# 0) 준비: 8주차 0) 준비(월드·모델 복사, 640x480)를 했으면 그대로. 비행 전 px4_msgs release/1.16 확인 (6주차 절)
# 터미널 1: 정면 카메라 기체 + 사람 월드
cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=person_world make px4_sitl gz_x500_mono_cam
# 터미널 2: uXRCE-DDS Agent (비행할 때만 — 드라이 런은 없어도 됨)
MicroXRCEAgent udp4 -p 8888
# 터미널 3: pxh> 가 뜬 뒤에 브리지
ros2 run drone_ros2_advanced w05_camera_bridge
# 터미널 4: 사람 인식 (14강 노드 그대로)
ros2 run drone_ros2_advanced w08_person --ros-args -p conf_threshold:=0.3
# 터미널 5: 추종 노드 — 단계별로
ros2 run drone_ros2_advanced w09_follow                                         # 0단계(15강) 드라이 런: 땅에서 부호 확인
ros2 run drone_ros2_advanced w09_follow --ros-args -p fly:=true -p kp_fwd:=0.0  # 1단계(16강) 이륙 → 회전만
ros2 param set /person_follow_base kp_fwd 6.0                                   # 2단계: 새 터미널에서, 비행 중에 전진 속도 켜기
```
- 로그 검산 (명령 = 게인 × 오차 — 6주차 로그에 검산 칸을 붙인 형식)
  - 드라이 런: `[드라이 런] ex=+0.25 → 요 각속도 +0.20 rad/s = 0.8 × +0.25 (오른쪽) | es=-0.020 → 전진 속도 +0.12 m/s = -6 × -0.020 | 추정 거리 약 4.3 m | conf 0.79`
  - 비행: `[FOLLOW] 기수 +92° (실측 요 각속도 +0.20 rad/s) | ex=+0.25 → 요 각속도 +0.20 rad/s = 0.8 × +0.25 (오른쪽) | es=-0.060 → 전진 속도 +0.36 m/s = -6 × -0.060 | 추정 거리 약 5.2 m | 고도 2.0 m | conf 0.79`
  - 최대값에 걸리면 포화(saturation) 표시: `요 각속도 +0.50 rad/s (포화, 0.8 × +1.00 = +0.80)` · 안전장치가 바꾼 값은 추정 거리 뒤(고도 앞)에 `| 보낸 전진 속도 +0.05 (가속도 제한)` (그 밖에 `(근접)`·`(지오펜스)`)
- 비행 중에 바꾸기: `ros2 param set /person_follow_base kp_yaw 0.8` · `kp_fwd 0.0`(전진 속도 끄기) — 0.1초마다 다시 읽으므로 착륙하지 않고 바로 바뀜 (`게인 변경: kp_fwd 0 → 6` 같은 한 줄이 찍힘)
- 일부러 틀리기 (전진 속도를 끈 채로): `ros2 param set /person_follow_base kp_yaw -0.8` → 사람 반대쪽으로 회전(양의 피드백) → 1초 뒤 재탐색 → 되돌리기 `kp_yaw 0.8`
- **끝내는 순서**: PX4 터미널(`pxh>`)에서 `commander land` → 착지 확인 → `w09_follow` Ctrl+C. 비행 중에 노드를 먼저 끄면 1초 뒤 PX4 가 Offboard 신호 끊김 비상 동작으로 넘어갑니다
- 그 밖의 파라미터: `-p search_rate:=0.0` (표적을 상실해도 재탐색하지 않고 제자리 호버링만 — 게인 0 으로 호버링하며 검출만 잴 때도 함께) · 목표 거리는 검출기 쪽에서 `w08_person … -p target_box_ratio:=0.33` (약 3 m)
- 드론은 원점으로 돌아오지 않고 사람을 따라 자리를 옮깁니다(한 바퀴 뒤 원점에서 약 4~5 m). **한 바퀴(약 76초) 추종한 뒤 착륙**하세요. 원점 6 m 지오펜스(geofence)는 전진하며 원점에서 멀어지는 것만 막고, 사람이 가까워서 하는 후진은 막지 않습니다

| 증상 | 원인 → 해결 |
|------|-------------|
| 이륙 뒤 `[HOLD] 표적 대기 (제자리 호버링)` 만 나옴 | 오차가 안 옴 → `ros2 topic hz /sjcu/person_error`, HUD 에 박스가 있는지. 사람이 화면 밖이면 들어올 때까지 표적 대기(고장 아님) |
| 이륙하지 않고 "Offboard 모드 전환 / Arm 명령 전송" 반복 | px4_msgs 가 main → 6주차 절의 `release/1.16` 전환 |
| 사람 반대쪽으로 돈다 | kp_yaw 부호가 음수(양의 피드백) → `ros2 param set /person_follow_base kp_yaw 0.8` |
| HOLD ↔ FOLLOW 가 자주 바뀜 | 검출이 타임아웃(1초)보다 오래 끊김(VM 부하) → `HEADLESS=1`, `conf_threshold:=0.3`, `w08_person` 창 줄이기 |
| HUD 가 갑자기 사라짐 | HUD 창이 선택된 채 `q` → `w08_person` 종료. 키보드·명령 입력은 터미널을 클릭해 둔 뒤에 |
| 구 `w09_follow_raw` 등이 아직 보임 | 예전 빌드의 실행 파일이 install 에 남은 것(정상). 치우려면 `cd ~/ros2_ws && rm -rf build/drone_ros2_advanced install/drone_ros2_advanced && colcon build --packages-select drone_ros2_advanced && source install/setup.bash` |
| HUD 에 바닥만 보임 | 5·6주차 습관으로 `gz_x500_mono_cam_down` 실행 → 브리지 감지 줄이 `model=x500_mono_cam_0` 인지 확인, `make px4_sitl gz_x500_mono_cam` 으로 다시 |
| 사람이 멀어져도 다가가지 않음 | 1단계 그대로(`kp_fwd 0`) → `ros2 param set /person_follow_base kp_fwd 6.0` 뒤 `게인 변경` 줄 확인. `근접`·`지오펜스` 경고가 찍히면 안전장치가 막는 중 |
| `ros2 param set` 이 노드를 못 찾음 | 노드 이름 오타 → `ros2 node list` 에서 `/person_follow_base` 확인 |

## 10주차 거리 센서 · LiDAR 지형 추종 (17·18강)
드론 아래를 보는 **거리 센서(distance sensor)** — 빔 1개짜리 하방 **라이다(LiDAR)** — 로 바로 아래 면까지의 거리를 재고(17강), 그 값으로 높이 1 m 상자를 넘으며 대지 고도를 지키는 노드 `w10_terrain_follow` 를 실행합니다(18강, PX4Base).
- 높이 두 가지: **상대 고도(relative altitude)** = 이륙 지점 기준 높이(NED `−z` — 2·6·9주차의 '고도') / **대지 고도(AGL, above ground level)** = 바로 아래 면(땅·상자 윗면)까지의 높이 = LiDAR − **장착 오프셋(mounting offset)** 약 0.18 m (계산). 평지에서는 둘이 같고, 상자 위에서는 대지 고도만 1 m 줄어듭니다
- 거리 값이 ROS2 로 오는 **두 경로** (PX4 v1.16.0 기본 설정에는 `/fmu/out/distance_sensor` 가 없음): ① 브리지 `w10_lidar_bridge` → `/lidar/down` (`sensor_msgs/LaserScan`, **원시값**, `ranges[0]`) / ② PX4 `/fmu/out/vehicle_local_position` 의 `dist_bottom` (EKF **추정값** — 값보다 `dist_bottom_valid` 를 먼저 확인). PX4 를 다시 빌드해 `/fmu/out/distance_sensor` 를 여는 경로 ③ 은 아래 3) 심화
- 18강 고도 한 줄은 9주차와 같은 비례 제어 `vz = kz × (높이 − 목표)` 에서 **높이만 상대 고도 → 대지 고도로** 바꾼 것입니다(**지형 추종, terrain following**). 기본값은 **드라이 런(dry run)** + 전진 속도 0(제자리) — 비행하려면 `-p fly:=true` 를 일부러 붙입니다

### 0) 준비 (한 번만)
```bash
# LiDAR 월드를 PX4 폴더로 복사 — 평지 + 상자 1개(동쪽 4~7 m, 남북 폭 4 m, 높이 1.0 m), Fuel·인터넷 불필요
cd ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/worlds
cp lidar_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
# 브리지는 5주차에 설치한 ros-humble-ros-gzharmonic 그대로 — 설치 확인 (Installed 줄에 버전이 찍히면 OK)
apt-cache policy ros-humble-ros-gzharmonic-bridge
```
- 기체는 PX4 내장 **`gz_x500_lidar_down`** — 카메라 없음, 기체 중심 아래 5 cm 에 아래를 보는 LiDAR 1개(측정 범위 0.1~100 m, 갱신 주기 50 Hz). 5·6·8·9주차 습관으로 다른 기체를 띄우면 LiDAR 가 없습니다
- 17강은 위치·자세 메시지만 읽어 px4_msgs 가 main 이어도 되지만, **18강 비행 전에는 `release/1.16` 필수** ([6주차 절](#6주차-시작-전에-한-번만--px4_msgs-버전-맞추기))

### 1) 17강 — 거리 값 읽기 (비행 없음, 키보드 비행은 선택)
```bash
# 터미널 1: 하방 LiDAR 기체 + LiDAR 월드 (상자를 보려면 GUI 켬 — VM 이 버거우면 앞에 HEADLESS=1)
cd ~/PX4-Autopilot && PX4_GZ_WORLD=lidar_world make px4_sitl gz_x500_lidar_down
#   pxh> listener distance_sensor     ← 코드 없이 첫 확인: current_distance 약 0.18 (땅), orientation 25 (아래)
# 터미널 2: uXRCE-DDS Agent (경로 ② dist_bottom 에 필요)
MicroXRCEAgent udp4 -p 8888
# 터미널 3: pxh> 가 뜬 뒤에 LiDAR 브리지 (경로 ①) — [감지] world=lidar_world model=x500_lidar_down_0 → /lidar/down 줄 확인
ros2 run drone_ros2_advanced w10_lidar_bridge
# 터미널 4: 측정 뷰어 — 비행하지 않음 (받아서 0.5초마다 한 줄 찍기만)
ros2 run drone_ros2_advanced w10_range_viewer
# 터미널 5 (선택): 2주차 키보드로 상자 위 통과 — T 이륙 → 약 2 m 에서 Space → W 한 번(동쪽 0.5 m/s) → 상자를 지나가면 Space
ros2 run drone_ros2_advanced keyboard_control
```
- 확인 명령 (새 터미널)
  - `ros2 topic list | grep distance` → `/fmu/in/distance_sensor`, `/fmu/in/obstacle_distance` 두 줄뿐 (둘 다 in = 내가 PX4 에 넣는 쪽 — PX4 가 내보내는 out 은 없음). 뷰어(터미널 4)를 켠 뒤에는 `/fmu/out/distance_sensor` 한 줄이 더 보이는데, 뷰어가 3) 심화의 경로 ③ 자리로 **구독만 해 둔 이름**입니다 — `ros2 topic info /fmu/out/distance_sensor` 의 `Publisher count: 0` (PX4 는 보내지 않음, `Subscription count` 가 뷰어)
  - 경로 ①: `ros2 topic echo --once /lidar/down` → `range_min: 0.1` · `range_max: 100.0` · `ranges:` 에 0.18 근처 한 칸 (빔 1개라 `angle_increment` 는 nan 일 수 있음). 발행 주기: `ros2 topic hz -w 50 /lidar/down` (약 50 Hz)
  - 경로 ②: `ros2 topic echo /fmu/out/vehicle_local_position --field dist_bottom` (Ctrl+C 로 멈춤) — 같은 방법으로 `dist_bottom_valid`(믿어도 되는가), `dist_bottom_sensor_bitfield`(1 = 거리 센서로 추정)
- 뷰어 로그 검산 (숫자는 계산 예 — 내 화면과 비교)
  - 땅: `[측정] LiDAR 0.18 m (유효) → 대지 고도 0.00 m | 상대 고도 0.00 m | dist_bottom 0.18 m (유효, 거리 센서)`
  - 상자 위: `[측정] LiDAR 1.18 m (유효) → 대지 고도 1.00 m | 상대 고도 2.00 m | 차이 1.00 m | dist_bottom 1.18 m (유효, 거리 센서)` → 1.18 − 0.18 = 1.00 (대지 고도), 2.00 − 1.00 = 1.00 (상자 높이). 드론이 내려간 것이 아니라 아래 면이 1 m 올라온 것
  - 대지 고도 = LiDAR × cos(기울기) − 장착 오프셋. `차이` 칸은 상대 고도 − 대지 고도가 0.15 m 를 넘을 때만(상자 위). `dist_bottom` 은 PX4 숫자 그대로(오프셋 포함)
- 뷰어 파라미터: `-p offset:=0.18` (장착 오프셋 — 내 VM 의 땅 값이 다르면 그 값으로) · `-p detail:=true` (기울기 · 1초 표본 수 · 최장 간격 · 1초 값 범위를 덧붙임) · `-p record:=true` (`~/w10_range_<시각>.csv` 에 0.1초마다 한 줄 — 그래프용)
- 일부러 틀리기 (선택): `make px4_sitl gz_x500`(거리 센서 없는 기체)로 띄우면 브리지는 `[경고] Gazebo에서 LiDAR 토픽을 찾지 못했습니다 …` (`gz topic -l` 에는 PX4 가 구독만 해 둔 `…/lidar/scan` 이름이 보이지만 발행자가 없음), `dist_bottom` 은 **숫자는 나오지만 `dist_bottom_valid` 가 false** → 숫자가 있다고 측정값은 아니다 (기체를 바꿀 때는 `pkill -9 -f "gz sim"` 먼저)

### 2) 18강 — 지형 추종 비행 (터미널 1·2·3·5)
```bash
# 터미널 1: PX4 (GUI 없이 — 상자 위에서 기체가 오르내리는 모습을 보려면 HEADLESS=1 을 뺌)
cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=lidar_world make px4_sitl gz_x500_lidar_down
# 터미널 2: uXRCE-DDS Agent (비행할 때만 — 드라이 런은 없어도 됨)
MicroXRCEAgent udp4 -p 8888
# 터미널 3: pxh> 가 뜬 뒤에 LiDAR 브리지 (0단계 드라이 런에서는 끈다)
ros2 run drone_ros2_advanced w10_lidar_bridge
# 터미널 5: 지형 추종 노드 — 단계별로
ros2 run drone_ros2_advanced w10_terrain_follow                          # 0단계: 드라이 런 (아래 모의 측정값과 함께, PX4 없이도 됨)
ros2 run drone_ros2_advanced w10_terrain_follow --ros-args -p fly:=true  # 1단계: 이륙 → 제자리 고도 유지 (전진 속도 기본 0)
ros2 param set /terrain_follow_base target_agl 3.0                       #   계단 응답: 새 터미널에서 목표 1 m 올리기 → 2.0 으로 되돌리기
ros2 param set /terrain_follow_base cruise_speed 0.3                     # 2단계: 비행 중에 전진 켜기 → 상자 넘기 → 동쪽 9 m 에서 코스 끝
```
- 4번 자리는 기록용 뷰어(`w10_range_viewer --ros-args -p record:=true`, 선택). **키보드 노드와 지형 추종 노드는 함께 켜지 않습니다** (Offboard 명령이 섞임)

**0단계 드라이 런 — 모의 측정값(simulated measurement)으로 부호 확인** (땅, 브리지(3번)는 끄고): 새 터미널에서 `ranges` 숫자만 바꿔 가며 발행합니다. "땅에 앉은 드론에게 여기가 상자 위라면?"을 물어보는 것.
```bash
ros2 topic pub -r 10 /lidar/down sensor_msgs/msg/LaserScan "{range_min: 0.1, range_max: 100.0, ranges: [1.18]}"
```

| 모의 측정값 (`ranges`) | 대지 고도 | 수직 속도 (NED: + = 하강) |
|------|------|------|
| `1.18` (상자 위라면) | 1.00 m | −0.50 m/s (포화, 상승) |
| `3.18` (상자에서 내려선 순간이라면) | 3.00 m | +0.50 m/s (포화, 하강) |
| `2.18` (평지 목표 고도라면) | 2.00 m | 0 (유지) |
| `.inf` | — | 계산하지 않음 → `LiDAR 측정 무효 (inf: 범위 밖)` |

- 노드를 발행보다 먼저 켜면 `[드라이 런] 거리 센서 신호 없음 …` 경고가 2초마다 나오는 것이 정상. 모의 측정값을 발행하면 **약 1초 뒤** `거리 센서 신호 복구 (1.0초 연속) → 지형 추종 재개` 한 줄과 함께 `[드라이 런] …` 검산 줄이 나옴 (유효한 값이 1.0초 이어져야 복구). 값을 바꾸려고 발행을 Ctrl+C 하면 0.5초 뒤 `거리 센서 신호 없음 (0.5초)`, 새 값으로 다시 발행하면 다시 약 1초 뒤 검산 줄
- `-p kz:=-1.0` 으로 다시 켜면 `1.18` 에서 '(하강)' → 부호가 틀렸다는 뜻. 오류 메시지 없는 부호 실수를 땅에서 잡습니다
- 드라이 런 로그는 최저·최대 고도 제한을 적용하기 **전**의 명령(포화까지 적용)을 찍습니다 — 땅에서는 최저 고도 제한이 늘 걸려 부호 실수를 가리기 때문

**1단계(`-p fly:=true`)로 넘어가기 전 — 모의 측정값 발행을 반드시 끕니다**: 0단계의 `ros2 topic pub …`(새 터미널)을 Ctrl+C 로 끄고, Agent(터미널 2)와 브리지(터미널 3, `pxh>` 뒤)를 켭니다. `ros2 topic info /lidar/down` 의 `Publisher count` 가 1(브리지)인지 확인하고 나서 비행 노드를 켭니다. 모의 발행이 남아 있으면 노드는 그 가짜 값으로 고도를 맞춥니다 — `1.18` 이면 최대 고도 제한 5 m 까지 계속 오르고, `3.18` 이면 땅까지 내려갑니다(최저 고도 제한도 같은 값을 보므로 막지 못함)

**로그 검산** (대지 고도 = LiDAR × cos(기울기) − 0.18 → 높이 오차 = 대지 고도 − 목표 → 수직 속도 = kz × 높이 오차, 최대 0.5 m/s)
- 드라이 런: `[드라이 런] 고도 기준 lidar | 대지 고도 1.00 m (LiDAR 1.18 m, 기울기 0°) | 높이 오차 -1.00 m → 수직 속도 -0.50 m/s (포화, 1 × -1.00 = -1.00) (상승)`
- 1단계 평지 호버: `[CRUISE] 동 +0.0 m | 고도 기준 lidar | 대지 고도 1.95 m (LiDAR 2.13 m, 기울기 1°) | 높이 오차 -0.05 m → 수직 속도 -0.05 m/s = 1 × -0.05 (상승) | 상대 고도 1.96 m | 전진 +0.00 m/s` (dist_bottom 칸 생략) — 대지 고도 ≈ 상대 고도이면 장착 오프셋이 맞음
- 2단계 상자에 들어선 직후 (계산): `[CRUISE] 동 +4.0 m | 고도 기준 lidar | 대지 고도 1.02 m (LiDAR 1.20 m, 기울기 1°) | 높이 오차 -0.98 m → 수직 속도 -0.50 m/s (포화, 1 × -0.98 = -0.98) (상승) | 상대 고도 2.03 m | 전진 +0.30 m/s` (dist_bottom 칸 생략) → 약 3초 뒤 대지 고도 2.0 근처, 상대 고도 약 3.0 (상자 위에서 기체가 1 m 올라감)
- 포화가 아니면 `= kz × 오차`, 최대값에 걸리면 `(포화, kz × 오차 = 값)`. 안전장치가 바꾼 값: `최저 고도 0.8 m 아래 (대지 고도 0.62 m) → 상승 0.30 m/s 이상 | 보낸 수직 속도 -0.30 (최저 고도)` · `최대 고도 제한 5.0 m → 상승 정지` · `코스 끝 (동 9.0 m) → 전진 정지 (되돌아가려면 cruise_speed 를 음수로)`
- 계단 응답(step response, 계산): `target_agl 3.0` → 처음 약 1초는 포화(0.5 m/s), 그 뒤로 점점 천천히 — 약 2.6초에 2.9 m. 상자에 올라서는 순간도 같은 모양(대지 고도가 한순간 1 m 줄어드는 계단 입력)

**비행 중에 바꾸기** — 0.1초마다 다시 읽으므로 착륙하지 않고 바로 바뀝니다(바뀌면 `게인 변경: kz 1 → -1` · `목표 대지 고도 변경: 2 → 3` · `전진 속도 변경: 0 → 0.3` · `고도 기준 변경: lidar → ned` 같은 한 줄)
- `kz` (기본 1.0) · `target_agl` (기본 2.0, 1.8~3.5 m 만 받음) · `cruise_speed` (기본 0.0, 최대 ±0.5 m/s, 음수 = 서쪽으로 되돌아오기) · `alt_source` = **고도 기준(height reference)**: `lidar`(기본, 경로 ① 원시값 − 장착 오프셋. `range_source px4` 면 경로 ③ 원시값 — 3) 심화) / `ekf`(경로 ② `dist_bottom` − 장착 오프셋) / `ned`(상대 고도 — 9주차와 같음). 범위 밖·철자 오류는 경고 후 이전 값 유지. `fly` 는 시작할 때만

**일부러 틀리기**
- 9주차 방식으로 되돌리기: 코스 끝(평지)에서 `ros2 param set /terrain_follow_base alt_source ned` → `cruise_speed -0.3` 으로 되돌아오며 상자 통과 → 상대 고도 2 m 그대로라 상자 위 대지 고도는 1 m (상자를 못 보는 고도 유지). 되돌리기 `alt_source lidar`
- 브리지 끄기 = **신호 없음(signal loss)**: 상자 위에서 3번 터미널 Ctrl+C → `거리 센서 신호 없음 (0.5초) → 상대 고도 2.98 m 유지, 전진 정지` → 브리지 다시 실행 → `거리 센서 신호 복구 (1.0초 연속) → 지형 추종 재개`
- `kz` 부호 뒤집기는 **땅 드라이 런(`-p kz:=-1.0`)에서만**. 비행 중 부호 뒤집기(양의 피드백)는 교수 시연입니다

- 안전장치 ([복붙 영역]): 유효성 검사(`inf` = 범위 밖, `range_min`·`range_max` 와 같은 경계값, 45° 넘게 기운 값은 버림) · 센서 타임아웃 0.5초 → 신호 없음(끊긴 순간의 상대 고도 유지 + 전진 정지, 유효한 값이 1.0초 이어지면 복구) · 최저 고도 제한 0.8 m(대지 고도 — 고도 기준과 상관없이) · 최대 고도 제한 5 m(상대 고도) · 포화(수직·전진 0.5 m/s)와 가속도 제한 · 코스 동쪽 0~9 m
- **끝내는 순서**: PX4 터미널(`pxh>`)에서 `commander land` → 착지 확인 → `w10_terrain_follow` Ctrl+C (그다음 3 → 2 → 1). 비행 중에 노드를 먼저 끄면 1초 뒤 PX4 가 Offboard 신호 끊김 비상 동작으로 넘어갑니다

### 막혔을 때
| 증상 | 원인 → 해결 |
|------|-------------|
| 브리지 `[경고] Gazebo에서 LiDAR 토픽을 찾지 못했습니다. 기본값으로 시도합니다.` | LiDAR 없는 기체(`gz_x500`·`gz_x500_mono_cam` 등)로 띄움 / PX4 보다 먼저 켬 → `gz topic -i -t $(gz topic -l \| grep "lidar/scan$")` 에 `No publishers on topic` 이 뜨면 LiDAR 없는 기체 (Harmonic 의 `gz topic -l` 은 PX4 가 구독만 한 scan 토픽도 보여 줌), `make px4_sitl gz_x500_lidar_down` 으로 다시, 브리지는 `pxh>` 뒤에 |
| 브리지 `[경고] 전방 LiDAR 기체입니다` | `gz_x500_lidar_front` 로 띄움 → `gz_x500_lidar_down` 으로 다시 |
| `Timed out waiting for Gazebo world` | 월드 복사 누락 → `ls ~/PX4-Autopilot/Tools/simulation/gz/worlds \| grep lidar`, 0) 준비의 `cp` |
| 이전 월드가 뜸 / `gazebo already running world:` | 이전 Gazebo 서버가 남음 → `pkill -9 -f "gz sim"` → `pgrep -af "gz sim"` 빈 줄 확인 후 다시 |
| `ros2 topic echo /lidar/down` 에 아무것도 없음 (브리지는 실행 중) | 브리지 감지 줄의 월드·기체 이름 확인 → `ros2 topic info -v /lidar/down`, 브리지 다시 실행 |
| 땅인데 LiDAR 가 `100.0` 고정 또는 `inf` | 렌더링 실패(VM) → `HEADLESS=1` → 그래도 안 되면 `export LIBGL_ALWAYS_SOFTWARE=1` (한 번에 하나씩 바꿔 보기) |
| 땅인데 LiDAR 가 0.18 이 아닌 다른 값 | 모델·버전 차이 → 뷰어 `-p detail:=true` 의 1초 값 범위를 보고 `-p offset:=<그 값>` |
| 뷰어 `PX4 위치 없음` / `자세 없음` | Agent 미실행 / source 누락 → 터미널 2 Agent, `source install/setup.bash` (땅 실습에서 자세 없음은 정상 — 기울기 0 으로 봄) |
| `dist_bottom_valid` 가 계속 false, 값은 −z + 0.1 정도로만 따라감 | 거리 센서 없는 기체 → 기체 이름 확인, `gz_x500_lidar_down` 으로 (17강 일부러 틀리기 상황) |
| 드라이 런 로그가 0.18 과 모의 측정값을 번갈아 냄 | 브리지가 켜진 채 모의 측정값 발행 → `ros2 topic info /lidar/down` 의 Publisher count 가 1 이 되게 브리지(3번)를 끔 |
| `ros2 topic pub` 가 오류 | YAML 따옴표·괄호 누락 → 위 명령을 그대로 복사 |
| 이륙하지 않고 "Offboard 모드 전환 / Arm 명령 전송" 반복 | px4_msgs 가 main → 6주차 절의 `release/1.16` 전환 |
| 이륙 직후 `거리 센서 신호 없음` | 브리지 미실행 / LiDAR 없는 기체 / `alt_source ekf` 인데 `dist_bottom_valid` 가 false → `ros2 topic hz /lidar/down`, 이 표 첫 줄 확인 |
| 신호 없음이 자주 뜸 | 갱신 주기가 낮음(VM 부하) / 브리지 꺼짐 → `HEADLESS=1`, 뷰어 `-p detail:=true` 로 1초 표본 수·최장 간격 확인 |
| 상자 위에서 오히려 내려감 | `kz` 음수 / `alt_source` 가 `ned` → 로그의 `게인 변경`·`고도 기준 변경` 줄 확인, `kz 1.0`, `alt_source lidar` |
| 이륙 뒤 앞으로 안 감 | 1단계 그대로(`cruise_speed` 0) / 코스 끝 / 신호 없음 상태 → `cruise_speed 0.3` (코스 끝에서 되돌아오려면 `-0.3`) |
| `ros2 param set` 이 노드를 못 찾음 / 값이 안 먹음 | 노드 이름 오타 / `alt_source` 철자 / `target_agl` 범위 밖 → `ros2 node list` 에서 `/terrain_follow_base`, `lidar`·`ned`·`ekf`, 1.8~3.5 m |
| 키보드 W 로 갔는데 상자를 비켜 감 / 키 반응 없음 | 기수가 동쪽이 아님 / 다른 창에 포커스 → 원점에서 다시 이륙 / 키보드 터미널 클릭 |

### 3) 심화 — 경로 ③: PX4 가 거리 센서 값을 직접 내보내게 하기 (실제 드론에서 쓰는 길)
**수업은 이 절 없이도 됩니다** — 두 노드의 기본값이 `range_source:=bridge`(경로 ① 브리지)라서 17·18강 명령은 그대로 동작합니다. 원시값을 PX4 쪽 길로도 받아 보고 싶은 사람을 위한 심화입니다.

- **왜**: 브리지(`w10_lidar_bridge`)는 Gazebo 토픽을 ROS2 로 옮기는 **시뮬레이션 전용** 길입니다. 실제 드론에는 Gazebo 가 없고, LiDAR 가 비행 컨트롤러(PX4)에 바로 연결됩니다. 그래서 실제 기체에서 ROS2 로 **원시값(raw measurement)** 을 받으려면 PX4 가 `distance_sensor` 를 ROS2 로 내보내도록 설정해야 합니다 (경로 ② `dist_bottom` 은 실제 기체에서도 되지만 EKF **추정값**)
- **주의**: PX4 **소스 파일을 고치고 다시 빌드**합니다 → 고정 환경(PX4 v1.16.0 원본)에서 벗어납니다. VM 에서 다시 빌드에 수 분 걸립니다. 끝나면 아래 '되돌리기'로 원본으로 돌릴 수 있습니다. px4_msgs 는 `release/1.16`([6주차 절](#6주차-시작-전에-한-번만--px4_msgs-버전-맞추기))에서 확인한 값입니다 — main 이면 먼저 전환

| 경로 | 토픽 | 값 | 실제 드론 | 준비 |
|------|------|----|-----------|------|
| ① 브리지 | `/lidar/down` (`sensor_msgs/LaserScan`) | 원시값 | 안 됨 (시뮬레이션 전용) | `w10_lidar_bridge` 실행 |
| ② EKF | `/fmu/out/vehicle_local_position` 의 `dist_bottom` | 추정값 | 됨 | 없음 (기본 목록에 있음) |
| ③ PX4 거리 센서 | `/fmu/out/distance_sensor` (`px4_msgs/DistanceSensor`) | 원시값 | 됨 | PX4 설정 파일에 두 줄 + 다시 빌드 |

**1단계 — PX4 내보내기 목록(`dds_topics.yaml`)에 두 줄 넣기** (PX4 를 Ctrl+C 로 끈 상태에서)
```bash
nano ~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
#   Ctrl+W → home_position 입력 → Enter 로 찾아감
#   publications: 목록의 맨 끝 = /fmu/out/home_position 항목 아래 (subscriptions: 줄보다 위) 에 빈 줄 하나 + 아래 두 줄
#   Ctrl+O → Enter (저장) → Ctrl+X (나가기)
```
넣을 두 줄 — 들여쓰기는 **스페이스로(Tab 금지)**, 위 항목들과 똑같이 `-` 앞 2칸 · `type` 앞 4칸:
```yaml
  - topic: /fmu/out/distance_sensor
    type: px4_msgs::msg::DistanceSensor
```
- 확인: `grep -n -A1 "distance_sensor" ~/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml` → 원래 있던 `/fmu/in/distance_sensor` 와 새 `/fmu/out/distance_sensor` 두 군데가 보이면 OK
- 토픽 이름에 `_v1` 이 붙지 않습니다 (PX4 v1.16.0 의 `msg/DistanceSensor.msg` — px4_msgs `release/1.16` 도 같음 — 에 `MESSAGE_VERSION` 이 없음) — 위 이름 그대로

**2단계 — 다시 빌드하고 확인** (터미널 1·2·4 — 브리지(3번)는 필요 없음)
```bash
# 터미널 1: 남은 Gazebo 서버 정리 → 실행하면 바뀐 목록 때문에 PX4 가 자동으로 다시 빌드 (VM 에서 수 분, pxh> 가 뜨면 끝)
pkill -9 -f "gz sim"
cd ~/PX4-Autopilot && PX4_GZ_WORLD=lidar_world make px4_sitl gz_x500_lidar_down
# 터미널 2: uXRCE-DDS Agent — 경로 ③ 은 Agent 를 거쳐 오므로 필수
MicroXRCEAgent udp4 -p 8888
# 새 터미널: 토픽 확인
ros2 topic info /fmu/out/distance_sensor                            # Publisher count: 1 = PX4 가 내보내는 중 (0 이거나 Unknown topic 이면 다시 빌드 안 됨)
ros2 topic echo /fmu/out/distance_sensor --field current_distance   # 땅에서 약 0.18 (Ctrl+C 로 멈춤)
# 터미널 4: 측정 뷰어 — 브리지 없이
ros2 run drone_ros2_advanced w10_range_viewer --ros-args -p range_source:=px4
```
- 뷰어 시작 로그에 `원시값 경로 px4 (/fmu/out/distance_sensor)` → 측정 줄은 `LiDAR(px4)` 로 구분됩니다: `[측정] LiDAR(px4) 0.18 m (유효) → 대지 고도 0.00 m | 상대 고도 0.00 m | dist_bottom 0.18 m (유효, 거리 센서)` (계산 예). 기본값(bridge)일 때는 17강 그대로 `LiDAR 0.18 m`
- `-p record:=true` CSV 는 맨 끝 열 `range_source` 에 어느 경로의 값인지 남습니다 (앞쪽 열은 그대로)
- **비행 중에 바꾸기**: 2) 18강 순서대로 브리지로 비행하다가 새 터미널에서 `ros2 param set /terrain_follow_base range_source px4` → `원시값 경로 변경: bridge → px4` 한 줄 뒤 `[CRUISE] … 대지 고도 1.95 m (LiDAR(px4) 2.13 m, 기울기 1°) …` (계산 예). 같은 Gazebo LiDAR 를 PX4 가 받아 내보낸 값이라 브리지 값과 거의 같아야 정상입니다. 되돌리기 `range_source bridge`. 처음부터 쓰려면 `ros2 run drone_ros2_advanced w10_terrain_follow --ros-args -p fly:=true -p range_source:=px4`
- 두 노드는 두 경로를 **늘 함께 구독**하고, **고른 경로의 값만** 대지 고도·신호 없음 판정·복구 판정에 씁니다 → 브리지를 켜 둔 채 px4 로 바꿔도 됩니다. 고른 경로의 값이 안 오면 신호 없음 규칙 그대로(0.5초 → 상대 고도 유지·전진 정지, 유효한 값이 1.0초 이어지면 복구)
- 늘 구독하므로 두 노드 중 하나라도 켜져 있으면 **다시 빌드하지 않아도** `ros2 topic list` 에 `/fmu/out/distance_sensor` 이름이 보입니다(구독만 해 둔 이름). PX4 가 실제로 보내는지는 목록이 아니라 `ros2 topic info /fmu/out/distance_sensor` 의 `Publisher count`(1 = 보내는 중) 또는 `ros2 topic echo --once` 로 확인합니다
- 0단계 모의 측정값(`ros2 topic pub /lidar/down …`)은 bridge 경로용입니다 — 드라이 런은 기본값(bridge)으로

**`px4_msgs/DistanceSensor` 메시지에서 보는 칸** (`ros2 topic echo --once /fmu/out/distance_sensor`)

| 칸 | 뜻 | 시뮬레이션 값 |
|----|----|---------------|
| `current_distance` | 측정 거리 [m] — 원시값 (기울기 보정 전) | 땅 약 0.18 |
| `min_distance` · `max_distance` | 측정 범위 [m] — 브리지의 `range_min` · `range_max` 와 같은 역할 | 0.1 · 100.0 |
| `orientation` | 센서가 보는 방향 — `25` = 아래 (`ROTATION_DOWNWARD_FACING`) | 25 |
| `signal_quality` | 신호 품질 — `-1` = 모름, `0` = 무효 신호, 1~100 = 품질 (클수록 좋음) | −1 (시뮬레이션은 늘 −1) |
| `type` | 센서 종류 — `0` = 레이저 | 0 |

- 노드의 유효성 검사는 브리지와 같고(`current_distance` 를 `min_distance`~`max_distance` 로 검사 — inf·nan·범위 밖·경계값·45° 넘게 기운 값은 버림) **두 가지가 더 있습니다**: `signal_quality` 가 `0` 이면 버림(`신호 품질 0 — 센서가 무효 신호로 보고`, −1 과 1~100 은 통과) · `orientation` 이 25 가 아닌 메시지(예: 전방 LiDAR)는 조용히 무시(처음 한 번만 `아래 방향(25)이 아닌 거리 센서 메시지 무시 (orientation 0)`)

**되돌리기** (고정 환경 원본으로)
```bash
cd ~/PX4-Autopilot && git checkout src/modules/uxrce_dds_client/dds_topics.yaml
make px4_sitl     # 다시 빌드만 (수 분) — 이후 /fmu/out/distance_sensor 는 사라짐, 노드는 range_source 기본값(bridge)으로
```
- 고친 상태인지 모를 때: `cd ~/PX4-Autopilot && git status --short src/modules/uxrce_dds_client/` → `M …/dds_topics.yaml` 이 보이면 고친 상태, 아무것도 안 나오면 원본

**교수 VM 실측** (두 줄 추가 → 다시 빌드 → 호버링 중): `current_distance` 4.93 · `min_distance` 0.1 · `max_distance` 100.0 · `orientation` 25 · `signal_quality` −1 · `variance` 0 · `type` 0. 같은 때 QGC 고도 4.8 m ≈ 4.93 − 0.18 = 4.75 → LiDAR 원시값에서 장착 오프셋을 빼면 고도와 맞습니다

| 증상 | 원인 → 해결 |
|------|-------------|
| `ros2 topic info /fmu/out/distance_sensor` 가 `Publisher count: 0` 또는 `Unknown topic` / 값이 안 옴 / 노드 경고 `LiDAR 없음 — /fmu/out/distance_sensor 가 안 옴: PX4 dds_topics.yaml 에 추가하고 다시 빌드했나요? (README 10주차 심화)` (`topic list` 에 이름이 보여도 노드가 구독만 해 둔 것일 수 있음) | 두 줄 추가 뒤 다시 빌드 안 함 / Agent 미실행 / 거리 센서 없는 기체 → 1단계 `grep` 으로 두 줄 확인, PX4 다시 실행(빌드 로그가 지나가는지), 터미널 2 Agent, 기체 이름 `gz_x500_lidar_down` 확인 |
| 노드 경고 `LiDAR 없음 — /fmu/out/distance_sensor 는 오지만 아래 방향(25)이 아님: 하방 LiDAR 기체(gz_x500_lidar_down)인가요?` | 전방 LiDAR 기체(`gz_x500_lidar_front`) — 그 전에 `아래 방향(25)이 아닌 거리 센서 메시지 무시 (orientation …)` 한 줄이 먼저 나옴 → `pkill -9 -f "gz sim"` 뒤 `gz_x500_lidar_down` 으로 다시 |
| 다시 빌드가 YAML 오류로 멈춤 | Tab 문자 / 칸 수 틀림 → nano 에서 두 줄을 지우고 스페이스로 다시 (`-` 앞 2칸, `type` 앞 4칸) |
| `range_source` 를 바꿔도 그대로 | 철자 → `bridge` · `px4` 둘 중 하나 (그 밖은 경고 후 이전 값 유지) |

## 11주차 2D LiDAR 장애물 감지 · 회피 비행 (19·20강)
앞을 보는 **2D 라이다(2D LiDAR)** — 한 평면 270° 를 1080칸으로 재는 LiDAR — 로 장애물을 찾고(19강), 기둥을 피해 동쪽 12 m 목표까지 가는 **반응형 회피(reactive avoidance)** 노드 `w11_avoid` 를 실행합니다(20강, PX4Base).
- 스캔 읽기: `i번째 칸 각도 = angle_min + i × angle_increment` — 각도 + = 왼쪽(반시계, REP 103), **0번 칸 = 오른쪽 뒤 −135°**, 정면 ≈ 540번, 바로 뒤 90° 는 **사각지대(blind spot)**. 10주차처럼 `ranges[0]` 을 그대로 쓰면 오른쪽 뒤를 봅니다
- 1080칸 → **섹터 최근접 거리(nearest obstacle distance)** 3개: 앞 ±30° · 왼쪽 +30~+90° · 오른쪽 −90~−30°. **같은 inf, 다른 뜻**: 10주차 하방은 `+inf` = 범위 밖(무효)이지만 2D 스캔의 `+inf` = **자유 공간(free space)** (그 방향 30 m 안에 장애물 없음). 섹터에 쓸 값이 하나도 없으면 측정 없음 = 막힘. 0.4 m 안 값은 자기 기체로 보고 버림
- 20강 규칙(VFH 3섹터판): 앞 1.5 m 미만 → 정지 → 넓은 쪽으로 **좌우 이동(lateral movement)** (동점이면 오른쪽) → 앞이 2.0 m 넘게 비면 전진 → 옆·뒤 대각선이 비면 코스 복귀 → 도착. **PX4 충돌 방지(Collision Prevention)는 Position 모드에서 사람이 조종할 때만 동작하고 Offboard 에서는 동작하지 않음** → 같은 원리를 노드가 직접 합니다. 기본값은 **드라이 런** + 전진 속도 0(제자리) — 비행하려면 `-p fly:=true`
- 부호: 스캔 각도 + = 왼쪽 ↔ `send_velocity_body` 의 `vy_body` + = 오른쪽(**몸체 좌표계(body frame, FRD)**) → 왼쪽 좌우 이동 = `vy_body` 음수

### 0) 준비 (한 번만)
```bash
# 기둥 월드를 PX4 폴더로 복사 — 기둥 1개(동쪽 6 m, 1×1×4 m) + 남쪽 벽 1개 + 목표 표시(동쪽 12 m), Fuel·인터넷 불필요
cd ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/worlds
cp obstacle_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```
- 기체는 PX4 내장 **`gz_x500_lidar_2d`** — 카메라 없음, 수평 2D LiDAR 1개(270° · 1080칸 · 각도 분해능 0.25° · 0.1~30 m · 30 Hz). 10주차 `gz_x500_lidar_down` 을 띄우면 앞을 못 봅니다 (브리지가 `[경고] 하방 LiDAR 기체` 를 띄움)
- 2D LiDAR 렌더링은 10주차 하방 1빔보다 무겁습니다 → VM 이 버거우면 PX4 명령 앞에 `HEADLESS=1`. 19강 필수(땅 값 읽기·계산)와 20강 0단계 드라이 런은 비행 없이 됩니다
- 20강 비행 전에는 **`release/1.16` 필수** ([6주차 절](#6주차-시작-전에-한-번만--px4_msgs-버전-맞추기))

### 1) 19강 — 스캔 읽기와 섹터 (비행 없음, 키보드 비행은 선택)
```bash
# 터미널 1: 2D LiDAR 기체 + 기둥 월드 (VM 이 버거우면 앞에 HEADLESS=1)
cd ~/PX4-Autopilot && PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500_lidar_2d
# 터미널 2: uXRCE-DDS Agent (뷰어의 위치·전진 속도용 — 없어도 스캔은 보임)
MicroXRCEAgent udp4 -p 8888
# 터미널 3: pxh> 가 뜬 뒤에 스캔 브리지 — [감지] world=obstacle_world model=x500_lidar_2d_0 → /lidar/scan 줄 확인
ros2 run drone_ros2_advanced w11_scan_bridge
# 터미널 4: 스캔 뷰어 — 비행하지 않음 (받아서 0.5초마다 한 줄 찍기만)
ros2 run drone_ros2_advanced w11_scan_viewer
# 터미널 5 (선택): 2주차 키보드로 기둥 앞까지 — T 이륙 → 약 2 m 에서 Space → W 한 번(동쪽 0.5 m/s) → 판단 칸이 '정지 판단'이 되면 Space → A(왼쪽)
ros2 run drone_ros2_advanced keyboard_control
```
- 확인 명령: `ros2 topic echo /lidar/scan --once --no-arr` → `angle_min: -2.356…` · `angle_increment: 0.00436…` · `range_min: 0.1` · `range_max: 30.0` · `ranges: '<sequence type: float, length: 1080>'`. 정면 칸 = (0 − angle_min) ÷ angle_increment ≈ 539.5. 발행 주기: `ros2 topic hz /lidar/scan` (약 30 Hz — VM 이 느리면 그만큼 낮음)
- 뷰어 로그 검산 (계산 예 — 내 화면과 비교)
  - 땅: `[스캔] 1080칸 | 정면 1칸(540) 5.38 m | 앞 5.38 · 왼쪽 자유 공간 · 오른쪽 3.54 m | 버림 0칸 | 판단: 전진` — 앞 5.38 = 기둥 서쪽 면(동 5.5) − 센서 위치(기체 중심 앞 0.12 m), 오른쪽 3.54 = 남쪽 벽 서쪽 끝 모서리까지. 정면 칸 번호는 반올림에 따라 539 로 찍힐 수도 있음(둘 다 정면에서 0.125°)
  - 기둥 앞에서 왼쪽으로 0.6 m 옮긴 곳: `[스캔] 1080칸 | 정면 1칸(540) 자유 공간 | 앞 1.38 · … | 판단: 정지 판단` — 정면 1칸은 기둥 북쪽 면 바깥을 지나 자유 공간인데, 기체 오른쪽 끝은 아직 기둥 앞 (1칸 = 폭 0인 선 → 기체 폭을 못 봄)
  - 키보드로 다가가면: `[정지 판단] 앞 1.50 m 통과 | 전진 0.50 m/s` → 멈추면 `[멈춤] 앞 0.94 m | 판단 뒤 1.5 s · 0.54 m 이동` = 알아채고 멈출 때까지 간 거리 = **정지 거리(stopping distance)** = 공주 거리 + 제동 거리. 19강 표의 0.5 m/s 칸(0.50 m)과 비교
  - 판단 칸: 전진(앞 3.0 m 이상) · 감속(3.0 m 안) · 정지 판단(1.5 m 안) · 근접(앞 ±45° 1.0 m 안) — 20강 노드와 같은 기준. `-inf` 칸·자기 기체 영역(0.4 m 안) 칸이 생기면 상태가 바뀔 때만 경고 한 줄
- 뷰어 파라미터: `ros2 param set /scan_viewer front_deg 1.0` (앞 섹터를 ±1° 로 — 정면 1칸과 비슷해짐) · `stop_dist` (정지 판단 거리, 판단 칸 기준이 같이 바뀜) · `-p record:=true` (`~/w11_scan_<시각>.csv` 에 0.1초마다 한 줄 — 그래프용)
- 선택: `PX4_GZ_WORLD=walls` (7강 월드 — PX4 문서가 충돌 방지 시험에 쓰는 월드)로 띄워 뷰어 비교. 브리지는 월드를 자동 감지 (월드를 바꿀 때는 `pkill -9 -f "gz sim"` 먼저)

### 2) 20강 — 회피 비행 (터미널 1·2·3·5)
```bash
# 터미널 1·2·3: 19강과 같음 (브리지는 pxh> 뒤, 0단계 드라이 런에서는 끈다)
# 터미널 5: 회피 노드 — 단계별로
ros2 run drone_ros2_advanced w11_avoid --ros-args -p cruise_speed:=0.5   # 0단계: 드라이 런 (아래 모의 스캔과 함께, PX4 없이도 됨)
ros2 run drone_ros2_advanced w11_avoid --ros-args -p fly:=true           # 1단계: 이륙 → 제자리 호버 (전진 속도 기본 0)
ros2 param set /avoid_base cruise_speed 0.5                              # 2단계: 비행 중에 전진 켜기 → 기둥 피해 동쪽 12 m 도착
```
- 4번 자리는 뷰어(선택). **키보드 노드와 회피 노드는 함께 켜지 않습니다** (Offboard 명령이 섞임)

**0단계 드라이 런 — 세 칸 모의 스캔** (땅, 브리지(3번)는 끄고): 새 터미널에서 `ranges` 만 바꿔 가며 **위에서부터 차례로** 발행합니다. 칸 3개 = −60° · 0° · +60° = **오른쪽 · 앞 · 왼쪽**, `.inf` = 자유 공간
```bash
ros2 topic pub -r 10 /lidar/scan sensor_msgs/msg/LaserScan "{angle_min: -1.0472, angle_increment: 1.0472, range_min: 0.1, range_max: 30.0, ranges: [2.0, 1.2, .inf]}"
```

| `ranges` (오른쪽 · 앞 · 왼쪽) | 판단 (로그 태그) | vx · vy [m/s] (vy − = 왼쪽) |
|------|------|------|
| `[2.0, 5.0, .inf]` | 전진 | +0.50 · 0.00 |
| `[2.0, 2.25, .inf]` | 감속 = 0.33 × (2.25 − 1.5) | +0.25 · 0.00 |
| `[2.0, 1.2, .inf]` | 정지 → 왼쪽 좌우 이동 | 0.00 · −0.30 |
| `[.inf, 1.2, 2.0]` | 왼쪽 유지 (오른쪽이 넓어져도 — 앞이 2.0 m 넘게 빌 때까지) | 0.00 · −0.30 |
| `[0.8, 1.2, 0.8]` | 막다른 곳 → 정지·호버 | 0.00 · 0.00 |

- 로그 예: `[드라이 런] [정지] 앞 1.20 m < 1.5 m → 왼쪽 좌우 이동 | vx +0.00 · vy -0.30 m/s (− = 왼쪽)` · 넷째 줄 `[드라이 런] [좌우 이동] 왼쪽 유지 | 앞 1.20 m (해제 2.0) | …` · 다시 앞을 `5.0` 으로 보내면 `앞 5.00 m ≥ 해제 2.0 m → 전진 +0.50 m/s` (방향을 풂)
- 노드를 발행보다 먼저 켜면 `[드라이 런] 스캔 신호 없음 …` 경고가 2초마다 나오는 것이 정상. 발행하면 약 1초 뒤 `스캔 신호 복구 (1.0초 연속) → 회피 비행 재개` 와 함께 판단 줄이 나옵니다
- 드라이 런은 위치를 쓰지 않아 코스 복귀·도착은 건너뛰고, 가속도 제한 **전**의 명령 값을 찍습니다
- `.inf` 대신 `29.0` 으로 써도 같은 뜻(먼 장애물)입니다 — `30.0` 은 경계값(`range_max`)이라 버려집니다

**1단계(`-p fly:=true`)로 넘어가기 전 — 모의 스캔 발행을 반드시 끕니다**: Ctrl+C 로 끄고, Agent(터미널 2)와 브리지(터미널 3, `pxh>` 뒤)를 켭니다. `ros2 topic info /lidar/scan` 의 `Publisher count` 가 1(브리지)인지 확인하고 나서 비행 노드를 켭니다. 모의 발행이 남아 있으면 노드는 가짜 스캔으로 판단합니다

**로그 검산** (계산 예 — PX4 속도 추종 지연을 넣은 간이 시뮬레이션. VM 숫자는 조금 다름)
- 1단계 호버: `[전진] 동 +0.0 m | 앞 5.38 · 왼쪽 자유 공간 · 오른쪽 3.54 m | 전진 +0.00 m/s`
- 2단계 (전진을 켠 뒤 약 35초. 행동이 바뀌면 바로 한 줄, 같은 행동은 1초마다 한 줄):
```
[감속] 동 +2.4 m | 앞 2.98 m | 전진 +0.49 = 0.33 × (2.98 - 1.5)
[정지] 앞 1.49 m < 1.5 m → 왼쪽 좌우 이동 (왼쪽 자유 공간 > 오른쪽 3.00 m)
[좌우 이동] 왼쪽 | 앞 1.43 m (해제 2.0) | vy -0.30 m/s (− = 왼쪽) | 북 +0.1 m
[감속] 동 +4.0 m · 북 +1.5 m | 앞 2.01 m ≥ 해제 2.0 m → 전진 +0.17 = 0.33 × (2.01 - 1.5) | … | 오른쪽 1.74 m < 2.0 → 복귀 보류
[복귀] 오른쪽 · 뒤 대각선 2.0 m 넘게 빔 | 동 +7.8 m · 북 +1.8 m → vy +0.30 m/s · 전진 +0.50 m/s
[도착] 동 +11.7 m (목표 12.0) → 정지·호버 — 끝내기: pxh> commander land
```
- 전진 속도 = k_slow × (앞 거리 − 정지 판단 거리) → 최소 0.1 · 최대 `cruise_speed` (**속도 제한**, 9주차 `kp × 오차 → 포화` 와 같은 꼴). 줄에는 판단이 정한 값이 찍히고, 가속도 제한(0.5 m/s²)으로 아직 따라가는 중이면 `| 보낸 값 vx … · vy … (가속도 제한)` 이 붙습니다
- 정지 판단 1.5 m 와 해제 2.0 m 를 다르게 두어 경계에서 정지·전진이 번갈아 나오지 않습니다(**히스테리시스, hysteresis**). 해제는 북 약 1.5 m 이고 감속·속도 추종 지연으로 약 1.8 m 까지 밀려, 기둥 옆을 지날 때 프로펠러와 기둥 사이는 약 1.0 m (계산)
- 기둥 옆에서는 복귀 쪽(오른쪽) 옆 + 뒤 대각선이 2.0 m 안이라 `복귀 보류` — 뒤 프로펠러까지 기둥을 지난 동쪽 약 7.8 m 에서 복귀를 시작합니다

**비행 중에 바꾸기** — 0.1초마다 다시 읽으므로 착륙하지 않고 바로 바뀝니다(바뀌면 `전진 속도 변경: 0 → 0.5` · `좌우 이동 속도 변경: 0.3 → -0.3` · `앞 섹터 반각 변경: 30 → 1` · `감속 비례 상수 변경: 0.33 → 10` · `정지 판단 거리 변경: 1.5 → 2 (해제 2.5)` 같은 한 줄)
- `cruise_speed` 0~0.5 (기본 0 — 0 이면 제자리, 좌우 이동도 멈춤 → 이상하면 `cruise_speed 0.0`) · `stop_dist` 1.2~3.0 (기본 1.5, 해제 = +0.5) · `k_slow` 0.1~10 (기본 0.33) · `v_side` −0.5~0.5 (기본 0.3) · `front_deg` 1~45 (기본 30). 범위 밖·숫자가 아닌 값은 경고 후 이전 값 유지 — `Set parameter successful` 이 떠도 노드 로그로 확인. `fly` 는 시작할 때만

**일부러 틀리기**
- 브리지 끄기 = **신호 없음** (학생): 비행 중 3번 터미널 Ctrl+C → 0.5초 뒤 `[신호 없음] 유효한 스캔 0.5초 없음 → 수평 정지·고도 유지` → 브리지 다시 실행 → `스캔 신호 복구 (1.0초 연속) → 회피 비행 재개`
- 좌우 이동 부호 — 학생은 드라이 런으로: `-p v_side:=-0.3` 으로 켜고 `ranges: [0.8, 1.2, .inf]` → `[안전] 오른쪽 0.80 m < 안전 거리 1.0 m → 좌우 이동 정지 (v_side 부호 확인)`. 교수 비행 시연: `ros2 param set /avoid_base v_side -0.3` → 로그는 `왼쪽` 인데 `vy +0.30` — 기체는 기둥 남쪽(기둥과 벽 사이 2.5 m)으로 통과하고, 복귀는 북쪽으로. 되돌리기 `v_side 0.3`
- 앞 섹터를 좁게 (교수 시연, `cruise_speed 0.3`): `ros2 param set /avoid_base front_deg 1.0` → 왼쪽으로 약 0.5 m 만에 '앞이 빔' 으로 풀려 전진 → `[근접] 앞쪽 0.99 m < 안전 거리 1.0 m → 전진 정지` (프로펠러와 기둥 약 0.5 m, 계산). 되돌리기 `front_deg 30` → 다시 정지 → 좌우 이동 → 통과
- (선택) `k_slow 10` → 감속 구간이 사실상 없어 0.5 m/s 그대로 다가가 정지 판단 (넘침이 커짐 — 19강 정지 거리 표와 비교)

- 안전장치 ([복붙 영역], 0 은 가속도 제한 없이 바로): 신호 없음(유효한 스캔 0.5초 없음 → 수평 정지·고도 유지, 1.0초 이어지면 재개) · 너무 가까움(`-inf` 한 칸이라도 → 수평 정지) · 근접(앞 ±45° 1.0 m 미만 → 전진 0, 좌우 이동은 허용 → 빠져나올 수 있음) · 옆 안전 거리(가는 쪽 옆 1.0 m 미만 → 좌우 0, 고른 쪽이면 막다른 곳 → 호버) · 사각지대(후진 없음) · 지오펜스(북 ±3 m 바깥쪽 좌우 0, 상대 고도 5 m 위로 상승 0)
- **측정한 방향으로만 이동**: 뒤 90° 는 측정하지 않아 후진 명령이 없고, 측정 없는 섹터는 막힌 것으로 봅니다. PX4 입력 `/fmu/in/obstacle_distance` 에는 쓰지 않습니다(PX4 가 같은 스캔을 이미 받음)
- **끝내는 순서**: PX4 터미널(`pxh>`)에서 `commander land` → 착지 확인 → `w11_avoid` Ctrl+C (그다음 3 → 2 → 1). 비행 중에 노드를 먼저 끄면 1초 뒤 PX4 가 Offboard 신호 끊김 비상 동작으로 넘어갑니다

### 막혔을 때
| 증상 | 원인 → 해결 |
|------|-------------|
| 브리지 `[경고] 하방 LiDAR 기체 — 11주차는 gz_x500_lidar_2d` | 10주차 습관으로 `gz_x500_lidar_down` 실행 → `pkill -9 -f "gz sim"` 뒤 `make px4_sitl gz_x500_lidar_2d` |
| 브리지 `[경고] Gazebo에서 LiDAR 토픽을 찾지 못했습니다. 기본값으로 시도합니다.` | LiDAR 없는 기체(`gz_x500` 등) / PX4 보다 먼저 켬 → 기체 이름 확인, 브리지는 `pxh>` 뒤에 (10주차 표 첫 줄과 같음) |
| 브리지 `[안내] 1빔 전방 LiDAR` · 뷰어 `1빔 기체 — 좌우 측정 없음` | `gz_x500_lidar_front` 등 1빔 기체 → 앞만 보이고 회피는 정지까지만 (좌우가 측정 없음 = 막힘). 11주차는 `gz_x500_lidar_2d` |
| `Timed out waiting for Gazebo world` | 월드 복사 누락 → `ls ~/PX4-Autopilot/Tools/simulation/gz/worlds \| grep obstacle`, 0) 준비의 `cp` |
| 이전 월드가 뜸 / `gazebo already running world:` | 이전 Gazebo 서버가 남음 → `pkill -9 -f "gz sim"` → `pgrep -af "gz sim"` 빈 줄 확인 후 다시 |
| Gazebo 가 매우 느림 / `ros2 topic hz /lidar/scan` 이 30 Hz 보다 한참 낮음 | 2D LiDAR 렌더 부하(VM) → `HEADLESS=1`. 그래도 느리면 드라이 런·땅 값까지 하고 비행은 시연 영상으로 |
| 땅에서 왼쪽이 `자유 공간` 이 아니라 `측정 없음` | 빈 방향이 `+inf` 가 아니라 최대값 `30.0` 으로 옴(경계값이라 버려짐) → `ros2 topic echo /lidar/scan --once` 의 `ranges` 화면을 게시판에 올려 주세요 |
| 뷰어 `자기 기체 영역 N칸 버림` 이 계속 | 프로펠러·프레임이 스캔 평면에 잡힘 → 0.4 m 안이라 버려지므로 판단에는 영향 없음 |
| 드라이 런 판단이 진짜 스캔 값과 번갈아 나옴 | 브리지가 켜진 채 모의 스캔 발행 → `ros2 topic info /lidar/scan` 의 Publisher count 가 1 이 되게 하나를 끔 |
| `스캔 칸 정보가 이상해 버림` | 모의 스캔에서 `angle_min`·`angle_increment`·`range_min`·`range_max` 중 하나가 빠짐 → 위 명령을 그대로 복사 |
| 이륙하지 않고 "Offboard 모드 전환 / Arm 명령 전송" 반복 | px4_msgs 가 main → 6주차 절의 `release/1.16` 전환 |
| 이륙 뒤 앞으로 안 감 | 1단계 그대로(`cruise_speed` 0) / 신호 없음 / 도착 → `cruise_speed 0.5`, 로그 태그 확인 |
| 기둥 앞에서 멈춘 채 그대로 (`[막다른 곳]`) | 고른 쪽 옆도 1.0 m 안 → 좌우 측정 확인(1빔 기체?) / `v_side` 가 0 → `v_side 0.3` |
| 로그는 `왼쪽` 인데 기체는 오른쪽으로 | `v_side` 음수(일부러 틀리기 그대로) → `ros2 param set /avoid_base v_side 0.3` |
| `[근접]` 이 반복되며 앞으로 안 감 | `front_deg` 를 좁힌 채(일부러 틀리기 그대로) → `ros2 param set /avoid_base front_deg 30` |
| `ros2 param set` 이 노드를 못 찾음 / 값이 안 먹음 | 노드 이름 오타 / 범위 밖 값 → `ros2 node list` 에서 `/avoid_base`, 노드 로그의 경고 확인 |

## 12주차 상태머신 미션 · 모드 전환 (21·22강)
11주차 회피 미션을 **상태머신, 유한 상태 머신(Finite State Machine, FSM)** 으로 다시 씁니다. 먼저 **전이 조건표(state transition table)** 를 PX4 없이 시뮬레이터로 시험하고(21강), 같은 표를 쓰는 노드 `w12_mission` 을 실행하면 이륙부터 착륙까지 자동으로 수행합니다(22강, PX4Base). 새 월드·새 기체 없음 — 11주차 `obstacle_world` + `gz_x500_lidar_2d` + `w11_scan_bridge` 그대로.

**볼 곳**: 코드는 읽지 않아도 됩니다. 아래 **전이 조건표 10줄**과, 실행하면 출력되는 **`[전이]` 로그**(로그 한 줄 = 표 한 줄)만 봅니다. 표는 [`week12/mission_table.py`](drone_ros2_advanced/drone_ros2_advanced/week12/mission_table.py) 한 파일에 있고 시뮬레이터와 미션 노드가 같이 씁니다.

- **상태(state)** = 수행 중인 동작, 7개: `IDLE`(준비) · `TAKEOFF`(이륙) · `CRUISE`(순항) · `AVOID`(회피) · `HOLD`(대기) · `LAND`(착륙) · `DONE`(끝). 현재 상태는 항상 하나입니다
- **사건(event)** = 상태를 바꾸는 계기(일어난 일 — 12강 상태 머신의 '넘어갈 조건'이 충족된 순간), 9개. **전이(transition)** = 다른 상태로 이동
- `HOLD` 는 노드 안의 대기 상태입니다 — PX4 는 Offboard 유지 (PX4 Hold 모드 아님)

**전이 조건표** — 한 줄 = 현재 상태 + 사건 → 다음 상태 = **상태도(state diagram)** 의 화살표 하나. 해당 줄 없음 → 현재 상태 유지

| 줄 | 현재 상태 | 사건 (뜻) | 다음 상태 |
|----|------|------|------|
| 1 | `IDLE` | `ready` (Offboard + Arm 완료) | `TAKEOFF` |
| 2 | `TAKEOFF` | `airborne` (이륙 고도 도달) | `CRUISE` |
| 3 | `CRUISE` | `front_blocked` (앞 1.5 m 미만) | `AVOID` |
| 4 | `AVOID` | `front_clear` (앞 2.0 m 이상) | `CRUISE` |
| 5 | `CRUISE` | `arrived` (목표 도착) | `LAND` |
| 6 | `LAND` | `landed` (착지) | `DONE` |
| 7 | `CRUISE` | `sensor_bad` (센서 신호 끊김) | `HOLD` |
| 8 | `AVOID` | `sensor_bad` (센서 신호 끊김) | `HOLD` |
| 9 | `HOLD` | `hold_timeout` (HOLD 20초) | `LAND` |
| 10 | `HOLD` | `sensor_ok` (신호 1초 이어짐) | `CRUISE` |

- 1~6 = 정상 미션. 7~10 = **페일세이프(failsafe)** — 고장 시 안전 동작: 센서 신호 끊김 → `HOLD` 에서 대기 → 복구 시 `CRUISE` · 시간 제한 경과 시(**타임아웃, timeout**) `LAND`
- 3번 1.5 m 와 4번 2.0 m 를 다르게 둔 것은 20강 히스테리시스 — 경계에서 `CRUISE` ⇄ `AVOID` 가 번갈아 나오지 않게
- 22강 노드의 사건 판정 조건: `airborne` 상대 고도 2 m ± 0.2 · `front_blocked` 앞 1.5 m 미만 또는 앞쪽 ±45° 1.0 m 미만(근접) · `arrived` 동쪽 11.7 m 넘음(목표 12 m) · `sensor_bad` 유효한 스캔 0.5초 없음 또는 `-inf` 칸(너무 가까움) · `sensor_ok` 유효한 스캔 1.0초 이어짐 · `hold_timeout` `HOLD` 에서 20초(파라미터로 바꿈)
- 22강 노드에는 표보다 먼저 확인하는 **표 밖 안전장치** 2개가 더 있습니다 (22강 절의 '표 밖 안전장치 2개')

### 0) 준비 (코드 받기)
```bash
cd ~/ros2_ws/src/drone_ros2_advanced && git fetch origin && git reset --hard origin/main
cd ~/ros2_ws && colcon build --packages-select drone_ros2_advanced && source install/setup.bash
ros2 pkg executables drone_ros2_advanced | grep w12     # 3개: w12_fsm_sim · w12_mission · w12_state_viewer
```
- 첫 두 줄은 [매주 실습 전에](#매주-실습-전에--최신-코드-받기) 와 같습니다 — `git reset` 은 내가 고친 제공 파일을 원래대로 돌립니다(새로 만든 파일은 남음)
- `w12_keyboard` 가 함께 보이면 옛 빌드가 남은 것입니다 — 쓰지 않습니다(미션 노드가 받지 않음). 지우려면 `rm -rf ~/ros2_ws/build/drone_ros2_advanced ~/ros2_ws/install/drone_ros2_advanced` 후 둘째 줄로 다시 빌드
- 22강 비행은 11주차 월드·기체 그대로입니다 — `obstacle_world.sdf` 를 PX4 폴더로 복사하지 않았으면 11주차 0) 준비의 `cp`. 비행 전에는 **`release/1.16` 필수** ([6주차 절](#6주차-시작-전에-한-번만--px4_msgs-버전-맞추기))

### 1) 21강 — 전이 조건표 시험 (터미널 하나, PX4·Gazebo 없음)
```bash
ros2 run drone_ros2_advanced w12_fsm_sim
```
사용자가 PX4 · 센서 역할입니다. 사건(일어난 일)의 **이름**을 한 줄에 하나씩 입력하면 표에 따라 상태가 바뀝니다. 실행하면 도움말(사건 9개와 뜻 · 명령)이 먼저 나옵니다. 명령: `t` 표 · `del N` 줄 삭제 · `r` 초기화(표·상태) · `v` 상태도 창 끄기·켜기 · `h` 도움말 · `q` 종료. 표 파일은 수정하지 않습니다.

**상태도 창**이 함께 뜹니다. 현재 상태 = 빨강, 방금 쓰인 줄의 화살표 = 빨강, 지나온 길 = 진한 회색. 화살표 옆 숫자가 표의 줄 번호입니다(화살표 하나 = 줄 하나).
- 입력은 **터미널에** 합니다. 창은 표시 전용입니다. 창이 앞에 오면 터미널을 한 번 클릭하세요
- 화면이 없거나(SSH 접속) OpenCV 가 없으면 창 없이 터미널만으로 됩니다

```
[IDLE] > ready
  사건 ready → 1번 줄 → TAKEOFF
  (TAKEOFF 에서 표에 있는 사건: airborne)
[TAKEOFF] > airborne
  사건 airborne → 2번 줄 → CRUISE
  (CRUISE 에서 표에 있는 사건: front_blocked · arrived · sensor_bad)
…
[LAND] > landed
  사건 landed → 6번 줄 → DONE
  미션 끝: IDLE → TAKEOFF → CRUISE → AVOID → CRUISE → LAND → DONE
  (끝 — r 로 초기화)
```
- 괄호 줄 = 현재 상태에서 표에 있는 사건(입력하면 상태가 바뀌는 사건)입니다. 표에 없는 사건이면 현재 상태를 유지합니다: `HOLD` 에서 `front_blocked` → `사건 front_blocked → 해당 줄 없음 → HOLD 유지`

| 실습 | 입력 (한 줄에 하나씩) | 쓰인 줄 | 결과 |
|------|------|------|------|
| 정상 미션 | `ready` `airborne` `front_blocked` `front_clear` `arrived` `landed` | 1 → 2 → 3 → 4 → 5 → 6 | `IDLE → TAKEOFF → CRUISE → AVOID → CRUISE → LAND → DONE` |
| 신호 끊김 → 복구 | `r` 뒤 `ready` `airborne` `sensor_bad` `sensor_ok` | 1 → 2 → 7 → 10 | `CRUISE → HOLD → CRUISE` |
| 신호 끊김 → 타임아웃 | `r` 뒤 `ready` `airborne` `sensor_bad` `hold_timeout` | 1 → 2 → 7 → 9 | `CRUISE → HOLD → LAND` |

**결함 주입 (fault injection)** — 일부러 문제를 넣고 시스템이 어떻게 반응하는지 보는 시험. 표 파일은 바뀌지 않고 시뮬레이터 안의 표만 바뀝니다
- `r` → `del 9 10` (`HOLD` 에서 나가는 줄 두 개 삭제) → 표 점검이 자동으로 실행됩니다. 창에서는 화살표 9 · 10 이 사라지고 `HOLD` 에 빨간 점선
- 복구와 같은 입력 `ready` `airborne` `sensor_bad` `sensor_ok` → `HOLD` 에서 교착 상태 (deadlock). 같은 입력, 다른 결과 — 비행 전에 표에서 결함을 찾습니다
```
[IDLE] > del 9 10
  9번 · 10번 줄 삭제
표 점검
  IDLE 에서 도달할 수 없는 상태           : 없음
  DONE 에 도달할 수 없는 상태 (교착 상태) : HOLD
  → 결함 있음 (r 로 초기화)
…
[CRUISE] > sensor_bad
  사건 sensor_bad → 7번 줄 → HOLD
  (HOLD 에서 나가는 줄 없음 — 교착 상태)
[HOLD] > sensor_ok
  사건 sensor_ok → 해당 줄 없음 → HOLD 유지
  (HOLD 에서 나가는 줄 없음 — 교착 상태)
```
- `r` 로 초기화합니다. `r` 은 표와 상태를 함께 초기화하므로 줄을 삭제하고 시험할 때는 `r` 먼저, `del` 은 그 뒤에
- 삭제해도 줄 번호는 처음 번호를 유지합니다 (`t` 로 확인 — 맨 아래 `(삭제한 줄: 9 10 — r 로 초기화)`)
- (심화) 한 줄에 여러 이름을 띄어 쓰면 차례로 입력한 것과 같습니다. 같은 순간에 생긴 사건은 `+` 로 잇습니다 — `CRUISE` 에서 `arrived+front_blocked` → `같은 순간 사건 2개: arrived · front_blocked → 3번 줄 front_blocked (위 줄이 먼저) → AVOID` (같은 순간 두 줄이 해당하면 위 줄 하나만)

### 2) 22강 — 미션 비행 (터미널 1·2·3·5, 상태도 창 4 는 선택)
```bash
# 터미널 1: 2D LiDAR 기체 + 기둥 월드 (11주차와 같음. VM 이 버거우면 앞에 HEADLESS=1)
cd ~/PX4-Autopilot && PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500_lidar_2d
# 터미널 2: uXRCE-DDS Agent
MicroXRCEAgent udp4 -p 8888
# 터미널 3: pxh> 가 뜬 뒤에 스캔 브리지 — [감지] world=obstacle_world model=x500_lidar_2d_0 → /lidar/scan 줄 확인
ros2 run drone_ros2_advanced w11_scan_bridge
# 터미널 4 (선택): 상태도 창 — PX4 에 명령하지 않음(표시 전용). 미션(5번)보다 먼저 켜면 처음부터 보임
ros2 run drone_ros2_advanced w12_state_viewer
# 터미널 5: 미션 노드 — 실행하면 이륙 → 회피 → 착륙까지 자동 수행
ros2 run drone_ros2_advanced w12_mission --ros-args -p fly:=true
```
- PX4 에 명령하는 노드는 5번 `w12_mission` 하나입니다. **2주차 `keyboard_control` 이나 `w11_avoid` 와 함께 실행하지 않습니다** (Offboard 명령이 섞임)
- 노드는 0.1초 주기로 세 단계를 반복합니다: **사건 판정**(PX4 상태 · 스캔 · 시간 → 조건이 충족된 사건 모두) → **표 조회**(표 밖 안전장치 2개를 먼저, 그다음 표를 위 줄부터 — 현재 상태의 줄만) → **상태별 동작**(명령 1번). 상태 전이 시 `[전이]` 한 줄
- 실습은 비행 2회(+ 선택 1회)이고, **비행마다 PX4(터미널 1)를 재시작**합니다 (아래 '재비행')

**비행 1 — 정상 미션**: 명령 한 줄로 이륙 · 회피 · 착륙까지 자동 수행합니다. 로그 (간이 모델(simple model) 계산 예 — PX4 지연을 넣은 시뮬레이션. VM 숫자는 조금 다름)
```
[전이] IDLE → TAKEOFF | 1번 줄 ready | Offboard + Arm 확인
[전이] TAKEOFF → CRUISE | 2번 줄 airborne | 상대 고도 1.80 m
[전이] CRUISE → AVOID | 3번 줄 front_blocked | 앞 1.49 m < 1.5 m
[AVOID] 왼쪽 선택 (왼쪽 자유 공간 · 오른쪽 3.00 m)
[전이] AVOID → CRUISE | 4번 줄 front_clear | 앞 2.00 m ≥ 해제 2.0 m
[전이] CRUISE → LAND | 5번 줄 arrived | 동 +11.7 m (목표 12.0)
[전이] LAND → DONE | 6번 줄 landed | 착지 감지
[미션 기록] 0.0 IDLE | 1.2 TAKEOFF | 3.2 CRUISE | 15.4 AVOID | 21.4 CRUISE | 38.5 LAND | 42.4 DONE
미션 끝 — Ctrl+C 로 노드를 끄세요
```
- `[전이]` 한 줄 = 표 한 줄: `N번 줄` 이 위 표의 줄 번호입니다. 정상 비행이면 표 1 → 6번 줄 순서로 나옵니다
- 11주차와 달리 `commander land` 가 필요 없습니다. `미션 끝 — Ctrl+C 로 노드를 끄세요` 가 출력되면 종료 순서대로
- 사이사이 `[CRUISE]` · `[AVOID]` 상태 줄이 1초마다 출력됩니다. 전이 줄만 보려면 `ros2 run drone_ros2_advanced w12_mission --ros-args -p fly:=true 2>&1 | grep -E "전이|미션"`, 현재 상태만 보려면 `ros2 topic echo /sjcu/mission_state`

**상태도 창 `w12_state_viewer` (터미널 4, 선택)** — 21강 시뮬레이터와 같은 그림으로 미션을 실시간으로 봅니다. PX4 에 명령하지 않으므로 언제 켜고 꺼도 미션에는 영향이 없습니다
- 위: 상태도. 현재 상태 = 빨강, 방금 쓰인 줄의 화살표 = 빨강, 화살표 옆 숫자 = 표의 줄 번호
- 가운데: 최근 전이 3줄 (`38.5 s  #5  arrived  CRUISE -> LAND`)
- 아래: 최근 60초 상태 띠 + 앞 거리(±30°) 곡선. 1.5 m 정지 판단 · 2.0 m 해제 선 사이에서 `AVOID` 가 시작하고 끝나는 것(히스테리시스)이 보입니다
- 미션 노드가 전이 때마다 `/sjcu/mission_transition` 에 보내는 한 줄을 받아 그립니다: `ros2 topic echo /sjcu/mission_transition` → `15.4 | CRUISE → AVOID | 3 front_blocked | 앞 1.49 m < 1.5 m`
- 미션 노드를 다시 실행하면 그림이 처음부터 그려집니다. 종료: 창에서 `q`
- (교수용) `--ros-args -p record:=true` → `~/w12_state_<시각>.csv` 에 0.1초마다 상태 · 앞 거리 · 전이

**비행 2 — 결함 주입(브리지 종료 후 재실행) → 사람의 PX4 모드 변경(`commander land`)** (PX4 를 재시작한 새 비행)
- 결함 주입: 이륙해 `CRUISE` 가 되면 3번 터미널(브리지) Ctrl+C → 0.5초 뒤 `HOLD` (7번 줄 · `AVOID` 중이면 8번 줄) — 제자리에서 수평 정지 · 고도 유지
- `HOLD` 확인 즉시 3번에서 ↑ + Enter 로 브리지 재실행 (다시 뜨는 데 시간이 걸림 — HOLD 타임아웃 20초 안에) → 스캔이 1.0초 이어지면 `CRUISE` (10번 줄)로 복귀해 미션을 계속합니다
- 사람의 PX4 모드 변경: 다시 전진하기 시작하면 1번 터미널에서 `pxh> commander land` → 약 1초 뒤 `DONE` (표 밖 안전장치) — 노드는 Offboard 재요청 없이 명령을 중단합니다(조종권(control authority) 이양). 착륙은 PX4 가 수행합니다 → 착지 확인 뒤 Ctrl+C
- 로그 (간이 모델 계산 예 — 브리지를 6.0초에 종료하고 9.0초에 재실행, 28.0초에 `commander land`):
```
[전이] CRUISE → HOLD | 7번 줄 sensor_bad | 유효한 스캔 0.5초 없음
[HOLD] 대기 0.0 / 20초 | 스캔 없음 | 수평 정지 · 고도 유지
…
[HOLD] 대기 3.0 / 20초 | 스캔 복구 0.5 / 1.0초 | 수평 정지 · 고도 유지
[전이] HOLD → CRUISE | 10번 줄 sensor_ok | 유효한 스캔 1.0초 이어짐
…
[전이] CRUISE → DONE | 안전장치 px4_takeover | Offboard 가 아닌 채 1초 (PX4 모드: Land) → 다시 요청하지 않음
[미션 기록] 0.0 IDLE | 1.2 TAKEOFF | 3.2 CRUISE | 6.6 HOLD | 10.1 CRUISE | 19.4 AVOID | 25.4 CRUISE | 29.2 DONE
PX4 가 조종 중 (PX4 모드: Land) — 이 노드는 명령 중단 (조종권 이양). 착지 확인 뒤 Ctrl+C (공중에 멈춰 있으면 pxh> commander land)
```

**노드 상태와 PX4 비행 모드** — 노드는 상태에 따라 PX4 비행 모드를 바꾸고(**모드 전환, mode switching**), 사람이나 PX4 가 모드를 바꾸면 명령을 중단합니다(조종권 이양)

| 노드 상태 | PX4 비행 모드 |
|------|------|
| `IDLE` | 노드가 Offboard + Arm 요청 |
| `TAKEOFF` · `CRUISE` · `AVOID` · `HOLD` | Offboard (`HOLD` 도 Offboard — PX4 Hold 모드 아님) |
| `LAND` | 노드가 Land 요청 → 착륙은 PX4 가 수행 |
| `DONE` | 노드가 명령 중단 (Offboard 신호도 중단) |

**표 밖 안전장치 2개** — 22강 노드에만 있고 표보다 먼저 확인합니다 (21강 표 · 시뮬레이터 · 상태도 그림에는 없음). 로그에는 줄 번호 대신 `안전장치`, 전이 토픽 · 상태도 창에는 `safety`

| 사건 | 조건 | 다음 상태 |
|------|------|------|
| `px4_takeover` | 비행 중(`TAKEOFF` · `CRUISE` · `AVOID` · `HOLD`) PX4 가 Offboard 가 아닌 채 1초 — 사람이 모드를 바꿈(`commander land`) · PX4 비상 동작 | `DONE` — 재요청 없이 명령 중단 |
| `dead_end` | `AVOID` 에서 막다른 곳 — 양옆 막힘 · 재탐색 뒤에도 막힘 · 30초 안에 앞이 열리지 않음 | `LAND` |

- 상태도 창은 화살표 없이 최근 전이 줄에만 `safety  px4_takeover  CRUISE -> DONE` (머리줄 오른쪽에 `(safety = checked before the table, no arrow)`), 전이 토픽은 `29.2 | CRUISE → DONE | safety px4_takeover | …`
- `IDLE`(Arm 전)과 `LAND`(노드가 요청한 Land 모드)에서는 `px4_takeover` 를 확인하지 않습니다 — `LAND` 중 `commander land` 는 6번 줄 `landed` 로 끝납니다
- 막다른 곳은 11주차 노드가 멈춰서 호버하던 곳입니다(착륙은 사람이). `obstacle_world` 비행에서는 생기지 않습니다 — 예: `[전이] AVOID → LAND | 안전장치 dead_end | 양옆 막힘 — 앞 1.49 · 왼쪽 0.90 · 오른쪽 0.90 m` (간이 모델, 기둥 앞 양옆에 벽을 둔 시험 월드)

**(선택) 비행 3 — 브리지 종료 후 대기 = 타임아웃**: `CRUISE` 에서 3번 Ctrl+C 후 재실행하지 않으면 `HOLD` 에서 20초 뒤 `LAND` (9번 줄) → 착지 → `DONE` (6번 줄). 11주차 노드는 신호가 복구되지 않으면 호버만 유지했습니다. 로그 (간이 모델 계산 예):
```
[전이] CRUISE → HOLD | 7번 줄 sensor_bad | 유효한 스캔 0.5초 없음
…
[전이] HOLD → LAND | 9번 줄 hold_timeout | 20초 안에 복구되지 않음
[전이] LAND → DONE | 6번 줄 landed | 착지 감지
[미션 기록] 0.0 IDLE | 1.2 TAKEOFF | 3.2 CRUISE | 6.6 HOLD | 26.6 LAND | 30.5 DONE
```
- 대기 시간(타임아웃) 변경: `ros2 param set /mission_base hold_timeout 3.0` (1~60초)

**참고: `fly` 를 빼면 드라이 런(dry run)** — `ros2 run drone_ros2_advanced w12_mission` 처럼 `-p fly:=true` 없이 실행하면 비행하지 않고(arm·이륙·속도 명령 없음) 전이만 출력합니다. 10·11주차와 같은 안전 기본값입니다. 시작 줄이 `미션 노드 시작 — 드라이 런 (fly:=false)` 이고 상태 줄 앞에 `[드라이 런]` 이 붙습니다. 모의 스캔(`ros2 topic pub`)으로 전이를 일으켜 보는 방법은 [`mission_base.py`](drone_ros2_advanced/drone_ros2_advanced/week12/mission_base.py) 머리 주석에 있습니다(브리지(3번)는 종료하고)

**파라미터**: `fly` 기본 false (시작할 때만 읽음) · `cruise_speed` 0~0.5 기본 0.5 (0 이면 `CRUISE` 에서 제자리, `AVOID` 의 좌우 이동은 계속) · `hold_timeout` 1~60 기본 20 (VM 에서 10초로는 브리지를 다시 실행해도 LAND 로 넘어가 20초로 올림). 뒤의 둘은 비행 중에 `ros2 param set /mission_base …` 로 바꿉니다(바뀌면 `전진 속도 변경: 0.5 → 0.3` 같은 한 줄, 범위 밖 값은 경고 후 이전 값 유지). 거리·속도 상수는 11주차와 같습니다(정지 판단 1.5 m · 해제 2.0 m · 좌우 이동 0.3 m/s · 안전 거리 1.0 m)
- 안전 필터 (11주차와 같음, 어느 상태든 명령을 보내기 직전에): 근접(앞 ±45° 1.0 m 미만 → 전진 0) · 옆 안전 거리(가는 쪽 옆 1.0 m 미만 → 좌우 0) · 지오펜스(북 ±3 m 바깥쪽 좌우 0 · 상대 고도 5 m 위로 상승 0) · 사각지대(뒤 90° 는 측정하지 않으므로 후진 없음)

**종료 순서**: `미션 끝 — Ctrl+C 로 노드를 끄세요` 가 출력되면 5 → 3 → 2 → 1 순서로 Ctrl+C (4 는 아무 때나). 비행 중 종료는 1번에서 `pxh> commander land` → 약 1초 뒤 안전장치로 `DONE` → 착지 확인 → 5 → 3 → 2 → 1. 비행 중에 노드를 먼저 종료하면 1초 뒤 PX4 가 Offboard 신호 끊김 비상 동작으로 넘어갑니다

**재비행 — PX4(터미널 1)도 재시작합니다** — 종료 순서대로 종료한 뒤 1 → 2 → 3 → (4) → 5. 노드만 다시 실행하면 착륙한 위치에서 이륙하므로, 목표 위였다면 `[전이] CRUISE → LAND | 5번 줄 arrived | 동 +12.0 m (목표 12.0)` 처럼 바로 착륙합니다

### 막혔을 때
| 증상 | 원인 → 해결 |
|------|-------------|
| 시뮬레이터가 `알 수 없는 사건: …` · `숫자는 표의 줄 번호입니다 …` · `한글이 입력됐습니다 …` · `… 는 상태 이름입니다 …` · `알 수 없는 입력: c` 같은 안내만 출력 | 사건은 영어 **이름**을 한 줄에 하나씩 → 한/영 키, 띄어쓰기 대신 밑줄(`front_blocked`), 상태 이름(`TAKEOFF`)이 아니라 사건 이름(`airborne`). 숫자는 `del 9` 처럼 줄을 삭제할 때만. 옛 명령 `c` · `s` · `swap` 은 없음(점검은 `del` 뒤에 자동 실행). `h` = 도움말 |
| 시뮬레이터에서 사건 이름을 입력해도 반응 없음 | 상태도 창이 선택됨 (창에 `type in the TERMINAL window` 가 뜸) → 터미널을 한 번 클릭 |
| `… → 미션이 끝났습니다 — r 로 초기화` | `DONE` 뒤에는 사건을 받지 않음 → `r` (표도 초기화되므로 줄 삭제는 `r` 뒤에) |
| 상태도 창이 안 뜸 (`상태도 창 없음 … — 터미널만으로 계속`) | 화면 없음(SSH) · OpenCV 없음 → 시뮬레이터는 터미널만으로 됩니다. `w12_state_viewer` 는 OpenCV 가 있어야 함 — 5주차 OpenCV 설치 확인 |
| `ros2 pkg executables` 에 `w12_keyboard` 도 보임 | 옛 빌드가 남음 → 쓰지 않음(미션 노드가 받지 않음). 지우려면 `rm -rf ~/ros2_ws/build/drone_ros2_advanced ~/ros2_ws/install/drone_ros2_advanced` 후 다시 빌드, `source install/setup.bash` |
| `[IDLE] 스캔 대기 — w11_scan_bridge(터미널 3) 실행 확인` 이 계속 | 브리지(3번) 미실행 / `pxh>` 보다 먼저 켬 → `ros2 topic hz /lidar/scan` 확인, 브리지 다시 실행 |
| 시작 줄 뒤로 아무것도 안 나오고 이륙하지 않음 (`Offboard 모드 전환` 도 없음) | PX4 위치가 안 옴 → 터미널 2 Agent 실행 확인, `ros2 topic info /fmu/out/vehicle_local_position` 의 `Publisher count` 가 1 인지 |
| 이륙하지 않고 "Offboard 모드 전환 / Arm 명령 전송" 반복 | px4_msgs 가 main → 6주차 절의 `release/1.16` 전환 |
| 시작 줄이 `드라이 런 (fly:=false)` · 로그에 `[드라이 런]` · 전이는 출력되는데 드론이 이륙하지 않음 | `-p fly:=true` 빠짐 → 5번 Ctrl+C 후 `ros2 run drone_ros2_advanced w12_mission --ros-args -p fly:=true` |
| 상태도 창이 두 상태를 오감 | 미션 노드가 둘 실행됨 → `ros2 node list` 에 `/mission_base` 가 둘이면 5번 하나만 남기고 Ctrl+C |
| `commander land` 를 친 적 없는데 `[전이] … → DONE \| 안전장치 px4_takeover` | PX4 가 Offboard 를 끊음(비상 동작 — 줄 끝 `PX4 모드:` 확인) → `pxh>` 로그 확인. 다른 노드가 PX4 에 명령 중인지 `ros2 node list` |
| 브리지를 재실행했는데 `HOLD → LAND` | 20초 안에 스캔이 복구되지 않음(HOLD 타임아웃 — 정상 동작) → 느린 PC 에서는 `ros2 param set /mission_base hold_timeout 30` 처럼 늘려 두기 |
| 이륙하자마자 `CRUISE → LAND` | PX4 는 재시작하지 않고 노드만 다시 실행함(이미 목표 위) → PX4 재시작 |
| PX4 를 다시 켰는데 `gazebo already running world:` | 이전 Gazebo 서버가 남음 → `pkill -9 -f "gz sim"` → `pgrep -af "gz sim"` 빈 줄 확인 후 다시 |
| 도착했는데 목표 판 밖에 착륙 | 전진 속도를 유지한 채 착륙 모드로 전환되어 조금 밀림 → `ros2 param set /mission_base cruise_speed 0.3` |
| 상태도 창이 `STATE -` · waiting 에서 안 바뀜 | `w12_mission` 미실행 → 미션 노드를 실행하면 바로 바뀝니다 |
| 상태도 창 아래가 `front (+-30 deg): no scan` | 브리지(3번) 종료됨 → 브리지 재실행 |
| `ros2 param set` 이 노드를 못 찾음 / 값이 안 먹음 | 노드 이름 오타 / 범위 밖 값 → `ros2 node list` 에서 `/mission_base`, 노드 로그의 경고 확인 |

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

Fuel(온라인 모델 저장소) 모델을 놓은 `my_custom_world_fuel.sdf` 도 있습니다. 물체 위치가 같아 **같은 미션 파일로 둘 다 비행**됩니다.
Fuel 모델은 첫 실행 때 받아지지만 **하나라도 못 받으면 월드 전체가 안 뜨므로**, 실습 전에 미리 받아 두세요.
```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Jersey Barrier"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction Cone"
```

## 폴더 구조
```
drone_ros2_advanced/
├── config/waypoints.yaml      # 3주차~ waypoint 미션 설정
├── worlds/                    # Gazebo World (PX4 폴더로 복사해 사용): 4주차 my_custom_world, 8주차 person_world + models/, 10주차 lidar_world, 11주차 obstacle_world
└── drone_ros2_advanced/
    ├── px4_base.py            # PX4Base 클래스 (base 버전의 부모)
    ├── week02/                # 노드 기초 + 키보드 제어
    ├── week03/                # 단일·다중 Waypoint 비행
    ├── week05/                # 카메라 · OpenCV · ArUco
    ├── week06/                # 오차 제어 · 정밀착륙
    ├── week08/                # 객체 인식 (dnn_demo · person_detector · video_publisher · models/ MobileNet-SSD · samples/ 예시 사진·영상)
    ├── week09/                # 사람 추종 비행 (person_follow_base = w09_follow)
    ├── week10/                # 거리 센서 · LiDAR 지형 추종 (lidar_bridge = w10_lidar_bridge · range_viewer = w10_range_viewer · terrain_follow_base = w10_terrain_follow)
    ├── week11/                # 2D LiDAR 장애물 감지 · 회피 (scan_bridge = w11_scan_bridge · scan_viewer = w11_scan_viewer · avoid_base = w11_avoid)
    ├── week12/                # 상태머신 미션 (mission_table = 전이 조건표 · fsm_sim = w12_fsm_sim · mission_base = w12_mission · state_view = 상태도 그림 · state_viewer = w12_state_viewer)
    ├── week13/                # 추종 안정화 (통합 미션)
    ├── week14/                # 심화 (YOLO)
    └── extras/                # 구 커리큘럼 자료 (참고용): GPS/지도(folium·flask 필요), 구 9주차 얼굴 추종 코드
```
※ 4주차는 파이썬 노드가 아니라 `worlds/` 의 SDF 파일이 실습 자산입니다.

## 참고
- 트러블슈팅: Gazebo 화면이 검게 나오면 `export LIBGL_ALWAYS_SOFTWARE=1` (VM 환경)
- 트러블슈팅: `import cv2` 에서 `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x` / `numpy.core.multiarray failed to import` → numpy 가 2.x 로 올라간 것. `pip install 'opencv-python>=4.7' 'numpy<2'` 로 1.x 로 내리면 해결 (실측 확인)
- 트러블슈팅: `/camera/image_raw` 가 안 보이면 → 터미널 3의 `w05_camera_bridge` 실행 여부, `gz topic -l | grep image` 로 Gazebo 쪽 토픽 확인
- 트러블슈팅: PX4 로그에 `gazebo already running world: …` 가 뜨고 Gazebo 창이 안 열리면 → 이전 Gazebo **서버**가 뒤에 남아 있는 것 (이때는 `PX4_GZ_WORLD` 를 바꿔도 이전 월드가 재사용됨). PX4 를 Ctrl+C 로 끈 뒤 `pkill -9 -f "gz sim"` → `pgrep -af "gz sim"` 이 아무것도 안 나오면 다시 실행. PX4 도 남았으면 `pkill -9 -f px4`. (`HEADLESS=1` 을 켜 둔 터미널이면 원래 창이 안 뜸 → `unset HEADLESS`)
- 비전 실습(5주차~)은 카메라 렌더링 부담이 큼 — VM에서는 `HEADLESS=1` + 해상도 축소 권장, GPU가 있는 네이티브 Ubuntu면 더 원활
- YOLOv8n은 CPU 환경에서도 동작 가능 (저사양 PC 대응)
