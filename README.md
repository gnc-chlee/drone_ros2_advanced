# drone_ros2_advanced

사이버대학교 **드론로봇프로그래밍심화(ROS2)** 실습 패키지

## 환경
- Ubuntu 22.04
- ROS2 Humble
- PX4 Autopilot
- Gazebo 시뮬레이션

## 설치 방법
```bash
cd ~/ros2_ws/src
git clone https://github.com/gnc-chlee/drone_ros2_advanced.git
cd ~/ros2_ws
colcon build --packages-select drone_ros2_advanced
source ~/.bashrc
```

추가 파이썬 패키지 (해당 주차에 설치):
```bash
pip install folium        # 5주차
pip install flask         # 6주차
pip install ultralytics   # 14주차
```

## 실습 코드는 두 가지 버전!

| 버전 | 설명 |
|------|------|
| **raw** | PX4 토픽에 직접 접근. QoS, heartbeat, 명령 조립까지 전부 코드에 보임 |
| **base** | `px4_base.py`(PX4Base 클래스)를 상속. 어려운 부분은 Base가 처리하고 로직만 작성 |

전반부에는 raw로 원리를 눈으로 확인하고, 뒤로 갈수록 base를 사용합니다.
코드에서 `[복붙 영역]`이라고 표시된 부분은 **복사해서 쓰고 원리만 이해**하면 됩니다.

## 커리큘럼 & 실습 코드

| 주차 | 내용 | 실행 명령 (`ros2 run drone_ros2_advanced ...`) |
|------|------|------|
| 1주차 | 기초 복습 (offboard, 단일 position 이동) / PX4 + Gazebo 환경 재점검 | `w01_takeoff_raw` / `w01_takeoff_base` |
| 2주차 | 다중 position 이동으로 확장 / ROS2 토픽·서비스·액션 복습 | `w02_multi_raw` / `w02_multi_base` |
| 3주차 | 다중 Waypoint 비행 / Waypoint 리스트 yaml 파일로 관리 | `w03_yaml_raw` / `w03_yaml_base` |
| 4주차 | 도착 판정 로직 (허용 오차 범위) / 미션 완료 후 자동 착륙 | `w04_mission_raw` / `w04_mission_base` |
| 5주차 | GPS ↔ Local NED 좌표계 이해 / Folium 기반 지도 UI | `w05_gps_viewer`, `w05_map_demo` |
| 6주차 | 지도 클릭 → ROS2 토픽으로 waypoint 전송 연동 | `w06_map_server` + `w06_goto_raw`/`w06_goto_base` |
| 7주차 | **중간고사** | - |
| 8주차 | OpenCV(contour) 복습 / Gazebo 카메라 → ROS2 image 토픽 | `w08_camera_viewer`, `w08_contour` |
| 9주차 | PX4 드론 적용 차이점 / 카메라 기반 드론 제어 개념 | `w09_center_error` |
| 10주차 | Haar Cascade 얼굴 인식 / 얼굴 위치 → 드론 이동 명령 변환 | `w10_face_detector`, `w10_face_command` |
| 11주차 | 화면 중앙 기준 오차 계산 / 비례 제어 개념 | `w11_p_control` |
| 12주차 | 얼굴 인식 + offboard 연동 / Following 비행 구현 | `w12_follow_raw` / `w12_follow_base` |
| 13주차 | Following 안정화 튜닝 / 얼굴 없을 때 호버링 처리 | `w13_stable_raw` / `w13_stable_base` |
| 14주차 | YOLOv8n 소개 / Haar Cascade vs YOLO 비교 / 사람 감지 데모 | `w14_yolo`, `w14_haar_vs_yolo` |
| 15주차 | **기말고사** | - |

※ 비전 노드(8~11, 14주차 검출기)는 드론 제어와 무관하므로 raw/base 구분이 없습니다.

## 기본 실행 순서 (비행 실습 공통)
```bash
# 터미널 1: PX4 SITL + Gazebo
cd ~/PX4-Autopilot && make px4_sitl gz_x500

# 터미널 2: uXRCE-DDS Agent (PX4 ↔ ROS2 다리)
MicroXRCEAgent udp4 -p 8888

# 터미널 3: 실습 노드
ros2 run drone_ros2_advanced w01_takeoff_raw
```

## 폴더 구조
```
drone_ros2_advanced/
├── config/waypoints.yaml      # 3주차~ waypoint 미션 설정
└── drone_ros2_advanced/
    ├── px4_base.py            # PX4Base 클래스 (base 버전의 부모)
    ├── week01/ ~ week06/      # 전반부: Waypoint 미션 비행
    └── week08/ ~ week14/      # 후반부: 비전 기반 Following
```

## 실습 내용
1. **Waypoint 미션 비행** - 다중 waypoint 자동 비행 + 지도 클릭 연동
2. **얼굴 인식 Following** - Haar Cascade 기반 얼굴 추적 비행
3. **YOLO 맛보기** - YOLOv8n 사람 감지 데모 (CPU 가능)

## 참고
- 기초 과목 저장소: [drone_ros2_basic](https://github.com/gnc-chlee)
- YOLOv8n은 CPU 환경에서도 동작 가능 (저사양 PC 대응)
