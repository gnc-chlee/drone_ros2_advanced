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
colcon build
source ~/.bashrc
```

## 커리큘럼

| 주차 | 내용 |
|------|------|
| 1주차 | 기초 과목 복습 (offboard, 단일 position 이동) / PX4 + Gazebo 환경 재점검 |
| 2주차 | 다중 position 이동으로 확장 / ROS2 토픽·서비스·액션 복습 |
| 3주차 | 다중 Waypoint 비행 / Waypoint 리스트 yaml 파일로 관리 |
| 4주차 | 도착 판정 로직 (허용 오차 범위) / 미션 완료 후 자동 착륙 |
| 5주차 | GPS ↔ Local NED 좌표계 이해 / Folium 기반 지도 UI |
| 6주차 | 지도 클릭 → ROS2 토픽으로 waypoint 전송 연동 |
| 7주차 | **중간고사** |
| 8주차 | 기초 과목 OpenCV(contour) 복습 / Gazebo 카메라 플러그인 → ROS2 image 토픽 |
| 9주차 | PX4 드론 적용 차이점 / 카메라 기반 드론 제어 개념 |
| 10주차 | Haar Cascade 얼굴 인식 / 얼굴 위치 → 드론 이동 명령 변환 |
| 11주차 | 화면 중앙 기준 오차 계산 / 비례 제어 개념 |
| 12주차 | 얼굴 인식 + offboard 연동 / Following 비행 구현 |
| 13주차 | Following 안정화 튜닝 / 얼굴 없을 때 호버링 처리 |
| 14주차 | YOLOv8n 소개 / Haar Cascade vs YOLO 비교 / 사람 감지 데모 |
| 15주차 | **기말고사** |

## 실습 내용
1. **Waypoint 미션 비행** - 다중 waypoint 자동 비행 + 지도 클릭 연동
2. **얼굴 인식 Following** - Haar Cascade 기반 얼굴 추적 비행
3. **YOLO 맛보기** - YOLOv8n 사람 감지 데모 (CPU 가능)

## 참고
- 기초 과목 저장소: [drone_ros2_basic](https://github.com/gnc-chlee)
- YOLOv8n은 CPU 환경에서도 동작 가능 (저사양 PC 대응)