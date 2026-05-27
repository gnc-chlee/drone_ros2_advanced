# drone_ros2_advanced

사이버대학교 드론로봇프로그래밍심화(ROS2) 실습 패키지

## 환경
- Ubuntu 22.04
- ROS2 Humble
- PX4 Autopilot

## 설치 방법
```bash
cd ~/ros2_ws/src
git clone https://github.com/gnc-chlee/drone_ros2_advanced.git
cd ~/ros2_ws
colcon build
source ~/.bashrc
```

## 실습 내용
1. Waypoint 미션 비행 (지도 연동)
2. 얼굴 인식 Following (Haar Cascade)
