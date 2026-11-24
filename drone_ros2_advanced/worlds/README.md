# worlds — 4주차 커스텀 Gazebo World

`my_custom_world.sdf` 는 4주차 실습의 **시연 시작점**입니다. PX4 내장 `aruco.sdf` 를 복사해
마커를 원점 밖으로 옮기고 Fuel 모델(Table)을 하나만 놓아 두었습니다 — 수업에서 `<include>` 를
하나씩 추가하며 배치를 익히는 용도입니다.
모델을 여러 개 배치한 **완성 예시**는 `my_custom_world_fuel.sdf` 입니다.

## 설치 (한 번만)

월드 파일은 ROS2 패키지가 아니라 **PX4 폴더**에 있어야 합니다. PX4가 월드를 찾는 경로가
`~/PX4-Autopilot/Tools/simulation/gz/worlds/` 로 고정돼 있기 때문입니다.

```bash
cp ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/worlds/my_custom_world.sdf \
   ~/PX4-Autopilot/Tools/simulation/gz/worlds/
```

## 실행

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=my_custom_world make px4_sitl gz_x500
```

카메라가 필요하면(5주차 이후) 기체만 바꿉니다.

```bash
PX4_GZ_WORLD=my_custom_world make px4_sitl gz_x500_mono_cam_down
```

## 규칙 세 가지

| 규칙 | 이유 |
|------|------|
| **파일명 = `<world name>`** | PX4가 `PX4_GZ_WORLD` 값을 파일 경로와 Gazebo 월드 이름 양쪽에 쓴다. 다르면 `Timed out waiting for Gazebo world` 로 종료 |
| **`<world>` 바로 아래 시스템 `<plugin>` 금지** | PX4는 Physics·Imu·NavSat·Sensors 등 13가지를 `server.config` 로 자동 적용하는데, 월드 직계에 plugin이 하나라도 있으면 그 자동 적용이 통째로 취소된다 (모델·GUI 플러그인은 해당 없음) |
| **`make px4_sitl gz_x500_<월드>` 형태는 쓰지 않기** | 그 타깃은 빌드 설정 시점에 폴더를 훑어 만들어지므로 새 월드에는 없다. 항상 `PX4_GZ_WORLD=` 방식으로 |

## 좌표 — Gazebo(ENU) ↔ PX4(NED)

SDF의 `<pose>` 여섯 숫자는 `x y z roll pitch yaw` 이고 **ENU** 입니다.

| | Gazebo `<pose>` (ENU) | PX4 setpoint (NED) |
|---|---|---|
| x | 동 | 북 |
| y | 북 | 동 |
| z | 위 | 아래 |

이 월드에 놓인 물체의 좌표 대응:

| 물체 | 있는 월드 | Gazebo `<pose>` x y z | PX4 (북, 동) |
|------|-----------|----------------------|--------------|
| `marker_a` (ArUco 마커) | 둘 다 | 5 3 0.001 | (3, 5) |
| `table_1` (테이블) | 둘 다 | -6 0 0 | (0, −6) |
| `barrier_1` (방호벽) | fuel판 | 0 9 0 | (9, 0) |
| `barrier_2` (방호벽) | fuel판 | -5 9 0 | (9, −5) |
| `tree_1` (소나무) | fuel판 | 8 -4 0 | (−4, 8) |
| `cone_1` (공사용 콘) | fuel판 | 2 -2 0 | (−2, 2) |

`config/my_custom_world_mission.yaml` 이 이 좌표들을 NED로 적어둔 미션 파일입니다.
**두 월드에 공통인 물체(테이블 → 마커)만 쓰므로 어느 월드에서든 그대로 비행됩니다.**
미션 노드는 waypoint에 도달하면 멈추지 않고 곧바로 다음 목표로 가며, 목록을 다 돌면 **그 자리에서 착륙**합니다.
그래서 마커를 마지막에 두어 **마커 위에 내려앉는 것**으로 좌표 변환을 확인합니다.
※ 위 대응표는 월드 원점과 PX4 로컬 원점이 같다는 전제(기본 스폰 위치)에서 성립합니다.

```bash
# 테이블 → 마커 순서로 비행한 뒤 마커 위에 착륙 (고도 5m)
ros2 run drone_ros2_advanced w03_mission_raw --ros-args \
  -p waypoint_file:=$(ros2 pkg prefix drone_ros2_advanced)/share/drone_ros2_advanced/config/my_custom_world_mission.yaml
```

## Fuel 모델 쓰기 (my_custom_world_fuel.sdf)

`my_custom_world_fuel.sdf` 는 같은 위치에 **Fuel(온라인 모델 저장소)** 모델을 놓은 버전입니다.
직접 도형을 만드는 대신 카탈로그에서 골라 `<pose>` 만 지정합니다.

```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table</uri>
  <name>table_1</name>          <!-- 같은 모델 여러 개면 이름을 다르게! -->
  <pose>-6 0 0 0 0 0</pose>     <!-- x(동) y(북) z(위) roll pitch yaw -->
</include>
```

### 내가 원하는 모델 쓰기 — 주소 확인

Fuel 사이트에서 마음에 드는 모델을 찾았다면, `<uri>` 에 넣을 주소를 정확히 알아내야 합니다.

| | 주소 | |
|---|---|---|
| 브라우저 주소창 | `https://app.gazebosim.org/OpenRobotics/fuel/models/Table` | ✗ 웹페이지 주소라 SDF에서 동작 안 함 |
| SDF `<uri>` | `https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table` | ✓ 호스트가 `fuel`, 중간에 `/1.0/` |

가장 확실한 방법은 **모델 페이지의 인용(citation) 블록**에서 `url={...}` 안의 주소를 그대로 복사하는 것입니다.

```
@online{GazeboFuel-OpenRobotics-Table,
    title={Table}, organization={Open Robotics}, date={2023}, month={September}, day={4},
    author={OpenRobotics},
    url={https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table},    ← 이 줄
}
```

주소 구조는 `https://fuel.gazebosim.org/1.0/<소유자>/models/<모델 이름>` 입니다.
같은 이름을 다른 사람이 올린 경우가 있으므로 **소유자까지 맞아야** 합니다.
모델 이름의 공백은 SDF에서 그대로 두고, 셸 명령에서만 따옴표로 감쌉니다.

### ★ 실습 전에 미리 받아두세요

Fuel 모델은 **첫 실행 때 내려받아** `~/.gz/fuel` 에 저장되고, 그 뒤로는 인터넷 없이 실행됩니다.
문제는 **모델 하나라도 못 받으면 월드 전체가 안 뜬다**는 점입니다(모델만 빠지는 게 아닙니다).

```bash
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Table"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Jersey Barrier"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Pine Tree"
gz fuel download -u "https://fuel.gazebosim.org/1.0/OpenRobotics/models/Construction Cone"
```

받아졌는지 확인: `ls ~/.gz/fuel/fuel.gazebosim.org/OpenRobotics/models/`

### 검증된 모델 목록 (VM 소프트웨어 렌더링 기준)

| 모델 | 이름 (URI 뒤에 붙일 것) | 크기 | 고정? | 메모 |
|------|------------------------|------|-------|------|
| 테이블 | `Table` | 1.5×0.8×1.0 m | 고정 | **가장 가벼움** — 메쉬 없이 박스·원통뿐. 상판(1m)에 착륙도 가능 |
| 방호벽 | `Jersey Barrier` | 4.07×0.81×1.14 m | 고정 | 파일 최소(167KB), 길어서 잘 보임. 벽·통로 만들기 좋음 |
| 소나무 | `Pine Tree` | 큼 | 고정 | 가볍지만 잎이 반투명이라 **5그루 이내** |
| 참나무 | `Oak tree` | 큼 | 고정 | 소나무와 동급 (※ 이름의 t 는 소문자) |
| 소화전 | `Fire hydrant` | 0.42×0.42×0.93 m | 고정 | 가볍지만 작아서 눈에 덜 띔 |
| 공사용 콘 | `Construction Cone` | 0.5×0.5×1.09 m | **움직임** | 드론이 스치면 넘어짐 — 충돌 시연에 좋음 |
| SUV | `SUV` | 차 한 대 | 고정 | 텍스처 1024×1024 두 장 — VM에서 1대까지 |
| 종이상자 | `Cardboard box` | 0.5×0.4×0.3 m | **움직임** | 모델에 z=0.15가 내장돼 있어 `<pose>` z를 0으로 두면 절반이 묻힘 |

**쓰지 말 것**: `Standing person`, `Walking person` — 4주차 점검 때 `<include>` 로 형상이 로드되지 않았습니다.
8~9주차의 사람은 이 두 모델이 아니라 **`person_world.sdf` 의 걷는 사람(actor)** 을 씁니다 (아래 "8주차 사람 월드").
`Casual female` 도 쓸 수는 있지만 첫 다운로드가 27 MB 라 미리 받아야 합니다(실제 텍스처는 2048² 이하 — 4096 은 쓰이지 않는 노멀맵).

### 라이선스와 출처

이 저장소가 쓰는 Fuel 모델 6개는 **전부 CC0 1.0 Universal(퍼블릭 도메인 헌정)** 이라 자유롭게 쓰고 배포할 수 있습니다.
법적으로 출처 표기 의무는 없지만, 수업 자료에는 아래 표를 남깁니다.

| 모델 | 버전 | 제작 | 라이선스 |
|------|------|------|----------|
| Table | 4 | Open Robotics | CC0 1.0 |
| Jersey Barrier | 6 | Open Robotics | CC0 1.0 |
| Construction Cone | 3 | Open Robotics | CC0 1.0 |
| Fire hydrant | 3 | Open Robotics | CC0 1.0 |
| Pine Tree | 6 | Open Robotics | CC0 1.0 |
| Oak tree | 7 | Open Robotics | CC0 1.0 |

출처: Gazebo Fuel <https://app.gazebosim.org/fuel/models> / CC0 전문 <https://creativecommons.org/publicdomain/zero/1.0/>

> **다른 모델을 고를 때는 라이선스를 반드시 확인하세요.** Fuel에는 출처 표기가 의무인 CC-BY 모델도 있습니다.
> 모델 페이지에 라이선스와 인용 정보(`@online{...}` 형식)가 함께 표시됩니다.

### Fuel 관련 주의

- URI는 `https://fuel.gazebosim.org/1.0/OpenRobotics/models/<이름>` 형식. 이름의 **공백은 그대로** 두고, 셸 명령에서는 따옴표로 감쌀 것
- 같은 모델을 여러 개 놓을 때는 `<name>` 을 반드시 다르게 (이름이 겹치면 하나만 나옴)
- 각 모델은 자기 원점 위치가 달라서, `<pose>` 의 z는 0으로 두고 **뜨거나 묻히면 그때 조정**
- 캐시를 통째로 배포할 수도 있습니다: `tar czf gz-fuel-cache.tgz -C ~/.gz/fuel fuel.gazebosim.org` → 학생이 `~/.gz/fuel` 에 풀기

## 8주차 사람 월드 (person_world.sdf)

14강(사람 인식 노드)·9주차(추종 비행)용. 땅 + 해(그림자 끔) + 걷는 사람 1명뿐인 가장 단순한 월드입니다.

```bash
cd ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/worlds
cp person_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cp -r models/* ~/PX4-Autopilot/Tools/simulation/gz/models/          # 사람 모델 (인터넷 불필요)
cd ~/PX4-Autopilot && HEADLESS=1 PX4_GZ_WORLD=person_world make px4_sitl gz_x500_mono_cam
```

- 사람은 Fuel 에서 받지 않고 `models/sjcu_walker/meshes/walk.dae`(chapulina Walking actor, CC0)를 씁니다 → 인터넷·Fuel 다운로드 실패와 무관
- 사람의 길 (Gazebo ENU, 드론은 원점에서 동쪽을 봄 — 화면에서 북쪽(+y)이 왼쪽):

| 시간(시뮬) | 장면 | 위치 |
|------|------|------|
| 0 ~ 11 s | 멀어짐 (뒷모습) | (4, 0) → (8, 0) |
| 13 ~ 21 s | 8 m 정지 (앞모습) | (8, 0) |
| 22 ~ 29 s | 북쪽으로 비켜섬 (옆모습) | (8, 0) → (8, 2.5) |
| 30 ~ 41 s | 다가옴 (앞모습) | (8, 2.5) → (4, 2.5) |
| 42 ~ 57 s | 4 m 앞을 가로지름 (옆모습) | (4, 2.5) → (4, −2.5) |
| 58 ~ 65 s | 가운데로 (옆모습) | (4, −2.5) → (4, 0) |
| 66 ~ 74 s | 4 m 정지 (앞모습) | (4, 0) |

- actor 의 `<pose>` z 는 **1.0** (메시 원점이 허리 높이 — 0 이면 땅에 묻힘, 4주차 종이상자 z 함정과 같은 이야기)
- 걸은 거리만큼 다리를 움직이므로, 멈춘 동안에는 걷던 자세 그대로 서 있습니다
- 강사 점검용 `person_check_world.sdf`: 사람이 3·4·5·6·8 m 에서 드론을 보고 10초씩 정지 (거리별 신뢰도 측정). 사진 입간판(`models/sjcu_person_board`, Plan B)도 주석을 풀어 비교

## 11주차 기둥 월드 (obstacle_world.sdf)

19·20강(2D LiDAR 장애물 감지·회피)용. 땅 + 해 + 기둥 1개 + 남쪽 벽 1개 + 목표 표시뿐인 월드입니다 (Fuel 불필요).

```bash
cd ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/worlds
cp obstacle_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cd ~/PX4-Autopilot && PX4_GZ_WORLD=obstacle_world make px4_sitl gz_x500_lidar_2d
```

| 모델 | `<pose>` (ENU) | 크기 (x·y·z) | 범위 |
|------|------|------|------|
| `pillar` (기둥) | 6.0 0 2.0 | 1 × 1 × 4 m | 동 5.5~6.5, 북 −0.5~0.5 |
| `south_wall` (남쪽 벽) | 7.5 −3.1 2.0 | 11 × 0.2 × 4 m | 동 2~13, 북쪽 면 = 북 −3.0 |
| `goal_pad` (목표 표시) | 12.0 0 0.005 | 1 × 1 × 0.01 m | 스캔 평면보다 낮아 LiDAR 에 안 보임 |

- 드론은 원점에서 동쪽을 봄 → 동쪽으로 곧장 가면 기둥에 막히고, 왼쪽(북쪽)이 넓어 왼쪽으로 돌아감. 기둥과 벽 사이 2.5 m 라 오른쪽으로 돌아가도 지나갈 수는 있음 (20강 `v_side -0.3` 일부러 틀리기)
- 높이 4 m 는 2D LiDAR 스캔 평면(상대 고도 2 m 에서 땅 위 약 2.3 m)보다 충분히 높게 — 평면보다 낮은 물체(10주차 1 m 상자)는 2D LiDAR 에 안 보입니다
- 스폰 위치의 섹터 값 (계산): 앞 5.38 m (기둥) · 왼쪽 자유 공간 · 오른쪽 3.54 m (벽 서쪽 끝 모서리)
- 7강 `walls` 월드(PX4 내장)는 벽 폭이 20 m 라 돌아가기에 길어서 이 월드를 따로 만들었습니다

## 13주차 주차장 월드 (parking_world.sdf) · 합성 기체 (x500_lidar_2d_cam_down)

24강(통합 미션 「주차장 지정 칸 착륙」)용. 11주차 기둥 월드의 코스(기둥 1개 + 남쪽 벽)에 주차장(주차칸 5개 + 칸마다 ArUco 마커)을 더한 월드와, 2D LiDAR + 아래 카메라를 함께 단 기체입니다 (Fuel 불필요 — 인터넷 없이 뜸).

```bash
# 한 번만: 월드 1개 + 모델 폴더(합성 기체 · 마커 그림)를 PX4 로 복사 (8주차에 models/ 를 복사했어도 다시)
cd ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/worlds
cp parking_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cp -r models/* ~/PX4-Autopilot/Tools/simulation/gz/models/
ls ~/PX4-Autopilot/Tools/simulation/gz/models | grep -E "x500_lidar_2d_cam_down|sjcu_parking_markers"     # 두 줄
# 실행 (남은 Gazebo 서버를 먼저 정리)
pkill -9 -f "gz sim"
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_lidar_2d_cam_down PX4_GZ_WORLD=parking_world ./build/px4_sitl_default/bin/px4
```

- 들어 있는 것: 월드 `parking_world.sdf` + 모델 폴더 2개 — `models/sjcu_parking_markers/`(주차칸 마커 그림 5장 = ArUco `DICT_4X4_50` 의 ID 0~4) · `models/x500_lidar_2d_cam_down/`(합성 기체 = x500 + 2D LiDAR + 아래 카메라). 출처 · 라이선스는 [`models/README.md`](models/README.md)
- 합성 기체는 `make px4_sitl gz_…` 로 띄울 수 없습니다(그 대상은 PX4 에 airframe 파일이 있는 기체만 만들어짐) → 위처럼 PX4 를 직접 실행합니다. 모델 폴더 복사만으로 뜨고 재빌드 · airframe 추가는 없습니다 (airframe 4001 = 기본 x500 과 같은 파라미터)
- 폴더 이름 = `<model name>` = `PX4_SIM_MODEL` 의 `gz_` 뒤. Gazebo 안의 기체 이름은 `x500_lidar_2d_cam_down_0`
- 센서 2개 (링크 · 센서 이름이 기본 기체와 같음 — Gazebo 토픽 이름은 기체 이름 부분만 다르고, 브리지 자동 감지 · ROS 쪽 이름은 그대로):
  - 2D LiDAR — 링크 `link` / 센서 `lidar_2d_v2`, 1080칸 · ±135° · 0.1~30 m · **10 Hz** (PX4 원본 30 Hz 에서 낮춤 — 2026-11-17 교수 VM: 스캔 평균 약 7.5 Hz)
  - 아래 카메라 — 링크 `camera_link` / 센서 `imager`, 640 × 480 · 15 Hz (8주차 표준과 같은 값)
- 기체가 안 뜨면(`pxh>` 는 떠도 Gazebo 에 기체가 없음): 시작 로그의 `[Err] [UserCommands.cc…` 줄 = 스폰 거절 → 모델 폴더 복사 · 폴더 이름 확인 (`GstCameraSystem` 의 `[Err]` 는 카메라 기체에서 날 수 있는 줄 — 무관)
- 화면 없이(느린 VM): `HEADLESS=1` 을 `&&` 바로 뒤에 — `cd ~/PX4-Autopilot && HEADLESS=1 PX4_SYS_AUTOSTART=4001 …` (줄 맨 앞에 쓰면 `cd` 에만 걸려 효과 없음)

| 모델 | `<pose>` (ENU) | 크기 (x·y·z) | 범위 |
|------|------|------|------|
| `pillar` (기둥) | 6.0 0 2.0 | 1 × 1 × 4 m | 동 5.5~6.5, 북 −0.5~0.5 (11주차와 같음) |
| `south_wall` (남쪽 벽) | 7.5 −3.1 2.0 | 11 × 0.2 × 4 m | 동 2~13, 북쪽 면 = 북 −3.0 (11주차와 같음) |
| `parking_floor` (주차 바닥) | 19 0 0.005 | 19 × 7 × 0.01 m | 동 9.5~28.5, 북 −3.5~3.5 |
| `parking_marker_0` ~ `_4` (마커 판) | 12.0 + 3.5 × N, 0, 0.012 | 0.6 × 0.6 m 평면 (그림만 — 충돌 없음) | N번 칸 = 동 12.0 · 15.5 · 19.0 · 22.5 · 26.0 |
| `building_1` (건물) | 21 23.5 4.0 | 10 × 6 × 8 m | 동 16~26, 북 20.5~26.5 |
| `start_pad` (시작 표시) | 0 0 0.001 | 반지름 2 m (그림만) | 드론이 뜨는 자리 |

- **N번 칸 마커 중심 = 목표 지점 = 동 (12 + 3.5 × N) m · 북 0** → PX4 좌표 (북 0, 동 12 + 3.5 × N). 0번 칸 = 11 · 12주차 목표 자리 (동 12 m)
- 칸 = 폭 3.5 m × 깊이 6 m (주차선 동 10.25 · 13.75 · 17.25 · 20.75 · 24.25 · 27.75). 마커 그림은 칸 번호와 같은 ID — 24강 미션 노드의 `target_slot` 과 마커 인식의 `target_marker_id` 를 같은 번호로
- 드론은 원점에서 동쪽을 봄 → 기둥에서 왼쪽(북쪽)으로 돌아간 뒤 0번 칸 위에 도착. 북쪽에 건물이 있어 스캔의 왼쪽 섹터는 '자유 공간' 대신 건물까지의 거리(약 24 m — 계산)로 찍힘 (회피 방향은 11주차와 같음)
- 고도 2 m 의 아래 카메라 화면(땅 약 5.0 × 3.7 m — 계산)에는 한 번에 한 칸의 마커만 들어옴
- 마커 판 · 주차선 · 시작 표시는 그림만(`<visual>`) — 밟아도 턱이 없음. 그림자는 두 곳 모두 끔 (그림자 경계가 마커 위를 지나면 인식이 끊김)
- 원점 스폰에서는 `w06_land` 를 쓰지 않습니다 — 고도 4 m 직선 이동이 높이 4 m 기둥을 스침

## 14주차 번호 칸 월드 (parking_number_world.sdf)

25강(통합 미션 프로젝트 시연)용. 13주차 주차장 월드 `parking_world` 에 칸 번호 판 5개(흰 숫자 0~4)를 더한 월드입니다 — 번호 판 말고는 `parking_world` 와 같습니다 (Fuel 불필요).

```bash
# 한 번만: 월드 1개 + 번호 그림 폴더를 PX4 로 복사 (13주차에 models/ 를 복사한 VM 기준)
cd ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/worlds
cp parking_number_world.sdf ~/PX4-Autopilot/Tools/simulation/gz/worlds/
cp -r models/sjcu_slot_numbers ~/PX4-Autopilot/Tools/simulation/gz/models/
ls ~/PX4-Autopilot/Tools/simulation/gz/models | grep -E "sjcu_slot_numbers|sjcu_parking_markers|x500_lidar_2d_cam_down"     # 세 줄
# 실행 (13주차 명령에서 월드 이름만 다름 — 남은 Gazebo 서버를 먼저 정리)
pkill -9 -f "gz sim"
cd ~/PX4-Autopilot && PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500_lidar_2d_cam_down PX4_GZ_WORLD=parking_number_world ./build/px4_sitl_default/bin/px4
# N번 칸으로 보내기 — 터미널 4 · 5 에 같은 번호 (예: 2번 칸)
ros2 run drone_ros2_advanced w05_aruco --ros-args -p target_marker_id:=2
ros2 run drone_ros2_advanced w13_mission --ros-args -p fly:=true -p target_slot:=2
```

- `ls … | grep` 이 세 줄이 아니면(13주차 복사를 하지 않은 VM): `cp -r models/* ~/PX4-Autopilot/Tools/simulation/gz/models/` 로 모델 폴더 전부 (마커 그림 · 합성 기체도 필요)

| 모델 | `<pose>` (ENU) | 크기 (x·y) | 범위 |
|------|------|------|------|
| `parking_number_0` ~ `_4` (번호 판) | 12.0 + 3.5 × N, 1.65, 0.012 | 2.0 × 1.5 m 평면 (그림만 — 충돌 없음, 모서리 둥금) | N번 칸 = 동 (11.0~13.0) + 3.5 × N, 북 0.9~2.4 (마커와 노란 앞줄 사이) |

- 칸 중앙 = 마커 중심 = 목표 지점: 동 12.0 · 15.5 · 19.0 · 22.5 · 26.0 m, 북 0 (13주차와 같음)
- 숫자의 위쪽이 동쪽을 향합니다 → 서쪽에서 날아오는 드론의 아래 카메라 화면(위 = 기체 앞 = 동)과 드론 뒤에서 본 Gazebo 화면에서 바로 읽힘. Gazebo 평면은 그림의 위쪽을 북(+y), 오른쪽을 동(+x)에 붙이므로 그림 파일 안에서는 숫자가 시계 방향으로 90° 누워 있습니다
- 번호 판은 바닥 그림이라 2D LiDAR 스캔(땅 위 약 2.3 m)에 잡히지 않습니다. 모서리가 둥근 까닭: 네모난 어두운 판은 마커 인식(ArUco)이 마커 후보 사각형으로 읽습니다 — 둥근 판은 후보가 되지 않음
- 터미널 3 `w13_bridge` 의 시작 줄 앞에 `[안내] 월드 = parking_number_world — 24강 미션 월드는 parking_world …` 한 줄이 나옵니다 — 이 월드에서는 정상 (연결은 그대로)

## 자주 나는 오류

| 증상 | 원인 | 해결 |
|------|------|------|
| `Timed out waiting for Gazebo world` | 파일명과 `<world name>` 불일치 | 두 이름을 같게 |
| Gazebo는 뜨는데 드론이 없음 | 위와 같은 원인 | 위와 같음 |
| 시동 거부 / `ekf2 missing data` | `<world>` 직계에 plugin을 일부만 넣어 server.config가 취소됨 | 월드에서 plugin을 모두 빼기 |
| 모델이 보이는데 통과함 | `<collision>` 없음 | `<visual>` 과 같은 geometry로 `<collision>` 추가 |
| 모델이 땅에 반쯤 묻힘 | `<pose>` 의 z는 **모델 원점** 높이 | 원점이 도형 중앙이면 높이 h 박스는 z = h/2 |
| 모델이 안 보임 | `model://` 이름이 PX4 models 폴더에 없음 | `ls ~/PX4-Autopilot/Tools/simulation/gz/models` 로 확인 |

## 참고

- 월드를 바꿔 실행할 때는 **기존 PX4·Gazebo를 먼저 종료**하세요. Gazebo 서버가 떠 있으면 `PX4_GZ_WORLD` 가 무시되고 이전 월드가 재사용됩니다 (로그에 `gazebo already running world:`). Gazebo 창을 닫아도 서버는 남을 수 있으니 `pkill -9 -f "gz sim"` 으로 끄고, `pgrep -af "gz sim"` 이 빈 줄인지 확인한 뒤 다시 실행하세요
- `Tools/simulation/gz` 는 git 서브모듈이라 여기에 파일을 넣으면 PX4 저장소가 변경된 것으로 표시됩니다. 실습에는 문제없습니다.
- PX4 v1.16 기본 제공 월드: `aruco`, `baylands`, `default`, `forest`, `frictionless`, `lawn`, `moving_platform`, `rover`, `walls`, `windy`
- 장애물이 더 필요하면 `walls` 월드를 참고하세요 (박스 4개가 인라인 `<model>` 로 정의돼 있음).
