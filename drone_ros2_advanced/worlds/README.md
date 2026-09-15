# worlds — 4주차 커스텀 Gazebo World

`my_custom_world.sdf` 는 4주차 실습의 **완성 예시**입니다. PX4 내장 `aruco.sdf` 를 복사해
마커를 원점 밖으로 옮기고 장애물 세 개(박스·벽·원기둥)를 놓았습니다.

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

| 물체 | Gazebo `<pose>` x y z | PX4 (북, 동) |
|------|----------------------|--------------|
| `marker_a` (ArUco 마커) | 5 3 0.001 | (3, 5) |
| `box_west` (박스 2×2×3) | -6 0 1.5 | (0, −6) |
| `wall_north` (벽 8×0.5×2) | 0 9 1 | (9, 0) |
| `pillar_east` (원기둥 r0.6 h4) | 8 -4 2 | (−4, 8) |

`config/my_custom_world_mission.yaml` 이 이 좌표들을 NED로 적어둔 미션 파일입니다.
미션 노드는 waypoint에 도달하면 멈추지 않고 곧바로 다음 목표로 가며, 목록을 다 돌면 **그 자리에서 착륙**합니다.
그래서 마커를 마지막에 두어 **마커 위에 내려앉는 것**으로 좌표 변환을 확인합니다.
※ 위 대응표는 월드 원점과 PX4 로컬 원점이 같다는 전제(기본 스폰 위치)에서 성립합니다.

```bash
# 박스 → 원기둥 → 마커 순서로 비행한 뒤 마커 위에 착륙 (고도 5m)
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

**쓰지 말 것**: `Standing person`, `Walking person` — 모델 안의 메쉬 경로가 Fuel 등록명과 달라서
`<include>` 로는 형상이 로드되지 않습니다. 사람 모델이 꼭 필요하면 `Casual female` 을 쓰되,
4096×4096 텍스처 때문에 VM에서는 느립니다(8~9주차에 재검토).

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

- 월드를 바꿔 실행할 때는 **기존 PX4·Gazebo를 먼저 종료**하세요. Gazebo 서버가 떠 있으면 `PX4_GZ_WORLD` 가 무시되고 이전 월드가 재사용됩니다 (로그에 `gazebo already running world:`)
- `Tools/simulation/gz` 는 git 서브모듈이라 여기에 파일을 넣으면 PX4 저장소가 변경된 것으로 표시됩니다. 실습에는 문제없습니다.
- PX4 v1.16 기본 제공 월드: `aruco`, `baylands`, `default`, `forest`, `frictionless`, `lawn`, `moving_platform`, `rover`, `walls`, `windy`
- 장애물이 더 필요하면 `walls` 월드를 참고하세요 (박스 4개가 인라인 `<model>` 로 정의돼 있음).
