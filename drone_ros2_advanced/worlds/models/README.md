# worlds/models — 8주차 사람 모델

PX4 의 모델 폴더로 복사해서 씁니다 (`person_world.sdf` 가 `model://` 로 찾음).

```bash
cp -r ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/worlds/models/* \
      ~/PX4-Autopilot/Tools/simulation/gz/models/
ls ~/PX4-Autopilot/Tools/simulation/gz/models | grep sjcu     # sjcu_person_board, sjcu_walker
```

| 폴더 | 내용 | 출처 · 라이선스 |
|------|------|-----------------|
| `sjcu_walker/` | 걷는 사람 actor. `meshes/walk.dae` (2,277,322 B, SHA-1 `37c2be7a884a54f4d222ae4b0f55b76a21b61fe9`) | Gazebo Fuel **chapulina / Walking actor** v2 — **CC0 1.0** (퍼블릭 도메인 헌정, 출처 표기 의무 없음). <https://app.gazebosim.org/chapulina/fuel/models/Walking%20actor> |
| `sjcu_person_board/` | 사진 입간판 (Plan B — actor 를 검출기가 못 잡을 때). 얇은 판에 전신 사진을 입힘 | 판: 직접 작성(MIT). 사진: `materials/textures/person.png` 를 **교수 촬영 사진으로 바꿔 넣을 것** (지금은 자리표시 그림) |

- 사람 모델을 Fuel 에서 내려받지 않고 저장소에 넣은 이유: 수업 중 인터넷·Fuel 서버 상태와 무관하게 월드가 뜨게 하려고.
- `walk.dae` 는 텍스처 없이 부위별 색(피부·초록 상의·청바지)만 쓰는 메시라 VM 소프트웨어 렌더링에서도 가볍습니다.
- 04주 자료의 "쓰지 말 것: Walking person" 은 **다른 모델**(OpenRobotics Walking person)입니다. 이 폴더의 walker 와 혼동하지 마세요.
