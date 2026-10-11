# 예시 사진 · 영상 (8주차 13·14강)

## 사진 — 13강 `w08_dnn_demo`

`w08_dnn_demo` 를 인자 없이 실행하면 이 폴더의 사진(`.jpg .jpeg .png .bmp .webp .tif`)을 차례로 보여 줍니다.

```bash
ros2 run drone_ros2_advanced w08_dnn_demo            # 이 폴더의 사진
ros2 run drone_ros2_advanced w08_dnn_demo ~/Pictures/my_photo.jpg   # 내 사진
```

- 사람과 함께 의자·모니터·병 같은 물체가 나오는 사진이 좋습니다 (VOC 20종 — `../models/README.md`).

| 사진 | 무엇이 보이나 (`w08_dnn_demo` 기준) | 13강에서 |
|------|-----------------------------------|----------|
| `01_group_7people.jpg` | 나란히 선 7명 — 기준값 0.3 이면 7명, 0.5 면 5~6명, 0.8 이면 0명 | 기준값 바꿔 보기 (헛잡음 · 놓침) |
| `02_room_chair_cat.jpg` | 의자는 잡고 **고양이는 놓침** (사진 전체에서 고양이가 작음) | 작으면 놓친다 |
| `03_room_cat_closeup.jpg` | 같은 사진을 잘라 고양이를 크게 → cat 0.99. 옆 스크래처를 cat 으로 잡는 **헛잡음**도 함께 (기준값을 올리면 사라짐) | 놓침 → 잡힘, 헛잡음 |
| `04_desk_person_cat.jpg` | 사람 1.00, 책상 위 **고양이는 놓침** (모니터 화면은 흐리게 처리) | 여러 종류 · 놓침 |
| `05_desk_cat_closeup.jpg` | 같은 사진을 잘라 고양이를 크게 → cat 1.00 | 크게 보이면 잡힌다 |

- 출처: 교수 제공(이충현, 2024~2026). `01` 은 2024년 드론작전사령부 방문 기념사진으로 언론 기사에 공개된 사진, `02`~`05` 는 교수 촬영(사진 속 인물 동의). 수업 실습용으로만 씁니다.
- 저장소에 넣을 때 긴 변 1280 px 로 줄이고 다시 저장해 촬영 정보(EXIF·GPS)를 지웠습니다.

## 영상 — 14강 `w08_video_pub`

| 파일 | 쓰는 곳 | 내용 |
|------|---------|------|
| `person_walk.mp4` (교수 촬영 후 추가 예정 — 그 전에는 `w08_video_pub cam` 이나 내 영상 경로) | 14강 실습 1 (`w08_video_pub` 인자 없이) | 사람이 걷는 실사 영상 약 30초 — 가까이 앞모습 → 옆 → 뒤 → 멀어짐 → 멀리서 앞 → 화면 밖 → 돌아옴 |
| `gazebo_front.mp4` | (선택) Gazebo 가 너무 느린 PC (`w08_video_pub gazebo`) | person_world 정면 카메라 녹화본 — `w08_video_pub --record gazebo_front.mp4` 로 만들어(지금 폴더에 저장) 이 폴더로 옮기고 `colcon build` |

- 권장 형식: 640×480(4:3), 15 fps, mp4(H.264). 더 큰 영상은 `w08_video_pub` 가 가로 640 으로 줄여 보내지만, 파일이 작을수록 저장소가 가볍습니다.
  ```bash
  # 휴대폰 영상(1920x1080)을 4:3 로 자르고 640x480 · 15fps 로 줄이기 (ffmpeg)
  ffmpeg -i input.mp4 -vf "crop=ih*4/3:ih,scale=640:480,fps=15" -an -c:v libx264 -crf 28 person_walk.mp4
  ```

**저작권이 깨끗한 사진·영상만** 넣습니다 (직접 찍은 것 권장). 넣은 뒤에는 `colcon build` 를 다시 해야 설치 경로에 복사됩니다.
