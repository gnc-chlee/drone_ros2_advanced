# 예시 사진 · 영상 (8주차 13·14강)

## 사진 — 13강 `w08_dnn_demo`

`w08_dnn_demo` 를 인자 없이 실행하면 이 폴더의 사진(`.jpg .jpeg .png .bmp .webp .tif`)을 차례로 보여 줍니다.

```bash
ros2 run drone_ros2_advanced w08_dnn_demo            # 이 폴더의 사진
ros2 run drone_ros2_advanced w08_dnn_demo ~/Pictures/my_photo.jpg   # 내 사진
```

- 사람과 함께 의자·모니터·병 같은 물체가 나오는 사진이 좋습니다 (VOC 20종 — `../models/README.md`).

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
