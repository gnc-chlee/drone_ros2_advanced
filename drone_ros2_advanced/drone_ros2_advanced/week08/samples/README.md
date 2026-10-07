# 예시 사진 (8주차 13강 데모)

`w08_dnn_demo` 를 인자 없이 실행하면 이 폴더의 사진(`.jpg .jpeg .png .bmp`)을 차례로 보여 줍니다.

```bash
ros2 run drone_ros2_advanced w08_dnn_demo            # 이 폴더의 사진
ros2 run drone_ros2_advanced w08_dnn_demo ~/Pictures/my_photo.jpg   # 내 사진
```

- 사람과 함께 의자·모니터·병 같은 물체가 나오는 사진이 좋습니다 (VOC 20종 — `../models/README.md`).
- **저작권이 깨끗한 사진만** 넣습니다 (직접 찍은 사진 권장). 사진을 넣은 뒤에는 `colcon build` 를 다시 해야 설치 경로에 복사됩니다.
