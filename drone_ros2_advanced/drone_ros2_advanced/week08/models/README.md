# MobileNet-SSD 모델 파일 (8주차)

8주차 사람 인식에 쓰는 사전 학습 모델입니다. 두 파일이 **짝**이라 항상 함께 씁니다.

| 파일 | 크기 | 하는 일 | SHA-1 |
|---|---|---|---|
| `deploy.prototxt` | 44,667 B | 설계도 — 층을 어떻게 쌓았는지 (BatchNorm 포함판) | `50cf80235a8fcccc641bf9f8efc803edbf21c615` |
| `mobilenet_iter_73000.caffemodel` | 23,306,119 B | 학습된 숫자(가중치) | `19e3ec38842f3e68b02c07a1c24424a1e9db57e9` |

- 출처: https://github.com/chuanqi305/MobileNet-SSD (커밋 `97406996b1eee2d40eb0a00ae567cf41e23369f9`, OpenCV 공식 테스트가 쓰는 것과 같은 파일)
- 라이선스: MIT — 같은 폴더의 `LICENSE` (Copyright (c) 2018 chuanqi305)
- 학습 데이터: PASCAL VOC 20종 (0 = 배경, **15 = 사람**)
- 입력 300×300, 전처리 `blobFromImage(frame, 0.007843, (300, 300), (127.5, 127.5, 127.5), swapRB=False)` — mean 은 반드시 숫자 3개
- 출력 `(1, 1, 100, 7)` = `[사진 번호, 종류 번호, 신뢰도, x1, y1, x2, y2]` (좌표는 0~1 비율, 모델 안에서 신뢰도 0.25 미만은 이미 버림)

**주의**
- 인터넷에 도는 `MobileNetSSD_deploy.prototxt`(BN 병합판)와 섞으면 **오류 없이 엉터리 결과**가 나옵니다. 이 폴더의 두 파일만 쓰세요.
- OpenCV **5.0 에는 `readNetFromCaffe` 가 없습니다.** 4.x 가 필요합니다: `pip install 'opencv-python==4.11.0.86' 'numpy<2'`
- 다시 받기:
  ```bash
  C=97406996b1eee2d40eb0a00ae567cf41e23369f9
  wget https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/$C/deploy.prototxt
  wget https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/$C/mobilenet_iter_73000.caffemodel
  ```
