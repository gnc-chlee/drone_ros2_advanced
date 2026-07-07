#!/usr/bin/env python3
# ==============================================================================
# File    : haar_vs_yolo.py  (14주차)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   Haar Cascade vs YOLOv8n 비교 데모 (웹캠, ROS2 없이 실행)
#
#   같은 화면을 두 방식으로 동시에 처리해서 나란히 보여줍니다:
#     왼쪽  : Haar Cascade 얼굴 인식 (10주차)
#     오른쪽: YOLOv8n 사람 감지 (오늘)
#   각각의 처리 시간(ms)도 표시 → 속도와 정확도를 직접 비교!
#
#   관찰 포인트:
#     - 고개를 돌려보세요 → Haar는 놓치고 YOLO는 잡습니다
#     - 뒤로 물러나보세요 → 몸 전체가 보이면 YOLO가 사람으로 인식
#     - 처리 시간: Haar가 훨씬 빠름 (그래서 아직도 쓰입니다)
#
#   실행 방법:
#     ros2 run drone_ros2_advanced w14_haar_vs_yolo
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import time
import cv2
import numpy as np

# ================================================================
# [복붙 영역] ultralytics(YOLO) 불러오기 — 환경에 맞게 수정
# ================================================================
import sys
sys.path.insert(0, '/home/sentiary/miniconda3/envs/dl_env/lib/python3.10/site-packages')
from ultralytics import YOLO


CAMERA_INDEX = 0


def main():
    # 두 모델 준비
    face_cascade = cv2.CascadeClassifier(
        cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    yolo = YOLO('yolov8n.pt')

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f'웹캠 {CAMERA_INDEX}번을 열 수 없습니다!')
        return

    print('Haar vs YOLO 비교 시작! (q: 종료)')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        haar_frame = frame.copy()
        yolo_frame = frame.copy()

        # ── 왼쪽: Haar Cascade 얼굴 인식 ─────────────────────────
        t0 = time.time()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(
            gray, scaleFactor=1.1, minNeighbors=5, minSize=(60, 60))
        haar_ms = (time.time() - t0) * 1000

        for (x, y, w, h) in faces:
            cv2.rectangle(haar_frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
            cv2.putText(haar_frame, 'face', (x, y-8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.putText(haar_frame, f'Haar: {haar_ms:.0f}ms  ({len(faces)} face)',
                    (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)

        # ── 오른쪽: YOLOv8n 사람 감지 ────────────────────────────
        t0 = time.time()
        results = yolo(frame, classes=[0], verbose=False)
        yolo_ms = (time.time() - t0) * 1000

        n_person = 0
        if results[0].boxes:
            for box in results[0].boxes:
                bx, by, bw, bh = box.xywh[0].tolist()
                conf = box.conf[0].item()
                left, top = int(bx - bw/2), int(by - bh/2)
                right, bottom = int(bx + bw/2), int(by + bh/2)
                cv2.rectangle(yolo_frame, (left, top), (right, bottom),
                              (0, 200, 0), 2)
                cv2.putText(yolo_frame, f'person {conf:.2f}',
                            (left, max(top-8, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
                n_person += 1

        cv2.putText(yolo_frame, f'YOLO: {yolo_ms:.0f}ms  ({n_person} person)',
                    (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 0), 2)

        # ── 나란히 붙여서 표시 ───────────────────────────────────
        combined = np.hstack([haar_frame, yolo_frame])
        cv2.imshow('Haar (left) vs YOLO (right) - q: quit', combined)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
