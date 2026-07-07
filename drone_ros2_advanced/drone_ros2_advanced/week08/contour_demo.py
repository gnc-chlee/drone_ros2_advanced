#!/usr/bin/env python3
# ==============================================================================
# File    : contour_demo.py  (8주차)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-07
# Version : 1.0.0
#
# Description:
#   기초 과목 OpenCV contour 복습 - 웹캠으로 물체 윤곽선 찾기
#   (ROS2 없이 실행되는 순수 OpenCV 스크립트)
#
#   처리 과정 (모든 비전 파이프라인의 기본!):
#     원본 → 그레이스케일 → 블러 → 이진화 → findContours → 가장 큰 것 선택
#
#   배우는 개념:
#     - cv2.findContours: 흰 영역의 "테두리"들을 찾아줌
#     - cv2.contourArea:  면적으로 가장 큰 물체 고르기
#     - cv2.boundingRect: 물체를 감싸는 사각형 → 중심점 계산
#     - "물체 중심점"이 나중에 드론 제어의 입력이 됩니다! (10주차~)
#
#   실행 방법:
#     ros2 run drone_ros2_advanced w08_contour
#     (웹캠 필요. 어두운 배경에 밝은 물체를 비춰보세요)
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import cv2

CAMERA_INDEX = 0     # 웹캠 번호 (안 되면 1, 2로 바꿔보세요)
THRESHOLD    = 127   # 이진화 기준 밝기 (0~255) — 바꿔가며 실험!
MIN_AREA     = 500   # 이 면적(픽셀)보다 작은 것은 노이즈로 무시


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f'웹캠 {CAMERA_INDEX}번을 열 수 없습니다!')
        return

    print('contour 데모 시작! (q: 종료)')
    print(f'이진화 기준: {THRESHOLD} — 코드에서 바꿔가며 실험해보세요')

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # ── 1. 그레이스케일 변환 ─────────────────────────────────
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ── 2. 블러 (노이즈 제거) ────────────────────────────────
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # ── 3. 이진화 (THRESHOLD보다 밝으면 흰색) ────────────────
        _, binary = cv2.threshold(
            blurred, THRESHOLD, 255, cv2.THRESH_BINARY)

        # ── 4. contour 찾기 ──────────────────────────────────────
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # ── 5. 가장 큰 contour 선택 + 중심점 계산 ────────────────
        if contours:
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > MIN_AREA:
                x, y, w, h = cv2.boundingRect(largest)
                cx = x + w // 2   # 중심 x
                cy = y + h // 2   # 중심 y

                cv2.drawContours(frame, [largest], -1, (0, 255, 0), 2)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(frame,
                            f'center=({cx},{cy}) area={int(area)}',
                            (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 0), 2)

        # ── 6. 화면 표시 (원본 + 이진화 결과) ────────────────────
        cv2.imshow('Contour Demo (q: quit)', frame)
        cv2.imshow('Binary', binary)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
