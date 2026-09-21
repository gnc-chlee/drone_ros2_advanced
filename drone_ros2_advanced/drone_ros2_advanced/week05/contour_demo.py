#!/usr/bin/env python3
# ==============================================================================
# File    : contour_demo.py  (5주차 1강)
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
#     - "물체 중심점"이 나중에 드론 제어의 입력이 됩니다! (6주차~)
#
#   실행 방법 (웹캠 전용 — PX4·브리지 불필요):
#     ros2 run drone_ros2_advanced w05_contour
#     (웹캠 필요. 어두운 배경에 밝은 물체를 비춰보세요)
#
#   웹캠 연결 (VMware):
#     VM 메뉴 ▸ Removable Devices ▸ (노트북 카메라) ▸ Connect 로 웹캠을 VM에 연결한 뒤
#     `ls /dev/video*` 로 확인 (/dev/video0 이 보이면 CAMERA_INDEX = 0)
#
#   VM에서 프레임이 뚝뚝 끊기고 `VIDEOIO(V4L2:/dev/video0): select() timeout` 이 뜨면:
#     VMware USB 패스스루 대역폭 부족. 이 코드는 압축(MJPG)+640x480 을 요청해 부담을 줄인다.
#     그래도 느리면 VM 설정 ▸ USB Controller ▸ USB compatibility 를 3.1(또는 2.0)로 바꿔 볼 것
#
#   밝은 방 함정:
#     배경이 밝으면 화면 전체가 가장 큰 contour로 잡힘
#     → THRESHOLD를 올리거나 어두운 배경 사용
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import cv2

CAMERA_INDEX = 0     # 웹캠 번호 (안 되면 1, 2로 바꿔보세요)
THRESHOLD    = 127   # 이진화 기준 밝기 (0~255) — 바꿔가며 실험!
                     # 배경이 밝으면 화면 전체가 가장 큰 contour로 잡힘 → THRESHOLD를 올리거나 어두운 배경 사용
MIN_AREA     = 500   # 이 면적(픽셀)보다 작은 것은 노이즈로 무시
FRAME_W, FRAME_H = 640, 480   # VM USB 패스스루를 고려한 안전한 해상도


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f'웹캠 {CAMERA_INDEX}번을 열 수 없습니다!')
        return

    # ── [복붙 영역] VM 웹캠 대응: 압축 포맷 + 낮은 해상도로 USB 부담 1/10 ──
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)          # 묵은 프레임 쌓이지 않게
    print(f'웹캠 설정: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}')

    print('contour 데모 시작! (q: 종료)')
    print(f'이진화 기준: {THRESHOLD} — 코드에서 바꿔가며 실험해보세요')

    fail = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            fail += 1
            if fail >= 30:               # 연속 30번 실패 = 카메라 죽음
                print('웹캠에서 프레임이 오지 않습니다 (USB 연결/대역폭 확인)')
                break
            continue
        fail = 0

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
