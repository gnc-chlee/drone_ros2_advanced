#!/usr/bin/env python3
# ==============================================================================
# File    : dnn_demo.py  (8주차 13강 - OpenCV DNN 데모)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-10-08
# Version : 1.1.0
#
# Description:
#   사진 · 영상 · 웹캠에서 OpenCV DNN(MobileNet-SSD)으로 물체를 찾아
#   박스 + 이름 + 신뢰도 + 처리 시간(ms)을 화면에 그립니다.
#   ROS 통신은 쓰지 않는 OpenCV 프로그램입니다 (실행만 ros2 run 으로, python3 로 직접 실행해도 됨).
#
#   DNN 4단계 (13강):
#     ① 읽기   : 모델 파일 2개(설계도 + 학습된 숫자)를 처음 한 번 읽는다      → load_model()
#     ② 전처리 : 사진을 300×300 으로 줄이고 밝기를 -1~1 로 맞춘다 (blob)     → detect()
#     ③ 추론   : net.forward() → 최대 100줄짜리 결과 표                     → detect()
#     ④ 해석   : 표를 한 줄씩 읽어 신뢰도가 기준값 이상인 것만 그린다       → draw()
#
#   결과 표 한 줄 = [사진 번호, 종류 번호, 신뢰도, x1, y1, x2, y2]
#     종류 번호: VOC 20종 (0 = 배경, 15 = 사람)   좌표: 화면 크기에 대한 비율(0~1)
#
#   실행 방법:
#     ros2 run drone_ros2_advanced w08_dnn_demo                          # samples/ 의 예시 사진
#     ros2 run drone_ros2_advanced w08_dnn_demo ~/Pictures/my_photo.jpg  # 내 사진 (폴더·영상 파일도 됨)
#     ros2 run drone_ros2_advanced w08_dnn_demo cam                      # 웹캠 0번 (1번이면 cam1)
#
#   조작 (그림 창을 클릭한 뒤):
#     슬라이더  기준값(%) — 30 이면 0.3. 0.25 아래는 모델이 이미 버려서 25 부터
#     n / 스페이스  다음 사진      p  이전 사진
#     t  지금 결과 표를 터미널에 출력      s  화면 저장(png)      q / ESC  종료
#
#   13강 실습 3) '두 줄 켜기: 사람만 보기' → draw() 안의 [실습] 표시
#     아래 두 줄 맨 앞의 # 한 글자씩만 지운다 (VS Code: 두 줄 선택 → Ctrl+/)
#     고친 파일: ~/ros2_ws/src/drone_ros2_advanced/drone_ros2_advanced/drone_ros2_advanced/week08/dnn_demo.py
#     고친 뒤 colcon build → source install/setup.bash 를 다시 해야 반영됩니다
#
#   모델: models/ (chuanqi305/MobileNet-SSD, MIT) — models/README.md 참고
#   OpenCV 4.x 필요 (5.0 에는 readNetFromCaffe 가 없다):
#     pip install 'opencv-python==4.11.0.86' 'numpy<2'
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import argparse
import glob
import os
import sys
import time

import cv2
import numpy as np


# ================================================================
# 파라미터
# ================================================================
HERE       = os.path.dirname(os.path.abspath(__file__))
MODEL_DIR  = os.path.join(HERE, 'models')
SAMPLE_DIR = os.path.join(HERE, 'samples')
PROTOTXT   = 'deploy.prototxt'                   # 설계도 (44 KB)
CAFFEMODEL = 'mobilenet_iter_73000.caffemodel'   # 학습된 숫자 (23 MB)

# VOC 20종 + 배경 — 번호표는 모델마다 다르다 (VOC 모델 15 · TF-COCO 모델 1 · YOLO 0)
CLASSES = ['background', 'aeroplane', 'bicycle', 'bird', 'boat', 'bottle', 'bus', 'car', 'cat',
           'chair', 'cow', 'diningtable', 'dog', 'horse', 'motorbike', 'person', 'pottedplant',
           'sheep', 'sofa', 'train', 'tvmonitor']
PERSON_ID = 15

CONF_DEFAULT = 0.5     # 처음 기준값 (슬라이더로 바꿈)
CONF_MIN     = 0.25    # 모델 안에서 0.25 미만은 이미 버린다 → 슬라이더 하한

IMAGE_EXT = ('.jpg', '.jpeg', '.png', '.bmp', '.webp', '.tif', '.tiff')
WINDOW    = 'DNN demo (q: quit)'
TRACKBAR  = 'threshold %'
VIEW_MAX_W, VIEW_MAX_H, VIEW_MIN_W = 1280, 720, 640   # 화면에 보일 크기 (큰 사진은 줄이고 작은 사진은 키움)

COLOR_PERSON = (57, 78, 224)     # BGR — 사람은 붉은 주황 (슬라이드 강조색)
COLOR_OTHER  = (216, 111, 58)    # BGR — 그 밖의 물체는 파랑
COLOR_TEXT   = (255, 255, 255)
FONT         = cv2.FONT_HERSHEY_SIMPLEX

FRAME_W, FRAME_H, FRAME_FPS = 640, 480, 15   # 웹캠: VM USB 패스스루를 고려한 안전한 값


# ================================================================
# ① 읽기 — 모델 파일 2개를 처음 한 번 읽는다
# ================================================================
def load_model(model_dir):
    if not hasattr(cv2.dnn, 'readNetFromCaffe'):
        print(f'[오류] 지금 OpenCV({cv2.__version__})에는 readNetFromCaffe 가 없습니다 (OpenCV 5 에서 삭제됨).')
        print("       4.x 로 되돌리세요:  pip install 'opencv-python==4.11.0.86' 'numpy<2'")
        sys.exit(1)
    proto = os.path.join(model_dir, PROTOTXT)
    model = os.path.join(model_dir, CAFFEMODEL)
    for p in (proto, model):
        if not os.path.isfile(p):
            print(f'[오류] 모델 파일이 없습니다: {p}')
            print('       코드 최신화(git) 뒤 colcon build → source install/setup.bash 를 다시 하세요.')
            sys.exit(1)
    print(f'모델 읽는 중: {PROTOTXT} + {CAFFEMODEL}')
    return cv2.dnn.readNetFromCaffe(proto, model)


# ================================================================
# ② 전처리 + ③ 추론
# ================================================================
def detect(net, frame):
    # 모델은 컬러(3채널) 사진만 받는다 → 흑백·투명(알파) 사진이면 컬러로 바꾼다
    if frame.ndim == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    elif frame.shape[2] == 4:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    # ② 300×300 으로 줄이고, (값 - 127.5) × 0.007843 → 밝기를 -1 ~ 1 로 맞춘다 (0.007843 = 1/127.5)
    #    mean 은 반드시 숫자 3개! (127.5 하나만 쓰면 파란색 채널에서만 빠진다)
    blob = cv2.dnn.blobFromImage(frame, 0.007843, (300, 300), (127.5, 127.5, 127.5), swapRB=False)
    # ③ 추론 — 결과는 늘 (1, 1, 100, 7) 표. 찾은 만큼만 채우고 나머지 줄은 0
    net.setInput(blob)
    t0 = time.perf_counter()
    out = net.forward()
    ms = (time.perf_counter() - t0) * 1000.0
    return out, ms


# ================================================================
# ④ 해석 — 표를 한 줄씩 읽어 기준값 이상만 그린다
# ================================================================
def draw(frame, out, conf_th, ms):
    view = frame.copy()
    h, w = view.shape[:2]
    found = people = 0

    # out[0, 0] = 100줄 × 7칸 표.  _ 는 사진 번호 (한 장씩 넣으니 늘 0 이라 안 씀)
    for _, cls, conf, x1, y1, x2, y2 in out[0, 0]:
        cls = int(cls)
        # ── [실습] 사람만 보기: 아래 두 줄 맨 앞의 # 한 글자씩만 지운다 ──
        #if cls != PERSON_ID:
        #    continue
        if conf < conf_th:
            continue                            # 기준값보다 확신이 낮으면 버린다
        found += 1
        if cls == PERSON_ID:
            people += 1

        # 좌표는 0~1 비율 → 0~1 밖으로 살짝 나간 값을 잘라 낸 뒤, 화면 크기를 곱해 픽셀로
        x1, y1, x2, y2 = [min(max(float(v), 0.0), 1.0) for v in (x1, y1, x2, y2)]
        bx1, by1 = int(x1 * (w - 1)), int(y1 * (h - 1))
        bx2, by2 = int(x2 * (w - 1)), int(y2 * (h - 1))
        color = COLOR_PERSON if cls == PERSON_ID else COLOR_OTHER
        cv2.rectangle(view, (bx1, by1), (bx2, by2), color, 2)

        # 이름표: 박스 왼쪽 위 (화면 밖으로 나가지 않게 안쪽으로 밀어 넣음)
        name = CLASSES[cls] if 0 <= cls < len(CLASSES) else str(cls)
        label = f'{name} {conf:.2f}'
        (tw, th), base = cv2.getTextSize(label, FONT, 0.6, 1)
        lx = max(0, min(bx1, w - tw - 6))
        ty = max(by1, th + base + 4)
        cv2.rectangle(view, (lx, ty - th - base - 4), (lx + tw + 6, ty), color, -1)
        cv2.putText(view, label, (lx + 3, ty - base - 2), FONT, 0.6, COLOR_TEXT, 1, cv2.LINE_AA)

    # 상태 표시줄은 왼쪽 아래 (위쪽에 두면 화면 위 끝 박스 이름표를 가린다)
    hud = f'threshold {conf_th:.2f} | {ms:.0f} ms | person {people} / all {found}'
    (tw, th), base = cv2.getTextSize(hud, FONT, 0.7, 2)
    cv2.rectangle(view, (0, h - th - base - 14), (tw + 16, h), (0, 0, 0), -1)
    cv2.putText(view, hud, (8, h - base - 7), FONT, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    return view


def print_table(out, title, conf_th):
    """모델이 낸 원본 결과 표에서 채워진 줄만 터미널에 출력 (화면에는 이 중 기준값 이상만 그림)"""
    rows = [r for r in out[0, 0] if r[2] > 0]
    print(f'\n[결과 표] {title} — 모델 원본 표 100줄 중 채워진 {len(rows)}줄  (✓ = 기준값 {conf_th:.2f} 이상 → 화면에 그림)')
    print('  사진  종류 번호        신뢰도    x1    y1    x2    y2')
    for img, cls, conf, x1, y1, x2, y2 in rows:
        cls = int(cls)
        kind = f'{cls} ({CLASSES[cls] if 0 <= cls < len(CLASSES) else "?"})'
        mark = '✓' if conf >= conf_th else ''
        print(f'  {int(img):>4}  {kind:<16} {conf:>5.2f}  {x1:>5.2f} {y1:>5.2f} {x2:>5.2f} {y2:>5.2f}  {mark}')


# ================================================================
# 화면 · 입력 도우미
# ================================================================
def fit_view(frame):
    """화면에 보일 크기로 맞춘다: 가로 1280·세로 720 안에, 너무 작으면 가로 640 까지 키움.
       (검출 좌표는 0~1 비율이라 크기를 바꿔도 그대로 맞는다)"""
    h, w = frame.shape[:2]
    s = min(VIEW_MAX_W / w, VIEW_MAX_H / h)
    if w * s < VIEW_MIN_W:
        s = VIEW_MIN_W / w
    if abs(s - 1.0) < 1e-3:
        return frame
    return cv2.resize(frame, None, fx=s, fy=s, interpolation=cv2.INTER_AREA if s < 1 else cv2.INTER_LINEAR)


def image_list(path):
    if os.path.isdir(path):
        return sorted(f for f in glob.glob(os.path.join(path, '*')) if f.lower().endswith(IMAGE_EXT))
    return [path]


def open_camera(index):
    backend = cv2.CAP_V4L2 if sys.platform.startswith('linux') else cv2.CAP_ANY
    cap = cv2.VideoCapture(index, backend)
    if not cap.isOpened():
        print(f'[오류] 웹캠 {index}번을 열 수 없습니다 (VM 메뉴에서 웹캠을 연결했는지, ls /dev/video* 확인).')
        sys.exit(1)
    # ── [복붙 영역] VM 웹캠 대응: 압축 포맷 + 낮은 해상도 (5주차 contour_demo 와 같음) ──
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
    cap.set(cv2.CAP_PROP_FPS, FRAME_FPS)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)          # 묵은 프레임 쌓이지 않게
    print(f'웹캠 설정: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} '
          f'@ {cap.get(cv2.CAP_PROP_FPS):.0f}fps')
    return cap


def setup_window():
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    # 슬라이더: 0~95 중 25 부터. 움직일 때 따로 할 일은 없다 (값은 루프에서 그때그때 읽음)
    cv2.createTrackbar(TRACKBAR, WINDOW, int(CONF_DEFAULT * 100), 95, lambda v: None)
    cv2.setTrackbarMin(TRACKBAR, WINDOW, int(CONF_MIN * 100))


def current_threshold():
    return max(CONF_MIN, cv2.getTrackbarPos(TRACKBAR, WINDOW) / 100.0)


def window_closed():
    """창의 X 를 눌렀는지. Qt 빌드(pip opencv-python)는 VISIBLE, GTK 빌드는 AUTOSIZE 로 판단"""
    try:
        v = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE)
        if v < 0:                                # 이 빌드는 VISIBLE 을 모름
            return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_AUTOSIZE) < 0
        return v < 1
    except cv2.error:
        return True


def save_view(view):
    name = os.path.abspath(time.strftime('dnn_result_%H%M%S.png'))
    if cv2.imwrite(name, view):
        print(f'저장: {name}')
    else:
        print(f'[오류] 저장 실패: {name}')


def read_key(delay_ms):
    # & 0xFF: 키 코드의 끝 8비트만 본다 (리눅스에서도 같은 값이 나오게)
    return cv2.waitKey(delay_ms) & 0xFF


# ================================================================
# 실행 루프 — 사진 목록 / 영상·웹캠
# ================================================================
def run_images(net, files):
    setup_window()
    idx = 0
    cache = {}                         # 사진마다 추론은 한 번만 (슬라이더를 움직이면 그리기만 다시)
    sized = False
    while True:
        path = files[idx]
        if path not in cache:
            frame = cv2.imread(path)   # 늘 컬러 3채널로 읽힌다
            if frame is None:
                print(f'[건너뜀] 사진을 읽을 수 없습니다: {path}')
                files.pop(idx)
                if not files:
                    return
                idx %= len(files)
                continue
            frame = fit_view(frame)    # 화면 크기로 줄여서 보관 (큰 사진 여러 장에도 메모리 걱정 없게)
            out, ms = detect(net, frame)
            cache[path] = (frame, out, ms)
            print(f'\n({idx + 1}/{len(files)}) {os.path.basename(path)}  추론 {ms:.0f} ms')
            print_table(out, os.path.basename(path), current_threshold())
        frame, out, ms = cache[path]
        view = draw(frame, out, current_threshold(), ms)
        cv2.imshow(WINDOW, view)
        if not sized:                  # 첫 화면에서 창 크기를 사진에 맞춤
            cv2.resizeWindow(WINDOW, view.shape[1], view.shape[0])
            sized = True
        key = read_key(30)
        if key in (ord('q'), 27) or window_closed():
            return
        if key in (ord('n'), ord(' ')):
            idx = (idx + 1) % len(files)
        elif key == ord('p'):
            idx = (idx - 1) % len(files)
        elif key == ord('t'):
            print_table(out, os.path.basename(path), current_threshold())
        elif key == ord('s'):
            save_view(view)


def run_stream(net, cap, title, is_file):
    setup_window()
    fail, view, out, sized = 0, None, None, False
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                if is_file and view is not None:          # 영상이 끝남 → 마지막 장면을 띄운 채 기다림
                    print('영상 끝 — 그림 창에서 아무 키나 누르면 종료')
                    while read_key(100) == 255 and not window_closed():
                        pass
                    return
                fail += 1
                if read_key(1) in (ord('q'), 27) or fail >= 30:
                    print('프레임이 오지 않습니다 (웹캠 연결·USB 대역폭 확인).')
                    return
                continue
            fail = 0
            frame = fit_view(frame)
            out, ms = detect(net, frame)
            view = draw(frame, out, current_threshold(), ms)
            cv2.imshow(WINDOW, view)
            if not sized:
                cv2.resizeWindow(WINDOW, view.shape[1], view.shape[0])
                sized = True
            key = read_key(1)
            if key in (ord('q'), 27) or window_closed():
                return
            if key == ord('t'):
                print_table(out, title, current_threshold())
            elif key == ord('s'):
                save_view(view)
    finally:
        cap.release()


def main(args=None):
    parser = argparse.ArgumentParser(description='OpenCV DNN(MobileNet-SSD) 데모 — 사진·영상·웹캠에서 물체 찾기')
    parser.add_argument('source', nargs='?', default=None,
                        help='사진·폴더·영상 파일 경로, 또는 cam / cam1 (없으면 samples/ 의 예시 사진)')
    parser.add_argument('--model-dir', default=MODEL_DIR, help='모델 파일 폴더 (기본: 이 파일 옆 models/)')
    argv = sys.argv[1:] if args is None else list(args)
    if '--ros-args' in argv:                    # 혹시 --ros-args 를 붙여도 그 뒤는 무시
        argv = argv[:argv.index('--ros-args')]
    opts = parser.parse_args(argv)

    # 입력 먼저 확인 (모델 읽기는 시간이 걸리므로 그 뒤에)
    src, files, cam = opts.source, None, None
    if src is None:
        files = image_list(SAMPLE_DIR)
        if not files:
            print(f'[안내] 예시 사진 폴더가 비어 있습니다: {SAMPLE_DIR}')
            print('       사진 경로를 주거나 웹캠을 쓰세요:')
            print('         ros2 run drone_ros2_advanced w08_dnn_demo ~/Pictures/my_photo.jpg')
            print('         ros2 run drone_ros2_advanced w08_dnn_demo cam')
            return
    elif src.lower().startswith('cam') and (src[3:] == '' or src[3:].isdigit()):
        cam = int(src[3:] or 0)
    else:
        src = os.path.expanduser(src)
        if os.path.isdir(src) or src.lower().endswith(IMAGE_EXT):
            files = image_list(src)
            if not files or not all(os.path.isfile(f) for f in files):
                print(f'[오류] 사진을 찾을 수 없습니다: {src}')
                return
        elif not os.path.isfile(src):
            print(f'[오류] 파일을 찾을 수 없습니다: {src}')
            return

    net = load_model(opts.model_dir)                    # ① 읽기 (처음 한 번)
    print('조작: 슬라이더=기준값(%)  n/스페이스=다음  p=이전  t=결과 표  s=저장  q=종료')
    try:
        if files:
            run_images(net, files)
        elif cam is not None:
            run_stream(net, open_camera(cam), f'웹캠 {cam}', is_file=False)
        else:
            cap = cv2.VideoCapture(src)
            if not cap.isOpened():
                print(f'[오류] 영상 파일을 열 수 없습니다: {src}')
                return
            run_stream(net, cap, os.path.basename(src), is_file=True)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
