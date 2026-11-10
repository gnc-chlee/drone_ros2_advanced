#!/usr/bin/env python3
# ==============================================================================
# File    : state_view.py  (12주차 21·22강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-09
# Version : 1.1.0   (2026-11-10 단순화)
#
# Description:
#   상태도 그림 — 21강 시뮬레이터(w12_fsm_sim)와 22강 상태 뷰어(w12_state_viewer)가 같이 쓴다
#   화살표는 전이 조건표에서 그린다 → 표에서 줄을 삭제하면 그 화살표가 사라진다
#   화살표 옆 숫자 = 표의 줄 번호. 현재 상태 = 빨강, 지나온 상태·화살표 = 진하게, 방금 지난 화살표 = 빨강
#   표 밖 안전장치 전이(safety)는 화살표가 없다 → 최근 전이 줄에만 'safety' 로 보인다
#   창 안 글자는 영어·숫자만 (OpenCV 는 한글을 못 쓴다 — 8주차 HUD 와 같음. '—' 같은 기호도 '???' 로 그려짐)
#   ROS 없음 (numpy + OpenCV 만). 학생은 읽지 않아도 되는 파일
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import math

import cv2
import numpy as np

from .mission_table import AVOID, CRUISE, DONE, EVENTS, HOLD, IDLE, LAND, TAKEOFF, TRANSITIONS

WIDTH = 640

# ── 색 (BGR) — 슬라이드 그림과 같은 강조색 ─────────────────────────
WHITE  = (255, 255, 255)
INK    = (60, 60, 60)        # 글자 · 지나온 길
SOFT   = (140, 140, 140)     # 보조 글자
FAINT  = (205, 205, 205)     # 아직 안 지난 화살표 · 상자 테두리
PALE   = (236, 236, 236)     # 지나온 상태 상자
SHADE  = ((222, 222, 222), (200, 200, 200))   # 상태 띠 (번갈아)
ACCENT = (57, 78, 224)       # #e04e39 현재 상태 · 방금 지난 화살표
BLUE   = (216, 111, 58)      # #3a6fd8 앞 거리 곡선
FONT   = cv2.FONT_HERSHEY_SIMPLEX

# ── 상태도 배치 (상자 가운데 x, y — 상태도 영역 안 좌표) ─────────────
#    윗줄 = 정상 경로 IDLE → TAKEOFF → CRUISE → LAND → DONE, 아랫줄 = CRUISE 아래 AVOID · LAND 아래 HOLD
#    화살표 10개가 모두 직선 (새 상태를 더하면 여기에 자리를 추가 — 자리가 없는 상태의 화살표는 그리지 않음)
HEADER_H  = 28
DIAGRAM_H = 226
SIM_H     = 480      # 시뮬레이터 창 높이
VIEW_H    = 480      # 뷰어 창 높이
BOX_W, BOX_H = 92, 32
POS = {
    IDLE: (52, 60), TAKEOFF: (170, 60), CRUISE: (300, 60), LAND: (440, 60), DONE: (570, 60),
    AVOID: (300, 170), HOLD: (440, 170),
}

GROUP_NAMES = {'미션': 'mission', '페일세이프': 'failsafe'}   # 사건 구분 → 창에 쓰는 영어
LEGEND_X = 84        # 사건 안내에서 사건 이름이 시작하는 x (구분 이름 'mission' · 'failsafe' 다음)


# ================================================================
# 그리기 도우미
# ================================================================
def _pt(p):
    return int(round(p[0])), int(round(p[1]))


def _text(img, s, org, scale=0.45, color=INK, thick=1, anchor='left', bg=None):
    """글자 한 줄. anchor: left(왼쪽 아래 기준) · center(가운데) · right(오른쪽 아래 기준). 쓴 폭을 돌려줌"""
    (w, h), base = cv2.getTextSize(s, FONT, scale, thick)
    x, y = org
    if anchor == 'center':
        x, y = x - w / 2, y + h / 2
    elif anchor == 'right':
        x = x - w
    x, y = int(round(x)), int(round(y))
    if bg is not None:
        cv2.rectangle(img, (x - 2, y - h - 2), (x + w + 2, y + base), bg, -1)
    cv2.putText(img, s, (x, y), FONT, scale, color, thick, cv2.LINE_AA)
    return w


def _dashed(img, p, q, color, dash=6, gap=4, thick=1):
    length = math.hypot(q[0] - p[0], q[1] - p[1])
    if length == 0:
        return
    ux, uy = (q[0] - p[0]) / length, (q[1] - p[1]) / length
    s = 0.0
    while s < length:
        e = min(s + dash, length)
        cv2.line(img, _pt((p[0] + ux * s, p[1] + uy * s)), _pt((p[0] + ux * e, p[1] + uy * e)),
                 color, thick, cv2.LINE_AA)
        s = e + gap


def _dashed_rect(img, x0, y0, x1, y1, color, thick=1):
    for p, q in (((x0, y0), (x1, y0)), ((x1, y0), (x1, y1)), ((x1, y1), (x0, y1)), ((x0, y1), (x0, y0))):
        _dashed(img, p, q, color, thick=thick)


def _border(center, toward, pad=3):
    """center 상자의 테두리 위, toward 쪽으로 나가는 점"""
    cx, cy = center
    dx, dy = toward[0] - cx, toward[1] - cy
    hw, hh = BOX_W / 2 + pad, BOX_H / 2 + pad
    t = min(hw / abs(dx) if dx else math.inf, hh / abs(dy) if dy else math.inf)
    return cx + dx * t, cy + dy * t


def _blocked(a, b):
    """같은 줄의 두 상자 사이에 다른 상자가 있는지 (있으면 아래로 돌아감 — 지금 표에는 없음, 줄을 더할 때 대비)"""
    (xa, ya), (xb, _) = POS[a], POS[b]
    return any(s not in (a, b) and y == ya and min(xa, xb) < x < max(xa, xb) for s, (x, y) in POS.items())


# ================================================================
# 화살표 = 표의 줄
# ================================================================
def edges(table=TRANSITIONS, numbers=None):
    """표 → 화살표 목록 [(출발 상태, 도착 상태, 줄 번호)] — 한 줄 = 화살표 하나
       numbers = 줄 번호 목록 (시뮬레이터에서 줄을 삭제해도 처음 번호를 유지하기 위해)"""
    numbers = list(numbers) if numbers is not None else list(range(1, len(table) + 1))
    out = []
    for no, (froms, _, to) in zip(numbers, table):
        out.extend((s, to, no) for s in froms if s != to and s in POS and to in POS)
    return out


def route(a, b, pair=False):
    """화살표 길 = (점 목록, 번호 자리). pair = 반대 방향 화살표도 있음 → 나란히 비켜 그림"""
    ca, cb = POS[a], POS[b]
    if ca[1] == cb[1] and _blocked(a, b):          # 같은 줄, 사이에 상자 → 아래로 돌아감
        y = ca[1] + BOX_H / 2 + 14
        pts = [(ca[0], ca[1] + BOX_H / 2 + 3), (ca[0], y), (cb[0], y), (cb[0], cb[1] + BOX_H / 2 + 3)]
        return pts, ((ca[0] + cb[0]) / 2, y)
    p, q = _border(ca, cb), _border(cb, ca)
    length = math.hypot(q[0] - p[0], q[1] - p[1]) or 1.0
    nx, ny = -(q[1] - p[1]) / length, (q[0] - p[0]) / length     # 진행 방향의 오른쪽 (화면 좌표)
    shift = 5 if pair else 0
    p, q = (p[0] + nx * shift, p[1] + ny * shift), (q[0] + nx * shift, q[1] + ny * shift)
    gap = shift + 9
    return [p, q], ((p[0] + q[0]) / 2 + nx * gap, (p[1] + q[1]) / 2 + ny * gap)


def _draw_edge(img, pts, color, thick):
    for p, q in zip(pts[:-2], pts[1:-1]):
        cv2.line(img, _pt(p), _pt(q), color, thick, cv2.LINE_AA)
    p, q = pts[-2], pts[-1]
    length = math.hypot(q[0] - p[0], q[1] - p[1]) or 1.0
    cv2.arrowedLine(img, _pt(p), _pt(q), color, thick, cv2.LINE_AA, tipLength=min(0.5, 9.0 / length))


def diagram(img, top, table, numbers, state, visited=(), done_rows=(), last=None, marks=()):
    """상태도. done_rows = 지나온 줄 {(줄 번호, 출발 상태)}, last = 방금 지난 전이 (출발, 도착, 줄 번호),
       marks = 점검에서 걸린 상태 (빨간 점선). 안전장치 전이(줄 번호 'safety')는 맞는 화살표가 없어 빨개지지 않는다"""
    area = img[top:top + DIAGRAM_H]
    arrows = edges(table, numbers)
    pairs = {(a, b) for a, b, _ in arrows}
    drawn = []
    for a, b, no in arrows:
        used = (no, a) in done_rows
        now = last is not None and last[2] == no and last[0] == a
        level = 2 if now else 1 if used else 0
        pts, label = route(a, b, pair=(b, a) in pairs)
        drawn.append((level, pts, label, no))
    style = {0: (FAINT, 1, SOFT), 1: (INK, 1, INK), 2: (ACCENT, 2, ACCENT)}
    for level, pts, _, _ in sorted(drawn, key=lambda d: d[0]):          # 진한 화살표가 위로
        color, thick, _ = style[level]
        _draw_edge(area, pts, color, thick)
    for level, _, label, no in sorted(drawn, key=lambda d: d[0]):
        _, _, tcolor = style[level]
        _text(area, str(no), label, 0.38, tcolor, anchor='center', bg=WHITE)

    for s, (cx, cy) in POS.items():
        bx0, by0, bx1, by1 = cx - BOX_W // 2, cy - BOX_H // 2, cx + BOX_W // 2, cy + BOX_H // 2
        if s == state:
            fill, edge, tcolor, thick = ACCENT, ACCENT, WHITE, 2
        elif s in visited:
            fill, edge, tcolor, thick = PALE, INK, INK, 1
        else:
            fill, edge, tcolor, thick = WHITE, FAINT, SOFT, 1
        cv2.rectangle(area, (bx0, by0), (bx1, by1), fill, -1)
        cv2.rectangle(area, (bx0, by0), (bx1, by1), edge, thick, cv2.LINE_AA)
        _text(area, s, (cx, cy), 0.5, tcolor, 1 if s != state else 2, anchor='center')
        if s in marks:
            _dashed_rect(area, bx0 - 5, by0 - 5, bx1 + 5, by1 + 5, ACCENT, 2)


# ================================================================
# 글 영역
# ================================================================
def _canvas(height):
    return np.full((height, WIDTH, 3), 255, np.uint8)


def _header(img, title, state, right):
    x = 10 + _text(img, title, (10, 19), 0.45, SOFT) + 18
    x += _text(img, 'STATE', (x, 19), 0.45, INK) + 8
    _text(img, state if state else '-', (x, 19), 0.55, ACCENT if state else SOFT, 2)
    _text(img, right, (WIDTH - 10, 19), 0.4, SOFT, anchor='right')
    cv2.line(img, (0, HEADER_H - 1), (WIDTH, HEADER_H - 1), FAINT, 1)


def _heading(img, y, s):
    cv2.line(img, (10, y), (WIDTH - 10, y), FAINT, 1)
    _text(img, s, (10, y + 15), 0.4, SOFT)
    return y + 20


def _recent(img, y, recent, timed=False):
    """최근 전이 3줄 — recent = [(시각 또는 None, 줄 번호 또는 'safety', 사건, 출발, 도착)], 최신이 맨 앞
       표의 줄은 '#3', 표 밖 안전장치는 'safety' (화살표가 없으므로 이 줄에만 보인다)"""
    rows = list(recent)[:3]
    _heading(img, y, 'recent transitions   (time)  #row  event  from -> to' if timed else
             'recent transitions   #row  event  from -> to')
    if any(not isinstance(no, int) for _, no, _, _, _ in rows):   # 안전장치 줄이 있으면 뜻을 머리줄 오른쪽에
        _text(img, '(safety = checked before the table, no arrow)', (WIDTH - 10, y + 15), 0.35, SOFT, anchor='right')
    y += 20
    x = 74 if timed else 14                         # 칸 위치를 고정 (글자 폭이 제각각이라 공백으로 못 맞춤)
    if not rows:
        _text(img, '(none yet)', (14, y + 13), 0.45, SOFT)
    for i, (t, no, event, a, b) in enumerate(rows):
        color, ty = (ACCENT if i == 0 else INK), y + 13 + 17 * i
        if t is not None:
            _text(img, f'{t:.1f} s', (x - 12, ty), 0.45, color, anchor='right')
        _text(img, f'#{no}' if isinstance(no, int) else str(no), (x, ty), 0.45, color)
        _text(img, event, (x + 56, ty), 0.45, color)
        _text(img, f'{a} -> {b}', (x + 172, ty), 0.45, color)
    return y + 13 + 17 * 2 + 10


def _fit_path(path, width, scale=0.45):
    """지나온 길 — 길면 앞을 '... >' 로 줄임"""
    text = ' > '.join(path)
    i = 0
    while cv2.getTextSize(text, FONT, scale, 1)[0][0] > width and i < len(path) - 1:
        i += 1
        text = '... > ' + ' > '.join(path[i:])
    return text


def sim_frame(table, numbers, state, visited, done_rows, last, recent, path,
              check=None, changed='', hint=False):
    """21강 시뮬레이터 창. check = check_table 결과 (IDLE 에서 도달할 수 없는 상태, DONE 에 도달할 수 없는 상태)
       — 줄을 삭제한 뒤에만. changed = 표를 바꾼 내용 ('deleted 9 10')"""
    img = _canvas(SIM_H)                            # 높이 고정 — 줄 수가 달라져도 창 크기는 같다
    _header(img, 'w12_fsm_sim', state, 'type in the terminal  |  v = close window')
    marks = set(check[1]) if check else set()       # DONE 에 도달할 수 없는 상태 (교착 상태) = 빨간 점선
    diagram(img, HEADER_H, table, numbers, state, visited, done_rows, last, marks)
    y = _recent(img, HEADER_H + DIAGRAM_H, recent)
    cv2.line(img, (10, y), (WIDTH - 10, y), FAINT, 1)
    _text(img, 'path', (14, y + 17), 0.4, SOFT)
    _text(img, _fit_path(path, WIDTH - 72), (58, y + 17), 0.45, INK)
    y += 24
    if changed:                                      # 표를 바꾼 내용 · 점검 결과 (세 줄 자리는 늘 비워 둠)
        _text(img, f'table changed: {changed}   (r = reset)', (14, y + 13), 0.45, ACCENT)
    if check is not None:
        unreachable, no_finish = check
        if not (unreachable or no_finish):
            _text(img, 'check: OK', (14, y + 31), 0.45, INK)
        else:
            _text(img, 'check', (14, y + 31), 0.45, ACCENT)
            for k, (name, states) in enumerate((('unreachable from IDLE: ', unreachable),
                                                ('deadlock (cannot reach DONE): ', no_finish))):
                _text(img, name + (' '.join(states) or 'none'), (70, y + 31 + 18 * k), 0.45,
                      ACCENT if states else SOFT)
    y = _heading(img, y + 58, 'events - type the NAME in the terminal (one per line)')
    for i, line in enumerate(event_lines()):
        ty = y + 13 + 16 * i
        _text(img, line[0], (14, ty), 0.4, SOFT)                # 구분 이름 (옅게)
        x = LEGEND_X
        for name in line[1:]:                                    # 사건 이름 (진하게)
            x += _text(img, name, (x, ty), 0.4, INK) + 14
    if hint:
        cv2.rectangle(img, (120, HEADER_H + 96), (WIDTH - 120, HEADER_H + 130), ACCENT, -1)
        _text(img, 'type in the TERMINAL window', (WIDTH / 2, HEADER_H + 113), 0.6, WHITE, 2, anchor='center')
    return img


def event_lines():
    """사건 안내 2줄: [['mission', 'ready', 'airborne', ...], ['failsafe', 'sensor_bad', ...]] — 첫 칸 = 구분 이름"""
    merged = {}
    for group, name, _ in EVENTS:
        merged.setdefault(GROUP_NAMES.get(group, group), []).append(name)
    return [[group] + names for group, names in merged.items()]


def viewer_frame(state, since, visited, done_rows, last, recent, bands, series, now,
                 front_text, stop, clear, front_label='front', table=TRANSITIONS, span=60.0):
    """22강 상태 뷰어 창. bands = [(시각, 상태)] 상태가 바뀐 시각, series = [(시각, 앞 거리 또는 None)]"""
    img = _canvas(VIEW_H)
    right = f'in state {since:.1f} s' if state else 'waiting for /sjcu/mission_state (w12_mission)'
    _header(img, 'w12_state_viewer', state, right)
    diagram(img, HEADER_H, table, None, state, visited, done_rows, last)
    y = _recent(img, HEADER_H + DIAGRAM_H, recent, timed=True)
    y = _heading(img, y, f'last {span:.0f} s   {front_label}: {front_text}')
    _timeline(img, y + 2, bands, series, now, stop, clear, span)
    return img


def _timeline(img, y, bands, series, now, stop, clear, span, top_m=4.0):
    """상태 띠 + 앞 거리 곡선 (위 = top_m 이상 · 자유 공간)"""
    x0, x1 = 44, WIDTH - 14
    t0 = now - span

    def tx(t):
        return x0 + (t - t0) / span * (x1 - x0)

    band_h = 18
    for i, (t, s) in enumerate(bands):
        t_end = bands[i + 1][0] if i + 1 < len(bands) else now
        if t_end <= t0:
            continue
        a, b = max(tx(t), x0), min(tx(t_end), x1)
        if b - a < 1:
            continue
        last = i == len(bands) - 1
        color = ACCENT if last else SHADE[i % 2]
        cv2.rectangle(img, _pt((a, y)), _pt((b, y + band_h)), color, -1)
        w = cv2.getTextSize(s, FONT, 0.38, 1)[0][0]
        if b - a >= w + 6:
            _text(img, s, ((a + b) / 2, y + band_h / 2), 0.38, WHITE if last else INK, anchor='center')
    _text(img, 'state', (4, y + 13), 0.36, SOFT)

    py0, py1 = y + band_h + 8, y + band_h + 8 + 96       # 곡선 영역 (위 = top_m)

    def my(m):
        return py1 - min(max(m, 0.0), top_m) / top_m * (py1 - py0)

    cv2.line(img, (x0, py1), (x1, py1), FAINT, 1)
    for m, name, color in ((stop, f'{stop:.1f}', INK), (clear, f'{clear:.1f}', SOFT)):
        _dashed(img, (x0, my(m)), (x1, my(m)), color)
        _text(img, name, (x0 - 6, my(m) + 4), 0.36, color, anchor='right')
    _text(img, f'{top_m:.0f} m', (x0 - 6, py0 + 4), 0.36, SOFT, anchor='right')
    # 시간 축 글자는 곡선 영역 맨 아래 안쪽에 (아래 선 밖에 두면 창 높이 480 을 넘어 잘림)
    _text(img, f'-{span:.0f} s', (x0 + 2, py1 - 3), 0.36, SOFT)
    _text(img, 'now', (x1 - 2, py1 - 3), 0.36, SOFT, anchor='right')

    run = []
    for t, m in series:
        if t < t0 or m is None:
            if len(run) > 1:
                cv2.polylines(img, [np.array(run, np.int32)], False, BLUE, 2, cv2.LINE_AA)
            run = []
            continue
        run.append(_pt((tx(t), my(m if math.isfinite(m) else top_m))))
    if len(run) > 1:
        cv2.polylines(img, [np.array(run, np.int32)], False, BLUE, 2, cv2.LINE_AA)
