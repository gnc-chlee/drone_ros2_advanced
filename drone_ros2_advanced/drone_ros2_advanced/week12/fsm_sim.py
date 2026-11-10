#!/usr/bin/env python3
# ==============================================================================
# File    : fsm_sim.py  (12주차 21강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-06
# Version : 1.2.0   (2026-11-10 단순화)
#
# Description:
#   전이 조건표 시뮬레이터 — PX4 · Gazebo 없이 터미널 하나로 표를 시험한다
#   사용자가 PX4 · 센서 역할: 사건(일어난 일) 이름을 입력하면 표에 따라 상태가 바뀐다
#   표는 22강 미션 노드와 같은 파일(mission_table.py)
#   상태도 창: 현재 상태 = 빨강, 방금 쓰인 줄의 화살표 = 빨강 (22강 w12_state_viewer 와 같은 그림)
#     입력은 터미널에 한다 (창은 표시 전용). 창이 앞에 오면 터미널을 한 번 클릭. v = 창 끄기·켜기
#     OpenCV 가 없거나 화면이 없으면(SSH 등) 창 없이 터미널만으로 된다
#
#   실행 방법:
#     ros2 run drone_ros2_advanced w12_fsm_sim
#
#   따라 하기 (사건 이름을 한 줄에 하나씩 입력):
#     정상 미션 : ready → airborne → front_blocked → front_clear → arrived → landed
#                 (1 → 2 → 3 → 4 → 5 → 6번 줄 순서로 전이 → DONE, 창의 화살표도 같은 순서로 빨강)
#     복구      : r → ready → airborne → sensor_bad → sensor_ok      (7 → 10번 줄, CRUISE 복귀)
#     타임아웃  : r → ready → airborne → sensor_bad → hold_timeout   (7 → 9번 줄, LAND)
#     ※ 이름은 2글자 이상 앞부분만 입력해도 된다 (re = ready). 한 줄에 여러 이름을 띄어 쓰면 차례로 입력한 것과 같다
#     ※ (심화) 같은 순간에 생긴 사건은 + 로 잇는다: arrived+front_blocked → 위 줄이 먼저 (3번 줄 → AVOID)
#
#   결함 주입 (fault injection) — 일부러 문제를 넣고 시스템이 어떻게 반응하는지 본다. 표 파일은 바뀌지 않음:
#     r → del 9 10 : HOLD 에서 나가는 줄을 삭제하면 점검이 자동으로 'DONE 에 도달할 수 없는 상태 (교착 상태) : HOLD'
#                    (창에서 화살표 9 · 10 이 사라지고 HOLD 에 빨간 점선)
#                    → ready → airborne → sensor_bad → sensor_ok → '해당 줄 없음 → HOLD 유지'
#     r            : 표와 상태 초기화
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import os
import queue
import re
import sys
import threading
import time
import unicodedata

from .mission_table import DONE, EVENTS, IDLE, SAFETY, STATES, TRANSITIONS, check_table, next_state

EVENT_NAMES = [name for _, name, _ in EVENTS]          # 입력할 수 있는 사건 9개
SAFETY_NAMES = [event for _, event, _ in SAFETY]       # 표 밖 안전장치 사건 (22강 미션 노드에만)
OLD_COMMANDS = ('c', 's', 'swap')                      # 단순화 전에 있던 명령 (지금은 없음)
WINDOW = 'w12_fsm_sim'
HINT_TIME = 3.0      # 창에 글자를 치면 '터미널에 입력' 안내를 이만큼 보여 줌 [s]
HELP_WIDTH = 78      # 도움말 한 줄 최대 폭 (터미널 칸, 한글은 두 칸)


def _width(s):
    """터미널에서 차지하는 칸 수 (한글 등 넓은 글자는 두 칸)"""
    return sum(2 if unicodedata.east_asian_width(c) in ('W', 'F') else 1 for c in s)


def _shown(s):
    """안내에 다시 찍는 입력 — 제어 문자(화살표 키의 ESC 등)는 '?' 로 (터미널 커서가 움직이지 않게)"""
    return ''.join(c if c.isprintable() else '?' for c in s)


def help_text():
    lines = ['상태머신 시뮬레이터 (w12_fsm_sim) — PX4 없이 전이 조건표만 시험합니다',
             '사용자가 PX4 · 센서 역할입니다. 사건(일어난 일) 이름을 한 줄에 하나씩 입력하면',
             '표에 따라 상태가 바뀝니다']
    groups = []
    for group, _, _ in EVENTS:
        if group not in groups:
            groups.append(group)
    for group in groups:
        items = [f'{name} ({meaning})' for g, name, meaning in EVENTS if g == group]
        head = '  ' + group + ' ' * (12 - _width(group))        # 구분 이름 칸 = 12칸
        line = head
        for item in items:                                      # 폭을 넘으면 다음 줄 (같은 자리에서 시작)
            if line != head and _width(line + ' · ' + item) > HELP_WIDTH:
                lines.append(line)
                line = ' ' * _width(head)
            elif line != head:
                line += ' · '
            line += item
        lines.append(line)
    lines.append('명령: t 표 · del N 줄 삭제 · r 초기화 · v 상태도 창 · h 도움말 · q 종료')
    return '\n'.join(lines)


class Sim:
    def __init__(self):
        self.restore()

    # ── 표 ──────────────────────────────────────────────────────
    def restore(self):
        """표 초기화 + 상태도 IDLE 로. 줄 번호는 처음 번호를 계속 쓴다 (삭제해도 안 변함)"""
        self.rows = list(enumerate(TRANSITIONS, 1))     # [(줄 번호, 줄), ...]
        self.check = None                               # 마지막 점검 결과 (창에 표시, r 로 초기화)
        self.restart()

    def restart(self):
        self.state = IDLE
        self.path = [IDLE]
        self.visited = {IDLE}     # 창: 지나온 상태
        self.done_rows = set()    # 창: 지나온 줄 {(줄 번호, 출발 상태)}
        self.last = None          # 창: 방금 지난 전이 (출발, 도착, 줄 번호)
        self.recent = []          # 창: 최근 전이 [(None, 줄 번호, 사건, 출발, 도착)], 최신이 맨 앞

    def table(self):
        return [row for _, row in self.rows]

    def _labels(self):
        return [label for label, _ in self.rows]

    def row_no(self, state, event):
        for label, (froms, ev, _) in self.rows:
            if state in froms and ev == event:
                return label
        return None

    def changed(self):
        """표를 바꾼 내용 (창 표시용): 'deleted 9 10' · 바꾸지 않았으면 ''"""
        labels = self._labels()
        deleted = [n for n in range(1, len(TRANSITIONS) + 1) if n not in labels]
        return 'deleted ' + ' '.join(map(str, deleted)) if deleted else ''

    def out_events(self, state):
        """현재 표에서 state 에서 나가는 줄의 사건들 (표 순서)"""
        out = []
        for _, (froms, event, _) in self.rows:
            if state in froms and event not in out:
                out.append(event)
        return out

    def out_hint(self, state):
        """'(CRUISE 에서 표에 있는 사건: front_blocked · arrived · sensor_bad)' — 나가는 줄이 없으면 교착 상태"""
        events = self.out_events(state)
        if not events:
            return f'  ({state} 에서 나가는 줄 없음 — 교착 상태)'
        return f'  ({state} 에서 표에 있는 사건: {" · ".join(events)})'

    # ── 사건 입력 (같은 순간 사건이 여럿이면 events 에 함께) ──────
    def apply(self, events):
        now = self.state
        names = ' · '.join(events)
        head = f'같은 순간 사건 {len(events)}개: {names}' if len(events) > 1 else f'사건 {names}'
        if now == DONE:
            print(f'  {head} → 미션이 끝났습니다 — r 로 초기화')
            return
        to, used = next_state(now, set(events), self.table())
        if used is None:
            print(f'  {head} → 해당 줄 없음 → {now} 유지')
            print(self.out_hint(now))
            return
        no = self.row_no(now, used)
        matched = [ev for ev in self.out_events(now) if ev in events]
        first = f' {used} (위 줄이 먼저)' if len(matched) > 1 else (f' {used}' if len(events) > 1 else '')
        print(f'  {head} → {no}번 줄{first} → {to}')
        if to != now:
            self.state = to
            self.path.append(to)
            self.visited.add(to)
            self.done_rows.add((no, now))
            self.last = (now, to, no)
            self.recent.insert(0, (None, no, used, now, to))
            del self.recent[3:]
        if to == DONE:
            print('  미션 끝: ' + ' → '.join(self.path))
            print('  (끝 — r 로 초기화)')
        else:
            print(self.out_hint(to))

    # ── 표 보기 · 점검 ──────────────────────────────────────────
    def show_table(self):
        group_of = {name: group for group, name, _ in EVENTS}
        print(' 줄  현재 상태  사건           다음 상태')
        group = None
        for label, (froms, event, to) in self.rows:
            if group_of.get(event) != group:              # 구분이 바뀌면 머리줄 (미션 · 페일세이프)
                group = group_of.get(event)
                print(f'     ── {group} ──')
            print(f'{label:>3}  {" ".join(froms):<10} {event:<14} {to}')
        if self.changed():
            print('  (삭제한 줄: ' + self.changed()[len('deleted '):] + ' — r 로 초기화)')

    def show_check(self):
        unreachable, no_finish = self.check = check_table(self.table())
        print('표 점검')
        print('  IDLE 에서 도달할 수 없는 상태           : ' + (' '.join(unreachable) or '없음'))
        print('  DONE 에 도달할 수 없는 상태 (교착 상태) : ' + (' '.join(no_finish) or '없음'))
        if unreachable or no_finish:
            print('  → 결함 있음 (r 로 초기화)')
        else:
            print('  → 통과')

    # ── 결함 주입: 줄 삭제 (삭제하면 점검 자동 실행) ──────────────
    def delete(self, numbers):
        numbers = list(dict.fromkeys(numbers))          # 같은 번호 두 번은 한 번으로
        for n in numbers:
            if n not in self._labels():
                print(f'  {n}번 줄이 없습니다 (t 로 표 확인)')
                return
        self.rows = [(label, row) for label, row in self.rows if label not in numbers]
        print('  ' + ' · '.join(f'{n}번' for n in numbers) + ' 줄 삭제')
        self.show_check()

    # ── 입력 한 줄 처리 (q 면 False) ────────────────────────────
    def handle(self, line):
        words = re.sub(r'\s*\+\s*', '+', line).split()   # 'arrived + front_blocked' → 'arrived+front_blocked'
        if not words:
            return True
        head = words[0].lower()
        if head in ('q', 'quit', 'exit'):
            return False
        if head in ('t', 'r', 'h', 'help') and len(words) > 1:   # 'r ready airborne' = r 다음에 ready · airborne 을 차례로
            self.handle(head)
            return self.handle(' '.join(words[1:]))
        if head == 't':
            self.show_table()
        elif head == 'r':
            self.restore()
            print('  표와 상태 초기화 (상태 IDLE)')
            print(self.out_hint(IDLE))
        elif head in ('h', 'help'):
            print(help_text())
        elif head == 'del':
            numbers = [int(w) for w in words[1:] if w.isascii() and w.isdigit()]
            if numbers and len(numbers) == len(words) - 1:
                self.delete(numbers)
            else:
                print('  사용법: del 9   또는   del 9 10')
        elif head in OLD_COMMANDS:
            print(f'  알 수 없는 입력: {_shown(line.strip())}  (h = 도움말)')
        else:
            steps = []                                  # 단어마다 [사건, ...] — 하나라도 모르면 아무것도 안 함
            for word in words:
                names = []
                for part in [p for p in word.split('+') if p]:
                    name, problem = self._event_name(part)
                    if problem:
                        print('  ' + problem)
                        return True
                    if name not in names:
                        names.append(name)
                if not names:                           # '+' 만 친 경우
                    print(f'  알 수 없는 입력: {_shown(word)}  (h = 도움말)')
                    return True
                steps.append(names)
            for names in steps:                         # 차례로 (두 번 입력한 것과 같음)
                self.apply(names)
        return True

    def _event_name(self, word):
        """입력 한 단어 → (사건 이름, None) 또는 (None, 안내 글)
           정확한 이름 또는 2글자 이상의 겹치지 않는 앞부분 (대소문자 무시)"""
        w = word.lower()
        if w in EVENT_NAMES:
            return w, None
        if not w.isprintable():                         # 화살표 키 ↑ 는 ESC[A 로 들어온다 (이 입력에는 줄 편집 기능이 없음)
            return None, '화살표 키(↑ ↓)는 쓰지 않습니다 — 사건 이름을 다시 입력하세요 (예: ready)'
        if w in ('t', 'r', 'h', 'del', 'v', 'q'):
            return None, f'{w} 는 명령입니다 — 명령은 줄 맨 앞에 (예: r ready airborne)'
        if not w.isascii():
            return None, '한글이 입력됐습니다 — 한/영 키로 영문 입력으로'
        if w.isdigit():
            return None, '숫자는 표의 줄 번호입니다 — 사건은 이름으로 (예: ready). 줄 삭제: del 9 10'
        if w in SAFETY_NAMES:
            return None, f'{w} 는 표 밖 안전장치 사건 — 22강 미션 노드에만 있습니다'
        if w.upper() in STATES:
            events = self.out_events(self.state)
            return None, (f'{w.upper()} 는 상태 이름입니다 — 사건(일어난 일) 이름을 입력하세요. '
                          f'{self.state} 에서 표에 있는 사건: {" · ".join(events) or "없음"}')
        if len(w) >= 2:
            hits = [name for name in EVENT_NAMES if name.startswith(w)]
            if len(hits) == 1:
                return hits[0], None
            if hits:
                count = '둘' if len(hits) == 2 else f'{len(hits)}개'
                return None, f"'{w}' 로 시작하는 사건이 {count}: {' · '.join(hits)} — 몇 글자 더 입력하세요"
        return None, f'알 수 없는 사건: {_shown(word)} — 입력 가능한 사건: {" · ".join(EVENT_NAMES)}'


# ================================================================
# [복붙 영역] 상태도 창 — 학생은 읽지 않아도 된다
#   OpenCV 는 창을 그리는 쪽(메인 스레드)에서만 부르고, 터미널 입력은 따로 받는다
#   (입력을 기다리는 동안 창이 멈춰 '응답 없음' 이 뜨지 않게)
# ================================================================
class View:
    def __init__(self):
        self.on = False
        self.cv2 = self.draw = None
        self.hint_until = 0.0

    def open(self):
        """창 열기 — 못 열면 이유를 찍고 False (터미널만으로 계속)"""
        if sys.platform.startswith('linux') and not (os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
            print('  상태도 창 없음: 화면이 없습니다 (SSH 접속 등) — 터미널만으로 계속')
            return False
        try:
            import cv2
            from . import state_view
            cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        except Exception as e:     # OpenCV 없음 · 화면 없는 빌드(cv2.error) 등
            print(f'  상태도 창 없음 ({type(e).__name__}) — 터미널만으로 계속')
            return False
        self.cv2, self.draw, self.on = cv2, state_view, True
        return True

    def close(self):
        if self.on:
            self.on = False
            try:
                self.cv2.destroyWindow(WINDOW)
                self.cv2.waitKey(1)
            except Exception:
                pass

    def toggle(self, sim):
        if self.on:
            self.close()
            print('  상태도 창 끔 (v 로 다시 켜기)')
        elif self.open():
            print('  상태도 창 켬 — 입력은 이 터미널에')

    def show(self, sim):
        if not self.on:
            return
        img = self.draw.sim_frame(sim.table(), sim._labels(), sim.state, sim.visited, sim.done_rows, sim.last,
                                  sim.recent, sim.path, sim.check, sim.changed(),
                                  hint=time.monotonic() < self.hint_until)
        self.cv2.imshow(WINDOW, img)
        self.cv2.waitKey(1)

    def pump(self, sim):
        """입력을 기다리는 동안 창을 살려 둔다. 창을 닫으면 끔, 창에 글자를 치면 '터미널에 입력' 안내"""
        if not self.on:
            return
        key = self.cv2.waitKey(30)
        if self._closed():
            self.on = False
            print(f'\n  상태도 창 닫힘 (v 로 다시 켜기)\n[{sim.state}] > ', end='', flush=True)
            return
        if key != -1 and key & 0xFF != 0xFF:
            self.hint_until = time.monotonic() + HINT_TIME
            self.show(sim)
        elif self.hint_until and time.monotonic() >= self.hint_until:
            self.hint_until = 0.0
            self.show(sim)

    def _closed(self):
        try:
            v = self.cv2.getWindowProperty(WINDOW, self.cv2.WND_PROP_VISIBLE)
            if v < 0:                                # 이 빌드는 VISIBLE 을 모름
                return self.cv2.getWindowProperty(WINDOW, self.cv2.WND_PROP_AUTOSIZE) < 0
            return v < 1
        except Exception:
            return True


def run(sim, view, read=input):
    """입력 한 줄씩 처리. 입력은 따로 받고(read), 기다리는 동안 창을 챙긴다"""
    lines, ready = queue.Queue(), threading.Event()

    def reader():
        while True:
            ready.wait()
            ready.clear()
            try:
                lines.put(read(f'[{sim.state}] > '))
            except (EOFError, KeyboardInterrupt):
                lines.put(None)
                return

    threading.Thread(target=reader, daemon=True).start()
    ready.set()
    try:
        while True:
            try:
                line = lines.get(timeout=0.03 if view.on else 0.5)
            except queue.Empty:
                view.pump(sim)
                continue
            if line is None:                         # Ctrl+D
                print()
                break
            if line.strip().lower() == 'v':
                view.toggle(sim)
            elif not sim.handle(line):
                break
            view.show(sim)
            ready.set()
    except KeyboardInterrupt:                        # Ctrl+C
        print()
    finally:
        view.close()


def main(args=None):
    sim, view = Sim(), View()
    print(help_text())
    if view.open():
        print('상태도 창을 띄웠습니다 — 입력은 이 터미널에 (창이 앞에 오면 터미널을 한 번 클릭) · v = 창 끄기·켜기')
        view.show(sim)
    print(sim.out_hint(IDLE).strip())
    run(sim, view)


if __name__ == '__main__':
    main()
