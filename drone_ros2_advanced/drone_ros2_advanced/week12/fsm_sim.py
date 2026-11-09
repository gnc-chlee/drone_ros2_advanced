#!/usr/bin/env python3
# ==============================================================================
# File    : fsm_sim.py  (12주차 21강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-06
# Version : 1.1.0   (2026-11-09 상태도 창 추가)
#
# Description:
#   전이 조건표 시뮬레이터 — PX4 · Gazebo 없이 터미널 하나로 표를 시험한다
#   사건 번호를 넣으면 상태가 어떻게 바뀌는지 보여 준다. 표는 22강 미션 노드와 같은 파일(mission_table.py)
#   상태도 창: 지금 상태에 불이 들어오고, 방금 쓰인 줄의 화살표가 빨개진다 (22강 w12_state_viewer 와 같은 그림)
#     입력은 터미널에 한다 (창은 보기만). 창이 앞에 오면 터미널을 한 번 클릭. v = 창 끄기·켜기
#     OpenCV 가 없거나 화면이 없으면(SSH 등) 창 없이 터미널만으로 된다
#
#   실행 방법:
#     ros2 run drone_ros2_advanced w12_fsm_sim
#
#   따라 하기:
#     정상 미션     : 1 → 2 → 10 → 11 → 13 → 3
#                     (ready → airborne → front_blocked → front_clear → arrived → landed)
#     같은 순간 둘  : 13 10  (arrived + front_blocked → 위 줄이 먼저)
#
#   결함 주입 (fault injection) — 일부러 표를 망가뜨려 어떤 문제가 생기는지 본다. 파일은 그대로:
#     del 6 7 → c   : HOLD 에서 스스로 나가는 줄을 지우면 '스스로 못 나가는 상태: HOLD' (창에서 화살표 6·7 이 사라짐)
#     del 10  → c   : TAKEOFF → CRUISE 를 지우면 CRUISE · AVOID · HOLD · MANUAL 에 닿지 못함
#     swap 11 12    : 도착과 막힘이 겹칠 때(13 10) LAND 대신 AVOID
#     r             : 표를 처음대로 되돌리기
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import os
import queue
import sys
import threading
import time

from .mission_table import EVENTS, FLYING, IDLE, DONE, TRANSITIONS, next_state, check_table

EVENT_NAMES = [name for _, name, _ in EVENTS]   # 사건 번호 = 이 목록의 순서 + 1
WINDOW = 'w12_fsm_sim'
HINT_TIME = 3.0      # 창에 글자를 치면 '터미널에 입력' 안내를 이만큼 보여 줌 [s]


def help_text():
    lines = ['상태머신 시뮬레이터 (w12_fsm_sim) — PX4 없이 전이 조건표만 시험합니다',
             '사건 번호를 넣으세요. 여러 개는 띄어쓰기 = 같은 순간에 생긴 사건']
    groups = []
    for group, _, _ in EVENTS:
        if group not in groups:
            groups.append(group)
    for group in groups:
        items = [f'{i} {name}' for i, (g, name, _) in enumerate(EVENTS, 1) if g == group]
        gap = ' ' * (5 - sum(2 if ord(c) > 127 else 1 for c in group))   # 한글은 두 칸 폭
        lines.append(f'  {group}{gap}' + '  '.join(items))
    lines.append('명령: t 표 · c 점검 · del N · swap N M · r 되돌리기 · s 처음부터 · v 상태도 창 · h 도움말 · q 끝')
    return '\n'.join(lines)


class Sim:
    def __init__(self):
        self.restore()

    # ── 표 ──────────────────────────────────────────────────────
    def restore(self):
        """표를 처음대로 + 상태도 IDLE 로. 줄 번호는 처음 번호를 계속 쓴다 (지우거나 바꿔도 안 변함)"""
        self.rows = list(enumerate(TRANSITIONS, 1))     # [(줄 번호, 줄), ...]
        self.check = None                               # 마지막 점검 결과 (창에 표시, 표를 바꾸면 지움)
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
        """표를 바꾼 내용 (창 표시용): 'deleted 6 7, order changed' · 그대로면 ''"""
        labels = self._labels()
        deleted = [n for n in range(1, len(TRANSITIONS) + 1) if n not in labels]
        parts = ['deleted ' + ' '.join(map(str, deleted))] if deleted else []
        if labels != sorted(labels):
            parts.append('order changed')
        return ', '.join(parts)

    # ── 사건 넣기 ───────────────────────────────────────────────
    def apply(self, events):
        now = self.state
        to, used = next_state(now, set(events), self.table())
        names = ' · '.join(events)
        if used is None:
            print(f'  사건 {names} → 맞는 줄 없음 → {now} 그대로')
            if now == DONE:
                print('  미션이 끝났습니다 — s 로 처음부터')
            return
        first = f' {used} (위 줄이 먼저)' if len(events) > 1 else ''
        no = self.row_no(now, used)
        print(f'  사건 {names} → {no}번 줄{first} → {to}')
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

    # ── 표 보기 · 점검 ──────────────────────────────────────────
    def show_table(self):
        print(' 줄  지금 상태          사건            다음 상태')
        for label, (froms, event, to) in self.rows:
            where = 'FLYING' if froms == FLYING else ' '.join(froms)
            print(f'{label:>3}  {where:<18} {event:<15} {to}')
        print('FLYING = 비행 중 전체 (' + ' '.join(FLYING) + ')  ·  위 줄이 먼저')

    def show_check(self):
        unreachable, no_finish, stuck = self.check = check_table(self.table())
        print('표 점검')
        print('  도달 못 하는 상태    : ' + (' '.join(unreachable) or '없음'))
        print('  끝에 못 가는 상태    : ' + (' '.join(no_finish) or '없음'))
        print('  스스로 못 나가는 상태: ' + (' '.join(stuck) or '없음'))
        if unreachable or no_finish or stuck:
            print('  → 결함 있음 (r 로 표 되돌리기)')
        else:
            print('  → 통과')

    # ── 결함 주입: 줄 지우기 · 순서 바꾸기 ──────────────────────
    def delete(self, numbers):
        for n in numbers:
            if n not in self._labels():
                print(f'  {n}번 줄이 없습니다 (t 로 표 확인)')
                return
        self.rows = [(label, row) for label, row in self.rows if label not in numbers]
        self.check = None
        print('  ' + ' · '.join(f'{n}번' for n in numbers) + ' 줄 지움 — c 로 점검, r 로 되돌리기')

    def swap(self, a, b):
        labels = self._labels()
        if a not in labels or b not in labels:
            print('  없는 줄 번호입니다 (t 로 표 확인)')
            return
        i, j = labels.index(a), labels.index(b)
        self.rows[i], self.rows[j] = self.rows[j], self.rows[i]
        self.check = None
        print(f'  {a}번 · {b}번 줄 순서 바꿈 — t 로 확인, r 로 되돌리기')

    # ── 입력 한 줄 처리 (q 면 False) ────────────────────────────
    def handle(self, line):
        words = line.split()
        if not words:
            return True
        head = words[0].lower()
        if head in ('q', 'quit', 'exit'):
            return False
        if head == 't':
            self.show_table()
        elif head == 'c':
            self.show_check()
        elif head == 'r':
            self.restore()
            print('  표를 처음대로 되돌림, 상태 IDLE')
        elif head == 's':
            self.restart()
            print('  상태 IDLE (표는 그대로)')
        elif head == 'h':
            print(help_text())
        elif head == 'del':
            numbers = [int(w) for w in words[1:] if w.isdigit()]
            if numbers and len(numbers) == len(words) - 1:
                self.delete(numbers)
            else:
                print('  사용법: del 6   또는   del 6 7')
        elif head == 'swap':
            if len(words) == 3 and words[1].isdigit() and words[2].isdigit():
                self.swap(int(words[1]), int(words[2]))
            else:
                print('  사용법: swap 11 12')
        else:
            events = [self._event_name(w) for w in words]
            if None in events:
                print(f'  모르는 입력: {line.strip()}  (h = 도움말)')
            else:
                self.apply(events)
        return True

    @staticmethod
    def _event_name(word):
        """'10' 또는 'front_blocked' → 사건 이름 (없으면 None)"""
        if word.isdigit() and 1 <= int(word) <= len(EVENT_NAMES):
            return EVENT_NAMES[int(word) - 1]
        return word if word in EVENT_NAMES else None


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
    run(sim, view)


if __name__ == '__main__':
    main()
