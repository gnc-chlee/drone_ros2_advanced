#!/usr/bin/env python3
# ==============================================================================
# File    : mission_table.py  (12주차 21·22강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-06
# Version : 1.0.0
#
# Description:
#   미션 상태머신의 전이 조건표 (state transition table)
#   21강 시뮬레이터(fsm_sim)와 22강 미션 노드(mission_base)가 이 표 하나를 같이 쓴다
#   → 표를 고치면 두 곳의 동작이 함께 바뀐다. ROS 없는 순수 파이썬
#
#   읽는 법: 지금 상태 + 생긴 사건 → 다음 상태. 위 줄이 먼저 (PX4 > 사람 > 안전 > 미션)
#     같은 순간 사건이 여러 개면 위에 있는 줄 하나만 쓴다
#     맞는 줄이 없으면 상태는 그대로다
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

# ── 상태 8개 ─────────────────────────────────────────────────────
IDLE    = 'IDLE'      # 스캔 확인 후 Offboard + Arm 요청
TAKEOFF = 'TAKEOFF'   # 제자리 이륙 (고도 2 m 까지)
CRUISE  = 'CRUISE'    # 동쪽으로 전진 (감속 · 코스 복귀 포함)
AVOID   = 'AVOID'     # 앞이 막힘 → 넓은 쪽으로 좌우 이동
HOLD    = 'HOLD'      # 센서 신호 끊김 → 수평 정지 · 고도 유지 (PX4 Hold 모드와 다름, 노드 안의 상태)
MANUAL  = 'MANUAL'    # 수동 개입: 사람이 키보드로 조종
LAND    = 'LAND'      # PX4 착륙 모드에 맡김
DONE    = 'DONE'      # 끝 (명령 없음)

STATES = [IDLE, TAKEOFF, CRUISE, AVOID, HOLD, MANUAL, LAND, DONE]
FLYING = (TAKEOFF, CRUISE, AVOID, HOLD, MANUAL)   # 비행 중 전체

# ── 사건 14개: (구분, 이름, 언제) — 시뮬레이터의 사건 번호는 이 순서 ──────
EVENTS = [
    ('PX4',  'ready',         'Offboard + Arm 확인'),
    ('PX4',  'airborne',      '상대 고도 2 m ± 0.2'),
    ('PX4',  'landed',        '착지 감지'),
    ('PX4',  'px4_takeover',  '비행 중 Offboard 가 아닌 채 1초'),
    ('사람', 'cmd_manual',    '키 M'),
    ('사람', 'cmd_auto',      '키 G'),
    ('사람', 'cmd_land',      '키 L'),
    ('센서', 'sensor_bad',    '유효한 스캔 0.5초 없음 또는 -inf'),
    ('센서', 'sensor_ok',     '유효한 스캔 1.0초 이어짐'),
    ('센서', 'front_blocked', '앞 1.5 m 미만'),
    ('센서', 'front_clear',   '앞 2.0 m 이상'),
    ('센서', 'dead_end',      '막다른 곳'),
    ('미션', 'arrived',       '동쪽 11.7 m 넘음'),
    ('시간', 'hold_timeout',  'HOLD 10초'),
]

# ================================================================
# [USER DEFINE] 전이 조건표 14줄 — (지금 상태들, 사건, 다음 상태)
# ================================================================
TRANSITIONS = [
    # PX4 가 조종권을 가져가면 손을 뗀다 (다시 요청하면 조종사와 싸우게 된다)
    (FLYING,                 'px4_takeover',  DONE),       # 1
    # 사람
    (FLYING,                 'cmd_land',      LAND),       # 2
    ((CRUISE, AVOID, HOLD),  'cmd_manual',    MANUAL),     # 3
    ((MANUAL,),              'cmd_auto',      CRUISE),     # 4
    # 안전
    ((CRUISE, AVOID),        'sensor_bad',    HOLD),       # 5
    ((HOLD,),                'hold_timeout',  LAND),       # 6  기다림에는 타임아웃
    ((HOLD,),                'sensor_ok',     CRUISE),     # 7  앞이 막혀 있으면 다음 틱에 AVOID 로
    ((AVOID,),               'dead_end',      LAND),       # 8
    # 미션
    ((IDLE,),                'ready',         TAKEOFF),    # 9
    ((TAKEOFF,),             'airborne',      CRUISE),     # 10
    ((CRUISE,),              'arrived',       LAND),       # 11
    ((CRUISE,),              'front_blocked', AVOID),      # 12  정지 판단 1.5 m
    ((AVOID,),               'front_clear',   CRUISE),     # 13  해제 2.0 m (둘이 달라서 히스테리시스)
    ((LAND,),                'landed',        DONE),       # 14
]


def next_state(state, events, table=TRANSITIONS):
    """표를 위 줄부터 읽어, 지금 상태 + 생긴 사건에 맞는 첫 줄을 찾는다.
       돌려주는 값 = (다음 상태, 그 사건). 맞는 줄이 없으면 (상태 그대로, None)"""
    for froms, event, to in table:
        if state in froms and event in events:
            return to, event
    return state, None


def row_number(state, event, table=TRANSITIONS):
    """지금 상태 + 사건에 맞는 첫 줄의 번호 (1부터, 없으면 None) — 상태 뷰어에 '몇 번 줄' 을 보여 줄 때"""
    for i, (froms, ev, _) in enumerate(table, 1):
        if state in froms and ev == event:
            return i
    return None


def check_table(table=TRANSITIONS):
    """표 점검 — 코드를 짜기 전에 표에서 찾을 수 있는 결함 세 가지. 돌려주는 값 = 상태 이름 목록 3개
       unreachable : IDLE 에서 갈 길이 없는 상태
       no_finish   : DONE 으로 가는 길이 없는 상태
       stuck       : 사람 명령·PX4 없이 스스로 나가는 줄이 없는 상태 (MANUAL 은 사람이 끝내는 상태라 제외)"""
    out = {s: set() for s in STATES}       # 상태 → 갈 수 있는 상태들
    back = {s: set() for s in STATES}      # 상태 → 그 상태로 오는 상태들
    self_exit = set()                      # 스스로 나가는 줄이 있는 상태
    for froms, event, to in table:
        for s in froms:
            out[s].add(to)
            back[to].add(s)
            if not event.startswith('cmd_') and event != 'px4_takeover':
                self_exit.add(s)

    def spread(start, links):              # start 에서 links 를 따라 닿는 상태 전부
        seen, todo = {start}, [start]
        while todo:
            for n in links[todo.pop()]:
                if n not in seen:
                    seen.add(n)
                    todo.append(n)
        return seen

    can_reach, can_finish = spread(IDLE, out), spread(DONE, back)
    unreachable = [s for s in STATES if s not in can_reach]
    no_finish = [s for s in STATES if s not in can_finish]
    stuck = [s for s in STATES if s not in (DONE, MANUAL) and s not in self_exit]
    return unreachable, no_finish, stuck
