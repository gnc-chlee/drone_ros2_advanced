#!/usr/bin/env python3
# ==============================================================================
# File    : mission_table.py  (12주차 21·22강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-06
# Version : 1.1.0   (2026-11-10 단순화)
#
# Description:
#   미션 상태머신의 전이 조건표 (state transition table)
#   21강 시뮬레이터(fsm_sim)와 22강 미션 노드(mission_base)가 이 표 하나를 같이 쓴다
#   → 표를 고치면 두 곳의 동작이 함께 바뀐다. ROS 없는 순수 파이썬
#
#   읽는 법: 한 줄 = 현재 상태 하나 + 사건 하나 → 다음 상태 하나 (= 상태도의 화살표 하나)
#     해당 줄이 없으면 현재 상태 유지
#     같은 순간 사건이 여러 개면 위에 있는 줄 하나만 쓴다
#   미션 노드는 표보다 먼저 표 밖 안전장치 2줄(SAFETY)을 본다 — 21강 표 · 시뮬레이터 · 상태도 그림에는 없음
#
#   같은 순간 사건 두 개 (obstacle_world 비행에서는 생기지 않음 — 시험으로 고정):
#     CRUISE + arrived · front_blocked → AVOID (3번 줄)
#     CRUISE + arrived · sensor_bad    → LAND  (5번 줄 — 목표 위라 바로 착륙해도 안전)
#     HOLD   + hold_timeout · sensor_ok → LAND (9번 줄)
#     sensor_bad 는 sensor_ok · front_blocked · front_clear · dead_end 와 같은 틱에 생기지 않는다
#     (스캔이 끊기거나 -inf 가 있으면 앞 · 옆 판단을 하지 않음)
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

# ── 상태 7개 ─────────────────────────────────────────────────────
IDLE    = 'IDLE'      # 준비: 스캔 확인 후 Offboard + Arm 요청
TAKEOFF = 'TAKEOFF'   # 이륙: 제자리에서 고도 2 m 까지
CRUISE  = 'CRUISE'    # 순항: 동쪽으로 전진 (감속 · 코스 복귀 포함)
AVOID   = 'AVOID'     # 회피: 앞이 막힘 → 넓은 쪽으로 좌우 이동
HOLD    = 'HOLD'      # 대기: 센서 신호 끊김 → 수평 정지 · 고도 유지
                      #       (노드 안의 상태 — PX4 는 Offboard 유지, PX4 Hold 모드 아님)
LAND    = 'LAND'      # 착륙: PX4 Land 모드에 맡김
DONE    = 'DONE'      # 끝: 명령 중단

STATES = [IDLE, TAKEOFF, CRUISE, AVOID, HOLD, LAND, DONE]

# ── 사건 9개: (구분, 이름, 뜻) — 시뮬레이터에서 칠 수 있는 사건은 이 9개뿐 ──────
#    정확한 조건은 오른쪽 주석 (22강 미션 노드가 이 조건으로 사건을 판정한다)
EVENTS = [
    ('미션',       'ready',         'Offboard + Arm 완료'),   # Offboard + Arm 확인
    ('미션',       'airborne',      '이륙 고도 도달'),         # 상대 고도 2 m ± 0.2
    ('미션',       'front_blocked', '앞 1.5 m 미만'),
    ('미션',       'front_clear',   '앞 2.0 m 이상'),
    ('미션',       'arrived',       '목표 도착'),              # 동쪽 11.7 m 넘음
    ('미션',       'landed',        '착지'),                   # 착지 감지
    ('페일세이프', 'sensor_bad',    '센서 신호 끊김'),         # 유효한 스캔 0.5초 없음 또는 -inf
    ('페일세이프', 'sensor_ok',     '신호 1초 이어짐'),        # 유효한 스캔 1.0초 이어짐
    ('페일세이프', 'hold_timeout',  'HOLD 20초'),              # HOLD 타임아웃 (파라미터 hold_timeout, 기본 20초)
]

# ================================================================
# [USER DEFINE] 전이 조건표 10줄 — (현재 상태, 사건, 다음 상태)
#   한 줄 = 상태도의 화살표 하나. 정상 미션 1~6 · 페일세이프 7~10
# ================================================================
TRANSITIONS = [
    # 정상 미션: 1 → 2 → 3 → 4 → 5 → 6
    ((IDLE,),    'ready',         TAKEOFF),   # 1
    ((TAKEOFF,), 'airborne',      CRUISE),    # 2
    ((CRUISE,),  'front_blocked', AVOID),     # 3  정지 판단 1.5 m
    ((AVOID,),   'front_clear',   CRUISE),    # 4  해제 2.0 m (둘이 달라서 히스테리시스)
    ((CRUISE,),  'arrived',       LAND),      # 5
    ((LAND,),    'landed',        DONE),      # 6
    # 페일세이프: 센서 신호 끊김 → HOLD 에서 대기 → 복구 시 CRUISE · 타임아웃 시 LAND
    ((CRUISE,),  'sensor_bad',    HOLD),      # 7
    ((AVOID,),   'sensor_bad',    HOLD),      # 8
    ((HOLD,),    'hold_timeout',  LAND),      # 9  대기 상태에는 타임아웃
    ((HOLD,),    'sensor_ok',     CRUISE),    # 10 앞이 막혀 있으면 다음 틱에 AVOID 로
]

# ================================================================
# [FIXED] 표 밖 안전장치 2줄 — 미션 노드가 표보다 먼저 본다
#   (21강 표 · 시뮬레이터 · 상태도 그림에는 없음. 전이 로그에는 줄 번호 대신 '안전장치')
# ================================================================
FLYING = (TAKEOFF, CRUISE, AVOID, HOLD)   # 비행 중 (IDLE 은 Arm 전, LAND 는 PX4 Land 모드라 Offboard 가 아니므로 뺌)
SAFETY = [
    (FLYING,   'px4_takeover', DONE),     # PX4 모드 변경(사람 · PX4 비상 동작) → 재요청 없이 명령 중단 (조종권 이양)
    ((AVOID,), 'dead_end',     LAND),     # 막다른 곳(양옆 막힘 · 재탐색 뒤에도 막힘 · 30초) → 착륙
]


def next_state(state, events, table=TRANSITIONS):
    """표를 위 줄부터 읽어, 현재 상태 + 발생한 사건에 해당하는 첫 줄을 찾는다.
       반환값 = (다음 상태, 그 사건). 해당 줄이 없으면 (현재 상태, None)
       미션 노드는 table = SAFETY + TRANSITIONS 로 불러 안전장치를 먼저 본다"""
    for froms, event, to in table:
        if state in froms and event in events:
            return to, event
    return state, None


def row_number(state, event, table=TRANSITIONS):
    """현재 상태 + 사건에 해당하는 첫 줄의 번호 (1부터, 없으면 None) — 상태도 창에 '몇 번 줄' 을 보여 줄 때"""
    for i, (froms, ev, _) in enumerate(table, 1):
        if state in froms and ev == event:
            return i
    return None


def rule_label(state, event, table=TRANSITIONS):
    """전이 이름표 — 표의 줄이면 줄 번호(int), 표 밖 안전장치면 'safety', 둘 다 아니면 None
       (전이 로그 · /sjcu/mission_transition · 상태도 창이 쓴다)"""
    for froms, ev, _ in SAFETY:
        if state in froms and ev == event:
            return 'safety'
    return row_number(state, event, table)


def check_table(table=TRANSITIONS):
    """표 점검 — 비행 전에 표에서 찾을 수 있는 결함 두 가지. 반환값 = 상태 이름 목록 2개
       unreachable : IDLE 에서 도달할 수 없는 상태
       no_finish   : DONE 에 도달할 수 없는 상태 (교착 상태 — 나가는 줄이 하나도 없는 상태 포함)
       ※ 점검을 통과해도 결함이 없다는 뜻은 아니다
         (예: 9번 줄만 삭제하면 통과하지만, 센서가 복구되지 않으면 HOLD 에서 무한 대기)"""
    out = {s: set() for s in STATES}       # 상태 → 갈 수 있는 상태들
    back = {s: set() for s in STATES}      # 상태 → 그 상태로 오는 상태들
    for froms, _, to in table:
        for s in froms:
            out[s].add(to)
            back[to].add(s)

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
    return unreachable, no_finish
