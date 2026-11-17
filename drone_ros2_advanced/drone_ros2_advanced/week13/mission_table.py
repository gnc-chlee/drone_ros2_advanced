#!/usr/bin/env python3
# ==============================================================================
# File    : mission_table.py  (13주차 23·24강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-11-17
# Version : 1.0.0
#
# Description:
#   통합 미션 「주차장 지정 칸 착륙」의 전이 조건표 (state transition table)
#   12주차 표(week12/mission_table.py)에 줄을 더한 것
#     1~10번 줄은 번호 그대로 · 5번 줄은 다음 상태만 LAND → ALIGN · 새 줄 11~14 (정밀착륙)
#   23강 시뮬레이터(fsm_sim)와 24강 미션 노드가 이 표 하나를 같이 쓴다
#   → 표를 고치면 두 곳의 동작이 함께 바뀐다. ROS 없는 순수 파이썬
#
#   읽는 법: 한 줄 = 현재 상태 하나 + 사건 하나 → 다음 상태 하나 (= 상태도의 화살표 하나)
#     해당 줄이 없으면 현재 상태 유지
#     같은 순간 사건이 여러 개면 위에 있는 줄 하나만 쓴다
#   미션 노드는 표보다 먼저 표 밖 안전장치 2줄(SAFETY)을 본다 — 23강 표 · 시뮬레이터 · 상태도 그림에는 없음
#
#   표를 읽는 함수 next_state · row_number 는 12주차 파일의 것을 그대로 쓴다 (import — 복사하지 않음)
#     두 함수의 표 인자 기본값은 12주차 10줄 표 → 13주차 코드는 표를 꼭 넘긴다
#     (넘기지 않으면 CRUISE + arrived → LAND — 12주차 5번 줄)
#
#   같은 순간 사건 두 개 (위 줄이 먼저 — 시험으로 고정):
#     CRUISE + arrived · front_blocked  → AVOID   (3번 줄)
#     CRUISE + arrived · sensor_bad     → ALIGN   (5번 줄 — 정렬 · 하강은 스캔을 쓰지 않음)
#     HOLD   + hold_timeout · sensor_ok → LAND    (9번 줄)
#     ALIGN  + aligned · align_timeout  → DESCEND (11번 줄)
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

from ..week12.mission_table import next_state, row_number  # noqa: F401  (12주차 함수 그대로 — 표를 꼭 넘긴다)

# ── 상태 9개 (12주차 7개 + ALIGN · DESCEND) ────────────────────────
IDLE    = 'IDLE'      # 준비: 연결 확인 후 Offboard + Arm 요청
TAKEOFF = 'TAKEOFF'   # 이륙: 제자리에서 고도 2 m 까지
CRUISE  = 'CRUISE'    # 순항: 동쪽으로 전진 (감속 · 코스 복귀 포함)
AVOID   = 'AVOID'     # 회피: 앞이 막힘 → 넓은 쪽으로 좌우 이동
HOLD    = 'HOLD'      # 대기: 센서 신호 끊김 → 수평 정지 · 고도 유지
                      #       (노드 안의 상태 — PX4 는 Offboard 유지, PX4 Hold 모드 아님)
ALIGN   = 'ALIGN'     # 정렬: 목표 칸 마커 바로 위로 (마커가 안 보이면 수평 정지 · 고도 유지)
DESCEND = 'DESCEND'   # 하강: 정렬을 유지하며 고도 1 m 까지
LAND    = 'LAND'      # 착륙: PX4 Land 모드에 맡김
DONE    = 'DONE'      # 끝: 명령 중단

STATES = [IDLE, TAKEOFF, CRUISE, AVOID, HOLD, ALIGN, DESCEND, LAND, DONE]

# ── 사건 13개: (구분, 이름, 뜻) — 시뮬레이터에서 칠 수 있는 사건은 이 13개뿐 ──────
#    정확한 조건은 오른쪽 주석 (24강 미션 노드가 이 조건으로 사건을 판정한다)
EVENTS = [
    ('미션',       'ready',         'Offboard + Arm 완료'),   # Offboard + Arm 확인
    ('미션',       'airborne',      '이륙 고도 도달'),         # 상대 고도 2 m ± 0.2
    ('미션',       'front_blocked', '앞 1.5 m 미만'),
    ('미션',       'front_clear',   '앞 2.0 m 이상'),
    ('미션',       'arrived',       '목표 도착'),              # 동쪽 (목표 지점 - 0.3) m 넘음 — 0번 칸 11.7 m
    ('미션',       'landed',        '착지'),                   # 착지 감지
    ('페일세이프', 'sensor_bad',    '센서 신호 끊김'),         # 유효한 스캔 0.5초 없음 또는 -inf
    ('페일세이프', 'sensor_ok',     '신호 1초 이어짐'),        # 유효한 스캔 1.0초 이어짐
    ('페일세이프', 'hold_timeout',  'HOLD 20초'),              # HOLD 타임아웃 (파라미터 hold_timeout, 기본 20초)
    ('정밀착륙',   'aligned',       '마커 정렬 완료'),         # 마커 오차(고도 보정)가 화면 반폭의 5 % 미만으로 1.0초 유지
    ('정밀착륙',   'marker_lost',   '마커 상실'),              # 마커 오차 0.5초 없음, 또는 하강 정체
                                                              #   (하강 정체 = DESCEND 에서 5초 동안 고도가 0.1 m 도 줄지 않음)
    ('정밀착륙',   'descended',     '하강 완료'),              # 상대 고도 1.0 m 이하 + 정렬 오차 5 % 미만
    ('정밀착륙',   'align_timeout', 'ALIGN 30초'),             # ALIGN 에 머문 시간의 합 (파라미터 align_timeout, 기본 30초)
]

# ================================================================
# [USER DEFINE] 전이 조건표 14줄 — (현재 상태, 사건, 다음 상태)
#   한 줄 = 상태도의 화살표 하나. 정상 미션 1~6 · 페일세이프 7~10 · 정밀착륙 11~14
#   12주차 표와 다른 곳: 5번 줄의 다음 상태 (LAND → ALIGN) · 새 줄 11~14
# ================================================================
TRANSITIONS = [
    # 정상 미션: 1 → 2 → 3 → 4 → 5 → (11 → 13) → 6
    ((IDLE,),    'ready',         TAKEOFF),   # 1
    ((TAKEOFF,), 'airborne',      CRUISE),    # 2
    ((CRUISE,),  'front_blocked', AVOID),     # 3  정지 판단 1.5 m
    ((AVOID,),   'front_clear',   CRUISE),    # 4  해제 2.0 m (둘이 달라서 히스테리시스)
    ((CRUISE,),  'arrived',       ALIGN),     # 5  12주차는 LAND — 도착하면 착륙 대신 마커 정렬
    ((LAND,),    'landed',        DONE),      # 6
    # 페일세이프 (12주차와 같음): 센서 신호 끊김 → HOLD 에서 대기 → 복구 시 CRUISE · 타임아웃 시 LAND
    ((CRUISE,),  'sensor_bad',    HOLD),      # 7
    ((AVOID,),   'sensor_bad',    HOLD),      # 8
    ((HOLD,),    'hold_timeout',  LAND),      # 9  대기 상태에는 타임아웃
    ((HOLD,),    'sensor_ok',     CRUISE),    # 10 앞이 막혀 있으면 다음 틱에 AVOID 로
    # 정밀착륙 (통합 미션에서 더한 줄): 정렬 → 하강 → 착륙, 놓치면 정렬로 · 시간 제한 뒤 그 자리 착륙
    #   ALIGN · DESCEND 에는 스캔 줄(sensor_bad)이 없다 — 정렬 · 하강은 카메라만 쓴다
    ((ALIGN,),   'aligned',       DESCEND),   # 11
    ((DESCEND,), 'marker_lost',   ALIGN),     # 12 놓치면 앞 단계(정렬)로
    ((DESCEND,), 'descended',     LAND),      # 13
    ((ALIGN,),   'align_timeout', LAND),      # 14 ALIGN 도 대기 상태 → 타임아웃 줄 (HOLD 의 9번 줄과 같은 모양)
]

# ================================================================
# [FIXED] 표 밖 안전장치 2줄 — 미션 노드가 표보다 먼저 본다
#   (23강 표 · 시뮬레이터 · 상태도 그림에는 없음. 전이 로그에는 줄 번호 대신 '안전장치')
#   12주차와 같은 2줄 — 비행 중 상태(FLYING)에 ALIGN · DESCEND 를 더함
# ================================================================
FLYING = (TAKEOFF, CRUISE, AVOID, HOLD, ALIGN, DESCEND)   # 비행 중 (IDLE 은 Arm 전, LAND 는 PX4 Land 모드라 Offboard 가 아니므로 뺌)
SAFETY = [
    (FLYING,   'px4_takeover', DONE),     # PX4 모드 변경(사람 · PX4 비상 동작) → 재요청 없이 명령 중단 (조종권 이양)
    ((AVOID,), 'dead_end',     LAND),     # 막다른 곳(양옆 막힘 · 재탐색 뒤에도 막힘 · 30초) → 착륙
]


def rule_label(state, event, table=TRANSITIONS):
    """전이 이름표 — 표의 줄이면 줄 번호(int), 표 밖 안전장치면 'safety', 둘 다 아니면 None
       (전이 로그 · /sjcu/mission_transition · 상태도 창이 쓴다)"""
    for froms, ev, _ in SAFETY:
        if state in froms and ev == event:
            return 'safety'
    return row_number(state, event, table)


def check_table(table=TRANSITIONS):
    """표 점검 — 비행 전에 표에서 찾을 수 있는 결함 두 가지. 반환값 = 상태 이름 목록 2개
       unreachable : IDLE 에서 도달할 수 없는 상태 (어떤 사건 순서로도 갈 수 없음 — 그 상태의 일은 실행되지 않는다)
       no_finish   : DONE 에 도달할 수 없는 상태 (교착 상태 — 나가는 줄이 하나도 없는 상태 포함)
       ※ 점검을 통과해도 결함이 없다는 뜻은 아니다
         (예: 14번 줄만 삭제하면 통과하지만, 마커가 끝내 보이지 않으면 ALIGN 에서 무한 대기)"""
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
