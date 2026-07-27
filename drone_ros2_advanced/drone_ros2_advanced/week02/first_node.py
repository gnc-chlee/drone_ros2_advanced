#!/usr/bin/env python3
# ==============================================================================
# File    : first_node.py
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-07-27
# Version : 1.0.0
#
# Description:
#   내 첫 ROS2 노드 (2주차: 노드의 뼈대 익히기)
#   PX4 없이, 1초마다 인사를 출력하는 가장 단순한 노드.
#
#   ROS2 노드의 4요소:
#     1. rclpy.init()  - ROS2 시작
#     2. Node 클래스   - 노드의 본체 (이름을 가진 프로그램 하나)
#     3. 콜백(callback) - "때가 되면 실행해줘"라고 예약해두는 함수
#     4. rclpy.spin()  - 예약된 콜백들을 계속 실행해주는 무한 루프
#
#   이 뼈대는 이번 학기 모든 노드에서 똑같이 반복된다.
#   position_listener.py는 여기에 Subscriber 하나를 더한 것뿐이다.
#
#   실행 방법:
#     cd ~/ros2_ws && colcon build --packages-select drone_ros2_advanced
#     source install/setup.bash
#     ros2 run drone_ros2_advanced first_node
#
#   다른 터미널에서 확인해보기:
#     ros2 node list        # /first_node 가 보인다
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from rclpy.node import Node


class FirstNode(Node):
    def __init__(self):
        super().__init__('first_node')   # 노드 이름 (ros2 node list에 보이는 이름)

        self.count = 0

        # ── Timer: 1초마다 timer_callback 실행을 "예약" ──────────
        # 예약만 할 뿐, 실제 실행은 main의 spin()이 해준다
        self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('FirstNode 시작!')

    def timer_callback(self):
        """1초마다 자동으로 실행된다 (내가 직접 호출하지 않는다!)"""
        self.count += 1
        self.get_logger().info(f'안녕하세요! {self.count}초 경과')


def main(args=None):
    rclpy.init(args=args)          # 1. ROS2 시작
    node = FirstNode()             # 2. 노드 생성
    try:
        rclpy.spin(node)           # 3. 콜백을 계속 처리 (Ctrl+C까지 여기서 대기)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()        # 4. 정리
        rclpy.shutdown()


if __name__ == '__main__':
    main()
