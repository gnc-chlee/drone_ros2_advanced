#!/usr/bin/env python3
"""
Drone Controller - YOLO Person Following
==========================================
PX4Base를 상속받아 YOLO 감지 결과로 사람을 따라가는 드론 제어 노드

제어 흐름:
  1. Offboard heartbeat 유지 (PX4Base에서 자동)
  2. YOLO 감지 토픽 수신
  3. 이미지 중심 기준으로 오차 계산
  4. 속도 명령으로 변환 (Simple P controller)
  5. PX4로 전송
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition

from .px4_base import PX4Base, PX4_QOS


# ================================================================
# 제어 파라미터 (튜닝 영역)
# ================================================================
IMAGE_WIDTH  = 640   # 카메라 해상도 (픽셀)
IMAGE_HEIGHT = 480

# P 게인 (오차 → 속도 변환 비율)
KP_XY  = 0.005   # 좌우/상하 게인
KP_Z   = 0.003   # 고도 유지 게인

# 속도 제한 (m/s)
MAX_VX = 2.0
MAX_VY = 2.0
MAX_VZ = 1.0

# 목표 Bounding Box 크기 (이 크기를 유지하며 거리 유지)
TARGET_BOX_HEIGHT = 200.0   # 픽셀

# Offboard 진입 카운터 임계값
OFFBOARD_THRESHOLD = 10


class DroneController(PX4Base):
    def __init__(self):
        super().__init__('drone_controller')

        # ============================================================
        # [USER DEFINE] Subscribers - 필요한 토픽 선택
        # ============================================================

        # YOLO 감지 결과 수신 [x_center, y_center, width, height, conf]
        self.detection_sub = self.create_subscription(
            Float32MultiArray,
            '/yolo/person_detection',
            self._detection_callback,
            10
        )

        # 로컬 포지션 (고도 유지용)
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_pos_callback,
            PX4_QOS
        )

        # ============================================================
        # [USER DEFINE] 상태 변수
        # ============================================================
        self.local_position = VehicleLocalPosition()
        self.detection      = None       # 최신 YOLO 감지 결과
        self.is_following   = False      # Following 활성화 여부
        self.target_altitude = -5.0      # 목표 고도 (NED, 음수가 위)

        # ============================================================
        # [USER DEFINE] Offboard heartbeat 모드 변경
        # velocity 제어 사용 → heartbeat에서 velocity=True로 변경
        # ============================================================
        # 기존 heartbeat 타이머 취소 후 재설정
        self.heartbeat_timer.cancel()
        self.heartbeat_timer = self.create_timer(
            0.1, self._velocity_heartbeat
        )

        # ============================================================
        # [USER DEFINE] 주기 실행 타이머
        # ============================================================
        self.create_timer(0.1, self.on_update)   # 10Hz 제어 루프

        self.get_logger().info('DroneController 초기화 완료')
        self.get_logger().info('YOLO Following 대기 중...')

    # ============================================================
    # [USER DEFINE] Subscriber 콜백
    # ============================================================
    def _detection_callback(self, msg: Float32MultiArray):
        """YOLO 감지 결과 수신"""
        self.detection = msg.data  # [x, y, w, h, conf]

    def _local_pos_callback(self, msg: VehicleLocalPosition):
        """로컬 포지션 업데이트"""
        self.local_position = msg

    # ============================================================
    # [USER DEFINE] Heartbeat 오버라이드 (velocity 제어용)
    # ============================================================
    def _velocity_heartbeat(self):
        """
        velocity 제어를 위한 heartbeat
        position=False, velocity=True
        """
        from px4_msgs.msg import OffboardControlMode
        msg = OffboardControlMode()
        msg.position     = False
        msg.velocity     = True
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)
        self.offboard_counter += 1

    # ============================================================
    # [USER DEFINE] 메인 제어 루프
    # ============================================================
    def on_update(self):
        """10Hz 제어 루프"""

        # Offboard 모드 진입 (충분한 heartbeat 후)
        if self.offboard_counter == OFFBOARD_THRESHOLD:
            self.set_offboard_mode()
            self.arm()
            self.get_logger().info('Offboard 모드 진입 + Arm!')

        # Offboard 진입 전이면 리턴
        if self.offboard_counter < OFFBOARD_THRESHOLD:
            return

        # YOLO 감지 결과 없으면 호버링
        if self.detection is None:
            self._hover()
            return

        # Following 제어
        self._follow_person()

    # ============================================================
    # [USER DEFINE] 제어 함수
    # ============================================================
    def _follow_person(self):
        """
        YOLO Bounding Box 기준으로 사람 Following
        
        좌표계:
          - vx: 전진/후진 (사람과의 거리 유지)
          - vy: 좌/우    (사람이 이미지 중앙에 오도록)
          - vz: 상/하    (고도 유지)
        """
        x_center, y_center, w, h, conf = self.detection

        # 이미지 중심 대비 오차 계산
        x_error = x_center - (IMAGE_WIDTH  / 2.0)   # 좌우 오차 (픽셀)
        y_error = y_center - (IMAGE_HEIGHT / 2.0)   # 상하 오차 (픽셀)

        # 거리 오차 (Bounding Box 높이 기준)
        dist_error = h - TARGET_BOX_HEIGHT           # 양수: 너무 가까움

        # P 제어 → 속도 변환
        vy = -KP_XY * x_error    # 좌우 (이미지 x → vy)
        vz = -KP_XY * y_error    # 상하 (이미지 y → vz)
        vx =  KP_XY * dist_error # 전후 (거리 유지)

        # 속도 제한 (클램핑)
        vx = max(-MAX_VX, min(MAX_VX, vx))
        vy = max(-MAX_VY, min(MAX_VY, vy))
        vz = max(-MAX_VZ, min(MAX_VZ, vz))

        self.send_velocity(vx, vy, vz)

        self.get_logger().debug(
            f'Following | err_x={x_error:.1f} err_y={y_error:.1f} '
            f'dist_err={dist_error:.1f} | '
            f'vx={vx:.2f} vy={vy:.2f} vz={vz:.2f}'
        )

    def _hover(self):
        """사람 감지 없을 때 제자리 호버링"""
        self.send_velocity(0.0, 0.0, 0.0)
        self.get_logger().info('사람 없음 - 호버링 중...')

    # ============================================================
    # [USER DEFINE] 추가 기능 (필요시 확장)
    # ============================================================
    def start_following(self):
        """Following 시작"""
        self.is_following = True
        self.get_logger().info('Following 시작!')

    def stop_following(self):
        """Following 중지 후 착륙"""
        self.is_following = False
        self.land()
        self.get_logger().info('Following 중지 - 착륙!')


# ================================================================
# 메인
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = DroneController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 중...')
        node.land()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()