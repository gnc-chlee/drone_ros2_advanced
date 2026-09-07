#!/usr/bin/env python3
# ==============================================================================
# File    : precision_land_ab.py  (6주차 2강)
# Author  : Choonghyun Lee (gnc-chlee)
# Date    : 2026-06-08
# Version : 1.0.0
#
# Description:
#   Arbitrator + ArUco 정밀 착륙 제어 노드
#   keyboard_control_ab.py와 함께 동작
#
#   상태머신:
#     MANUAL   → 키보드 cmd 그대로 PX4 전달
#     ALIGN    → x,y 오차 줄이며 마커 위 정렬
#     DESCEND  → 정렬 유지하며 하강
#     LANDED   → 착지 감지 → disarm
#
#   우선순위: LAND > MANUAL
#
#   구독 토픽:
#     /sjcu/cmd                       : [vx,vy,vz,yaw_rate,hover_flag,target_z]  (keyboard_control_ab)
#     /sjcu/mode                      : "manual" / "land"                          (keyboard_control_ab)
#     /sjcu/error                     : [x_error, y_error, z_error]  (w05_aruco, 마커 미검출 시 발행 없음)
#                                        x_error = 마커중심x − 화면중심x [px, 오른쪽 +]
#                                        y_error = 마커중심y − 화면중심y [px, 아래 +]
#                                        z_error = 마커 변 길이 평균(px) − target_marker_size
#     /fmu/out/vehicle_local_position
#     /fmu/out/vehicle_land_detected  : 착지 감지 → disarm
#
#   퍼블리시 토픽:
#     /fmu/in/offboard_control_mode   : (현재 미사용 — heartbeat 는 keyboard_control_ab 가 발행)
#     /fmu/in/trajectory_setpoint
#     /fmu/in/vehicle_command
#
#   동작 조합: w05_aruco + w06_keyboard_ab + w06_precision_land
#
#   실행 방법 (터미널 6개, 순서대로):
#     터미널 1: cd ~/PX4-Autopilot && PX4_GZ_WORLD=aruco make px4_sitl gz_x500_mono_cam_down
#     터미널 2: MicroXRCEAgent udp4 -p 8888
#     터미널 3: ros2 run drone_ros2_advanced w05_camera_bridge     # Gazebo 이미지 → /camera/image_raw
#     터미널 4: ros2 run drone_ros2_advanced w05_aruco
#     터미널 5: ros2 run drone_ros2_advanced w06_keyboard_ab
#     터미널 6: ros2 run drone_ros2_advanced w06_precision_land     # ← 이 노드
#
# Repository:
#   https://github.com/gnc-chlee/drone_ros2_advanced
#
# License : MIT
# ==============================================================================

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy,
    HistoryPolicy, DurabilityPolicy,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleLandDetected,
)
from std_msgs.msg import String, Float32MultiArray


# ================================================================
# QoS
# ================================================================
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

# ================================================================
# 상태
# ================================================================
class State:
    MANUAL  = 'MANUAL'
    ALIGN   = 'ALIGN'
    DESCEND = 'DESCEND'
    LANDED  = 'LANDED'

# ================================================================
# 파라미터
# ================================================================
KP_XY        = 0.1  # x,y 오차 → 속도 게인
MAX_VXY      = 0.2     # [m/s]
DESCEND_VZ   = 0.2     # [m/s] 하강 속도
ALIGN_THRESH = 30.0    # [pixel] 정렬 완료 임계값
ALIGN_HOLD   = 1.0     # [s] 정렬 유지 시간
DETECT_TIMEOUT = 2.0   # [s] 마커 감지 타임아웃
LAND_ALT     = 0.4     # [m] 이 고도 이하면 착지 명령 (현재 미사용 — 착지는 vehicle_land_detected 로 판단)


class PrecisionLandAB(Node):
    def __init__(self):
        super().__init__('precision_land_ab')

        # ================================================================
        # Publishers
        # ================================================================
        # (현재 미사용 — Offboard heartbeat 는 keyboard_control_ab 가 발행)
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            PX4_QOS
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            PX4_QOS
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            PX4_QOS
        )

        # ================================================================
        # Subscribers
        # ================================================================
        self.cmd_sub = self.create_subscription(
            Float32MultiArray, '/sjcu/cmd',
            self._cmd_callback, 10
        )
        self.mode_sub = self.create_subscription(
            String, '/sjcu/mode',
            self._mode_callback, 10
        )
        # /sjcu/error = [x_error, y_error, z_error]  (w05_aruco 가 발행, 마커 미검출 시 발행 없음)
        #   x_error : 마커중심x − 화면중심x [px, 오른쪽 +]
        #   y_error : 마커중심y − 화면중심y [px, 아래 +]
        #   z_error : 마커 변 길이 평균(px) − target_marker_size  (이 노드에선 사용 안 함)
        self.error_sub = self.create_subscription(
            Float32MultiArray, '/sjcu/error',
            self._error_callback, 10
        )
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self._local_pos_callback, PX4_QOS
        )
        self.land_detected_sub = self.create_subscription(
            VehicleLandDetected,
            '/fmu/out/vehicle_land_detected',
            self._land_detected_callback, PX4_QOS
        )

        # ================================================================
        # 상태 변수
        # ================================================================
        self.state          = State.MANUAL
        self.current_mode   = 'manual'

        # keyboard cmd
        self.kbd_vx         = 0.0
        self.kbd_vy         = 0.0
        self.kbd_vz         = 0.0
        self.kbd_yaw_rate   = 0.0
        self.kbd_hovering   = False
        self.kbd_target_z   = 0.0

        # aruco error
        self.error              = None
        self._last_detect_time  = self.get_clock().now()
        self._align_start_time  = None

        # local position
        self.local_position = VehicleLocalPosition()
        self.is_landed      = False

        # ================================================================
        # 제어 루프 (20Hz)
        # ================================================================
        self.create_timer(0.05, self.on_update)

        self.get_logger().info('PrecisionLandAB (Arbitrator) 초기화 완료')
        self.get_logger().info('keyboard_control_ab 노드를 같이 실행하세요!')

    # ================================================================
    # Subscriber 콜백
    # ================================================================
    def _cmd_callback(self, msg: Float32MultiArray):
        d = msg.data
        self.kbd_vx       = d[0]
        self.kbd_vy       = d[1]
        self.kbd_vz       = d[2]
        self.kbd_yaw_rate = d[3]
        self.kbd_hovering = bool(d[4])
        self.kbd_target_z = d[5]

    def _mode_callback(self, msg: String):
        prev = self.current_mode
        self.current_mode = msg.data

        if msg.data == 'land' and prev != 'land':
            self.state             = State.ALIGN
            self._align_start_time = None
            self.get_logger().info('=== 정밀 착륙 모드 → ALIGN ===')

        elif msg.data == 'manual' and prev != 'manual':
            self.state = State.MANUAL
            self.get_logger().info('=== 수동 모드 복귀 → MANUAL ===')

    def _error_callback(self, msg: Float32MultiArray):
        self.error             = msg.data
        self._last_detect_time = self.get_clock().now()

    def _local_pos_callback(self, msg: VehicleLocalPosition):
        self.local_position = msg

    def _land_detected_callback(self, msg: VehicleLandDetected):
        if msg.landed and self.state == State.DESCEND:
            self.get_logger().info('착지 감지! → Disarm')
            self.state     = State.LANDED
            self.is_landed = True
            self._disarm()

    # ================================================================
    # 메인 제어 루프 (20Hz)
    # ================================================================
    def on_update(self):
        """
        Arbitrator: 모드에 따라 setpoint 선택

        MANUAL  → keyboard cmd 그대로 전달
        ALIGN   → ArUco 오차로 x,y 정렬
        DESCEND → 정렬 유지 + 하강
        LANDED  → 아무것도 안 함
        """
        if self.state == State.LANDED:
            return

        if   self.state == State.MANUAL:  self._do_manual()
        elif self.state == State.ALIGN:   self._do_align()
        elif self.state == State.DESCEND: self._do_descend()

    # ================================================================
    # MANUAL 모드
    # ================================================================
    def _do_manual(self):
        """키보드 cmd 그대로 PX4 setpoint로 전달 (heartbeat는 keyboard가 담당)"""
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        setpoint           = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.yaw       = float('nan')
        setpoint.yawspeed  = float(self.kbd_yaw_rate)

        if self.kbd_hovering:
            setpoint.position = [float('nan'), float('nan'), float(self.kbd_target_z)]
            setpoint.velocity = [0.0, 0.0, 0.0]
            setpoint.yawspeed = 0.0
        else:
            setpoint.position = [float('nan'), float('nan'), float('nan')]
            setpoint.velocity = [self.kbd_vx, self.kbd_vy, self.kbd_vz]

        # setpoint 는 이 노드가 발행 (heartbeat 는 keyboard_control_ab 담당)
        self.trajectory_pub.publish(setpoint)

    # ================================================================
    # ALIGN 모드
    # ================================================================
    def _do_align(self):
        """ArUco 오차로 x,y 정렬"""
        elapsed = (
            self.get_clock().now() - self._last_detect_time
        ).nanoseconds * 1e-9

        if self.error is None or elapsed > DETECT_TIMEOUT:
            self.get_logger().warn(
                f'마커 감지 타임아웃 ({elapsed:.1f}s) → 호버링',
                throttle_duration_sec=2.0
            )
            self._hover()
            self._align_start_time = None
            return

        x_error, y_error, _ = self.error

        # 하방 카메라 좌표계 → body frame
        # 이미지 x(+우) → 드론 y(+우)
        # 이미지 y(+아래) → 드론 x(+전방)
        vy_body =  KP_XY * x_error
        vx_body =  - KP_XY * y_error
        vz      =  0.0

        vx_body = max(-MAX_VXY, min(MAX_VXY, vx_body))
        vy_body = max(-MAX_VXY, min(MAX_VXY, vy_body))

        self._send_velocity_body(vx_body, vy_body, vz)

        # 정렬 완료 판단
        aligned = (
            abs(x_error) < ALIGN_THRESH and
            abs(y_error) < ALIGN_THRESH
        )

        if aligned:
            if self._align_start_time is None:
                self._align_start_time = self.get_clock().now()
                self.get_logger().info(f'정렬 완료! {ALIGN_HOLD}s 유지 대기...')
            else:
                held = (
                    self.get_clock().now() - self._align_start_time
                ).nanoseconds * 1e-9
                if held >= ALIGN_HOLD:
                    self.state = State.DESCEND
                    self.get_logger().info('정렬 유지 완료 → DESCEND')
        else:
            self._align_start_time = None

        self.get_logger().debug(
            f'ALIGN | ex={x_error:.0f} ey={y_error:.0f}'
        )

    # ================================================================
    # DESCEND 모드
    # ================================================================
    def _do_descend(self):
        """정렬 유지하며 하강 + 착지 감지"""
        elapsed = (
            self.get_clock().now() - self._last_detect_time
        ).nanoseconds * 1e-9

        if self.error is None or elapsed > DETECT_TIMEOUT:
            self.get_logger().warn('마커 감지 타임아웃 → ALIGN 복귀')
            self.state             = State.ALIGN
            self._align_start_time = None
            return

        x_error, y_error, _ = self.error

        # 정렬 이탈 시 복귀
        if abs(x_error) > ALIGN_THRESH * 2 or abs(y_error) > ALIGN_THRESH * 2:
            self.state             = State.ALIGN
            self._align_start_time = None
            self.get_logger().warn('정렬 이탈 → ALIGN 복귀')
            return

        # 정렬 유지 + 하강
        vy_body =  KP_XY * x_error
        vx_body =  - KP_XY * y_error
        vz      =  DESCEND_VZ   # NED: 양수 = 하강

        vx_body = max(-MAX_VXY, min(MAX_VXY, vx_body))
        vy_body = max(-MAX_VXY, min(MAX_VXY, vy_body))

        self._send_velocity_body(vx_body, vy_body, vz)

        current_alt = abs(self.local_position.z)
        self.get_logger().debug(
            f'DESCEND | alt={current_alt:.2f}m '
            f'ex={x_error:.0f} ey={y_error:.0f}'
        )

    # ================================================================
    # 헬퍼 함수
    # ================================================================
    def _send_velocity_body(self, vx_body, vy_body, vz, yaw_rate=0.0):
        """body 속도를 velocity setpoint 로 발행 (※ yaw 회전 없이 NED 에 그대로 넣음)"""
        # ※ yaw=0 가정 — 기수를 돌린 상태에서는 보정 방향이 어긋남.
        #    PX4Base.send_velocity_body 처럼 current_yaw 회전이 필요 (base판에서 해결 예정)

        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint           = TrajectorySetpoint()
        setpoint.timestamp = timestamp
        setpoint.position  = [float('nan'), float('nan'), float('nan')]
        setpoint.velocity  = [float(vx_body), float(vy_body), float(vz)]
        setpoint.yaw       = float('nan')
        setpoint.yawspeed  = float(yaw_rate)
        self.trajectory_pub.publish(setpoint)

    def _hover(self):
        """호버링"""
        self._send_velocity_body(0.0, 0.0, 0.0)

    def _disarm(self):
        """Disarm 명령"""
        msg = VehicleCommand()
        msg.command          = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1           = 0.0
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
        self.get_logger().info('Disarm 완료!')

    def _publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        # (현재 미사용 — _disarm 이 직접 VehicleCommand 를 조립함)
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = param1
        msg.param2           = param2
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)


# ================================================================
# 메인
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandAB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('종료 중...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()