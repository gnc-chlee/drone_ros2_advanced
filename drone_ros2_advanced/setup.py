from setuptools import find_packages, setup

package_name = 'drone_ros2_advanced'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Choonghyun Lee',
    maintainer_email='chungh6577@gmail.com',
    description='드론로봇프로그래밍심화(ROS2) 주차별 실습 패키지',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ── 기존 노드 ────────────────────────────────────────
            'drone_controller = drone_ros2_advanced.drone_controller:main',
            'keyboard_control = drone_ros2_advanced.keyboard_control:main',
            'keyboard_control_v2 = drone_ros2_advanced.keyboard_control_v2:main',
            'aruco_detector = drone_ros2_advanced.aruco_detector:main',
            'precision_land_ab = drone_ros2_advanced.precision_land_ab:main',
            'keyboard_control_ab = drone_ros2_advanced.keyboard_control_ab:main',

            # ── 1주차: Offboard 이륙 + 단일 position ─────────────
            'w01_takeoff_raw = drone_ros2_advanced.week01.takeoff_single_raw:main',
            'w01_takeoff_base = drone_ros2_advanced.week01.takeoff_single_base:main',

            # ── 2주차: 다중 position 이동 ────────────────────────
            'w02_multi_raw = drone_ros2_advanced.week02.multi_position_raw:main',
            'w02_multi_base = drone_ros2_advanced.week02.multi_position_base:main',

            # ── 3주차: Waypoint YAML 관리 ────────────────────────
            'w03_yaml_raw = drone_ros2_advanced.week03.waypoint_yaml_raw:main',
            'w03_yaml_base = drone_ros2_advanced.week03.waypoint_yaml_base:main',

            # ── 4주차: 도착 판정 + 자동 착륙 ─────────────────────
            'w04_mission_raw = drone_ros2_advanced.week04.waypoint_mission_raw:main',
            'w04_mission_base = drone_ros2_advanced.week04.waypoint_mission_base:main',

            # ── 5주차: GPS ↔ NED, Folium 지도 ────────────────────
            'w05_gps_viewer = drone_ros2_advanced.week05.gps_ned_viewer:main',
            'w05_map_demo = drone_ros2_advanced.week05.folium_map_demo:main',

            # ── 6주차: 지도 클릭 → waypoint 비행 ─────────────────
            'w06_map_server = drone_ros2_advanced.week06.map_click_server:main',
            'w06_goto_raw = drone_ros2_advanced.week06.goto_gps_raw:main',
            'w06_goto_base = drone_ros2_advanced.week06.goto_gps_base:main',

            # ── 8주차: 카메라 + OpenCV contour ───────────────────
            'w08_camera_viewer = drone_ros2_advanced.week08.camera_viewer:main',
            'w08_contour = drone_ros2_advanced.week08.contour_demo:main',

            # ── 9주차: 카메라 기반 제어 개념 ─────────────────────
            'w09_center_error = drone_ros2_advanced.week09.center_error_viewer:main',

            # ── 10주차: Haar 얼굴 인식 ───────────────────────────
            'w10_face_detector = drone_ros2_advanced.week10.face_detector:main',
            'w10_face_command = drone_ros2_advanced.week10.face_to_command:main',

            # ── 11주차: 비례(P) 제어 ─────────────────────────────
            'w11_p_control = drone_ros2_advanced.week11.p_control_demo:main',

            # ── 12주차: Following 비행 ───────────────────────────
            'w12_follow_raw = drone_ros2_advanced.week12.face_following_raw:main',
            'w12_follow_base = drone_ros2_advanced.week12.face_following_base:main',

            # ── 13주차: Following 안정화 ─────────────────────────
            'w13_stable_raw = drone_ros2_advanced.week13.face_following_stable_raw:main',
            'w13_stable_base = drone_ros2_advanced.week13.face_following_stable_base:main',

            # ── 14주차: YOLOv8n ──────────────────────────────────
            'w14_yolo = drone_ros2_advanced.week14.yolo_detector:main',
            'w14_haar_vs_yolo = drone_ros2_advanced.week14.haar_vs_yolo:main',
        ],
    },
)
