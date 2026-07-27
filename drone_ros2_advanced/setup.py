from setuptools import find_packages, setup

package_name = 'drone_ros2_advanced'

setup(
    name=package_name,
    version='2.0.0',
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
    description='로봇운영체제(ROS2)응용 주차별 실습 패키지',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # ── 2주차: 노드 기초 + 키보드 제어 ───────────────────
            'first_node = drone_ros2_advanced.week02.first_node:main',
            'position_listener = drone_ros2_advanced.week02.position_listener:main',
            'keyboard_control = drone_ros2_advanced.week02.keyboard_control:main',

            # ── 3주차: 단일·다중 Waypoint ────────────────────────
            'w03_takeoff_raw = drone_ros2_advanced.week03.takeoff_single_raw:main',
            'w03_takeoff_base = drone_ros2_advanced.week03.takeoff_single_base:main',
            'w03_multi_raw = drone_ros2_advanced.week03.multi_position_raw:main',
            'w03_multi_base = drone_ros2_advanced.week03.multi_position_base:main',
            'w03_yaml_raw = drone_ros2_advanced.week03.waypoint_yaml_raw:main',
            'w03_yaml_base = drone_ros2_advanced.week03.waypoint_yaml_base:main',
            'w03_mission_raw = drone_ros2_advanced.week03.waypoint_mission_raw:main',
            'w03_mission_base = drone_ros2_advanced.week03.waypoint_mission_base:main',

            # ── 5주차: 카메라 · OpenCV · ArUco ───────────────────
            'w05_camera_viewer = drone_ros2_advanced.week05.camera_viewer:main',
            'w05_contour = drone_ros2_advanced.week05.contour_demo:main',
            'w05_aruco = drone_ros2_advanced.week05.aruco_detector:main',

            # ── 6주차: 오차 제어 · 정밀착륙 ──────────────────────
            'w06_center_error = drone_ros2_advanced.week06.center_error_viewer:main',
            'w06_keyboard_v2 = drone_ros2_advanced.week06.keyboard_control_v2:main',
            'w06_keyboard_ab = drone_ros2_advanced.week06.keyboard_control_ab:main',
            'w06_precision_land = drone_ros2_advanced.week06.precision_land_ab:main',

            # ── 8주차: 객체 인식 (Haar 원형, DNN판 추가 예정) ────
            'w08_face_detector = drone_ros2_advanced.week08.face_detector:main',

            # ── 9주차: 사람 추종 비행 ────────────────────────────
            'w09_face_command = drone_ros2_advanced.week09.face_to_command:main',
            'w09_p_control = drone_ros2_advanced.week09.p_control_demo:main',
            'w09_follow_raw = drone_ros2_advanced.week09.face_following_raw:main',
            'w09_follow_base = drone_ros2_advanced.week09.face_following_base:main',
            'w09_drone_controller = drone_ros2_advanced.week09.drone_controller:main',

            # ── 13주차: 추종 안정화 (통합 미션 소재) ─────────────
            'w13_stable_raw = drone_ros2_advanced.week13.face_following_stable_raw:main',
            'w13_stable_base = drone_ros2_advanced.week13.face_following_stable_base:main',

            # ── 14주차: 심화 (YOLO) ──────────────────────────────
            'w14_yolo = drone_ros2_advanced.week14.yolo_detector:main',
            'w14_haar_vs_yolo = drone_ros2_advanced.week14.haar_vs_yolo:main',

            # ── extras: 구 커리큘럼 GPS/지도 (참고용) ────────────
            'extras_gps_viewer = drone_ros2_advanced.extras.gps_ned_viewer:main',
            'extras_map_demo = drone_ros2_advanced.extras.folium_map_demo:main',
            'extras_map_server = drone_ros2_advanced.extras.map_click_server:main',
            'extras_goto_raw = drone_ros2_advanced.extras.goto_gps_raw:main',
            'extras_goto_base = drone_ros2_advanced.extras.goto_gps_base:main',
        ],
    },
)
