from setuptools import find_packages, setup

package_name = 'drone_ros2_advanced'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lch',
    maintainer_email='lch@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'px4_base = drone_ros2_advanced.px4_base:main',
            'drone_controller = drone_ros2_advanced.drone_controller:main',
            'keyboard_control = drone_ros2_advanced.keyboard_control:main',
            'keyboard_control_v2 = drone_ros2_advanced.keyboard_control_v2:main',
            'aruco_detector = drone_ros2_advanced.aruco_detector:main',
            'precision_land_ab = drone_ros2_advanced.precision_land_ab:main',
            'keyboard_control_ab = drone_ros2_advanced.keyboard_control_ab:main',
        ],
    },
)
