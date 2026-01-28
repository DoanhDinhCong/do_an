from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'mecanum_robot_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro')) + glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
        (os.path.join('share', package_name, 'maps'),
            glob(os.path.join('maps', '*.yaml')) + glob(os.path.join('maps', '*.pgm'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dcd',
    maintainer_email='doanha1k12@gmail.com',
    description='Mecanum robot bringup package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'velocity_bridge.py = mecanum_robot_bringup.velocity_bridge:main',
            'mecanum_odom_real.py = mecanum_robot_bringup.mecanum_odom_real:main',
            'laser_scan_merger.py = mecanum_robot_bringup.laser_scan_merger:main',
            'hwt901b_driver.py = mecanum_robot_bringup.hwt901b_driver:main',
            'test_odometry_comparison.py = mecanum_robot_bringup.test_odometry_comparison:main',
            'circle_path_gui.py = mecanum_robot_bringup.circle_path_gui:main',
        ],
    },
)
