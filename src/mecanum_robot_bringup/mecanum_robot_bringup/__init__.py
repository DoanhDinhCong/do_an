"""
Mecanum Robot Bringup Package
==============================
Package chứa các nodes Python cho robot mecanum 4 bánh với dual lidar.

Nodes:
- velocity_bridge: Giao tiếp STM32 qua Serial
- mecanum_odom_real: Tính odometry từ encoder
- laser_scan_merger: Merge 2 lidar thành 1 scan
- hwt901b_driver: Driver IMU HWT901B
- test_odometry_comparison: Test so sánh odometry
"""

__version__ = '1.0.0'
