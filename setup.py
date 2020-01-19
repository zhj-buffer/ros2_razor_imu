from glob import glob
import os
from setuptools import setup

PACKAGE_NAME = "razor_imu_9dof"
SHARE_DIR = os.path.join("share", PACKAGE_NAME)

setup(
    name=PACKAGE_NAME,
    version='1.2.0',
    packages=["nodes", "nodes.lib"],
    data_files=[
        (os.path.join(SHARE_DIR, "launch"), glob(os.path.join("launch", "*.launch.py"))),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*.yaml")))],
    py_modules=[],
    zip_safe=True,
    install_requires=['setuptools',
                      'pyserial',
                      'pyyaml',
                      'transforms3d'],
    author='Kristof Robot, Tang Tiong Yew, Paul Bouchier, Peter Bartz',
    maintainer='Kristof Robot',
    keywords=['ROS2'],
    description='razor_imu_9dof is a package that provides a ROS2 driver for the Sparkfun Razor IMU 9DOF.',
    license='BSD',
    entry_points={
        'console_scripts': ['imu_node = nodes.imu_node:main',
                            'display_3D_visualization_node = nodes.display_3D_visualization:main'
                            ],
    }
)
