cmake_minimum_required(VERSION 2.8.3)
project(razor_imu_9dof)

find_package(catkin REQUIRED)

catkin_package(CATKIN_DEPENDS)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY magnetometer_calibration
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS 
	scripts/node.py
	scripts/display_3D_visualization.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
