cmake_minimum_required(VERSION 2.8.3)
project(razor_imu_9dof)

find_package(catkin REQUIRED COMPONENTS genmsg std_msgs roscpp message_generation)

add_message_files(FILES RazorImu.msg)

generate_messages(DEPENDENCIES std_msgs) 

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS nodes/node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS nodes/display_3D_visualization.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
