cmake_minimum_required(VERSION 2.8.3)
project(robotiq_2f_85_gripper_visualization)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs message_generation)

add_service_files(
  FILES
  Control2F85.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_install_python(
  PROGRAMS
  scripts/simulated_gripper_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package()

install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
