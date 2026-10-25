cmake_minimum_required(VERSION 3.0.2)
project(vlp16_bev)
find_package(catkin REQUIRED COMPONENTS rospy interfaces sensor_msgs)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/vlp16_bev_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS launch.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
