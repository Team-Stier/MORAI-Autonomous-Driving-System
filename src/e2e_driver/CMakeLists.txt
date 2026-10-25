cmake_minimum_required(VERSION 3.0.2)
project(e2e_driver)
find_package(catkin REQUIRED COMPONENTS rospy interfaces morai_msgs nav_msgs sensor_msgs std_msgs)
catkin_python_setup()
catkin_package()
catkin_install_python(PROGRAMS scripts/e2e_driver_node.py scripts/latency_probe.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS launch.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
