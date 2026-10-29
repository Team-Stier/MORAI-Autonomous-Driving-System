cmake_minimum_required(VERSION 3.0.2)
project(visualizer)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(catkin REQUIRED COMPONENTS node_pipeline roscpp rospy geometry_msgs nav_msgs
  visualization_msgs interfaces morai_msgs lanelet2_core lanelet2_io tf2 tf2_geometry_msgs rviz pluginlib)
np_add_nodes(LIBRARIES pugixml Qt5::Widgets)
catkin_install_python(PROGRAMS scripts/object_info_gt.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
configure_file(scripts/udp_receiver.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/udp_receiver.py COPYONLY)
install(FILES scripts/udp_receiver.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES rviz_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS scripts/sudo_askpass.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)
