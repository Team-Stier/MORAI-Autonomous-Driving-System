cmake_minimum_required(VERSION 3.0.2)
project(morai_tf)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs roscpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros
)

catkin_package(CATKIN_DEPENDS
  geometry_msgs roscpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(map_velodyne_tf_node src/map_velodyne_tf_node.cpp)
target_compile_features(map_velodyne_tf_node PUBLIC cxx_std_14)
target_link_libraries(map_velodyne_tf_node ${catkin_LIBRARIES})

install(TARGETS map_velodyne_tf_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
