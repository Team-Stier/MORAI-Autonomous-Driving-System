cmake_minimum_required(VERSION 3.0.2)
project(path_planner)
find_package(catkin REQUIRED COMPONENTS node_pipeline geometry_msgs interfaces nav_msgs sensor_msgs tf2_msgs visualization_msgs
  lanelet2_core lanelet2_io lanelet2_routing lanelet2_traffic_rules)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(pugixml REQUIRED CONFIG)
get_target_property(pugixml_INCLUDE_DIRS pugixml INTERFACE_INCLUDE_DIRECTORIES)
np_add_nodes(INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} ${pugixml_INCLUDE_DIRS} LIBRARIES pugixml OpenMP::OpenMP_CXX)
