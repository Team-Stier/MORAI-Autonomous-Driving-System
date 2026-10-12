cmake_minimum_required(VERSION 3.0.2)
project(object_detector)
find_package(catkin REQUIRED COMPONENTS node_pipeline interfaces tf2_msgs sensor_msgs pcl_conversions
  lanelet2_core lanelet2_io lanelet2_routing lanelet2_traffic_rules)
find_package(PCL REQUIRED COMPONENTS common filters kdtree search)
find_package(pugixml REQUIRED CONFIG)
get_target_property(pugixml_INCLUDE_DIRS pugixml INTERFACE_INCLUDE_DIRECTORIES)
np_add_nodes(INCLUDE_DIRS ${PCL_INCLUDE_DIRS} ${pugixml_INCLUDE_DIRS} LIBRARIES ${PCL_LIBRARIES} pugixml)
