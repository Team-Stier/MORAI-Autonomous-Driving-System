cmake_minimum_required(VERSION 3.0.2)
project(control)
find_package(catkin REQUIRED COMPONENTS node_pipeline geometry_msgs interfaces morai_msgs nav_msgs
  sensor_msgs std_msgs tf tf2 tf2_geometry_msgs)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(osqp REQUIRED CONFIG)
np_add_nodes(INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} LIBRARIES osqp::osqp fmt::fmt)
