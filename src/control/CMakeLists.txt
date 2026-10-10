cmake_minimum_required(VERSION 3.0.2)
project(control)
find_package(catkin REQUIRED COMPONENTS node_pipeline geometry_msgs interfaces morai_msgs nav_msgs
  sensor_msgs std_msgs tf tf2 tf2_geometry_msgs osqp_vendor qpoases_vendor)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
np_add_nodes(INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
  ${osqp_vendor_PREFIX}/include/osqp ${qpoases_vendor_PREFIX}/include/qpoases_vendor
  LIBRARIES fmt::fmt)
