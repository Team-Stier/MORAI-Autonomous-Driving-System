cmake_minimum_required(VERSION 3.0.2)
project(control)
find_package(catkin REQUIRED COMPONENTS node_pipeline geometry_msgs interfaces morai_msgs nav_msgs)
np_add_nodes()
