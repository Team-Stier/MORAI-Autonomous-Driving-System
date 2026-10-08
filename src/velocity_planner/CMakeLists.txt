cmake_minimum_required(VERSION 3.0.2)
project(velocity_planner)
find_package(catkin REQUIRED COMPONENTS node_pipeline geometry_msgs interfaces nav_msgs tf2 tf2_geometry_msgs tf2_msgs)
np_add_nodes()
