cmake_minimum_required(VERSION 3.0.2)
project(tf_publisher)
find_package(catkin REQUIRED COMPONENTS node_pipeline geometry_msgs nav_msgs tf2_msgs)
np_add_nodes()
