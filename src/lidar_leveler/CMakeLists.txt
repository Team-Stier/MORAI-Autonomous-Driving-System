cmake_minimum_required(VERSION 3.0.2)
project(lidar_leveler)
find_package(catkin REQUIRED COMPONENTS node_pipeline geometry_msgs interfaces sensor_msgs tf2 tf2_msgs)
np_add_nodes()
