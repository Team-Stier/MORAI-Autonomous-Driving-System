cmake_minimum_required(VERSION 3.0.2)
project(speed_filter)
find_package(catkin REQUIRED COMPONENTS node_pipeline interfaces morai_msgs)
np_add_nodes()
