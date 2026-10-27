cmake_minimum_required(VERSION 3.0.2)
project(section_annotator)
find_package(catkin REQUIRED COMPONENTS node_pipeline interfaces nav_msgs)
np_add_nodes()
