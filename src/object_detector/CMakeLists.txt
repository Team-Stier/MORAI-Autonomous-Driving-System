cmake_minimum_required(VERSION 3.0.2)
project(object_detector)
find_package(catkin REQUIRED COMPONENTS node_pipeline interfaces sensor_msgs)
np_add_nodes()
