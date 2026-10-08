cmake_minimum_required(VERSION 3.0.2)
project(traffic_light)
find_package(catkin REQUIRED COMPONENTS node_pipeline interfaces sensor_msgs)
np_add_nodes()
