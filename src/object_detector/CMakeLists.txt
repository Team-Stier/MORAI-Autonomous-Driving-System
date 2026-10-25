cmake_minimum_required(VERSION 3.0.2)
project(object_detector)
find_package(catkin REQUIRED COMPONENTS node_pipeline interfaces tf2_msgs sensor_msgs pcl_conversions
  lanelet2_core lanelet2_io lanelet2_routing lanelet2_traffic_rules)
find_package(PCL REQUIRED COMPONENTS common filters kdtree search)
find_package(pugixml REQUIRED CONFIG)
find_package(PythonLibs 3.12 REQUIRED)
find_library(UTONIA_ROSPACK_LIBRARY rospack
  PATHS "${CMAKE_CURRENT_SOURCE_DIR}/src/libraries/utonia_runtime/lib" NO_DEFAULT_PATH)
if(NOT UTONIA_ROSPACK_LIBRARY)
  message(FATAL_ERROR "Build the private Python 3.12 rospack dependency for Utonia")
endif()
get_filename_component(UTONIA_ROSPACK_LIBRARY_DIR ${UTONIA_ROSPACK_LIBRARY} DIRECTORY)
get_filename_component(UTONIA_PYTHON_LIBRARY_DIR ${PYTHON_LIBRARIES} DIRECTORY)
# Noetic's default rospack embeds Python 3.8; this node must use one Python ABI.
list(FILTER catkin_LIBRARIES EXCLUDE REGEX "(^|/)(lib)?python3\\.")
list(FILTER catkin_LIBRARIES EXCLUDE REGEX "(^|/)(lib)?rospack(\\.so.*)?$")
get_target_property(pugixml_INCLUDE_DIRS pugixml INTERFACE_INCLUDE_DIRECTORIES)
np_add_nodes(INCLUDE_DIRS ${PCL_INCLUDE_DIRS} ${pugixml_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS}
  LIBRARIES ${PCL_LIBRARIES} pugixml ${UTONIA_ROSPACK_LIBRARY} ${PYTHON_LIBRARIES}
  -Wl,--disable-new-dtags -Wl,-rpath,${UTONIA_ROSPACK_LIBRARY_DIR} -Wl,-rpath,${UTONIA_PYTHON_LIBRARY_DIR})
