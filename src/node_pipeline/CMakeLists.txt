cmake_minimum_required(VERSION 3.0.2)
project(node_pipeline)

find_package(catkin REQUIRED COMPONENTS interfaces roscpp roslib)
find_package(yaml-cpp REQUIRED)
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES node_pipeline
  CATKIN_DEPENDS interfaces roscpp roslib
  CFG_EXTRAS node_pipeline-extras.cmake)

add_library(node_pipeline src/runtime.cpp)
target_include_directories(node_pipeline PUBLIC include ${catkin_INCLUDE_DIRS})
target_include_directories(node_pipeline PRIVATE ${YAML_CPP_INCLUDE_DIR})
target_compile_features(node_pipeline PUBLIC cxx_std_14)
target_compile_options(node_pipeline PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(node_pipeline ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
add_dependencies(node_pipeline ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  foreach(test_name test_pipeline test_param_reader test_ros_header_injector)
    catkin_add_gtest(${test_name} test/unit/${test_name}.cpp)
    if(TARGET ${test_name})
      target_compile_features(${test_name} PRIVATE cxx_std_14)
      target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wpedantic)
      target_link_libraries(${test_name} node_pipeline gtest_main)
    endif()
  endforeach()
  catkin_add_nosetests(test/unit/test_np_check.py)
  catkin_add_nosetests(test/unit/test_np_generate.py)
endif()

install(TARGETS node_pipeline
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY cmake scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} PATTERN "*.in" EXCLUDE)
install(FILES src/node_main.cpp DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src)
install(FILES README.md CORE_REGISTRATION.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
