cmake_minimum_required(VERSION 3.0.2)
project(morai_path_manager)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES morai_route_path morai_route_path_messages
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(morai_route_path
  src/route/route_path.cpp
)
target_compile_features(morai_route_path PUBLIC cxx_std_14)
target_link_libraries(morai_route_path
  ${catkin_LIBRARIES}
)

add_library(morai_route_path_messages
  src/route/route_path_messages.cpp
)
target_compile_features(morai_route_path_messages PUBLIC cxx_std_14)
target_link_libraries(morai_route_path_messages
  morai_route_path
  ${catkin_LIBRARIES}
)

add_executable(route_path_publisher_node
  src/nodes/route_path_publisher_node.cpp
)
target_compile_features(route_path_publisher_node PUBLIC cxx_std_14)
target_link_libraries(route_path_publisher_node
  morai_route_path
  morai_route_path_messages
  ${catkin_LIBRARIES}
)

install(
  TARGETS morai_route_path morai_route_path_messages route_path_publisher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY config docs launch map
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/unit/test_config_contract.py)

  catkin_add_gtest(test_route_path
    test/unit/test_route_path.cpp
  )
  if(TARGET test_route_path)
    target_compile_features(test_route_path PUBLIC cxx_std_14)
    target_compile_definitions(test_route_path PRIVATE
      MORAI_PATH_MANAGER_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
      MORAI_PATH_MANAGER_OFFICIAL_PATH="${CMAKE_CURRENT_SOURCE_DIR}/map/R-KR_PG_K-City_2025/2026_molit_comp_global_path.txt"
    )
    target_link_libraries(test_route_path
      morai_route_path
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_route_path_messages
    test/unit/test_route_path_messages.cpp
  )
  if(TARGET test_route_path_messages)
    target_compile_features(test_route_path_messages PUBLIC cxx_std_14)
    target_compile_definitions(test_route_path_messages PRIVATE
      MORAI_PATH_MANAGER_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data"
    )
    target_link_libraries(test_route_path_messages
      morai_route_path
      morai_route_path_messages
      ${catkin_LIBRARIES}
    )
  endif()
endif()
