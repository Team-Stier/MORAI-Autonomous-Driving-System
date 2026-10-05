cmake_minimum_required(VERSION 3.0.2)
project(control)
find_package(catkin REQUIRED COMPONENTS nav_msgs geometry_msgs interfaces morai_msgs node_pipeline roscpp
  tf tf2 tf2_geometry_msgs)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED CONFIG)
catkin_package(INCLUDE_DIRS include LIBRARIES control_tracking control_morai_mpc control_morai_mpc_qpoases
  CATKIN_DEPENDS nav_msgs geometry_msgs interfaces morai_msgs node_pipeline roscpp
  tf tf2 tf2_geometry_msgs
  DEPENDS Eigen3)
include_directories(include ${catkin_INCLUDE_DIRS})
file(GLOB MORAI_MPC_QPOASES_SOURCES src/controllers/morai_mpc/vendor/qpoases/*.cpp)
add_library(control_morai_mpc_qpoases SHARED ${MORAI_MPC_QPOASES_SOURCES})
target_compile_definitions(control_morai_mpc_qpoases PRIVATE __NO_COPYRIGHT__)
add_library(control_morai_mpc
  src/controllers/morai_mpc/autoware_lateral.cpp
  src/controllers/morai_mpc/longitudinal_mpc.cpp
  src/controllers/morai_mpc/steering_prediction.cpp
  src/controllers/morai_mpc/parameters.cpp
  src/controllers/morai_mpc/osqp_sqp_solver.cpp
  src/controllers/morai_mpc/vendor/amathutils/Amathutils.cpp
  src/controllers/morai_mpc/autoware/mpc_utils.cpp
  src/controllers/morai_mpc/autoware/mpc_trajectory.cpp
  src/controllers/morai_mpc/autoware/lowpass_filter.cpp
  src/controllers/morai_mpc/autoware/vehicle_model/vehicle_model_interface.cpp
  src/controllers/morai_mpc/autoware/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/controllers/morai_mpc/autoware/qp_solver/qp_solver_qpoases.cpp)
target_compile_features(control_morai_mpc PUBLIC cxx_std_14)
target_compile_options(control_morai_mpc PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(control_morai_mpc PUBLIC ${EIGEN3_INCLUDE_DIRS})
add_dependencies(control_morai_mpc ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_morai_mpc control_morai_mpc_qpoases ${catkin_LIBRARIES} osqp::osqp)
add_library(control_tracking
  src/common/control_timing_bounds.cpp
  src/common/osqp_sqp_solver.cpp
  src/controllers/lateral/pure_pursuit.cpp
  src/controllers/lateral/stanley_controller.cpp
  src/controllers/lateral/imm_two_model_filter.cpp
  src/controllers/lateral/hybrid_controller.cpp
  src/controllers/lateral/mpc/path.cpp
  src/controllers/lateral/mpc/optimizer.cpp
  src/controllers/lateral/mpc/mpc_controller.cpp
  src/controllers/lateral/mpc/mpc_lateral_controller.cpp
  src/controllers/longitudinal/pid_controller.cpp
  src/controllers/longitudinal/mpc/longitudinal_mpc.cpp
  src/controllers/tracking_commands.cpp
  src/planning/curvature_speed_planner.cpp
  src/planning/wheel_corridor.cpp)
target_compile_features(control_tracking PUBLIC cxx_std_14)
target_compile_options(control_tracking PRIVATE -Wall -Wextra -Wpedantic)
add_dependencies(control_tracking ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_tracking ${catkin_LIBRARIES})
add_executable(control_node src/ros_node.cpp)
target_compile_features(control_node PRIVATE cxx_std_14)
target_compile_options(control_node PRIVATE -Wall -Wextra -Wpedantic)
add_dependencies(control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_node control_tracking ${catkin_LIBRARIES})
install(TARGETS control_tracking control_morai_mpc control_morai_mpc_qpoases control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(PROGRAMS launch.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/analysis/analyze_tracking_bag.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_morai_mpc test/unit/test_morai_mpc.cpp)
  if(TARGET test_morai_mpc)
    target_compile_features(test_morai_mpc PRIVATE cxx_std_14)
    target_compile_definitions(test_morai_mpc PRIVATE
      MORAI_MPC_TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
    add_dependencies(test_morai_mpc ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test_morai_mpc control_morai_mpc ${catkin_LIBRARIES})
  endif()
  foreach(test_name path_tracking_core pure_pursuit pid_controller longitudinal_mpc stanley_controller imm_two_model_filter hybrid_controller mpc_lateral_controller curvature_speed_planner wheel_corridor control_timing_bounds)
    catkin_add_gtest(test_${test_name} test/unit/test_${test_name}.cpp)
    if(TARGET test_${test_name})
      target_compile_features(test_${test_name} PRIVATE cxx_std_14)
      add_dependencies(test_${test_name} ${catkin_EXPORTED_TARGETS})
      target_link_libraries(test_${test_name} control_tracking ${catkin_LIBRARIES})
    endif()
  endforeach()
  catkin_add_nosetests(test/unit/test_tracking_bag_analysis.py)
endif()
