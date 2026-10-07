cmake_minimum_required(VERSION 3.14)
project(control)
find_package(catkin REQUIRED COMPONENTS nav_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs interfaces morai_msgs node_pipeline roscpp)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
include(FetchContent)
# OSQP 0.6 API and source hash are pinned for reproducible Noetic builds.
set(UNITTESTS OFF CACHE BOOL "" FORCE)
set(ENABLE_MKL_PARDISO OFF CACHE BOOL "" FORCE)
FetchContent_Declare(osqp
  URL https://github.com/osqp/osqp/releases/download/v0.6.3/osqp-v0.6.3-src.tar.gz
  URL_HASH SHA256=285b2a60f68d113a1090767ec8a9c81a65b3af2d258f8c78a31cc3f98ba58456)
FetchContent_MakeAvailable(osqp)
catkin_package(INCLUDE_DIRS include LIBRARIES control_tracking control_universe CATKIN_DEPENDS nav_msgs geometry_msgs sensor_msgs tf2 tf2_geometry_msgs interfaces morai_msgs node_pipeline roscpp DEPENDS EIGEN3)
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
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
add_library(control_universe
  src/universe/controller.cpp
  src/universe/parameters.cpp
  src/universe/osqp_solver.cpp
  src/universe/mpc/mpc.cpp
  src/universe/mpc/mpc_utils.cpp
  src/universe/mpc/mpc_trajectory.cpp
  src/universe/mpc/lowpass_filter.cpp
  src/universe/mpc/steering_predictor.cpp
  src/universe/mpc/vehicle_model/vehicle_model_interface.cpp
  src/universe/mpc/vehicle_model/vehicle_model_bicycle_kinematics.cpp
  src/universe/pid/pid.cpp
  src/universe/pid/pid_longitudinal_controller.cpp
  src/universe/pid/smooth_stop.cpp
  src/universe/pid/longitudinal_controller_utils.cpp
  src/universe/interpolation/linear_interpolation.cpp
  src/universe/interpolation/spline_interpolation.cpp
  src/universe/interpolation/spherical_linear_interpolation.cpp)
target_compile_features(control_universe PUBLIC cxx_std_14)
target_compile_options(control_universe PRIVATE -Wall -Wextra -Wpedantic)
add_dependencies(control_universe ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_universe PUBLIC osqpstatic fmt::fmt ${catkin_LIBRARIES})
node_pipeline_add_node(control_node)
target_compile_features(control_node PRIVATE cxx_std_14)
target_compile_options(control_node PRIVATE -Wall -Wextra -Wpedantic)
add_dependencies(control_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(control_node control_universe control_tracking ${catkin_LIBRARIES})
install(TARGETS control_tracking control_universe control_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES NOTICE DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY licenses DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(PROGRAMS launch.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY config launch docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
catkin_install_python(PROGRAMS scripts/analysis/analyze_tracking_bag.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
if(CATKIN_ENABLE_TESTING)
  foreach(test_name control_inputs autoware_universe_core path_tracking_core pure_pursuit pid_controller longitudinal_mpc stanley_controller imm_two_model_filter hybrid_controller mpc_lateral_controller curvature_speed_planner wheel_corridor control_timing_bounds)
    catkin_add_gtest(test_${test_name} test/unit/test_${test_name}.cpp)
    if(TARGET test_${test_name})
      if(test_name MATCHES "^(control_inputs|autoware_universe_core|path_tracking_core)$")
        target_compile_options(test_${test_name} PRIVATE -include ${CMAKE_CURRENT_BINARY_DIR}/control_node_registration_numeric.hpp)
        add_dependencies(test_${test_name} control_node_extension_contract)
      endif()
      target_compile_features(test_${test_name} PRIVATE cxx_std_14)
      add_dependencies(test_${test_name} ${catkin_EXPORTED_TARGETS})
      target_link_libraries(test_${test_name} control_universe control_tracking ${catkin_LIBRARIES})
    endif()
  endforeach()
  catkin_add_nosetests(test/unit/test_tracking_bag_analysis.py)
endif()
