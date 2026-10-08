# 실행 패키지의 유일한 빌드 진입점이다. config/<node>/node.yaml마다 공통 main으로 노드를 만든다.
# 사용: find_package(catkin REQUIRED COMPONENTS node_pipeline ...) 후 np_add_nodes([INCLUDE_DIRS ...] [LIBRARIES ...])
macro(np_add_nodes)
  cmake_parse_arguments(NP "" "" "INCLUDE_DIRS;LIBRARIES" ${ARGN})
  find_package(PythonInterp 3 REQUIRED)
  catkin_package()

  set(_np_package_dir "${PROJECT_SOURCE_DIR}")
  set(_np_build_dir "${PROJECT_BINARY_DIR}/np_generated")
  set(_np_include_dirs "${_np_package_dir}/include" ${catkin_INCLUDE_DIRS} ${NP_INCLUDE_DIRS})
  string(REPLACE ";" "\n" _np_include_lines "${_np_include_dirs}")
  file(WRITE "${_np_build_dir}/include_dirs.txt" "${_np_include_lines}\n")

  file(GLOB_RECURSE _np_watched CONFIGURE_DEPENDS
    "${_np_package_dir}/config/*.yaml" "${_np_package_dir}/include/*" "${_np_package_dir}/src/*"
    "${_np_package_dir}/package.xml")
  file(GLOB _np_node_files CONFIGURE_DEPENDS "${_np_package_dir}/config/*/node.yaml")
  file(GLOB_RECURSE _np_library_sources CONFIGURE_DEPENDS "${_np_package_dir}/src/libraries/*.cpp")

  # 경로·YAML·include·AST 계약 검사는 모든 노드·라이브러리·테스트보다 먼저 통과해야 한다.
  add_custom_command(
    OUTPUT "${_np_build_dir}/contract.stamp"
    COMMAND ${PYTHON_EXECUTABLE} "${node_pipeline_SCRIPTS_DIR}/np_check.py"
      --package-dir "${_np_package_dir}" --package "${PROJECT_NAME}" --src-dir "${_np_package_dir}/.."
      --include-dirs-file "${_np_build_dir}/include_dirs.txt" --stamp "${_np_build_dir}/contract.stamp"
    DEPENDS ${_np_watched} "${node_pipeline_SCRIPTS_DIR}/np_check.py" "${node_pipeline_SCRIPTS_DIR}/np_generate.py"
    COMMENT "node_pipeline contract check: ${PROJECT_NAME}"
    VERBATIM)
  add_custom_target(${PROJECT_NAME}_np_contract ALL DEPENDS "${_np_build_dir}/contract.stamp")
  if(catkin_EXPORTED_TARGETS)
    add_dependencies(${PROJECT_NAME}_np_contract ${catkin_EXPORTED_TARGETS})
  endif()

  set(_np_link_libraries ${catkin_LIBRARIES} ${NP_LIBRARIES})
  if(_np_library_sources)
    add_library(${PROJECT_NAME}_libraries ${_np_library_sources})
    target_include_directories(${PROJECT_NAME}_libraries PUBLIC "${_np_package_dir}/include" ${catkin_INCLUDE_DIRS} ${NP_INCLUDE_DIRS})
    target_compile_features(${PROJECT_NAME}_libraries PUBLIC cxx_std_14)
    target_compile_options(${PROJECT_NAME}_libraries PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(${PROJECT_NAME}_libraries ${catkin_LIBRARIES} ${NP_LIBRARIES})
    add_dependencies(${PROJECT_NAME}_libraries ${PROJECT_NAME}_np_contract)
    install(TARGETS ${PROJECT_NAME}_libraries
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
    list(APPEND _np_link_libraries ${PROJECT_NAME}_libraries)
  endif()

  foreach(_np_node_file ${_np_node_files})
    get_filename_component(_np_node_dir "${_np_node_file}" DIRECTORY)
    get_filename_component(_np_node "${_np_node_dir}" NAME)
    file(GLOB _np_registrations CONFIGURE_DEPENDS "${_np_node_dir}/registrations/*.yaml")
    set(_np_registry_dir "${_np_build_dir}/${_np_node}")
    # node.yaml·등록 YAML은 재빌드 때만 반영되도록 빌드 디렉터리 사본을 실행 시 읽는다.
    set(_np_config_copy "${_np_registry_dir}/config")
    add_custom_command(
      OUTPUT "${_np_registry_dir}/np_registry.hpp" "${_np_config_copy}/node.yaml"
      COMMAND ${PYTHON_EXECUTABLE} "${node_pipeline_SCRIPTS_DIR}/np_generate.py"
        --package-dir "${_np_package_dir}" --package "${PROJECT_NAME}" --node "${_np_node}"
        --output "${_np_registry_dir}/np_registry.hpp" --config-copy "${_np_config_copy}"
      DEPENDS "${_np_node_file}" ${_np_registrations} ${_np_watched} "${node_pipeline_SCRIPTS_DIR}/np_generate.py"
      COMMENT "node_pipeline registry: ${PROJECT_NAME}/${_np_node}"
      VERBATIM)
    add_executable(${_np_node}_node "${node_pipeline_MAIN_SOURCE}" "${_np_registry_dir}/np_registry.hpp"
      "${_np_config_copy}/node.yaml")
    target_include_directories(${_np_node}_node PRIVATE "${_np_registry_dir}" "${_np_package_dir}/include"
      ${catkin_INCLUDE_DIRS} ${NP_INCLUDE_DIRS})
    target_compile_definitions(${_np_node}_node PRIVATE "NP_NODE_CONFIG_DIR=\"${_np_config_copy}\"")
    target_compile_features(${_np_node}_node PRIVATE cxx_std_14)
    target_compile_options(${_np_node}_node PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(${_np_node}_node ${_np_link_libraries})
    add_dependencies(${_np_node}_node ${PROJECT_NAME}_np_contract)
    install(TARGETS ${_np_node}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  endforeach()

  if(CATKIN_ENABLE_TESTING)
    file(GLOB _np_cpp_tests CONFIGURE_DEPENDS "${_np_package_dir}/test/unit/test_*.cpp")
    foreach(_np_test_file ${_np_cpp_tests})
      get_filename_component(_np_test "${_np_test_file}" NAME_WE)
      catkin_add_gtest(${_np_test} "${_np_test_file}")
      if(TARGET ${_np_test})
        target_include_directories(${_np_test} PRIVATE "${_np_package_dir}/include" ${catkin_INCLUDE_DIRS} ${NP_INCLUDE_DIRS})
        target_compile_features(${_np_test} PRIVATE cxx_std_14)
        target_compile_definitions(${_np_test} PRIVATE "NP_PACKAGE_DIR=\"${_np_package_dir}\"")
        target_link_libraries(${_np_test} ${_np_link_libraries} gtest_main)
        add_dependencies(${_np_test} ${PROJECT_NAME}_np_contract)
      endif()
    endforeach()
    file(GLOB _np_python_tests CONFIGURE_DEPENDS "${_np_package_dir}/test/unit/test_*.py")
    foreach(_np_test_file ${_np_python_tests})
      catkin_add_nosetests("${_np_test_file}")
    endforeach()
  endif()

  install(PROGRAMS launch.sh DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endmacro()
