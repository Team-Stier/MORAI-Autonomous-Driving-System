# ROS Noetic catkin 패키지에서 사용할 최소 CMake 버전을 지정한다.
cmake_minimum_required(VERSION 3.0.2)
# 패키지 이름을 선언한다. PROJECT_NAME과 설치 경로에 같은 이름을 사용한다.
project(node_pipeline)

# 공통 라이브러리 자체는 ROS 메시지/roscpp에 의존하지 않고 catkin만 필요하다.
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs nav_msgs tf2_msgs interfaces morai_msgs)
# 다른 실행 패키지가 include 경로를 받아 header-only 템플릿을 사용할 수 있게 export한다.
catkin_package(INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs tf2_msgs interfaces morai_msgs
  CFG_EXTRAS node_pipeline.cmake)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_core_sequence test/unit/test_core_sequence.cpp)
  if(TARGET test_core_sequence)
    target_include_directories(test_core_sequence PRIVATE include)
    target_compile_features(test_core_sequence PRIVATE cxx_std_14)
    target_compile_definitions(test_core_sequence PRIVATE NODE_PIPELINE_LEGACY_PATH_PLANNER)
    target_compile_options(test_core_sequence PRIVATE -Wall -Wextra -Wpedantic)
    target_link_libraries(test_core_sequence gtest_main)
  endif()
  add_executable(test_sealed_runtime test/unit/test_sealed_runtime.cpp)
  target_include_directories(test_sealed_runtime PRIVATE include)
  target_compile_features(test_sealed_runtime PRIVATE cxx_std_14)
  target_compile_options(test_sealed_runtime PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  add_test(NAME test_sealed_runtime COMMAND test_sealed_runtime)
  add_executable(test_execution_contract test/unit/test_execution_contract.cpp)
  target_include_directories(test_execution_contract PRIVATE include)
  target_compile_features(test_execution_contract PRIVATE cxx_std_14)
  target_compile_options(test_execution_contract PRIVATE -Wall -Wextra -Wpedantic -UNDEBUG)
  add_test(NAME test_execution_contract COMMAND test_execution_contract)
  add_test(NAME test_extension_contracts COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_extension_contracts.py")
  add_test(NAME test_registration_tools COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_registration_tools.py")
  add_test(NAME test_ros_runtime COMMAND python3 "${CMAKE_CURRENT_SOURCE_DIR}/test/unit/test_ros_runtime.py")
endif()

# 서비스 패키지는 헤더와 가이드 템플릿 소스를 배포한다. ROS 가이드 템플릿 실행 파일은 여기서 빌드하지 않는다.
# 공개 헤더 디렉터리의 내용을 설치 대상에 넣는다. 뒤의 /는 디렉터리 내용 복사를 뜻한다.
install(DIRECTORY include/${PROJECT_NAME}/
  # include/node_pipeline 아래로 설치해 include 이름을 개발/설치 환경에서 동일하게 유지한다.
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# 복사해서 전문화할 가이드 템플릿 코드와 합성 가이드 템플릿 설정을 함께 설치한다.
install(DIRECTORY templates scripts src
  # 패키지 share 디렉터리에 가이드 템플릿/설정 하위 디렉터리를 보존한다.
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# 처리 계약과 다이어그램이 담긴 README도 설치 패키지에 포함한다.
install(FILES README.md CORE_REGISTRATION.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
