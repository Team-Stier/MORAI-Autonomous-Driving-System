# ROS Noetic catkin 패키지에서 사용할 최소 CMake 버전을 지정한다.
cmake_minimum_required(VERSION 3.0.2)
# 패키지 이름을 선언한다. PROJECT_NAME과 설치 경로에 같은 이름을 사용한다.
project(node_pipeline)

# 공통 라이브러리 자체는 ROS 메시지/roscpp에 의존하지 않고 catkin만 필요하다.
find_package(catkin REQUIRED)
# 다른 실행 패키지가 include 경로를 받아 header-only 템플릿을 사용할 수 있게 export한다.
catkin_package(INCLUDE_DIRS include)

# 서비스 패키지는 헤더와 가이드 템플릿 소스를 배포한다. ROS 가이드 템플릿 실행 파일은 여기서 빌드하지 않는다.
# 공개 헤더 디렉터리의 내용을 설치 대상에 넣는다. 뒤의 /는 디렉터리 내용 복사를 뜻한다.
install(DIRECTORY include/${PROJECT_NAME}/
  # include/node_pipeline 아래로 설치해 include 이름을 개발/설치 환경에서 동일하게 유지한다.
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# 복사해서 전문화할 가이드 템플릿 코드와 합성 가이드 템플릿 설정을 함께 설치한다.
install(DIRECTORY templates config
  # 패키지 share 디렉터리에 가이드 템플릿/설정 하위 디렉터리를 보존한다.
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# 처리 계약과 다이어그램이 담긴 README도 설치 패키지에 포함한다.
install(FILES README.md DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
