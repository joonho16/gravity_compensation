cmake_minimum_required(VERSION 3.8)
project(gravity_compensation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. 패키지 찾기
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(ament_index_cpp REQUIRED) # ROS 패키지
find_package(pinocchio REQUIRED)       # 시스템 패키지
find_package(Eigen3 REQUIRED)          # 시스템 패키지

add_executable(gravity_node src/gravity_node.cpp)

# 2. 시스템 라이브러리 (Pinocchio, Eigen) 헤더 경로 및 링크
target_include_directories(gravity_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(gravity_node
  ${pinocchio_LIBRARIES}
)

# 3. [핵심 수정] ROS 2 패키지 의존성 링크 (이렇게 해야 rclcpp 경로를 자동으로 찾습니다)
ament_target_dependencies(gravity_node
  rclcpp
  sensor_msgs
  dynamixel_sdk
  ament_index_cpp
)

# 4. 설치 설정
install(DIRECTORY urdf meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS gravity_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()