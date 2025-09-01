cmake_minimum_required(VERSION 3.8)
project(dongsoo_cpp_pkg)

# C++ 표준 (권장)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp      REQUIRED)
find_package(std_msgs    REQUIRED)
find_package(dynamixel_sdk REQUIRED)

find_package(eigen3_cmake_module QUIET)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
# nlohmann_json는 헤더 전용 라이브러리로 사용 (시스템 설치 필요)
# find_package(nlohmann_json 3.2.0 REQUIRED)

add_executable(motor_connect src/motor_connect.cpp)

target_include_directories(motor_connect PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Eigen 링크
target_link_libraries(motor_connect Eigen3::Eigen)
# nlohmann_json은 헤더 전용이므로 링크 불필요
if(NOT TARGET Eigen3::Eigen)
  message(WARNING "Eigen3::Eigen 타깃을 못 찾음 -> /usr/include/eigen3 수동 추가")
  target_include_directories(motor_connect PRIVATE /usr/include/eigen3)
endif()

ament_target_dependencies(motor_connect
  rclcpp
  std_msgs
  dynamixel_sdk
)

install(TARGETS motor_connect
  DESTINATION lib/${PROJECT_NAME}
)

# test_motor_connect executable (improved version)
add_executable(test_motor_connect src/test_motor_connect.cpp)

target_include_directories(test_motor_connect PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Eigen & nlohmann_json 링크
target_link_libraries(test_motor_connect Eigen3::Eigen)
# nlohmann_json은 헤더 전용이므로 링크 불필요
if(NOT TARGET Eigen3::Eigen)
  message(WARNING "Eigen3::Eigen 타깃을 못 찾음 -> /usr/include/eigen3 수동 추가")
  target_include_directories(test_motor_connect PRIVATE /usr/include/eigen3)
endif()

ament_target_dependencies(test_motor_connect
  rclcpp
  std_msgs
  dynamixel_sdk
)

install(TARGETS test_motor_connect
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()