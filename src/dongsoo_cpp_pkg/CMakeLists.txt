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

add_executable(motor_connect src/motor_connect.cpp)

target_include_directories(motor_connect PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Eigen 필요하면 링크(무해합니다)
target_link_libraries(motor_connect Eigen3::Eigen)
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

ament_package()

