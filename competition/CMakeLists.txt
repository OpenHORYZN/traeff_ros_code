cmake_minimum_required(VERSION 3.8)
project(competition)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(main_routine src/main_routine.cpp)

target_include_directories(main_routine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_features(main_routine PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(main_routine rclcpp nav_msgs)

install(TARGETS main_routine
  DESTINATION lib/${PROJECT_NAME})


ament_package()
