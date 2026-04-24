cmake_minimum_required(VERSION 3.8)
project(mission_control)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(main_mission src/main_mission.cpp)
target_include_directories(main_mission PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include>)

ament_target_dependencies(main_mission 
  rclcpp
  nav_msgs
  mavros_msgs
  cv_bridge
)

target_link_libraries(main_mission
  ${OpenCV_LIBS}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS main_mission
  DESTINATION lib/${PROJECT_NAME})

ament_package()
