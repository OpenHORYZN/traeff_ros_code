cmake_minimum_required(VERSION 3.8)
project(mission_control)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── ROS 2 dependencies ────────────────────────────────────────────────────── #
find_package(ament_cmake   REQUIRED)
find_package(rclcpp        REQUIRED)
find_package(nav_msgs      REQUIRED)
find_package(mavros_msgs   REQUIRED)
find_package(cv_bridge     REQUIRED)
find_package(sensor_msgs   REQUIRED)
find_package(std_msgs      REQUIRED)
find_package(geometry_msgs REQUIRED)

# ── OpenCV 4.5 (apt / Jetson system install) ─────────────────────────────── #
find_package(OpenCV 4.5 REQUIRED
  COMPONENTS
    core
    imgproc
    highgui
    calib3d
    aruco
)

# ── Shared dependency list (used by both executables) ─────────────────────── #
set(ROS_DEPS
  rclcpp
  nav_msgs
  mavros_msgs
  cv_bridge
  sensor_msgs
  std_msgs
  geometry_msgs
)

# ── main_mission ──────────────────────────────────────────────────────────── #
add_executable(main_mission src/main_mission.cpp)
target_include_directories(main_mission PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(main_mission ${ROS_DEPS})
target_link_libraries(main_mission ${OpenCV_LIBS})

# ── manual_node ───────────────────────────────────────────────────────────── #
add_executable(manual_node src/manual_node.cpp)
target_include_directories(manual_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(manual_node ${ROS_DEPS})
target_link_libraries(manual_node ${OpenCV_LIBS})

# ── Install ───────────────────────────────────────────────────────────────── #
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(TARGETS
  main_mission
  manual_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()