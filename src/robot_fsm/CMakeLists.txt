cmake_minimum_required(VERSION 3.8)
project(robot_fsm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)

add_executable(robot_fsm_main
  src/main.cpp
  src/mission/mission_controller.cpp
  src/stages/stage1_wetland_fsm.cpp
  src/stages/stage2_clam_fsm.cpp
  src/stages/stage3_hay_fsm.cpp
  src/stages/stage4_mazu_fsm.cpp
  src/stages/stage5_new_fsm.cpp
)

target_include_directories(robot_fsm_main PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(stm_communication_node
  src/communication/stm_communication_node.cpp
)

target_include_directories(stm_communication_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(robot_fsm_main
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  robot_interfaces
)

ament_target_dependencies(stm_communication_node
  rclcpp
  geometry_msgs
  robot_interfaces
)

install(TARGETS
  robot_fsm_main
  stm_communication_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()
