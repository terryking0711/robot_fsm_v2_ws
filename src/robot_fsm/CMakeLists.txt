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
find_package(robot_interfaces REQUIRED)

include_directories(include)

add_executable(robot_fsm_main
  src/main.cpp
  src/mission/mission_controller.cpp
  src/stages/stage3_hay_fsm.cpp
)

ament_target_dependencies(robot_fsm_main
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  robot_interfaces
)

install(TARGETS
  robot_fsm_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()