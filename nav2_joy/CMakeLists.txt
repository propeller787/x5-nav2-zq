cmake_minimum_required(VERSION 3.8)
project(nav2_joy)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(nav2_joy_teleop
  src/nav2_joy_teleop.cpp
  src/main_joy_teleop.cpp
)
target_include_directories(nav2_joy_teleop
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(nav2_joy_teleop PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

add_executable(joy_to_twist
  src/joy_to_twist.cpp
  src/main_joy_to_twist.cpp
)
target_include_directories(joy_to_twist
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(joy_to_twist PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

install(
  TARGETS nav2_joy_teleop joy_to_twist
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(
  PROGRAMS nav2_joy/simple_loopback_sim.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(geometry_msgs rclcpp sensor_msgs)
ament_package()
