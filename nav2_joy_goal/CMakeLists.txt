cmake_minimum_required(VERSION 3.8)
project(nav2_joy_goal)

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
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_navfn_planner REQUIRED)

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

add_library(quintic_planner SHARED
  src/quintic_planner.cpp
)
target_include_directories(quintic_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(quintic_planner
  PUBLIC
    geometry_msgs
    rclcpp
    nav2_core
    nav2_costmap_2d
    nav2_util
    nav_msgs
    pluginlib
    tf2_ros
    tf2_geometry_msgs
    nav2_navfn_planner
)
# nav2_navfn_planner's CMake config exports its library under the same name
# as the package, so the standard _TARGETS variable is empty; we have to
# link the .so by absolute path. Without this the plugin loads but crashes
# with "undefined symbol: ...NavfnPlannerC1Ev" at startup.
find_library(_navfn_impl NAMES nav2_navfn_planner REQUIRED)
target_link_libraries(quintic_planner PUBLIC ${_navfn_impl})

install(
  TARGETS nav2_joy_teleop joy_to_twist quintic_planner
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(FILES quintic_planner_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

install(
  PROGRAMS nav2_joy/simple_loopback_sim.py
          nav2_joy/keyboard_goal_setter.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs
  rclcpp
  sensor_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  nav2_navfn_planner
)
pluginlib_export_plugin_description_file(nav2_core quintic_planner_plugin.xml)
ament_package()
