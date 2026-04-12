cmake_minimum_required(VERSION 3.8)
project(all_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)

add_library(all_launch_rviz_plugins SHARED
  src/docking_panel.cpp
)
target_include_directories(all_launch_rviz_plugins PUBLIC
  include
)
target_link_libraries(all_launch_rviz_plugins
  Qt5::Widgets
)
ament_target_dependencies(all_launch_rviz_plugins
  geometry_msgs
  pluginlib
  rclcpp
  rviz_common
  std_msgs
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/controller_switcher.py
  scripts/dock_pose_bridge.py
  scripts/nav2_goal_metrics.py
  scripts/apriltag_dock_pose_publisher.py
  scripts/logitech_camera_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS all_launch_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(all_launch_rviz_plugins)
ament_export_dependencies(
  geometry_msgs
  pluginlib
  rclcpp
  rviz_common
  std_msgs
)

ament_package()
