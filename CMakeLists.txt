cmake_minimum_required(VERSION 3.22)
project(ros2_rviz_plugins)

# C++17 (Qt5 compatible), matching the Jazzy RViz default-plugins toolchain.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# rviz_common requires PIC for Qt MOC generation in shared libraries.
set(CMAKE_AUTOMOC ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(w_behavior_tree_interfaces REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# Headers are listed alongside .cpp so CMAKE_AUTOMOC always scans the
# Q_OBJECT classes; otherwise dlopen dies with an undefined-symbol vtable
# error for the panel/tool class.
add_library(${PROJECT_NAME} SHARED
  src/nav_single_goal_tool.cpp
  src/nav_single_panel.cpp
  include/ros2_rviz_plugins/nav_single_goal_tool.hpp
  include/ros2_rviz_plugins/nav_single_panel.hpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Qt5::Widgets
)
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    rclcpp
    rclcpp_action
    pluginlib
    rviz_common
    rviz_rendering
    rviz_default_plugins
    geometry_msgs
    tf2
    tf2_geometry_msgs
    w_behavior_tree_interfaces
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# pluginlib requires this define so the panel/tool classes register with
# rviz_common::Panel / rviz_common::Tool on dlopen.
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  FILES plugins_description.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY icons
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  pluginlib
  rviz_common
  rviz_rendering
  rviz_default_plugins
  geometry_msgs
  tf2
  tf2_geometry_msgs
  w_behavior_tree_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
