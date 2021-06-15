cmake_minimum_required(VERSION 3.5)
project(moveit_visual_tools)

find_package(moveit_common REQUIRED)
moveit_package()

# Load  all dependencies required for this package
find_package(rviz_visual_tools REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  Boost
  geometry_msgs
  graph_msgs
  moveit_core
  moveit_ros_planning
  rclcpp
  rviz_visual_tools
  std_msgs
  tf2_eigen
  tf2_ros
  trajectory_msgs
  visualization_msgs
)

# Visualization Tools Library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/imarker_robot_state.cpp
  src/imarker_end_effector.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${rviz_visual_tools_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
)
ament_target_dependencies(${PROJECT_NAME}
  Boost
  geometry_msgs
  graph_msgs
  moveit_core
  moveit_ros_planning
  rclcpp
  rviz_visual_tools
  std_msgs
  tf2_eigen
  tf2_ros
  trajectory_msgs
  visualization_msgs
)

# Demo executable
add_executable(${PROJECT_NAME}_demo
  src/${PROJECT_NAME}_demo.cpp
)
target_link_libraries(${PROJECT_NAME}_demo
  ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_demo
  Boost
  geometry_msgs
  graph_msgs
  moveit_core
  moveit_ros_planning
  rclcpp
  rviz_visual_tools
  std_msgs
  tf2_eigen
  tf2_ros
  trajectory_msgs
  visualization_msgs
)


#############
## Install ##
#############

# Install libraries
install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT
    export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include DESTINATION include)

# Install shared resources
install(DIRECTORY launch resources DESTINATION share/${PROJECT_NAME})

# Install executables
install(TARGETS ${PROJECT_NAME}_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)

# Exports
# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
