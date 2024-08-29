cmake_minimum_required(VERSION 3.5)
project(moveit_visual_tools)

find_package(moveit_common REQUIRED)
moveit_package()

# Load  all dependencies required for this package
find_package(Boost REQUIRED system)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

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
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
  src/imarker_robot_state.cpp
  src/imarker_end_effector.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Demo executable
add_executable(${PROJECT_NAME}_demo
  src/${PROJECT_NAME}_demo.cpp
)
target_link_libraries(${PROJECT_NAME}_demo
  ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_demo ${THIS_PACKAGE_INCLUDE_DEPENDS})


# Exports
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

#############
## Install ##
#############

# Install libraries
install(
  TARGETS ${PROJECT_NAME}
  EXPORT
    export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/ DESTINATION include)

# Install shared resources
install(DIRECTORY launch resources DESTINATION share/${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_CONFIG_FILE ".ament_flake8.ini")
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
