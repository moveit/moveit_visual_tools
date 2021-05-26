cmake_minimum_required(VERSION 3.5)
project(moveit_visual_tools)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-W -Wall -Wextra
    -Wwrite-strings -Wunreachable-code -Wpointer-arith
    -Winit-self -Wredundant-decls
    -Wno-unused-parameter -Wno-unused-function)
endif()

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

## Build
# include_directories(include)
# include_directories(SYSTEM
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIRS}
# )

# Visualization Tools Library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/imarker_robot_state.cpp
  src/imarker_end_effector.cpp
)
# set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
# add_dependencies(${PROJECT_NAME} graph_msgs_generate_messages_cpp)
# message("rviz_visual_tools_LIBRARIES=${rviz_visual_tools_LIBRARIES}")
target_link_libraries(
  ${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${rviz_visual_tools_LIBRARIES}
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    include
    ${Boost_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2_eigen
  tf2_ros
  rviz_visual_tools
  moveit_ros_planning
  moveit_core
  geometry_msgs
  visualization_msgs
  graph_msgs
  std_msgs
  trajectory_msgs
)
ament_export_dependencies(
  rviz_visual_tools
)

# Demo executable
add_executable(${PROJECT_NAME}_demo
  src/${PROJECT_NAME}_demo.cpp
)
target_link_libraries(${PROJECT_NAME}_demo
  ${PROJECT_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_demo
  rclcpp
  tf2_eigen
  tf2_ros
  rviz_visual_tools
  moveit_ros_planning
  moveit_core
  geometry_msgs
  visualization_msgs
  graph_msgs
  std_msgs
  trajectory_msgs
)


#############
## Install ##
#############

# Install libraries
install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT
    ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
