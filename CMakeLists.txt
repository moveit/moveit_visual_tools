cmake_minimum_required(VERSION 2.8.3)
project(moveit_visual_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rviz_visual_tools
  eigen_conversions
  geometry_msgs
  moveit_ros_robot_interaction
  moveit_core
  roscpp
  tf_conversions
  visualization_msgs
  graph_msgs
  std_msgs
  trajectory_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system)

# Catkin
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    rviz_visual_tools
    moveit_ros_robot_interaction
    moveit_core
    geometry_msgs
    visualization_msgs
    graph_msgs
    std_msgs
    moveit_msgs
    trajectory_msgs
  INCLUDE_DIRS include
)

# Enable gdb to show line numbers
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -W -Wno-unused-parameter -fno-strict-aliasing")

## Build

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Visualization Tools Library
add_library(${PROJECT_NAME}
  src/moveit_visual_tools.cpp
)
add_dependencies(${PROJECT_NAME} graph_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Demo executable
add_executable(${PROJECT_NAME}_demo src/${PROJECT_NAME}_demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo
  ${catkin_LIBRARIES} ${PROJECT_NAME}
)

# Demo executable
add_executable(${PROJECT_NAME}_collision_objects_demo src/collision_objects_demo.cpp)
set_target_properties(${PROJECT_NAME}_collision_objects_demo
  PROPERTIES OUTPUT_NAME collision_objects_demo PREFIX ""
)
target_link_libraries(${PROJECT_NAME}_collision_objects_demo
  ${catkin_LIBRARIES} ${PROJECT_NAME}
)

## Install

# Install libraries
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install shared resources
install(DIRECTORY launch    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install executables
install(TARGETS ${PROJECT_NAME}_demo ${PROJECT_NAME}_collision_objects_demo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
