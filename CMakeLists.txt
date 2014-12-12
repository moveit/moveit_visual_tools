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

# Test executable
add_executable(moveit_visual_tools_test src/moveit_visual_tools_test.cpp)
target_link_libraries(moveit_visual_tools_test 
  ${catkin_LIBRARIES} ${PROJECT_NAME}
)

# Test executable
add_executable(collision_objects_test src/collision_objects_test.cpp)
target_link_libraries(collision_objects_test 
  ${catkin_LIBRARIES} ${PROJECT_NAME}
)

## Install

# Install libraries
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install header files
install(DIRECTORY include/$PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install shared resources
install(DIRECTORY launch    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install executables
install(TARGETS moveit_visual_tools_test collision_objects_test
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

