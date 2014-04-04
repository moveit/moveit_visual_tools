cmake_minimum_required(VERSION 2.8.3)
project(moveit_visualization_tools)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED
  eigen_conversions 
  geometry_msgs
  moveit_ros_planning 
  roscpp
  tf_conversions
  visualization_msgs
)

# Catkin
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    eigen_conversions 
    geometry_msgs
    moveit_ros_planning 
    roscpp
    tf_conversions
    visualization_msgs 
  INCLUDE_DIRS include
)

# Enable gdb to show line numbers
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g")
#set(CMAKE_BUILD_TYPE Debug)

## Build 
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

# Visualization Tools Library
add_library(${PROJECT_NAME}
  src/visualization_tools.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Test executable
add_executable(visualization_tools_test src/visualization_tools_test.cpp)
target_link_libraries(visualization_tools_test 
  ${catkin_LIBRARIES} ${PROJECT_NAME} ${PROJECT_NAME}_visualization_tools
)