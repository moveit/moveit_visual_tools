cmake_minimum_required(VERSION 2.8.3)
project(moveit_visual_tools)

# C++ 11 required for OMPL
add_compile_options(-std=c++11)

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
  roslint
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system)

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
    trajectory_msgs
  INCLUDE_DIRS include
)

## Build
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Visualization Tools Library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/imarker_robot_state.cpp
  src/imarker_end_effector.cpp
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


#############
## Testing ##
#############

## Test for correct C++ source code
roslint_cpp()

#############
## Install ##
#############

# Install libraries
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install shared resources
install(DIRECTORY launch    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install executables
install(TARGETS ${PROJECT_NAME}_demo
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
