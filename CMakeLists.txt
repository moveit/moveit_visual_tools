cmake_minimum_required(VERSION 3.0.2)
project(moveit_visual_tools)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  rviz_visual_tools
  tf2_eigen
  geometry_msgs
  moveit_ros_planning
  moveit_core
  roscpp
  tf2_ros
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
    roscpp
    rviz_visual_tools
    moveit_ros_planning
    moveit_core
    geometry_msgs
    visualization_msgs
    graph_msgs
    std_msgs
    trajectory_msgs
  INCLUDE_DIRS include
)

## Build
include_directories(include)
include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# Visualization Tools Library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/imarker_robot_state.cpp
  src/imarker_end_effector.cpp
)
target_compile_options(${PROJECT_NAME} PRIVATE
  -Wall -Wextra -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
add_dependencies(${PROJECT_NAME} graph_msgs_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Demo executable
add_executable(${PROJECT_NAME}_demo src/${PROJECT_NAME}_demo.cpp)
target_compile_options(${PROJECT_NAME}_demo PRIVATE
  -Wall -Wextra -Wwrite-strings -Wunreachable-code -Wpointer-arith -Wredundant-decls)
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
