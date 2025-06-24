cmake_minimum_required(VERSION 3.20)
project(moveit_visual_tools)

find_package(moveit_common REQUIRED)
moveit_package()

# Load  all dependencies required for this package
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_occupancy_map_monitor REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Visualization Tools Library
add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}.cpp
  src/imarker_robot_state.cpp
  src/imarker_end_effector.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include> ${BULLET_INCLUDE_DIR}
)


target_link_libraries(moveit_visual_tools PUBLIC
  ${geometry_msgs_TARGETS}
  ${graph_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${trajectory_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  moveit_core::moveit_collision_detection
  moveit_core::moveit_collision_detection_bullet
  moveit_core::moveit_collision_detection_fcl
  moveit_core::moveit_collision_distance_field
  moveit_core::moveit_constraint_samplers
  moveit_core::moveit_distance_field
  moveit_core::moveit_dynamics_solver
  moveit_core::moveit_exceptions
  moveit_core::moveit_kinematic_constraints
  moveit_core::moveit_kinematics_base
  moveit_core::moveit_kinematics_metrics
  moveit_core::moveit_macros
  moveit_core::moveit_planning_interface
  moveit_core::moveit_planning_scene
  moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state
  moveit_core::moveit_robot_trajectory
  moveit_core::moveit_smoothing_base
  moveit_core::moveit_test_utils
  moveit_core::moveit_trajectory_processing
  moveit_core::moveit_transforms
  moveit_core::moveit_utils
  moveit_ros_planning::default_request_adapter_parameters
  moveit_ros_planning::default_response_adapter_parameters
  moveit_ros_planning::kinematics_parameters
  moveit_ros_planning::moveit_collision_plugin_loader
  moveit_ros_planning::moveit_constraint_sampler_manager_loader
  moveit_ros_planning::moveit_cpp
  moveit_ros_planning::moveit_kinematics_plugin_loader
  moveit_ros_planning::moveit_plan_execution
  moveit_ros_planning::moveit_planning_pipeline
  moveit_ros_planning::moveit_planning_pipeline_interfaces
  moveit_ros_planning::moveit_planning_scene_monitor
  moveit_ros_planning::moveit_rdf_loader
  moveit_ros_planning::moveit_robot_model_loader
  moveit_ros_planning::moveit_trajectory_execution_manager
  moveit_ros_planning::planning_pipeline_parameters
  moveit_ros_planning::srdf_publisher_node
  rclcpp::rclcpp
  rviz_visual_tools::rviz_visual_tools
  rviz_visual_tools::rviz_visual_tools_gui
  rviz_visual_tools::rviz_visual_tools_imarker_simple
  rviz_visual_tools::rviz_visual_tools_remote_control
  tf2_eigen::tf2_eigen
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  Boost::filesystem
)

# Exports
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)


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

# Install header files
install(DIRECTORY include/ DESTINATION include)

# Install shared resources
install(DIRECTORY launch resources DESTINATION share/${PROJECT_NAME})

#############
## Testing ##
#############

if(BUILD_TESTING)
# Demo executable
add_executable(${PROJECT_NAME}_demo
  src/${PROJECT_NAME}_demo.cpp
)

target_link_libraries(${PROJECT_NAME}_demo
  ${PROJECT_NAME} rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cppcheck_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_flake8_CONFIG_FILE ".ament_flake8.ini")
  ament_lint_auto_find_test_dependencies()

endif()

ament_package()
