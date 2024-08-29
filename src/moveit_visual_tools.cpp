// Copyright 2015 University of Colorado, Boulder
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Author: Dave Coleman
// Desc:   Simple tools for showing parts of a robot in Rviz, such as the gripper or arm

#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt Messages
#include <moveit_msgs/msg/collision_object.hpp>

// MoveIt
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/macros/console_colors.h>

// Conversions
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

// Transforms
#include <tf2_ros/transform_listener.h>

// Shape tools
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>

// C++
#include <string>
#include <algorithm>
#include <utility>
#include <vector>
#include <set>
#include <limits>
#include <iomanip>

using namespace std::literals::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_visual_tools");
namespace moveit_visual_tools
{
MoveItVisualTools::MoveItVisualTools(const rclcpp::Node::SharedPtr& node)
  : RvizVisualTools("", rviz_visual_tools::RVIZ_MARKER_TOPIC, node)
  , robot_state_topic_(DISPLAY_ROBOT_STATE_TOPIC)
  , planning_scene_topic_(PLANNING_SCENE_TOPIC)
  , node_(node)
{
  loadSharedRobotState();
  setBaseFrame(robot_model_->getModelFrame());
}

MoveItVisualTools::MoveItVisualTools(const rclcpp::Node::SharedPtr& node, const std::string& base_frame,
                                     const std::string& marker_topic,
                                     planning_scene_monitor::PlanningSceneMonitorPtr psm)
  : RvizVisualTools::RvizVisualTools(base_frame, marker_topic, node)
  , psm_(std::move(psm))
  , robot_state_topic_(DISPLAY_ROBOT_STATE_TOPIC)
  , planning_scene_topic_(PLANNING_SCENE_TOPIC)
  , node_(node)
{
}

MoveItVisualTools::MoveItVisualTools(const rclcpp::Node::SharedPtr& node, const std::string& base_frame,
                                     const std::string& marker_topic, moveit::core::RobotModelConstPtr robot_model)
  : RvizVisualTools::RvizVisualTools(base_frame, marker_topic, node)
  , robot_model_(std::move(robot_model))
  , planning_scene_topic_(PLANNING_SCENE_TOPIC)
  , node_(node)
{
}

bool MoveItVisualTools::loadPlanningSceneMonitor()
{
  // Check if we already have one
  if (psm_)
  {
    RCLCPP_WARN_STREAM(LOGGER, "Will not load a new planning scene monitor when one has "
                               "already been set for Visual Tools");
    return false;
  }
  RCLCPP_INFO_STREAM(LOGGER, "Loading planning scene monitor");

  // Regular version b/c the other one causes problems with recognizing end effectors
  psm_.reset(new planning_scene_monitor::PlanningSceneMonitor(node_, ROBOT_DESCRIPTION, "visual_tools_scene"));

  if (psm_->getPlanningScene())
  {
    // Optional monitors to start:
    // psm_->startWorldGeometryMonitor();
    // psm_->startSceneMonitor("/move_group/monitored_planning_scene");
    // psm_->startStateMonitor("/joint_states", "/attached_collision_object");
    psm_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                       planning_scene_topic_);
    RCLCPP_INFO_STREAM(LOGGER, "Publishing planning scene on " << planning_scene_topic_);

    planning_scene_monitor::LockedPlanningSceneRW planning_scene(psm_);
    planning_scene->setName("visual_tools_scene");
  }
  else
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Planning scene not configured");
    return false;
  }

  return true;
}

bool MoveItVisualTools::processCollisionObjectMsg(const moveit_msgs::msg::CollisionObject& msg,
                                                  const rviz_visual_tools::Colors& color)
{
  // Apply command directly to planning scene to avoid a ROS msg call
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(getPlanningSceneMonitor());
    scene->getCurrentStateNonConst().update();  // TODO: remove hack to prevent bad transforms
    scene->processCollisionObjectMsg(msg);
    scene->setObjectColor(msg.id, getColor(color));
  }
  // Trigger an update

  if (!manual_trigger_update_)
  {
    triggerPlanningSceneUpdate();
  }

  return true;
}

bool MoveItVisualTools::processAttachedCollisionObjectMsg(const moveit_msgs::msg::AttachedCollisionObject& msg)
{
  // Apply command directly to planning scene to avoid a ROS msg call
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(getPlanningSceneMonitor());
    // scene->getCurrentStateNonConst().update(); // hack to prevent bad transforms
    scene->processAttachedCollisionObjectMsg(msg);
  }

  // Trigger an update
  if (!manual_trigger_update_)
  {
    triggerPlanningSceneUpdate();
  }

  return true;
}

bool MoveItVisualTools::moveCollisionObject(const Eigen::Isometry3d& pose, const std::string& name,
                                            const rviz_visual_tools::Colors& color)
{
  return moveCollisionObject(convertPose(pose), name, color);
}

bool MoveItVisualTools::moveCollisionObject(const geometry_msgs::msg::Pose& pose, const std::string& name,
                                            const rviz_visual_tools::Colors& color)
{
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.header.stamp = node_->now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::msg::CollisionObject::MOVE;

  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = pose;

  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::triggerPlanningSceneUpdate()
{
  // TODO(davetcoleman): perhaps switch to using the service call?
  getPlanningSceneMonitor()->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_GEOMETRY);

  return true;
}

bool MoveItVisualTools::loadSharedRobotState()
{
  // Get robot state
  if (!shared_robot_state_)
  {
    // Check if a robot model was passed in
    if (!robot_model_)
    {
      // Fall back on using planning scene monitor.
      robot_model_ = getPlanningSceneMonitor()->getRobotModel();
    }
    shared_robot_state_.reset(new moveit::core::RobotState(robot_model_));

    // TODO(davetcoleman): this seems to be a work around for a weird NaN bug
    shared_robot_state_->setToDefaultValues();
    shared_robot_state_->update(true);

    // hidden_robot_state_.reset(new moveit::core::RobotState(robot_model_));
    // hidden_robot_state_->setToDefaultValues();
    // hidden_robot_state_->update(true);

    hidden_robot_state_.reset(new moveit::core::RobotState(*shared_robot_state_));
    root_robot_state_.reset(new moveit::core::RobotState(*shared_robot_state_));
  }

  return !(shared_robot_state_ == nullptr);
}

moveit::core::RobotStatePtr& MoveItVisualTools::getSharedRobotState()
{
  // Always load the robot state before using
  loadSharedRobotState();
  return shared_robot_state_;
}

moveit::core::RobotModelConstPtr MoveItVisualTools::getRobotModel()
{
  // Always load the robot state before using
  loadSharedRobotState();
  return shared_robot_state_->getRobotModel();
}

bool MoveItVisualTools::loadEEMarker(const moveit::core::JointModelGroup* ee_jmg,
                                     const std::vector<double>& ee_joint_pos)
{
  // Get joint state group
  if (ee_jmg == nullptr)  // make sure EE_GROUP exists
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to find joint model group with address" << ee_jmg);
    return false;
  }

  // Always load the robot state before using
  loadSharedRobotState();
  shared_robot_state_->setToDefaultValues();
  shared_robot_state_->update();

  if (!ee_joint_pos.empty())
  {
    if (ee_joint_pos.size() != ee_jmg->getActiveJointModels().size())
    {
      RCLCPP_ERROR_STREAM(LOGGER, "The number of joint positions given ("
                                      << ee_joint_pos.size() << ") does not match the number of active joints in "
                                      << ee_jmg->getName() << "(" << ee_jmg->getActiveJointModels().size() << ")");
      return false;
    }
    shared_robot_state_->setJointGroupPositions(ee_jmg, ee_joint_pos);
    shared_robot_state_->update(true);
  }

  // Clear old EE markers and EE poses
  ee_markers_map_[ee_jmg].markers.clear();
  ee_poses_map_[ee_jmg].clear();

  // Remember joint state
  ee_joint_pos_map_[ee_jmg] = ee_joint_pos;

  // Keep track of how many unique markers we have between different EEs
  static std::size_t marker_id_offset = 0;

  // Get end effector group

  // Create color to use for EE markers
  std_msgs::msg::ColorRGBA marker_color = getColor(rviz_visual_tools::GREY);

  // Get link names that are in end effector
  const std::vector<std::string>& ee_link_names = ee_jmg->getLinkModelNames();

  // Get EE link markers for Rviz
  shared_robot_state_->getRobotMarkers(ee_markers_map_[ee_jmg], ee_link_names, marker_color, ee_jmg->getName(),
                                       rclcpp::Duration::from_seconds(0));
  RCLCPP_DEBUG_STREAM(LOGGER, "Number of rviz markers in end effector: " << ee_markers_map_[ee_jmg].markers.size());

  // Error check
  if (ee_markers_map_[ee_jmg].markers.empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER,
                        "No links found to visualize in end effector for joint model group: " << ee_jmg->getName());
    return false;
  }

  const std::string& ee_parent_link_name = ee_jmg->getEndEffectorParentGroup().second;
  const moveit::core::LinkModel* ee_parent_link = robot_model_->getLinkModel(ee_parent_link_name);

  Eigen::Isometry3d ee_marker_global_transform = shared_robot_state_->getGlobalLinkTransform(ee_parent_link);
  Eigen::Isometry3d ee_marker_pose;

  // Process each link of the end effector
  for (std::size_t i = 0; i < ee_markers_map_[ee_jmg].markers.size(); ++i)
  {
    // Header
    ee_markers_map_[ee_jmg].markers[i].header.frame_id = base_frame_;

    // Options for meshes
    if (ee_markers_map_[ee_jmg].markers[i].type == visualization_msgs::msg::Marker::MESH_RESOURCE)
    {
      ee_markers_map_[ee_jmg].markers[i].mesh_use_embedded_materials = true;
    }

    // Unique id
    ee_markers_map_[ee_jmg].markers[i].id += marker_id_offset;

    // Copy original marker poses to a vector
    ee_marker_pose = ee_marker_global_transform.inverse() * convertPose(ee_markers_map_[ee_jmg].markers[i].pose);
    ee_poses_map_[ee_jmg].push_back(ee_marker_pose);
  }

  marker_id_offset += ee_markers_map_[ee_jmg].markers.size();

  return true;
}

void MoveItVisualTools::loadTrajectoryPub(const std::string& display_planned_path_topic, bool blocking)
{
  if (pub_display_path_)
    return;

  // Trajectory paths
  pub_display_path_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(display_planned_path_topic, 10);
  RCLCPP_DEBUG_STREAM(LOGGER, "Publishing MoveIt trajectory on topic " << pub_display_path_->get_topic_name());

  // Wait for topic to be ready
  if (blocking)
  {
    const bool subscribed = waitForSubscriber(pub_display_path_, 5.0 /* seconds */);
    if (subscribed)
    {
      RCLCPP_DEBUG_STREAM(LOGGER, "Subscribed to display trajectory topic: " << display_planned_path_topic);
    }
    else
    {
      RCLCPP_WARN_STREAM(LOGGER, "Cannot subscribe to display trajectory topic: " << display_planned_path_topic);
    }
  }
}

void MoveItVisualTools::loadRobotStatePub(const std::string& robot_state_topic, bool blocking)
{
  if (pub_robot_state_)
    return;

  // Update global var if new topic was passed in
  if (!robot_state_topic.empty())
    robot_state_topic_ = robot_state_topic;

  // RobotState Message
  pub_robot_state_ = node_->create_publisher<moveit_msgs::msg::DisplayRobotState>(robot_state_topic, 1);
  RCLCPP_DEBUG_STREAM(LOGGER, "Publishing MoveIt robot state on topic " << pub_robot_state_->get_topic_name());

  // Wait for topic to be ready
  if (blocking)
  {
    const bool subscribed = waitForSubscriber(pub_robot_state_, 5.0 /* seconds */);
    if (subscribed)
    {
      RCLCPP_DEBUG_STREAM(LOGGER, "Subscribed to robot state topic: " << robot_state_topic);
    }
    else
    {
      RCLCPP_WARN_STREAM(LOGGER, "Cannot subscribe to robot state topic: " << robot_state_topic);
    }
  }
}

bool MoveItVisualTools::publishEEMarkers(const geometry_msgs::msg::Pose& pose,
                                         const moveit::core::JointModelGroup* ee_jmg,
                                         const std::vector<double>& ee_joint_pos,
                                         const rviz_visual_tools::Colors& color, const std::string& ns)
{
  // Check if we have not loaded the EE markers
  if (ee_markers_map_[ee_jmg].markers.empty() || ee_poses_map_[ee_jmg].empty() ||
      ee_joint_pos_map_[ee_jmg] != ee_joint_pos)
  {
    if (!loadEEMarker(ee_jmg, ee_joint_pos))
    {
      RCLCPP_ERROR_STREAM(LOGGER, "Unable to publish EE marker, unable to load EE markers");
      return false;
    }
  }

  Eigen::Isometry3d eigen_goal_ee_pose = convertPose(pose);
  Eigen::Isometry3d eigen_this_marker;

  // Process each link of the end effector
  for (std::size_t i = 0; i < ee_markers_map_[ee_jmg].markers.size(); ++i)
  {
    // Make sure ROS is still spinning
    if (!rclcpp::ok())
      break;

    // Header
    ee_markers_map_[ee_jmg].markers[i].header.stamp = node_->now();

    // Namespace
    ee_markers_map_[ee_jmg].markers[i].ns = ns;

    // Lifetime
    ee_markers_map_[ee_jmg].markers[i].lifetime = marker_lifetime_;

    // Color
    if (color != rviz_visual_tools::DEFAULT)
      ee_markers_map_[ee_jmg].markers[i].color = getColor(color);

    // Convert pose
    eigen_this_marker = eigen_goal_ee_pose * ee_poses_map_[ee_jmg][i];
    ee_markers_map_[ee_jmg].markers[i].pose = convertPose(eigen_this_marker);
  }

  // Helper for publishing rviz markers
  // Does not require trigger() because publishing array auto-triggers
  if (!publishMarkers(ee_markers_map_[ee_jmg]))
  {
    RCLCPP_WARN_STREAM(LOGGER, "Unable to publish EE markers");
    return false;
  }

  return true;
}

bool MoveItVisualTools::publishGrasps(const std::vector<moveit_msgs::msg::Grasp>& possible_grasps,
                                      const moveit::core::JointModelGroup* ee_jmg, double animate_speed)
{
  RCLCPP_DEBUG_STREAM(LOGGER, "Visualizing " << possible_grasps.size() << " grasps with EE joint model group "
                                             << ee_jmg->getName());

  // Loop through all grasps
  for (std::size_t i = 0; i < possible_grasps.size(); ++i)
  {
    if (!rclcpp::ok())  // Check that ROS is still ok and that user isn't trying to quit
      break;

    publishEEMarkers(possible_grasps[i].grasp_pose.pose, ee_jmg);

    rclcpp::sleep_for(std::chrono::milliseconds(int(animate_speed * 1000)));
  }

  return true;
}

bool MoveItVisualTools::publishAnimatedGrasps(const std::vector<moveit_msgs::msg::Grasp>& possible_grasps,
                                              const moveit::core::JointModelGroup* ee_jmg, double animate_speed)
{
  RCLCPP_DEBUG_STREAM(LOGGER, "Visualizing " << possible_grasps.size() << " grasps with joint model group "
                                             << ee_jmg->getName() << " at speed " << animate_speed);

  // Loop through all grasps
  for (std::size_t i = 0; i < possible_grasps.size(); ++i)
  {
    if (!rclcpp::ok())  // Check that ROS is still ok and that user isn't trying to quit
      break;

    publishAnimatedGrasp(possible_grasps[i], ee_jmg, animate_speed);
    rclcpp::sleep_for(std::chrono::milliseconds(int(animate_speed * 1000)));
  }

  return true;
}

bool MoveItVisualTools::publishAnimatedGrasp(const moveit_msgs::msg::Grasp& grasp,
                                             const moveit::core::JointModelGroup* ee_jmg, double animate_speed)
{
  // Grasp Pose Variables
  geometry_msgs::msg::Pose grasp_pose = grasp.grasp_pose.pose;

#if 0  // Debug
  publishArrow(grasp_pose, rviz_visual_tools::GREEN);
  publishEEMarkers(grasp_pose, ee_jmg);
  ros::Duration(0.5).sleep();
#endif

  Eigen::Isometry3d grasp_pose_eigen;
  tf2::fromMsg(grasp_pose, grasp_pose_eigen);

  // Pre-grasp pose variables
  geometry_msgs::msg::Pose pre_grasp_pose;
  Eigen::Isometry3d pre_grasp_pose_eigen;

  // Approach direction variables
  Eigen::Vector3d pre_grasp_approach_direction_local;

  // Display Grasp Score
  // std::string text = "Grasp Quality: " +
  // boost::lexical_cast<std::string>(int(grasp.grasp_quality*100)) + "%";
  // publishText(grasp_pose, text);

  // Convert the grasp pose into the frame of reference of the approach/retreat frame_id

  // Animate the movement - for ee approach direction
  double animation_resulution = 0.1;  // the lower the better the resolution
  for (double percent = 0; percent < 1; percent += animation_resulution)
  {
    if (!rclcpp::ok())  // Check that ROS is still ok and that user isn't trying to quit
      break;

    // Copy original grasp pose to pre-grasp pose
    pre_grasp_pose_eigen = grasp_pose_eigen;

    // The direction of the pre-grasp
    // Calculate the current animation position based on the percent
    Eigen::Vector3d pre_grasp_approach_direction = Eigen::Vector3d(
        -1 * grasp.pre_grasp_approach.direction.vector.x * grasp.pre_grasp_approach.min_distance * (1 - percent),
        -1 * grasp.pre_grasp_approach.direction.vector.y * grasp.pre_grasp_approach.min_distance * (1 - percent),
        -1 * grasp.pre_grasp_approach.direction.vector.z * grasp.pre_grasp_approach.min_distance * (1 - percent));

    // Decide if we need to change the approach_direction to the local frame of the end effector
    // orientation
    const std::string& ee_parent_link_name = ee_jmg->getEndEffectorParentGroup().second;

    if (grasp.pre_grasp_approach.direction.header.frame_id == ee_parent_link_name)
    {
      // Apply/compute the approach_direction vector in the local frame of the grasp_pose
      // orientation
      pre_grasp_approach_direction_local = grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
    }
    else
    {
      pre_grasp_approach_direction_local = pre_grasp_approach_direction;  // grasp_pose_eigen.rotation() *
                                                                          // pre_grasp_approach_direction;
    }

    // Update the grasp pose usign the new locally-framed approach_direction
    pre_grasp_pose_eigen.translation() += pre_grasp_approach_direction_local;

    // Convert eigen pre-grasp position back to regular message
    pre_grasp_pose = tf2::toMsg(pre_grasp_pose_eigen);

    publishEEMarkers(pre_grasp_pose, ee_jmg);

    rclcpp::sleep_for(std::chrono::milliseconds(int(animate_speed * 1000)));

    // Pause more at initial pose for debugging purposes
    if (percent == 0)
      rclcpp::sleep_for(std::chrono::milliseconds(int(animate_speed * 2000)));
  }
  return true;
}

bool MoveItVisualTools::publishIKSolutions(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& ik_solutions,
                                           const moveit::core::JointModelGroup* arm_jmg, double display_time)
{
  if (ik_solutions.empty())
  {
    RCLCPP_WARN_STREAM(LOGGER, "Empty ik_solutions vector passed into publishIKSolutions()");
    return false;
  }

  loadSharedRobotState();

  RCLCPP_DEBUG_STREAM(LOGGER, "Visualizing " << ik_solutions.size() << " inverse kinematic solutions");

  // Apply the time to the trajectory
  trajectory_msgs::msg::JointTrajectoryPoint trajectory_pt_timed;

  // Create a trajectory with one point
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  trajectory_msg.joint_trajectory.header.frame_id = base_frame_;
  trajectory_msg.joint_trajectory.joint_names = arm_jmg->getActiveJointModelNames();

  // Overall length of trajectory
  double running_time = 0;

  // Loop through all inverse kinematic solutions
  for (std::size_t i = 0; i < ik_solutions.size(); ++i)
  {
    trajectory_pt_timed = ik_solutions[i];
    trajectory_pt_timed.time_from_start = rclcpp::Duration::from_seconds(running_time);
    trajectory_msg.joint_trajectory.points.push_back(trajectory_pt_timed);

    running_time += display_time;
  }

  // Re-add final position so the last point is displayed fully
  trajectory_pt_timed = trajectory_msg.joint_trajectory.points.back();
  trajectory_pt_timed.time_from_start = rclcpp::Duration::from_seconds(running_time);
  trajectory_msg.joint_trajectory.points.push_back(trajectory_pt_timed);

  return publishTrajectoryPath(trajectory_msg, shared_robot_state_, true);
}

bool MoveItVisualTools::removeAllCollisionObjects()
{
  // Apply command directly to planning scene to avoid a ROS msg call
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(getPlanningSceneMonitor());
    scene->removeAllCollisionObjects();
  }

  return true;
}

bool MoveItVisualTools::cleanupCO(const std::string& name)
{
  // Clean up old collision objects
  moveit_msgs::msg::CollisionObject co;
  co.header.stamp = node_->now();
  co.header.frame_id = base_frame_;
  co.id = name;
  co.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  return processCollisionObjectMsg(co);
}

bool MoveItVisualTools::cleanupACO(const std::string& /*name*/)
{
  // Clean up old attached collision object
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.object.header.stamp = node_->now();
  aco.object.header.frame_id = base_frame_;

  // aco.object.id = name;
  aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  return processAttachedCollisionObjectMsg(aco);
}

bool MoveItVisualTools::attachCO(const std::string& name, const std::string& ee_parent_link)
{
  // Attach a collision object
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.object.header.stamp = node_->now();
  aco.object.header.frame_id = base_frame_;

  aco.object.id = name;
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Link to attach the object to
  aco.link_name = ee_parent_link;

  return processAttachedCollisionObjectMsg(aco);
}

bool MoveItVisualTools::publishCollisionBlock(const geometry_msgs::msg::Pose& block_pose, const std::string& name,
                                              double block_size, const rviz_visual_tools::Colors& color)
{
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.header.stamp = node_->now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::BOX>());
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = block_size;
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = block_size;
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = block_size;
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = block_pose;

  // RCLCPP_INFO_STREAM(LOGGER,"CollisionObject: \n " << collision_obj);
  // RCLCPP_DEBUG_STREAM(LOGGER,"Published collision object " << name);
  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::publishCollisionCuboid(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2,
                                               const std::string& name, const rviz_visual_tools::Colors& color)
{
  return publishCollisionCuboid(convertPoint(point1), convertPoint(point2), name, color);
}

bool MoveItVisualTools::publishCollisionCuboid(const geometry_msgs::msg::Point& point1,
                                               const geometry_msgs::msg::Point& point2, const std::string& name,
                                               const rviz_visual_tools::Colors& color)
{
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.header.stamp = node_->now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Calculate center pose
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0].position.x = (point1.x - point2.x) / 2.0 + point2.x;
  collision_obj.primitive_poses[0].position.y = (point1.y - point2.y) / 2.0 + point2.y;
  collision_obj.primitive_poses[0].position.z = (point1.z - point2.z) / 2.0 + point2.z;

  // Calculate scale
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::BOX>());
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = fabs(point1.x - point2.x);
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = fabs(point1.y - point2.y);
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = fabs(point1.z - point2.z);

  // Prevent scale from being zero
  if (!collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X])
    collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = rviz_visual_tools::SMALL_SCALE;
  if (!collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y])
    collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = rviz_visual_tools::SMALL_SCALE;
  if (!collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z])
    collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = rviz_visual_tools::SMALL_SCALE;

  // RCLCPP_INFO_STREAM(LOGGER,"CollisionObject: \n " << collision_obj);
  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::publishCollisionCuboid(const Eigen::Isometry3d& pose, double width, double depth, double height,
                                               const std::string& name, const rviz_visual_tools::Colors& color)
{
  geometry_msgs::msg::Pose pose_msg = tf2::toMsg(pose);
  return publishCollisionCuboid(pose_msg, width, depth, height, name, color);
}

bool MoveItVisualTools::publishCollisionCuboid(const geometry_msgs::msg::Pose& pose, double width, double depth,
                                               double height, const std::string& name,
                                               const rviz_visual_tools::Colors& color)
{
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.header.stamp = node_->now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Calculate center pose
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = pose;

  // Calculate scale
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::BOX>());
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = width;
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = depth;
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = height;

  // Prevent scale from being zero
  if (!collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X])
    collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = rviz_visual_tools::SMALL_SCALE;
  if (!collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y])
    collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = rviz_visual_tools::SMALL_SCALE;
  if (!collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z])
    collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = rviz_visual_tools::SMALL_SCALE;

  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::publishCollisionFloor(double z, const std::string& plane_name,
                                              const rviz_visual_tools::Colors& color)
{
  // Instead just generate a rectangle
  geometry_msgs::msg::Point point1;
  geometry_msgs::msg::Point point2;

  point1.x = rviz_visual_tools::LARGE_SCALE;
  point1.y = rviz_visual_tools::LARGE_SCALE;
  point1.z = z;

  point2.x = -rviz_visual_tools::LARGE_SCALE;
  point2.y = -rviz_visual_tools::LARGE_SCALE;
  point2.z = z - rviz_visual_tools::SMALL_SCALE;

  return publishCollisionCuboid(point1, point2, plane_name, color);
}

bool MoveItVisualTools::publishCollisionCylinder(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b,
                                                 const std::string& object_name, double radius,
                                                 const rviz_visual_tools::Colors& color)
{
  return publishCollisionCylinder(convertPoint(a), convertPoint(b), object_name, radius, color);
}

bool MoveItVisualTools::publishCollisionCylinder(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                                                 const std::string& object_name, double radius,
                                                 const rviz_visual_tools::Colors& color)
{
  // Distance between two points
  double height = (a - b).lpNorm<2>();

  // Find center point
  Eigen::Vector3d pt_center = getCenterPoint(a, b);

  // Create vector
  Eigen::Isometry3d pose;
  pose = getVectorBetweenPoints(pt_center, b);

  // Convert pose to be normal to cylindar axis
  Eigen::Isometry3d rotation;
  rotation = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY());
  pose = pose * rotation;

  return publishCollisionCylinder(pose, object_name, radius, height, color);
}

bool MoveItVisualTools::publishCollisionCylinder(const Eigen::Isometry3d& object_pose, const std::string& object_name,
                                                 double radius, double height, const rviz_visual_tools::Colors& color)
{
  return publishCollisionCylinder(convertPose(object_pose), object_name, radius, height, color);
}

bool MoveItVisualTools::publishCollisionCylinder(const geometry_msgs::msg::Pose& object_pose,
                                                 const std::string& object_name, double radius, double height,
                                                 const rviz_visual_tools::Colors& color)
{
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.header.stamp = node_->now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = object_name;
  collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::CYLINDER>());
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = height;
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = radius;
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = object_pose;

  // RCLCPP_INFO_STREAM(LOGGER,"CollisionObject: \n " << collision_obj);
  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::publishCollisionMesh(const Eigen::Isometry3d& object_pose, const std::string& object_name,
                                             const std::string& mesh_path, const rviz_visual_tools::Colors& color)
{
  return publishCollisionMesh(convertPose(object_pose), object_name, mesh_path, color);
}

bool MoveItVisualTools::publishCollisionMesh(const geometry_msgs::msg::Pose& object_pose,
                                             const std::string& object_name, const std::string& mesh_path,
                                             const rviz_visual_tools::Colors& color)
{
  shapes::Shape* mesh = shapes::createMeshFromResource(mesh_path);  // make sure its prepended by file://
  shapes::ShapeMsg shape_msg;  // this is a boost::variant type from shape_messages.h
  if (!mesh || !shapes::constructMsgFromShape(mesh, shape_msg))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to create mesh shape message from resource " << mesh_path);
    return false;
  }

  if (!publishCollisionMesh(object_pose, object_name, boost::get<shape_msgs::msg::Mesh>(shape_msg), color))
    return false;

  RCLCPP_DEBUG(LOGGER, "Loaded mesh from '%s'", mesh_path.c_str());
  return true;
}

bool MoveItVisualTools::publishCollisionMesh(const Eigen::Isometry3d& object_pose, const std::string& object_name,
                                             const shape_msgs::msg::Mesh& mesh_msg,
                                             const rviz_visual_tools::Colors& color)
{
  return publishCollisionMesh(convertPose(object_pose), object_name, mesh_msg, color);
}

bool MoveItVisualTools::publishCollisionMesh(const geometry_msgs::msg::Pose& object_pose,
                                             const std::string& object_name, const shape_msgs::msg::Mesh& mesh_msg,
                                             const rviz_visual_tools::Colors& color)
{
  // Create collision message
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.header.stamp = node_->now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = object_name;
  collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  collision_obj.mesh_poses.resize(1);
  collision_obj.mesh_poses[0] = object_pose;
  collision_obj.meshes.resize(1);
  collision_obj.meshes[0] = mesh_msg;

  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::publishCollisionGraph(const graph_msgs::msg::GeometryGraph& graph,
                                              const std::string& object_name, double radius,
                                              const rviz_visual_tools::Colors& color)
{
  RCLCPP_INFO_STREAM(LOGGER, "Preparing to create collision graph");

  // The graph is one collision object with many primitives
  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.header.stamp = node_->now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = object_name;
  collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;

  // Track which pairs of nodes we've already connected since graph is bi-directional
  typedef std::pair<std::size_t, std::size_t> node_ids;
  std::set<node_ids> added_edges;
  std::pair<std::set<node_ids>::iterator, bool> return_value;

  Eigen::Vector3d a, b;
  for (std::size_t i = 0; i < graph.nodes.size(); ++i)
  {
    for (std::size_t j = 0; j < graph.edges[i].node_ids.size(); ++j)
    {
      // Check if we've already added this pair of nodes (edge)
      return_value = added_edges.insert(node_ids(i, j));
      if (!return_value.second)
      {
        // Element already existed in set, so don't add a new collision object
      }
      else
      {
        // Create a cylinder from two points
        a = convertPoint(graph.nodes[i]);
        b = convertPoint(graph.nodes[graph.edges[i].node_ids[j]]);

        // add other direction of edge
        added_edges.insert(node_ids(j, i));

        // Distance between two points
        double height = (a - b).lpNorm<2>();

        // Find center point
        Eigen::Vector3d pt_center = getCenterPoint(a, b);

        // Create vector
        Eigen::Isometry3d pose;
        pose = getVectorBetweenPoints(pt_center, b);

        // Convert pose to be normal to cylindar axis
        Eigen::Isometry3d rotation;
        rotation = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY());
        pose = pose * rotation;

        // Create the solid primitive
        shape_msgs::msg::SolidPrimitive cylinder;
        cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        cylinder.dimensions.resize(
            geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::CYLINDER>());
        cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = height;
        cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = radius;

        // Add to the collision object
        collision_obj.primitives.push_back(cylinder);

        // Add the pose
        collision_obj.primitive_poses.push_back(convertPose(pose));
      }
    }
  }

  return processCollisionObjectMsg(collision_obj, color);
}

void MoveItVisualTools::getCollisionWallMsg(double x, double y, double z, double angle, double width, double height,
                                            const std::string& name, moveit_msgs::msg::CollisionObject& collision_obj)
{
  collision_obj.header.stamp = node_->now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::BOX>());

  geometry_msgs::msg::Pose rec_pose;

  // ----------------------------------------------------------------------------------
  // Name
  collision_obj.id = name;

  double depth = 0.1;

  // Position
  rec_pose.position.x = x;
  rec_pose.position.y = y;
  rec_pose.position.z = height / 2 + z;

  // Size
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = depth;
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = width;
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = height;
  // ----------------------------------------------------------------------------------

  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(static_cast<double>(angle), Eigen::Vector3d::UnitZ()));
  rec_pose.orientation.x = quat.x();
  rec_pose.orientation.y = quat.y();
  rec_pose.orientation.z = quat.z();
  rec_pose.orientation.w = quat.w();

  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = rec_pose;
}

bool MoveItVisualTools::publishCollisionWall(double x, double y, double angle, double width, double height,
                                             const std::string& name, const rviz_visual_tools::Colors& color)
{
  return publishCollisionWall(x, y, 0.0, angle, width, height, name, color);
}

bool MoveItVisualTools::publishCollisionWall(double x, double y, double z, double angle, double width, double height,
                                             const std::string& name, const rviz_visual_tools::Colors& color)
{
  moveit_msgs::msg::CollisionObject collision_obj;
  getCollisionWallMsg(x, y, z, angle, width, height, name, collision_obj);

  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::publishCollisionTable(double x, double y, double z, double angle, double width, double height,
                                              double depth, const std::string& name,
                                              const rviz_visual_tools::Colors& color)
{
  geometry_msgs::msg::Pose table_pose;

  // Center of table
  table_pose.position.x = x;
  table_pose.position.y = y;
  table_pose.position.z = z + height / 2.0;

  // Orientation
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(static_cast<double>(angle), Eigen::Vector3d::UnitZ()));
  table_pose.orientation.x = quat.x();
  table_pose.orientation.y = quat.y();
  table_pose.orientation.z = quat.z();
  table_pose.orientation.w = quat.w();

  moveit_msgs::msg::CollisionObject collision_obj;
  collision_obj.header.stamp = node_->now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::solidPrimitiveDimCount<shape_msgs::msg::SolidPrimitive::BOX>());

  // Size
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = depth;
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = width;
  collision_obj.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = height;

  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = table_pose;
  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::loadCollisionSceneFromFile(const std::string& path)
{
  return loadCollisionSceneFromFile(path, Eigen::Isometry3d::Identity());
}

bool MoveItVisualTools::loadCollisionSceneFromFile(const std::string& path, const Eigen::Isometry3d& offset)
{
  // Open file
  std::ifstream fin(path.c_str());
  if (fin.good())
  {
    // Load directly to the planning scene
    planning_scene_monitor::LockedPlanningSceneRW scene(getPlanningSceneMonitor());
    {
      if (scene)
      {
        scene->loadGeometryFromStream(fin, offset);
      }
      else
      {
        RCLCPP_WARN_STREAM(LOGGER, "Unable to get locked planning scene RW");
        return false;
      }
    }
    RCLCPP_INFO(LOGGER, "Loaded scene geometry from '%s'", path.c_str());
  }
  else
    RCLCPP_WARN(LOGGER, "Unable to load scene geometry from '%s'", path.c_str());

  fin.close();

  return triggerPlanningSceneUpdate();
}

bool MoveItVisualTools::publishWorkspaceParameters(const moveit_msgs::msg::WorkspaceParameters& params)
{
  return publishCuboid(convertPoint(params.min_corner), convertPoint(params.max_corner), rviz_visual_tools::TRANSLUCENT,
                       "Planning_Workspace", 1);
}

bool MoveItVisualTools::checkAndPublishCollision(const moveit::core::RobotState& robot_state,
                                                 const planning_scene::PlanningScene* planning_scene,
                                                 const rviz_visual_tools::Colors& highlight_link_color,
                                                 const rviz_visual_tools::Colors& contact_point_color)
{
  // Compute the contacts if any
  collision_detection::CollisionRequest c_req;
  collision_detection::CollisionResult c_res;
  c_req.contacts = true;
  c_req.max_contacts = 10;
  c_req.max_contacts_per_pair = 3;
  c_req.verbose = true;

  // Check for collisions
  planning_scene->checkCollision(c_req, c_res, robot_state);
  std::vector<std::string> highlight_links;
  for (const auto& contact : c_res.contacts)
  {
    highlight_links.push_back(contact.first.first);
    highlight_links.push_back(contact.first.second);
  }

  publishRobotState(robot_state, highlight_link_color, highlight_links);
  publishContactPoints(c_res.contacts, planning_scene, contact_point_color);
  return c_res.collision;
}

bool MoveItVisualTools::publishContactPoints(const moveit::core::RobotState& robot_state,
                                             const planning_scene::PlanningScene* planning_scene,
                                             const rviz_visual_tools::Colors& color)
{
  // Compute the contacts if any
  collision_detection::CollisionRequest c_req;
  collision_detection::CollisionResult c_res;
  c_req.contacts = true;
  c_req.max_contacts = 10;
  c_req.max_contacts_per_pair = 3;
  c_req.verbose = true;

  // Check for collisions
  planning_scene->checkCollision(c_req, c_res, robot_state);
  return publishContactPoints(c_res.contacts, planning_scene, color);
}

bool MoveItVisualTools::publishContactPoints(const collision_detection::CollisionResult::ContactMap& contacts,
                                             const planning_scene::PlanningScene* planning_scene,
                                             const rviz_visual_tools::Colors& color)
{
  // Display
  if (!contacts.empty())
  {
    visualization_msgs::msg::MarkerArray arr;
    collision_detection::getCollisionMarkersFromContacts(arr, planning_scene->getPlanningFrame(), contacts);
    RCLCPP_INFO_STREAM(LOGGER, "Completed listing of explanations for invalid states.");

    // Check for markers
    if (arr.markers.empty())
      return true;

    // Convert markers to same namespace and other custom stuff
    for (std::size_t i = 0; i < arr.markers.size(); ++i)
    {
      arr.markers[i].ns = "Collision";
      arr.markers[i].id = i;
      arr.markers[i].scale.x = 0.04;
      arr.markers[i].scale.y = 0.04;
      arr.markers[i].scale.z = 0.04;
      arr.markers[i].color = getColor(color);
    }

    return publishMarkers(arr);
  }
  return true;
}

bool MoveItVisualTools::publishTrajectoryPoint(const trajectory_msgs::msg::JointTrajectoryPoint& trajectory_pt,
                                               const std::string& planning_group, double display_time)
{
  // Get joint state group
  const moveit::core::JointModelGroup* jmg = robot_model_->getJointModelGroup(planning_group);

  if (jmg == nullptr)  // not found
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Could not find joint model group " << planning_group);
    return false;
  }

  // Apply the time to the trajectory
  trajectory_msgs::msg::JointTrajectoryPoint trajectory_pt_timed = trajectory_pt;
  trajectory_pt_timed.time_from_start = rclcpp::Duration::from_seconds(display_time);

  // Create a trajectory with one point
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  trajectory_msg.joint_trajectory.header.frame_id = base_frame_;
  trajectory_msg.joint_trajectory.joint_names = jmg->getJointModelNames();
  trajectory_msg.joint_trajectory.points.push_back(trajectory_pt);
  trajectory_msg.joint_trajectory.points.push_back(trajectory_pt_timed);

  return publishTrajectoryPath(trajectory_msg, shared_robot_state_, true);
}

bool MoveItVisualTools::publishTrajectoryPath(const std::vector<moveit::core::RobotStatePtr>& trajectory,
                                              const moveit::core::JointModelGroup* jmg, double speed, bool blocking)
{
  // Copy the vector of RobotStates to a RobotTrajectory
  robot_trajectory::RobotTrajectoryPtr robot_trajectory(
      new robot_trajectory::RobotTrajectory(robot_model_, jmg->getName()));

  double duration_from_previous = 0;
  for (std::size_t k = 0; k < trajectory.size(); ++k)
  {
    duration_from_previous += speed;
    robot_trajectory->addSuffixWayPoint(trajectory[k], duration_from_previous);
  }

  // Convert trajectory to a message
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);

  // Use first trajectory point as reference state
  moveit_msgs::msg::RobotState robot_state_msg;
  if (!trajectory.empty())
    moveit::core::robotStateToRobotStateMsg(*trajectory[0], robot_state_msg);

  return publishTrajectoryPath(trajectory_msg, robot_state_msg, blocking);
}

bool MoveItVisualTools::publishTrajectoryPath(const robot_trajectory::RobotTrajectoryPtr& trajectory, bool blocking)
{
  return publishTrajectoryPath(*trajectory, blocking);
}

bool MoveItVisualTools::publishTrajectoryPath(const robot_trajectory::RobotTrajectory& trajectory, bool blocking)
{
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  trajectory.getRobotTrajectoryMsg(trajectory_msg);

  // Add time from start if none specified
  if (trajectory_msg.joint_trajectory.points.size() > 1)
  {
    if (trajectory_msg.joint_trajectory.points[1].time_from_start ==
        rclcpp::Duration::from_seconds(0))  // assume no timestamps exist
    {
      for (std::size_t i = 0; i < trajectory_msg.joint_trajectory.points.size(); ++i)
      {
        trajectory_msg.joint_trajectory.points[i].time_from_start = rclcpp::Duration::from_seconds(i * 2);  // 1 hz
      }
    }
  }

  // Use first trajectory point as reference state
  moveit_msgs::msg::RobotState robot_state_msg;
  if (!trajectory.empty())
    moveit::core::robotStateToRobotStateMsg(trajectory.getFirstWayPoint(), robot_state_msg);

  return publishTrajectoryPath(trajectory_msg, robot_state_msg, blocking);
}

bool MoveItVisualTools::publishTrajectoryPath(const moveit_msgs::msg::RobotTrajectory& trajectory_msg,
                                              const moveit::core::RobotStateConstPtr& robot_state, bool blocking)
{
  return publishTrajectoryPath(trajectory_msg, *robot_state, blocking);
}

bool MoveItVisualTools::publishTrajectoryPath(const moveit_msgs::msg::RobotTrajectory& trajectory_msg,
                                              const moveit::core::RobotState& robot_state, bool blocking)
{
  // Convert the robot state to a ROS message
  moveit_msgs::msg::RobotState robot_state_msg;
  moveit::core::robotStateToRobotStateMsg(robot_state, robot_state_msg);
  return publishTrajectoryPath(trajectory_msg, robot_state_msg, blocking);
}

bool MoveItVisualTools::publishTrajectoryPath(const moveit_msgs::msg::RobotTrajectory& trajectory_msg,
                                              const moveit_msgs::msg::RobotState& robot_state, bool blocking)
{
  // Check if we have enough points
  if (trajectory_msg.joint_trajectory.points.empty())
  {
    RCLCPP_WARN_STREAM(LOGGER, "Unable to publish trajectory path because trajectory has zero points");
    return false;
  }

  // Create the message
  moveit_msgs::msg::DisplayTrajectory display_trajectory_msg;
  display_trajectory_msg.model_id = robot_model_->getName();
  display_trajectory_msg.trajectory.resize(1);
  display_trajectory_msg.trajectory[0] = trajectory_msg;
  display_trajectory_msg.trajectory_start = robot_state;

  publishTrajectoryPath(display_trajectory_msg);

  // Wait the duration of the trajectory
  if (blocking)
  {
    auto duration = std::chrono::milliseconds(1000 * trajectory_msg.joint_trajectory.points.back().time_from_start.sec);
    // If trajectory has not been parameterized, assume each waypoint takes 50 milliseconds (based on Rviz)
    if (duration.count() < std::numeric_limits<double>::epsilon() * 1000 /* ms */)
    {
      duration = std::chrono::milliseconds(50 * trajectory_msg.joint_trajectory.points.size());
    }
    RCLCPP_DEBUG_STREAM(LOGGER, "Waiting for trajectory animation " << duration.count() * 1000 << " seconds");

    // Check if ROS is ok in intervals
    auto counter = 0ms;
    const auto CHECK_TIME_INTERVAL = 250ms;  // check every fourth second
    while (rclcpp::ok() && counter <= duration)
    {
      counter += CHECK_TIME_INTERVAL;
      rclcpp::sleep_for(CHECK_TIME_INTERVAL);
    }
  }

  return true;
}

void MoveItVisualTools::publishTrajectoryPath(const moveit_msgs::msg::DisplayTrajectory& display_trajectory_msg)
{
  // Publish message
  loadTrajectoryPub();  // always call this before publishing
  pub_display_path_->publish(display_trajectory_msg);
}

bool MoveItVisualTools::publishTrajectoryLine(const moveit_msgs::msg::RobotTrajectory& trajectory_msg,
                                              const moveit::core::LinkModel* ee_parent_link,
                                              const moveit::core::JointModelGroup* arm_jmg,
                                              const rviz_visual_tools::Colors& color)
{
  // Error check
  if (!arm_jmg)
  {
    RCLCPP_FATAL_STREAM(LOGGER, "arm_jmg is NULL");
    return false;
  }

  // Always load the robot state before using
  loadSharedRobotState();

  // Convert trajectory into a series of RobotStates
  robot_trajectory::RobotTrajectoryPtr robot_trajectory(
      new robot_trajectory::RobotTrajectory(robot_model_, arm_jmg->getName()));
  robot_trajectory->setRobotTrajectoryMsg(*shared_robot_state_, trajectory_msg);

  return publishTrajectoryLine(robot_trajectory, ee_parent_link, color);
}

bool MoveItVisualTools::publishTrajectoryLine(const robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
                                              const moveit::core::LinkModel* ee_parent_link,
                                              const rviz_visual_tools::Colors& color)
{
  return publishTrajectoryLine(*robot_trajectory, ee_parent_link, color);
}

bool MoveItVisualTools::publishTrajectoryLine(const robot_trajectory::RobotTrajectory& robot_trajectory,
                                              const moveit::core::LinkModel* ee_parent_link,
                                              const rviz_visual_tools::Colors& color)
{
  // Error check
  if (!ee_parent_link)
  {
    RCLCPP_FATAL_STREAM(LOGGER, "ee_parent_link is NULL");
    return false;
  }

  // Point location datastructure
  EigenSTL::vector_Vector3d path;

  // Visualize end effector position of cartesian path
  for (std::size_t i = 0; i < robot_trajectory.getWayPointCount(); ++i)
  {
    const Eigen::Isometry3d& tip_pose = robot_trajectory.getWayPoint(i).getGlobalLinkTransform(ee_parent_link);

    // Error Check
    if (tip_pose.translation().x() != tip_pose.translation().x())
    {
      RCLCPP_ERROR_STREAM(LOGGER, "NAN DETECTED AT TRAJECTORY POINT i=" << i);
      return false;
    }

    path.push_back(tip_pose.translation());
    publishSphere(tip_pose, color, rviz_visual_tools::MEDIUM);
  }

  const double radius = 0.005;
  publishPath(path, color, radius);

  return true;
}

bool MoveItVisualTools::publishTrajectoryLine(const moveit_msgs::msg::RobotTrajectory& trajectory_msg,
                                              const moveit::core::JointModelGroup* arm_jmg,
                                              const rviz_visual_tools::Colors& color)
{
  if (!arm_jmg)
  {
    RCLCPP_FATAL_STREAM(LOGGER, "arm_jmg is NULL");
    return false;
  }

  std::vector<const moveit::core::LinkModel*> tips;
  if (!arm_jmg->getEndEffectorTips(tips) || tips.empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to get end effector tips from jmg");
    return false;
  }

  // For each end effector
  for (const moveit::core::LinkModel* ee_parent_link : tips)
  {
    if (!publishTrajectoryLine(trajectory_msg, ee_parent_link, arm_jmg, color))
      return false;
  }

  return true;
}

bool MoveItVisualTools::publishTrajectoryLine(const robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
                                              const moveit::core::JointModelGroup* arm_jmg,
                                              const rviz_visual_tools::Colors& color)
{
  return publishTrajectoryLine(*robot_trajectory, arm_jmg, color);
}

bool MoveItVisualTools::publishTrajectoryLine(const robot_trajectory::RobotTrajectory& robot_trajectory,
                                              const moveit::core::JointModelGroup* arm_jmg,
                                              const rviz_visual_tools::Colors& color)
{
  if (!arm_jmg)
  {
    RCLCPP_FATAL_STREAM(LOGGER, "arm_jmg is NULL");
    return false;
  }

  std::vector<const moveit::core::LinkModel*> tips;
  if (!arm_jmg->getEndEffectorTips(tips) || tips.empty())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Unable to get end effector tips from jmg");
    return false;
  }

  // For each end effector
  for (const moveit::core::LinkModel* ee_parent_link : tips)
  {
    if (!publishTrajectoryLine(robot_trajectory, ee_parent_link, color))
      return false;
  }

  return true;
}

bool MoveItVisualTools::publishTrajectoryPoints(const std::vector<moveit::core::RobotStatePtr>& robot_state_trajectory,
                                                const moveit::core::LinkModel* ee_parent_link,
                                                const rviz_visual_tools::Colors& color)
{
  // Visualize end effector position of cartesian path
  for (std::size_t i = 0; i < robot_state_trajectory.size(); ++i)
  {
    const Eigen::Isometry3d& tip_pose = robot_state_trajectory[i]->getGlobalLinkTransform(ee_parent_link);

    publishSphere(tip_pose, color);
  }
  return true;
}

void MoveItVisualTools::enableRobotStateRootOffet(const Eigen::Isometry3d& offset)
{
  robot_state_root_offset_enabled_ = true;
  robot_state_root_offset_ = offset;
}

void MoveItVisualTools::disableRobotStateRootOffet()
{
  robot_state_root_offset_enabled_ = false;
}

bool MoveItVisualTools::publishRobotState(const trajectory_msgs::msg::JointTrajectoryPoint& trajectory_pt,
                                          const moveit::core::JointModelGroup* jmg,
                                          const rviz_visual_tools::Colors& color)
{
  return publishRobotState(trajectory_pt.positions, jmg, color);
}

bool MoveItVisualTools::publishRobotState(const std::vector<double>& joint_positions,
                                          const moveit::core::JointModelGroup* jmg,
                                          const rviz_visual_tools::Colors& color)
{
  // Always load the robot state before using
  loadSharedRobotState();

  // Set robot state
  shared_robot_state_->setToDefaultValues();  // reset the state just in case
  shared_robot_state_->setJointGroupPositions(jmg, joint_positions);

  // Publish robot state
  return publishRobotState(*shared_robot_state_, color);
}

bool MoveItVisualTools::publishRobotState(const moveit::core::RobotStatePtr& robot_state,
                                          const rviz_visual_tools::Colors& color,
                                          const std::vector<std::string>& highlight_links)
{
  return publishRobotState(*robot_state.get(), color, highlight_links);
}

bool MoveItVisualTools::publishRobotState(const moveit::core::RobotState& robot_state,
                                          const rviz_visual_tools::Colors& color,
                                          const std::vector<std::string>& highlight_links)
{
  // when only a subset of links should be colored, the default message is used rather than the cached solid robot
  // messages
  rviz_visual_tools::Colors base_color = color;
  if (!highlight_links.empty())
    base_color = rviz_visual_tools::DEFAULT;

  // Reference to the correctly colored version of message (they are cached)
  // May not exist yet but this will create it
  moveit_msgs::msg::DisplayRobotState& display_robot_msg = display_robot_msgs_[base_color];

  // Check if a robot state message already exists for this color
  if (display_robot_msg.highlight_links.empty())  // has not been colored yet, lets create that
  {
    if (color != rviz_visual_tools::DEFAULT)  // ignore color highlights when set to default
    {
      // Get links names
      const std::vector<std::string>& link_names =
          highlight_links.empty() ? robot_state.getRobotModel()->getLinkModelNamesWithCollisionGeometry() :
                                    highlight_links;
      display_robot_msg.highlight_links.resize(link_names.size());

      // Get color
      const std_msgs::msg::ColorRGBA& color_rgba = getColor(color);

      // Color every link
      for (std::size_t i = 0; i < link_names.size(); ++i)
      {
        display_robot_msg.highlight_links[i].id = link_names[i];
        display_robot_msg.highlight_links[i].color = color_rgba;
      }
    }
  }

  // Apply the offset
  if (robot_state_root_offset_enabled_)
  {
    loadSharedRobotState();

    // Copy robot state
    *shared_robot_state_ = robot_state;

    applyVirtualJointTransform(*shared_robot_state_, robot_state_root_offset_);

    // Convert state to message
    moveit::core::robotStateToRobotStateMsg(*shared_robot_state_, display_robot_msg.state);
  }
  else
  {
    // Convert state to message
    moveit::core::robotStateToRobotStateMsg(robot_state, display_robot_msg.state);
  }

  // Publish
  publishRobotState(display_robot_msg);

  // remove highlight links from default message
  if (!highlight_links.empty())
    display_robot_msg.highlight_links.clear();

  return true;
}

void MoveItVisualTools::publishRobotState(const moveit_msgs::msg::DisplayRobotState& robot_state_msg)
{
  loadRobotStatePub();
  pub_robot_state_->publish(robot_state_msg);
}

bool MoveItVisualTools::hideRobot()
{
  moveit_msgs::msg::DisplayRobotState display_robot_state_msg;
  // Hide the robot state
  display_robot_state_msg.hide = true;

  // Publish
  publishRobotState(display_robot_state_msg);
  return true;
}

void MoveItVisualTools::showJointLimits(const moveit::core::RobotStatePtr& robot_state)
{
  const std::vector<const moveit::core::JointModel*>& joints = robot_model_->getActiveJointModels();

  // Loop through joints
  for (std::size_t i = 0; i < joints.size(); ++i)
  {
    // Assume all joints have only one variable
    if (joints[i]->getVariableCount() > 1)
    {
      // RCLCPP_WARN_STREAM(LOGGER, "Unable to handle joints with more than one var, skipping '"
      //<< joints[i]->getName() << "'");
      continue;
    }

    double current_value = robot_state->getVariablePosition(joints[i]->getName());

    // check if bad position
    bool out_of_bounds = !robot_state->satisfiesBounds(joints[i]);

    const moveit::core::VariableBounds& bound = joints[i]->getVariableBounds()[0];

    if (out_of_bounds)
      std::cout << MOVEIT_CONSOLE_COLOR_RED;

    std::cout << "   " << std::fixed << std::setprecision(5) << bound.min_position_ << "\t";
    double delta = bound.max_position_ - bound.min_position_;
    // std::cout << "delta: " << delta << " ";
    double step = delta / 20.0;

    bool marker_shown = false;
    for (double value = bound.min_position_; value < bound.max_position_; value += step)
    {
      // show marker of current value
      if (!marker_shown && current_value < value)
      {
        std::cout << "|";
        marker_shown = true;
      }
      else
        std::cout << "-";
    }
    // show max position
    std::cout << " \t" << std::fixed << std::setprecision(5) << bound.max_position_ << "  \t" << joints[i]->getName()
              << " current: " << std::fixed << std::setprecision(5) << current_value << std::endl;

    if (out_of_bounds)
      std::cout << MOVEIT_CONSOLE_COLOR_RESET;
  }
}

/**
 * --------------------------------------------------------------------------------------
 * Private Functions
 * --------------------------------------------------------------------------------------
 */

planning_scene_monitor::PlanningSceneMonitorPtr MoveItVisualTools::getPlanningSceneMonitor()
{
  if (!psm_)
  {
    RCLCPP_INFO_STREAM(LOGGER, "No planning scene passed into moveit_visual_tools, creating one.");
    loadPlanningSceneMonitor();
  }
  return psm_;
}

bool MoveItVisualTools::checkForVirtualJoint(const moveit::core::RobotState& robot_state)
{
  static const std::string VJOINT_NAME = "virtual_joint";

  // Check if joint exists
  if (!robot_state.getRobotModel()->hasJointModel(VJOINT_NAME))
  {
    RCLCPP_WARN_STREAM(LOGGER, "Joint '" << VJOINT_NAME << "' does not exist.");
    return false;
  }

  // Check if variables exist
  if (!robot_state.getRobotModel()->getJointModel(VJOINT_NAME)->hasVariable(VJOINT_NAME + "/trans_x"))
  {
    // Debug
    RCLCPP_WARN_STREAM(LOGGER, "Variables for joint '" << VJOINT_NAME
                                                       << "' do not exist. Try making this vjoint "
                                                          "floating");
    RCLCPP_WARN_STREAM(LOGGER, "The only available joint variables are:");
    const std::vector<std::string>& var_names =
        robot_state.getRobotModel()->getJointModel(VJOINT_NAME)->getVariableNames();
    std::copy(var_names.begin(), var_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    return false;
  }

  return true;
}

bool MoveItVisualTools::applyVirtualJointTransform(moveit::core::RobotState& robot_state,
                                                   const Eigen::Isometry3d& offset)
{
  static const std::string VJOINT_NAME = "virtual_joint";

  // Error check
  if (!checkForVirtualJoint(robot_state))
  {
    RCLCPP_WARN_STREAM(LOGGER, "Unable to apply virtual joint transform, hideRobot() functionality is disabled");
    return false;
  }

  // Apply translation
  robot_state.setVariablePosition(VJOINT_NAME + "/trans_x", offset.translation().x());
  robot_state.setVariablePosition(VJOINT_NAME + "/trans_y", offset.translation().y());
  robot_state.setVariablePosition(VJOINT_NAME + "/trans_z", offset.translation().z());

  // Apply rotation
  Eigen::Quaterniond q(offset.rotation());
  robot_state.setVariablePosition(VJOINT_NAME + "/rot_x", q.x());
  robot_state.setVariablePosition(VJOINT_NAME + "/rot_y", q.y());
  robot_state.setVariablePosition(VJOINT_NAME + "/rot_z", q.z());
  robot_state.setVariablePosition(VJOINT_NAME + "/rot_w", q.w());

  return true;
}

}  // namespace moveit_visual_tools
