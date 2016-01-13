/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Dave Coleman
// Desc:   Simple tools for showing parts of a robot in Rviz, such as the gripper or arm

#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt Messages
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>

// MoveIt
#include <moveit/robot_state/conversions.h>
#include <moveit/collision_detection/collision_tools.h>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// Shape tools
#include <geometric_shapes/solid_primitive_dims.h>
#include <geometric_shapes/shape_operations.h>

namespace moveit_visual_tools
{
MoveItVisualTools::MoveItVisualTools(
    const std::string& base_frame, const std::string& marker_topic,
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
  : RvizVisualTools::RvizVisualTools(base_frame, marker_topic)
  , planning_scene_monitor_(planning_scene_monitor)
  , mannual_trigger_update_(false)
  , robot_state_topic_(DISPLAY_ROBOT_STATE_TOPIC)
  , planning_scene_topic_(PLANNING_SCENE_TOPIC)
  , name_("visual_tools")
{
}

MoveItVisualTools::MoveItVisualTools(const std::string& base_frame, const std::string& marker_topic,
                                     robot_model::RobotModelConstPtr robot_model)
  : RvizVisualTools::RvizVisualTools(base_frame, marker_topic)
  , robot_model_(robot_model)
  , mannual_trigger_update_(false)
{
}

bool MoveItVisualTools::loadPlanningSceneMonitor()
{
  // Check if we already have one
  if (planning_scene_monitor_)
  {
    ROS_WARN_STREAM_NAMED(name_, "Will not load a new planning scene monitor when one has "
                                          "already been set for Visual Tools");
    return false;
  }
  ROS_DEBUG_STREAM_NAMED(name_, "Loading planning scene monitor");

  // Create tf transformer
  boost::shared_ptr<tf::TransformListener> tf;
  // tf.reset(new tf::TransformListener(nh_));
  // ros::spinOnce();

  // Regular version b/c the other one causes problems with recognizing end effectors
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(
                                                                                 //ROBOT_DESCRIPTION, boost::shared_ptr<tf::Transformer>(), "visual_tools_scene"));
                                                                                 ROBOT_DESCRIPTION, tf, "visual_tools_scene"));
  ros::spinOnce();
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    // planning_scene_monitor_->startWorldGeometryMonitor();
    // planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
    // planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");

    planning_scene_monitor_->startPublishingPlanningScene(
        planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, planning_scene_topic_);
    ROS_DEBUG_STREAM_NAMED(name_, "Publishing planning scene on "
                                               << planning_scene_topic_);

    planning_scene_monitor_->getPlanningScene()->setName("visual_tools_scene");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(name_, "Planning scene not configured");
    return false;
  }

  return true;
}

bool MoveItVisualTools::processCollisionObjectMsg(const moveit_msgs::CollisionObject& msg,
                                                  const rviz_visual_tools::colors& color)
{
  // Apply command directly to planning scene to avoid a ROS msg call
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(getPlanningSceneMonitor());
    scene->getCurrentStateNonConst().update();  // hack to prevent bad transforms
    scene->processCollisionObjectMsg(msg);
    scene->setObjectColor(msg.id, getColor(color));
  }
  // Trigger an update
  if (!mannual_trigger_update_)
  {
    triggerPlanningSceneUpdate();
  }

  return true;
}

bool MoveItVisualTools::processAttachedCollisionObjectMsg(
    const moveit_msgs::AttachedCollisionObject& msg)
{
  // Apply command directly to planning scene to avoid a ROS msg call
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(getPlanningSceneMonitor());
    // scene->getCurrentStateNonConst().update(); // hack to prevent bad transforms
    scene->processAttachedCollisionObjectMsg(msg);
  }

  // Trigger an update
  if (!mannual_trigger_update_)
  {
    triggerPlanningSceneUpdate();
  }

  return true;
}

bool MoveItVisualTools::triggerPlanningSceneUpdate()
{
  getPlanningSceneMonitor()->triggerSceneUpdateEvent(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
  ros::spinOnce();
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
      planning_scene_monitor::PlanningSceneMonitorPtr psm = getPlanningSceneMonitor();
      robot_model_ = psm->getRobotModel();
    }
    shared_robot_state_.reset(new robot_state::RobotState(robot_model_));
    hidden_robot_state_.reset(new robot_state::RobotState(robot_model_));

    // TODO: this seems to be a work around for a weird NaN number bug
    shared_robot_state_->setToDefaultValues();
    shared_robot_state_->update(true);
    hidden_robot_state_->setToDefaultValues();
    hidden_robot_state_->update(true);
  }

  return shared_robot_state_;
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

bool MoveItVisualTools::loadEEMarker(const robot_model::JointModelGroup* ee_jmg)
{
  // Get joint state group
  if (ee_jmg == NULL)  // make sure EE_GROUP exists
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to find joint model group with address"
                                               << ee_jmg);
    return false;
  }

  // Always load the robot state before using
  loadSharedRobotState();
  shared_robot_state_->setToDefaultValues();
  shared_robot_state_->update();

  // Clear old EE markers and EE poses
  ee_markers_map_[ee_jmg].markers.clear();
  ee_poses_map_[ee_jmg].clear();

  // Keep track of how many unique markers we have between different EEs
  static std::size_t marker_id_offset = 0;

  // -----------------------------------------------------------------------------------------------
  // Get end effector group

  // Create color to use for EE markers
  std_msgs::ColorRGBA marker_color = getColor(rviz_visual_tools::GREY);

  // Get link names that are in end effector
  const std::vector<std::string>& ee_link_names = ee_jmg->getLinkModelNames();

  // -----------------------------------------------------------------------------------------------
  // Get EE link markers for Rviz

  shared_robot_state_->getRobotMarkers(ee_markers_map_[ee_jmg], ee_link_names, marker_color,
                                       ee_jmg->getName(), ros::Duration());
  ROS_DEBUG_STREAM_NAMED(name_, "Number of rviz markers in end effector: "
                                             << ee_markers_map_[ee_jmg].markers.size());

  const std::string& ee_parent_link_name = ee_jmg->getEndEffectorParentGroup().second;
  // ROS_DEBUG_STREAM_NAMED(name_,"EE Parent link: " << ee_parent_link_name);
  const moveit::core::LinkModel* ee_parent_link = robot_model_->getLinkModel(ee_parent_link_name);

  Eigen::Affine3d ee_marker_global_transform =
      shared_robot_state_->getGlobalLinkTransform(ee_parent_link);
  Eigen::Affine3d ee_marker_pose;

  // Process each link of the end effector
  for (std::size_t i = 0; i < ee_markers_map_[ee_jmg].markers.size(); ++i)
  {
    // Header
    ee_markers_map_[ee_jmg].markers[i].header.frame_id = base_frame_;

    // Options for meshes
    if (ee_markers_map_[ee_jmg].markers[i].type == visualization_msgs::Marker::MESH_RESOURCE)
    {
      ee_markers_map_[ee_jmg].markers[i].mesh_use_embedded_materials = true;
    }

    // Unique id
    ee_markers_map_[ee_jmg].markers[i].id += marker_id_offset;

    // Copy original marker poses to a vector
    ee_marker_pose =
        ee_marker_global_transform.inverse() * convertPose(ee_markers_map_[ee_jmg].markers[i].pose);
    ee_poses_map_[ee_jmg].push_back(ee_marker_pose);
  }

  marker_id_offset += ee_markers_map_[ee_jmg].markers.size();

  return true;
}

void MoveItVisualTools::loadTrajectoryPub(const std::string& display_planned_path_topic)
{
  if (pub_display_path_)
    return;

  // Trajectory paths
  pub_display_path_ =
      nh_.advertise<moveit_msgs::DisplayTrajectory>(display_planned_path_topic, 10, false);
  ROS_DEBUG_STREAM_NAMED(name_, "Publishing MoveIt trajectory on topic "
                                             << pub_display_path_.getTopic());

  // Wait for topic to be ready
  waitForSubscriber(pub_display_path_);
}

void MoveItVisualTools::loadRobotStatePub(const std::string& robot_state_topic)
{
  if (pub_robot_state_)
    return;

  // Update global var if new topic was passed in
  if (!robot_state_topic.empty())
    robot_state_topic_ = robot_state_topic;

  // RobotState Message
  pub_robot_state_ = nh_.advertise<moveit_msgs::DisplayRobotState>(robot_state_topic_, 1);
  ROS_DEBUG_STREAM_NAMED(name_, "Publishing MoveIt robot state on topic "
                                             << pub_robot_state_.getTopic());

  // Wait for topic to be ready
  waitForSubscriber(pub_robot_state_);
}

planning_scene_monitor::PlanningSceneMonitorPtr MoveItVisualTools::getPlanningSceneMonitor()
{
  if (!planning_scene_monitor_)
  {
    ROS_INFO_STREAM_NAMED(name_,
                          "No planning scene passed into moveit_visual_tools, creating one.");
    loadPlanningSceneMonitor();
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
  return planning_scene_monitor_;
}

bool MoveItVisualTools::publishEEMarkers(const geometry_msgs::Pose& pose,
                                         const robot_model::JointModelGroup* ee_jmg,
                                         const rviz_visual_tools::colors& color,
                                         const std::string& ns)
{
  // Check if we have not loaded the EE markers
  if (ee_markers_map_[ee_jmg].markers.empty() || ee_poses_map_[ee_jmg].empty())
  {
    if (!loadEEMarker(ee_jmg))
    {
      ROS_ERROR_STREAM_NAMED(name_,
                             "Unable to publish EE marker, unable to load EE markers");
      return false;
    }
  }

  Eigen::Affine3d eigen_goal_ee_pose = convertPose(pose);
  Eigen::Affine3d eigen_this_marker;

  // publishArrow( pose, rviz_visual_tools::RED, rviz_visual_tools::LARGE );

  // -----------------------------------------------------------------------------------------------
  // Process each link of the end effector
  for (std::size_t i = 0; i < ee_markers_map_[ee_jmg].markers.size(); ++i)
  {
    // Make sure ROS is still spinning
    if (!ros::ok())
      break;

    // Header
    ee_markers_map_[ee_jmg].markers[i].header.stamp = ros::Time::now();

    // Namespace
    ee_markers_map_[ee_jmg].markers[i].ns = ns;

    // Lifetime
    ee_markers_map_[ee_jmg].markers[i].lifetime = marker_lifetime_;

    // Color
    ee_markers_map_[ee_jmg].markers[i].color = getColor(color);

    // Convert pose
    eigen_this_marker = eigen_goal_ee_pose * ee_poses_map_[ee_jmg][i];
    ee_markers_map_[ee_jmg].markers[i].pose = convertPose(eigen_this_marker);
  }

  // Helper for publishing rviz markers
  if (!publishMarkers(ee_markers_map_[ee_jmg]))
    return false;

  return true;
}

bool MoveItVisualTools::publishGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
                                      const robot_model::JointModelGroup* ee_jmg,
                                      double animate_speed)
{
  ROS_DEBUG_STREAM_NAMED(name_, "Visualizing " << possible_grasps.size()
                                                        << " grasps with EE joint model group "
                                                        << ee_jmg->getName());

  // Loop through all grasps
  for (std::size_t i = 0; i < possible_grasps.size(); ++i)
  {
    if (!ros::ok())  // Check that ROS is still ok and that user isn't trying to quit
      break;

    publishEEMarkers(possible_grasps[i].grasp_pose.pose, ee_jmg);

    ros::Duration(animate_speed).sleep();
  }

  return true;
}

bool MoveItVisualTools::publishAnimatedGrasps(
    const std::vector<moveit_msgs::Grasp>& possible_grasps,
    const robot_model::JointModelGroup* ee_jmg, double animate_speed)
{
  ROS_DEBUG_STREAM_NAMED(
      name_, "Visualizing " << possible_grasps.size() << " grasps with joint model group "
                                     << ee_jmg->getName() << " at speed " << animate_speed);

  // Loop through all grasps
  for (std::size_t i = 0; i < possible_grasps.size(); ++i)
  {
    if (!ros::ok())  // Check that ROS is still ok and that user isn't trying to quit
      break;

    publishAnimatedGrasp(possible_grasps[i], ee_jmg, animate_speed);

    ros::Duration(0.1).sleep();
  }

  return true;
}

bool MoveItVisualTools::publishAnimatedGrasp(const moveit_msgs::Grasp& grasp,
                                             const robot_model::JointModelGroup* ee_jmg,
                                             double animate_speed)
{
  // Grasp Pose Variables
  geometry_msgs::Pose grasp_pose = grasp.grasp_pose.pose;

  // Debug
  if (false)
  {
    publishArrow(grasp_pose, rviz_visual_tools::GREEN);
    publishEEMarkers(grasp_pose, ee_jmg);
    ros::Duration(0.5).sleep();
  }

  Eigen::Affine3d grasp_pose_eigen;
  tf::poseMsgToEigen(grasp_pose, grasp_pose_eigen);

  // Pre-grasp pose variables
  geometry_msgs::Pose pre_grasp_pose;
  Eigen::Affine3d pre_grasp_pose_eigen;

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
    if (!ros::ok())  // Check that ROS is still ok and that user isn't trying to quit
      break;

    // Copy original grasp pose to pre-grasp pose
    pre_grasp_pose_eigen = grasp_pose_eigen;

    // The direction of the pre-grasp
    // Calculate the current animation position based on the percent
    Eigen::Vector3d pre_grasp_approach_direction =
        Eigen::Vector3d(-1 * grasp.pre_grasp_approach.direction.vector.x *
                            grasp.pre_grasp_approach.min_distance * (1 - percent),
                        -1 * grasp.pre_grasp_approach.direction.vector.y *
                            grasp.pre_grasp_approach.min_distance * (1 - percent),
                        -1 * grasp.pre_grasp_approach.direction.vector.z *
                            grasp.pre_grasp_approach.min_distance * (1 - percent));

    // Decide if we need to change the approach_direction to the local frame of the end effector
    // orientation
    const std::string& ee_parent_link_name = ee_jmg->getEndEffectorParentGroup().second;

    if (grasp.pre_grasp_approach.direction.header.frame_id == ee_parent_link_name)
    {
      // Apply/compute the approach_direction vector in the local frame of the grasp_pose
      // orientation
      pre_grasp_approach_direction_local =
          grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
    }
    else
    {
      pre_grasp_approach_direction_local =
          pre_grasp_approach_direction;  // grasp_pose_eigen.rotation() *
                                         // pre_grasp_approach_direction;
    }

    // Update the grasp pose usign the new locally-framed approach_direction
    pre_grasp_pose_eigen.translation() += pre_grasp_approach_direction_local;

    // Convert eigen pre-grasp position back to regular message
    tf::poseEigenToMsg(pre_grasp_pose_eigen, pre_grasp_pose);

    // publishArrow(pre_grasp_pose, moveit_visual_tools::BLUE);
    publishEEMarkers(pre_grasp_pose, ee_jmg);

    ros::Duration(animate_speed).sleep();

    // Pause more at initial pose for debugging purposes
    if (percent == 0)
      ros::Duration(animate_speed * 2).sleep();
  }
  return true;
}

bool MoveItVisualTools::publishIKSolutions(
    const std::vector<trajectory_msgs::JointTrajectoryPoint>& ik_solutions,
    const robot_model::JointModelGroup* arm_jmg, double display_time)
{
  if (ik_solutions.empty())
  {
    ROS_WARN_STREAM_NAMED(name_,
                          "Empty ik_solutions vector passed into publishIKSolutions()");
    return false;
  }

  loadSharedRobotState();

  ROS_DEBUG_STREAM_NAMED(name_, "Visualizing " << ik_solutions.size()
                                                        << " inverse kinematic solutions");

  // Apply the time to the trajectory
  trajectory_msgs::JointTrajectoryPoint trajectory_pt_timed;

  // Create a trajectory with one point
  moveit_msgs::RobotTrajectory trajectory_msg;
  trajectory_msg.joint_trajectory.header.frame_id = base_frame_;
  trajectory_msg.joint_trajectory.joint_names = arm_jmg->getActiveJointModelNames();

  // Overall length of trajectory
  double running_time = 0;

  // Loop through all inverse kinematic solutions
  for (std::size_t i = 0; i < ik_solutions.size(); ++i)
  {
    trajectory_pt_timed = ik_solutions[i];
    trajectory_pt_timed.time_from_start = ros::Duration(running_time);
    trajectory_msg.joint_trajectory.points.push_back(trajectory_pt_timed);

    running_time += display_time;
  }

  // Re-add final position so the last point is displayed fully
  trajectory_pt_timed = trajectory_msg.joint_trajectory.points.back();
  trajectory_pt_timed.time_from_start = ros::Duration(running_time);
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
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = base_frame_;
  co.id = name;
  co.operation = moveit_msgs::CollisionObject::REMOVE;

  return processCollisionObjectMsg(co);
}

bool MoveItVisualTools::cleanupACO(const std::string& name)
{
  // Clean up old attached collision object
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.header.stamp = ros::Time::now();
  aco.object.header.frame_id = base_frame_;

  // aco.object.id = name;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;

  return processAttachedCollisionObjectMsg(aco);
}

bool MoveItVisualTools::attachCO(const std::string& name, const std::string& ee_parent_link)
{
  // Attach a collision object
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.header.stamp = ros::Time::now();
  aco.object.header.frame_id = base_frame_;

  aco.object.id = name;
  aco.object.operation = moveit_msgs::CollisionObject::ADD;

  // Link to attach the object to
  aco.link_name = ee_parent_link;

  return processAttachedCollisionObjectMsg(aco);
}

bool MoveItVisualTools::publishCollisionBlock(const geometry_msgs::Pose& block_pose,
                                              const std::string& name, double block_size,
                                              const rviz_visual_tools::colors& color)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;

  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = block_size;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = block_size;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = block_size;
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = block_pose;

  // ROS_INFO_STREAM_NAMED(name_,"CollisionObject: \n " << collision_obj);
  // ROS_DEBUG_STREAM_NAMED(name_,"Published collision object " << name);
  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::publishCollisionCuboid(const Eigen::Vector3d& point1,
                                               const Eigen::Vector3d& point2,
                                               const std::string& name,
                                               const rviz_visual_tools::colors& color)
{
  return publishCollisionCuboid(convertPoint(point1), convertPoint(point2), name, color);
}

bool MoveItVisualTools::publishCollisionCuboid(const geometry_msgs::Point& point1,
                                               const geometry_msgs::Point& point2,
                                               const std::string& name,
                                               const rviz_visual_tools::colors& color)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;

  // Calculate center pose
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0].position.x = (point1.x - point2.x) / 2.0 + point2.x;
  collision_obj.primitive_poses[0].position.y = (point1.y - point2.y) / 2.0 + point2.y;
  collision_obj.primitive_poses[0].position.z = (point1.z - point2.z) / 2.0 + point2.z;

  // Calculate scale
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] =
      fabs(point1.x - point2.x);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] =
      fabs(point1.y - point2.y);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] =
      fabs(point1.z - point2.z);

  // Prevent scale from being zero
  if (!collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X])
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] =
        rviz_visual_tools::SMALL_SCALE;
  if (!collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y])
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] =
        rviz_visual_tools::SMALL_SCALE;
  if (!collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z])
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] =
        rviz_visual_tools::SMALL_SCALE;

  // ROS_INFO_STREAM_NAMED(name_,"CollisionObject: \n " << collision_obj);
  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::publishCollisionFloor(double z, const std::string& plane_name,
                                              const rviz_visual_tools::colors& color)
{
  // Instead just generate a rectangle
  geometry_msgs::Point point1;
  geometry_msgs::Point point2;

  point1.x = rviz_visual_tools::LARGE_SCALE;
  point1.y = rviz_visual_tools::LARGE_SCALE;
  point1.z = z;

  point2.x = -rviz_visual_tools::LARGE_SCALE;
  point2.y = -rviz_visual_tools::LARGE_SCALE;
  point2.z = z - rviz_visual_tools::SMALL_SCALE;
  ;

  return publishCollisionCuboid(point1, point2, plane_name, color);
}

bool MoveItVisualTools::publishCollisionCylinder(const geometry_msgs::Point& a,
                                                 const geometry_msgs::Point& b,
                                                 const std::string& object_name, double radius,
                                                 const rviz_visual_tools::colors& color)
{
  return publishCollisionCylinder(convertPoint(a), convertPoint(b), object_name, radius, color);
}

bool MoveItVisualTools::publishCollisionCylinder(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                                                 const std::string& object_name, double radius,
                                                 const rviz_visual_tools::colors& color)
{
  // Distance between two points
  double height = (a - b).lpNorm<2>();

  // Find center point
  Eigen::Vector3d pt_center = getCenterPoint(a, b);

  // Create vector
  Eigen::Affine3d pose;
  pose = getVectorBetweenPoints(pt_center, b);

  // Convert pose to be normal to cylindar axis
  Eigen::Affine3d rotation;
  rotation = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY());
  pose = pose * rotation;

  return publishCollisionCylinder(pose, object_name, radius, height, color);
}

bool MoveItVisualTools::publishCollisionCylinder(const Eigen::Affine3d& object_pose,
                                                 const std::string& object_name, double radius,
                                                 double height,
                                                 const rviz_visual_tools::colors& color)
{
  return publishCollisionCylinder(convertPose(object_pose), object_name, radius, height, color);
}

bool MoveItVisualTools::publishCollisionCylinder(const geometry_msgs::Pose& object_pose,
                                                 const std::string& object_name, double radius,
                                                 double height,
                                                 const rviz_visual_tools::colors& color)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = object_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = object_pose;

  // ROS_INFO_STREAM_NAMED(name_,"CollisionObject: \n " << collision_obj);
  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::publishCollisionMesh(const Eigen::Affine3d& object_pose,
                                             const std::string& object_name,
                                             const std::string& mesh_path,
                                             const rviz_visual_tools::colors& color)
{
  return publishCollisionMesh(convertPose(object_pose), object_name, mesh_path, color);
}

bool MoveItVisualTools::publishCollisionMesh(const geometry_msgs::Pose& object_pose,
                                             const std::string& object_name,
                                             const std::string& mesh_path,
                                             const rviz_visual_tools::colors& color)
{
  shapes::Shape* mesh =
      shapes::createMeshFromResource(mesh_path);  // make sure its prepended by file://
  shapes::ShapeMsg shape_msg;  // this is a boost::variant type from shape_messages.h
  if (!mesh || !shapes::constructMsgFromShape(mesh, shape_msg))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to create mesh shape message from resource "
                                               << mesh_path);
    return false;
  }

  if (!publishCollisionMesh(object_pose, object_name, boost::get<shape_msgs::Mesh>(shape_msg),
                            color))
    return false;

  ROS_DEBUG_NAMED(name_, "Loaded mesh from '%s'", mesh_path.c_str());
  return true;
}

bool MoveItVisualTools::publishCollisionMesh(const Eigen::Affine3d& object_pose,
                                             const std::string& object_name,
                                             const shape_msgs::Mesh& mesh_msg,
                                             const rviz_visual_tools::colors& color)
{
  return publishCollisionMesh(convertPose(object_pose), object_name, mesh_msg, color);
}

bool MoveItVisualTools::publishCollisionMesh(const geometry_msgs::Pose& object_pose,
                                             const std::string& object_name,
                                             const shape_msgs::Mesh& mesh_msg,
                                             const rviz_visual_tools::colors& color)
{
  // Create collision message
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = object_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.mesh_poses.resize(1);
  collision_obj.mesh_poses[0] = object_pose;
  collision_obj.meshes.resize(1);
  collision_obj.meshes[0] = mesh_msg;

  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::publishCollisionGraph(const graph_msgs::GeometryGraph& graph,
                                              const std::string& object_name, double radius,
                                              const rviz_visual_tools::colors& color)
{
  ROS_INFO_STREAM_NAMED("publishCollisionGraph", "Preparing to create collision graph");

  // The graph is one collision object with many primitives
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = object_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;

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
      if (return_value.second == false)
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
        Eigen::Affine3d pose;
        pose = getVectorBetweenPoints(pt_center, b);

        // Convert pose to be normal to cylindar axis
        Eigen::Affine3d rotation;
        rotation = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitY());
        pose = pose * rotation;

        // Create the solid primitive
        shape_msgs::SolidPrimitive cylinder;
        cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
        cylinder.dimensions.resize(
            geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
        cylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
        cylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;

        // Add to the collision object
        collision_obj.primitives.push_back(cylinder);

        // Add the pose
        collision_obj.primitive_poses.push_back(convertPose(pose));
      }
    }
  }

  return processCollisionObjectMsg(collision_obj, color);
}

void MoveItVisualTools::getCollisionWallMsg(double x, double y, double angle, double width,
                                            const std::string name,
                                            moveit_msgs::CollisionObject& collision_obj)
{
  double floor_to_base_height = 0; // TODO(davetcoleman): set this to a better value

  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);

  geometry_msgs::Pose rec_pose;

  // ----------------------------------------------------------------------------------
  // Name
  collision_obj.id = name;

  double depth = 0.1;
  double height = 2.5;

  // Position
  rec_pose.position.x = x;
  rec_pose.position.y = y;
  rec_pose.position.z = height / 2 + floor_to_base_height;

  // Size
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = depth;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;
  // ----------------------------------------------------------------------------------

  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  rec_pose.orientation.x = quat.x();
  rec_pose.orientation.y = quat.y();
  rec_pose.orientation.z = quat.z();
  rec_pose.orientation.w = quat.w();

  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = rec_pose;
}

bool MoveItVisualTools::publishCollisionWall(double x, double y, double angle, double width,
                                             const std::string name,
                                             const rviz_visual_tools::colors& color)
{
  moveit_msgs::CollisionObject collision_obj;
  getCollisionWallMsg(x, y, angle, width, name, collision_obj);

  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::publishCollisionTable(double x, double y, double angle, double width,
                                              double height, double depth, const std::string name,
                                              const rviz_visual_tools::colors& color)
{
  double floor_to_base_height = 0; // TODO(davetcoleman): set this to a better value

  geometry_msgs::Pose table_pose;

  // Position
  table_pose.position.x = x;
  table_pose.position.y = y;
  table_pose.position.z = height / 2 + floor_to_base_height;

  // Orientation
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  table_pose.orientation.x = quat.x();
  table_pose.orientation.y = quat.y();
  table_pose.orientation.z = quat.z();
  table_pose.orientation.w = quat.w();

  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(
      geometric_shapes::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);

  // Size
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = depth;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;

  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = table_pose;
  return processCollisionObjectMsg(collision_obj, color);
}

bool MoveItVisualTools::loadCollisionSceneFromFile(const std::string& path)
{
  return loadCollisionSceneFromFile(path, Eigen::Affine3d::Identity());
}

bool MoveItVisualTools::loadCollisionSceneFromFile(const std::string& path,
                                                   const Eigen::Affine3d& offset)
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
        ROS_WARN_STREAM_NAMED(name_, "Unable to get locked planning scene RW");
        return false;
      }
    }
    ROS_INFO_NAMED(name_, "Loaded scene geometry from '%s'", path.c_str());
  }
  else
    ROS_WARN_NAMED(name_, "Unable to load scene geometry from '%s'", path.c_str());

  fin.close();

  return triggerPlanningSceneUpdate();
}

bool MoveItVisualTools::publishCollisionTests() { ROS_ERROR_STREAM_NAMED("temp", "Depricated"); }

bool MoveItVisualTools::publishWorkspaceParameters(const moveit_msgs::WorkspaceParameters& params)
{
  return publishCollisionCuboid(convertPoint(params.min_corner), convertPoint(params.max_corner),
                                "workspace", rviz_visual_tools::TRANSLUCENT);
}

bool MoveItVisualTools::publishContactPoints(const moveit::core::RobotState& robot_state,
                                             const planning_scene::PlanningScene* planning_scene,
                                             const rviz_visual_tools::colors& color)
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

  // Display
  if (c_res.contact_count > 0)
  {
    visualization_msgs::MarkerArray arr;
    collision_detection::getCollisionMarkersFromContacts(arr, planning_scene->getPlanningFrame(),
                                                         c_res.contacts);
    ROS_INFO_STREAM_NAMED(name_, "Completed listing of explanations for invalid states.");

    // Check for markers
    if (arr.markers.empty())
      return true;

    // Convert markers to same namespace and other custom stuff
    for (std::size_t i = 0; i < arr.markers.size(); ++i)
    {
      arr.markers[i].ns = "Collision";
      arr.markers[i].scale.x = 0.04;
      arr.markers[i].scale.y = 0.04;
      arr.markers[i].scale.z = 0.04;
      arr.markers[i].color = getColor(color);
    }

    return publishMarkers(arr);
  }
  return true;
}

bool MoveItVisualTools::publishTrajectoryPoint(
    const trajectory_msgs::JointTrajectoryPoint& trajectory_pt, const std::string& planning_group,
    double display_time)
{
  // Get joint state group
  const robot_model::JointModelGroup* jmg = robot_model_->getJointModelGroup(planning_group);

  if (jmg == NULL)  // not found
  {
    ROS_ERROR_STREAM_NAMED("publishTrajectoryPoint", "Could not find joint model group "
                                                         << planning_group);
    return false;
  }

  // Apply the time to the trajectory
  trajectory_msgs::JointTrajectoryPoint trajectory_pt_timed = trajectory_pt;
  trajectory_pt_timed.time_from_start = ros::Duration(display_time);

  // Create a trajectory with one point
  moveit_msgs::RobotTrajectory trajectory_msg;
  trajectory_msg.joint_trajectory.header.frame_id = base_frame_;
  trajectory_msg.joint_trajectory.joint_names = jmg->getJointModelNames();
  trajectory_msg.joint_trajectory.points.push_back(trajectory_pt);
  trajectory_msg.joint_trajectory.points.push_back(trajectory_pt_timed);

  return publishTrajectoryPath(trajectory_msg, shared_robot_state_, true);
}

bool MoveItVisualTools::publishTrajectoryPath(
    const std::vector<robot_state::RobotStatePtr>& trajectory,
    const moveit::core::JointModelGroup* jmg, double speed, bool blocking)
{
  // Copy the vector of RobotStates to a RobotTrajectory
  robot_trajectory::RobotTrajectoryPtr robot_trajectory(
      new robot_trajectory::RobotTrajectory(robot_model_, jmg->getName()));
  ;

  double duration_from_previous = 0;
  for (std::size_t k = 0; k < trajectory.size(); ++k)
  {
    duration_from_previous += speed;
    robot_trajectory->addSuffixWayPoint(trajectory[k], duration_from_previous);
  }

  // Convert trajectory to a message
  moveit_msgs::RobotTrajectory trajectory_msg;
  robot_trajectory->getRobotTrajectoryMsg(trajectory_msg);

  return publishTrajectoryPath(trajectory_msg, shared_robot_state_, blocking);
}

bool MoveItVisualTools::publishTrajectoryPath(const robot_trajectory::RobotTrajectory& trajectory,
                                              bool blocking)
{
  moveit_msgs::RobotTrajectory trajectory_msg;
  trajectory.getRobotTrajectoryMsg(trajectory_msg);

  // Add time from start if none specified
  if (trajectory_msg.joint_trajectory.points.size() > 1)
  {
    if (trajectory_msg.joint_trajectory.points[1].time_from_start ==
        ros::Duration(0))  // assume no timestamps exist
    {
      for (std::size_t i = 0; i < trajectory_msg.joint_trajectory.points.size(); ++i)
      {
        trajectory_msg.joint_trajectory.points[i].time_from_start = ros::Duration(i * 2);  // 1 hz
      }
    }
  }

  return publishTrajectoryPath(trajectory_msg, shared_robot_state_, blocking);
}

bool MoveItVisualTools::publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg,
                                              const robot_state::RobotStateConstPtr robot_state,
                                              bool blocking)
{
  // Check if we have enough points
  if (!trajectory_msg.joint_trajectory.points.size())
  {
    ROS_WARN_STREAM_NAMED(name_,
                          "Unable to publish trajectory path because trajectory has zero points");
    return false;
  }

  // Create the message
  moveit_msgs::DisplayTrajectory display_trajectory_msg;
  display_trajectory_msg.model_id = robot_model_->getName();
  display_trajectory_msg.trajectory.resize(1);
  display_trajectory_msg.trajectory[0] = trajectory_msg;

  // Convert the current robot state to the trajectory start, so that we can e.g. provide vjoint
  // positions
  robot_state::robotStateToRobotStateMsg(*robot_state, display_trajectory_msg.trajectory_start);

  // Publish message
  loadTrajectoryPub();  // always call this before publishing
  pub_display_path_.publish(display_trajectory_msg);
  ros::spinOnce();

  // Wait the duration of the trajectory
  if (blocking)
  {
    ROS_INFO_STREAM_NAMED(name_,
                          "Waiting for trajectory animation "
                              << trajectory_msg.joint_trajectory.points.back().time_from_start
                              << " seconds");

    // Check if ROS is ok in intervals
    double counter = 0;
    while (ros::ok() &&
           counter < trajectory_msg.joint_trajectory.points.back().time_from_start.toSec())
    {
      counter += 0.25;  // check every fourth second
      ros::Duration(0.25).sleep();
    }
  }

  return true;
}

bool MoveItVisualTools::publishTrajectoryLine(const moveit_msgs::RobotTrajectory& trajectory_msg,
                                              const moveit::core::LinkModel* ee_parent_link,
                                              const robot_model::JointModelGroup* arm_jmg,
                                              const rviz_visual_tools::colors& color,
                                              bool clear_all_markers)
{
  // Error check
  if (!arm_jmg)
  {
    ROS_FATAL_STREAM_NAMED(name_, "arm_jmg is NULL");
    return false;
  }

  // Always load the robot state before using
  loadSharedRobotState();

  // Convert trajectory into a series of RobotStates
  robot_trajectory::RobotTrajectoryPtr
    robot_trajectory(new robot_trajectory::RobotTrajectory(robot_model_, arm_jmg->getName()));
  robot_trajectory->setRobotTrajectoryMsg(*shared_robot_state_, trajectory_msg);

  return publishTrajectoryLine(robot_trajectory, ee_parent_link, color, clear_all_markers);
}

bool MoveItVisualTools::publishTrajectoryLine(const robot_trajectory::RobotTrajectoryPtr robot_trajectory,
                                              const moveit::core::LinkModel* ee_parent_link,
                                              const rviz_visual_tools::colors& color,
                                              bool clear_all_markers)
{
  // Error check
  if (!ee_parent_link)
  {
    ROS_FATAL_STREAM_NAMED(name_, "ee_parent_link is NULL");
    return false;
  }

  // Point location datastructure
  std::vector<geometry_msgs::Point> path;

  // Group together messages
  enableInternalBatchPublishing(true);

  if (clear_all_markers)
    publishMarker(reset_marker_);

  // Visualize end effector position of cartesian path
  for (std::size_t i = 0; i < robot_trajectory->getWayPointCount(); ++i)
  {
    const Eigen::Affine3d& tip_pose =
        robot_trajectory->getWayPoint(i).getGlobalLinkTransform(ee_parent_link);

    // Error Check
    if (tip_pose.translation().x() != tip_pose.translation().x())
    {
      ROS_ERROR_STREAM_NAMED(name_, "NAN DETECTED AT TRAJECTORY POINT i=" << i);
      return false;
    }

    path.push_back(convertPose(tip_pose).position);
    publishSphere(tip_pose, color, rviz_visual_tools::LARGE);
  }

  publishPath(path, color, rviz_visual_tools::XSMALL);

  return triggerInternalBatchPublishAndDisable();
}

bool MoveItVisualTools::publishTrajectoryPoints(
    const std::vector<robot_state::RobotStatePtr>& robot_state_trajectory,
    const moveit::core::LinkModel* ee_parent_link, const rviz_visual_tools::colors& color)
{
  // Visualize end effector position of cartesian path
  for (std::size_t i = 0; i < robot_state_trajectory.size(); ++i)
  {
    const Eigen::Affine3d& tip_pose =
        robot_state_trajectory[i]->getGlobalLinkTransform(ee_parent_link);

    publishSphere(tip_pose, color);
  }
  return true;
}

bool MoveItVisualTools::publishRobotState(const robot_state::RobotStatePtr& robot_state,
                                          const rviz_visual_tools::colors& color)
{
  publishRobotState(*robot_state.get(), color);
}

bool MoveItVisualTools::publishRobotState(const robot_state::RobotState& robot_state,
                                          const rviz_visual_tools::colors& color)
{
  // Reference to the correctly colored version of message (they are cached)
  // May not exist yet but this will create it
  moveit_msgs::DisplayRobotState& display_robot_msg = display_robot_msgs_[color];

  // Check if a robot state message already exists for this color
  if (display_robot_msg.highlight_links.size() == 0)  // has not been colored yet, lets create that
  {
    if (color != rviz_visual_tools::DEFAULT)  // ignore color highlights when set to default
    {
      // Get links names
      const std::vector<const moveit::core::LinkModel*>& link_names =
          robot_state.getRobotModel()->getLinkModelsWithCollisionGeometry();
      display_robot_msg.highlight_links.resize(link_names.size());

      // Get color
      const std_msgs::ColorRGBA& color_rgba = getColor(color);

      // Color every link
      for (std::size_t i = 0; i < link_names.size(); ++i)
      {
        display_robot_msg.highlight_links[i].id = link_names[i]->getName();
        display_robot_msg.highlight_links[i].color = color_rgba;
      }
    }
  }

  // Modify colors to also indicate which are fixed
  // TODO not compatibile with mainstream moveit
  /*
  if (robot_state.hasFixedLinks())
  {
    // Get links names
    const std::vector<const moveit::core::LinkModel*>& link_names =
  robot_state.getRobotModel()->getLinkModelsWithCollisionGeometry();

    for (std::size_t i = 0; i < robot_model_->getFixableLinks().size(); ++i)
    {
      if (robot_state.fixedLinkEnabled(i))
      {
        for (std::size_t j = 0; j < link_names.size(); ++j)
        {
          if (link_names[j] == robot_model_->getFixableLinks()[i])
            if ( robot_state.getPrimaryFixedLinkID() == i ) // is primary
              display_robot_msg.highlight_links[j].color = getColor(rviz_visual_tools::BLUE);
            else
              display_robot_msg.highlight_links[j].color = getColor(rviz_visual_tools::RED);
        }
      }
    }
  }
  */

  // Convert state to message
  robot_state::robotStateToRobotStateMsg(robot_state, display_robot_msg.state);

  // Publish
  loadRobotStatePub();
  pub_robot_state_.publish(display_robot_msg);
  ros::spinOnce();

  return true;
}

bool MoveItVisualTools::publishRobotState(
    const trajectory_msgs::JointTrajectoryPoint& trajectory_pt,
    const robot_model::JointModelGroup* jmg, const rviz_visual_tools::colors& color)
{
  // Always load the robot state before using
  loadSharedRobotState();

  // Set robot state
  shared_robot_state_->setToDefaultValues();  // reset the state just in case
  shared_robot_state_->setJointGroupPositions(jmg, trajectory_pt.positions);

  // Publish robot state
  publishRobotState(*shared_robot_state_, color);

  return true;
}

bool MoveItVisualTools::hideRobot()
{
  static const std::string VJOINT_NAME = "virtual_joint";

  // Always load the robot state before using
  loadSharedRobotState();

  // Check if joint
  if (!hidden_robot_state_->getRobotModel()->hasJointModel(VJOINT_NAME))
  {
    ROS_WARN_STREAM_NAMED(name_, "Unable to hide robot because joint '" << VJOINT_NAME
                          << "' does not exist.");
    const std::vector<std::string>& names = hidden_robot_state_->getRobotModel()->getJointModelNames();
    ROS_WARN_STREAM_NAMED(name_, "Available names:");
    std::copy(names.begin(), names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

    return false;
  }

  // Check if variables exist
  if (!hidden_robot_state_->getRobotModel()->getJointModel(VJOINT_NAME)
      ->hasVariable(VJOINT_NAME + "/trans_x"))
  {
    // Debug
    ROS_WARN_STREAM_NAMED(name_, "Unable to hide robot because variables for joint '" << VJOINT_NAME
                          << "' do not exist. Try making this vjoint floating");
    ROS_WARN_STREAM_NAMED(name_, "The only available joint variables are:");
    const std::vector<std::string>& var_names =
      hidden_robot_state_->getRobotModel()->getJointModel(VJOINT_NAME)->getVariableNames();
    std::copy(var_names.begin(), var_names.end(),
              std::ostream_iterator<std::string>(std::cout, "\n"));
    return false;
  }

  // Hide the robot
  hidden_robot_state_->setVariablePosition(VJOINT_NAME + "/trans_x",
                                           rviz_visual_tools::LARGE_SCALE);
  hidden_robot_state_->setVariablePosition(VJOINT_NAME + "/trans_y",
                                           rviz_visual_tools::LARGE_SCALE);
  hidden_robot_state_->setVariablePosition(VJOINT_NAME + "/trans_z",
                                           rviz_visual_tools::LARGE_SCALE);
  return publishRobotState(hidden_robot_state_);
}

}  // namespace
