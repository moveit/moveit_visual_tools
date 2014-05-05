/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
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

#include <moveit_visual_tools/visual_tools.h>
#include <moveit/robot_interaction/robot_interaction.h>

// MoveIt Messages
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>

// MoveIt
#include <moveit/robot_state/conversions.h>

// Conversions
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

#include <shape_tools/solid_primitive_dims.h>

namespace moveit_visual_tools
{

VisualTools::VisualTools(const std::string& base_link,
  const std::string& marker_topic,
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
  :  base_link_(base_link),
     marker_topic_(marker_topic),
     planning_scene_monitor_(planning_scene_monitor)
{
  initialize();
}

void VisualTools::initialize()
{
  floor_to_base_height_ = 0;
  marker_lifetime_ = ros::Duration(30.0);
  muted_ = false;
  alpha_ = 0.8;

  // Initialize counters to zero
  resetMarkerCounts();

  // Cache the reusable markers
  loadRvizMarkers();
}

void VisualTools::loadMarkerPub()
{
  if (pub_rviz_marker_)
    return;

  // Rviz marker publisher
  pub_rviz_marker_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing Rviz markers on topic " << marker_topic_);

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  ros::spinOnce();
}

void VisualTools::loadCollisionPub()
{
  if (pub_collision_obj_)
    return;

  // Collision object creator
  pub_collision_obj_ = nh_.advertise<moveit_msgs::CollisionObject>(COLLISION_TOPIC, 10);
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing collision objects on topic " << COLLISION_TOPIC);

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  ros::spinOnce();
}

void VisualTools::loadAttachedPub()
{
  if (pub_attach_collision_obj_)
    return;

  // Collision object attacher
  pub_attach_collision_obj_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>(ATTACHED_COLLISION_TOPIC, 10);
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing attached collision objects on topic " << ATTACHED_COLLISION_TOPIC);

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  ros::spinOnce();
}

void VisualTools::loadPlanningPub()
{
  if (pub_planning_scene_diff_)
    return;

  // Planning scene diff publisher
  pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 1);
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing planning scene on topic " << PLANNING_SCENE_TOPIC);

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  ros::spinOnce();
}

void VisualTools::loadTrajectoryPub()
{
  if (pub_display_path_)
    return;

  // Trajectory paths
  pub_display_path_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(DISPLAY_PLANNED_PATH_TOPIC, 10, false);
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing MoveIt trajectory on topic " << DISPLAY_PLANNED_PATH_TOPIC);

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  ros::spinOnce();
}

void VisualTools::loadRobotPub()
{
  if (pub_robot_state_)
    return;

  // RobotState Message
  pub_robot_state_ = nh_.advertise<moveit_msgs::DisplayRobotState>(DISPLAY_ROBOT_STATE_TOPIC, 1 );
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing MoveIt Robot State on topic " << DISPLAY_ROBOT_STATE_TOPIC);

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  ros::spinOnce();
}

void VisualTools::setFloorToBaseHeight(double floor_to_base_height)
{
  floor_to_base_height_ = floor_to_base_height;
}

void VisualTools::setGraspPoseToEEFPose(geometry_msgs::Pose grasp_pose_to_eef_pose)
{
  grasp_pose_to_eef_pose_ = grasp_pose_to_eef_pose;
}

void VisualTools::setLifetime(double lifetime)
{
  marker_lifetime_ = ros::Duration(lifetime);

  // Update cached markers
  arrow_marker_.lifetime = marker_lifetime_;
  rectangle_marker_.lifetime = marker_lifetime_;
  line_marker_.lifetime = marker_lifetime_;
  sphere_marker_.lifetime = marker_lifetime_;
  block_marker_.lifetime = marker_lifetime_;
  cylinder_marker_.lifetime = marker_lifetime_;
  text_marker_.lifetime = marker_lifetime_;
}

std_msgs::ColorRGBA VisualTools::getColor(const rviz_colors &color)
{
  std_msgs::ColorRGBA result;
  result.a = alpha_;
  switch(color)
  {
    case RED:
      result.r = 0.8;
      result.g = 0.1;
      result.b = 0.1;
      break;
    case GREEN:
      result.r = 0.1;
      result.g = 0.8;
      result.b = 0.1;
      break;
    case GREY:
      result.r = 0.9;
      result.g = 0.9;
      result.b = 0.9;
      break;
    case WHITE:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 1.0;
      break;
    case ORANGE:
      result.r = 1.0;
      result.g = 0.5;
      result.b = 0.0;
      break;
    case BLACK:
      result.r = 0.0;
      result.g = 0.0;
      result.b = 0.0;
      break;
    case YELLOW:
      result.r = 1.0;
      result.g = 1.0;
      result.b = 0.0;
      break;
    case BLUE:
    default:
      result.r = 0.1;
      result.g = 0.1;
      result.b = 0.8;
  }

  return result;
}

geometry_msgs::Vector3 VisualTools::getScale(const rviz_scales &scale, bool arrow_scale, double marker_scale)
{
  geometry_msgs::Vector3 result;
  double val;
  switch(scale)
  {
    case XXSMALL:
      val = 0.005;
      break;
    case XSMALL:
      val = 0.01;
      break;
    case SMALL:
      val = 0.03;
      break;
    case REGULAR:
      val = 0.05;
      break;
    case LARGE:
      val = 0.1;
      break;
    case XLARGE:
      val = 0.5;
      break;
    default:
      ROS_ERROR_STREAM_NAMED("visualization_tools","Not implemented yet");
      break;
  }

  result.x = val * marker_scale;
  result.y = val * marker_scale;
  result.z = val * marker_scale;

  // The y and z scaling is smaller for arrows
  if (arrow_scale)
  {
    result.y *= 0.1;
    result.z *= 0.1;
  }

  return result;
}

planning_scene_monitor::PlanningSceneMonitorPtr VisualTools::getPlanningSceneMonitor()
{
  if( !planning_scene_monitor_ )
  {
    loadPlanningSceneMonitor();
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
  return planning_scene_monitor_;
}

bool VisualTools::loadSharedRobotState()
{
  // Get robot state
  if (!shared_robot_state_)
  {
    planning_scene_monitor::PlanningSceneMonitorPtr psm = getPlanningSceneMonitor();

    shared_robot_state_.reset(new robot_state::RobotState(psm->getRobotModel() ));
  }

  return true;
}

Eigen::Vector3d VisualTools::getCenterPoint(Eigen::Vector3d a, Eigen::Vector3d b)
{
  Eigen::Vector3d center;
  center[0] = (a[0] + b[0]) / 2;
  center[1] = (a[1] + b[1]) / 2;
  center[2] = (a[2] + b[2]) / 2;
  return center;
}

Eigen::Affine3d VisualTools::getVectorBetweenPoints(Eigen::Vector3d a, Eigen::Vector3d b)
{
  // from http://answers.ros.org/question/31006/how-can-a-vector3-axis-be-used-to-produce-a-quaternion/

  // Goal pose:
  Eigen::Quaterniond q;

  Eigen::Vector3d axis_vector = b - a;
  axis_vector.normalize();

  Eigen::Vector3d up_vector(0.0, 0.0, 1.0);
  Eigen::Vector3d right_axis_vector = axis_vector.cross(up_vector);
  right_axis_vector.normalized();
  double theta = axis_vector.dot(up_vector);
  double angle_rotation = -1.0*acos(theta);

  //-------------------------------------------
  // Method 1 - TF - works
  //Convert to TF
  tf::Vector3 tf_right_axis_vector;
  tf::vectorEigenToTF(right_axis_vector, tf_right_axis_vector);

  // Create quaternion
  tf::Quaternion tf_q(tf_right_axis_vector, angle_rotation);

  // Convert back to Eigen
  tf::quaternionTFToEigen(tf_q, q);
  //-------------------------------------------
  //std::cout << q.toRotationMatrix() << std::endl;

  //-------------------------------------------
  // Method 2 - Eigen - broken TODO
  //q = Eigen::AngleAxis<double>(angle_rotation, right_axis_vector);
  //-------------------------------------------
  //std::cout << q.toRotationMatrix() << std::endl;

  // Rotate so that vector points along line
  Eigen::Affine3d pose;
  q.normalize();
  pose = q * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY());
  pose.translation() = a;

  return pose;
}

bool VisualTools::loadRvizMarkers()
{
  // Load arrow ----------------------------------------------------

  arrow_marker_.header.frame_id = base_link_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  arrow_marker_.ns = "Arrow";
  // Set the marker type.
  arrow_marker_.type = visualization_msgs::Marker::ARROW;
  // Set the marker action.  Options are ADD and DELETE
  arrow_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  arrow_marker_.lifetime = marker_lifetime_;

  // Load rectangle ----------------------------------------------------

  rectangle_marker_.header.frame_id = base_link_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  rectangle_marker_.ns = "Rectangle";
  // Set the marker type.
  rectangle_marker_.type = visualization_msgs::Marker::CUBE;
  // Set the marker action.  Options are ADD and DELETE
  rectangle_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  rectangle_marker_.lifetime = marker_lifetime_;

  // Load line ----------------------------------------------------

  line_marker_.header.frame_id = base_link_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  line_marker_.ns = "Line";
  // Set the marker type.
  line_marker_.type = visualization_msgs::Marker::LINE_STRIP;
  // Set the marker action.  Options are ADD and DELETE
  line_marker_.action = visualization_msgs::Marker::ADD;
  // Lifetime
  line_marker_.lifetime = marker_lifetime_;

  // Load Block ----------------------------------------------------
  block_marker_.header.frame_id = base_link_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  block_marker_.ns = "Block";
  // Set the marker action.  Options are ADD and DELETE
  block_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  block_marker_.type = visualization_msgs::Marker::CUBE;
  // Lifetime
  block_marker_.lifetime = marker_lifetime_;

  // Load Cylinder ----------------------------------------------------
  cylinder_marker_.header.frame_id = base_link_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  cylinder_marker_.ns = "Cylinder";
  // Set the marker action.  Options are ADD and DELETE
  cylinder_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  cylinder_marker_.type = visualization_msgs::Marker::CYLINDER;
  // Lifetime
  cylinder_marker_.lifetime = marker_lifetime_;

  // Load Sphere -------------------------------------------------
  sphere_marker_.header.frame_id = base_link_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  sphere_marker_.ns = "Sphere";
  // Set the marker type.
  sphere_marker_.type = visualization_msgs::Marker::SPHERE_LIST;
  // Set the marker action.  Options are ADD and DELETE
  sphere_marker_.action = visualization_msgs::Marker::ADD;
  // Marker group position and orientation
  sphere_marker_.pose.position.x = 0;
  sphere_marker_.pose.position.y = 0;
  sphere_marker_.pose.position.z = 0;
  sphere_marker_.pose.orientation.x = 0.0;
  sphere_marker_.pose.orientation.y = 0.0;
  sphere_marker_.pose.orientation.z = 0.0;
  sphere_marker_.pose.orientation.w = 1.0;
  // Create a sphere point
  geometry_msgs::Point point_a;
  // Add the point pair to the line message
  sphere_marker_.points.push_back( point_a );
  sphere_marker_.colors.push_back( getColor( BLUE ) );
  // Lifetime
  sphere_marker_.lifetime = marker_lifetime_;

  // Load Text ----------------------------------------------------
  text_marker_.header.frame_id = base_link_;
  // Set the namespace and id for this marker.  This serves to create a unique ID
  text_marker_.ns = "Text";
  // Set the marker action.  Options are ADD and DELETE
  text_marker_.action = visualization_msgs::Marker::ADD;
  // Set the marker type.
  text_marker_.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // Lifetime
  text_marker_.lifetime = marker_lifetime_;
}

bool VisualTools::loadPlanningSceneMonitor()
{
  ROS_DEBUG_STREAM_NAMED("visual_tools","Loading planning scene monitor");

  // Create planning scene monitor
  // We create it the harder, more manual way so that we can tell MoveIt! to skip loading IK solvers, since we will
  // never use them within the context of moveit_visual_tools. This saves loading time
  /*
  robot_model_loader::RobotModelLoader::Options rml_options(ROBOT_DESCRIPTION);
  rml_options.load_kinematics_solvers_ = false;
  rm_loader_.reset(new robot_model_loader::RobotModelLoader(rml_options));

  std::string monitor_name = "visual_tools_planning_scene_monitor";

  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(
      planning_scene::PlanningScenePtr(),
      rm_loader_,
      boost::shared_ptr<tf::Transformer>(),
      monitor_name
      ));
  */

  // Regular version b/c the other one causes problems with recognizing end effectors
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  ros::spinOnce();

  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    //planning_scene_monitor_->startWorldGeometryMonitor();
    //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
    //planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
    //planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, "test_planning_scene");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("visual_tools","Planning scene not configured");
    return false;
  }

  return true;
}

bool VisualTools::loadRobotMarkers()
{
  // Always oad the robot state before using
  loadSharedRobotState();

  // Get all link names
  const std::vector<std::string> &link_names = shared_robot_state_->getRobotModel()->getLinkModelNames();;

  ROS_DEBUG_STREAM_NAMED("visual_tools","Number of links in robot: " << link_names.size());
  //    std::copy(link_names.begin(), link_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

  // Get EE link markers for Rviz
  visualization_msgs::MarkerArray robot_marker_array;
  shared_robot_state_->getRobotMarkers(robot_marker_array, link_names, getColor( GREY ), "test", ros::Duration());

  ROS_DEBUG_STREAM_NAMED("visual_tools","Number of rviz markers: " << robot_marker_array.markers.size());

  // Publish the markers
  for (std::size_t i = 0 ; i < robot_marker_array.markers.size() ; ++i)
  {
    // Make sure ROS is still spinning
    if( !ros::ok() )
      break;

    // Header
    robot_marker_array.markers[i].header.frame_id = base_link_;
    robot_marker_array.markers[i].header.stamp = ros::Time::now();

    // Options for meshes
    if( robot_marker_array.markers[i].type == visualization_msgs::Marker::MESH_RESOURCE )
      robot_marker_array.markers[i].mesh_use_embedded_materials = true;

    pub_rviz_marker_.publish( robot_marker_array.markers[i] );
    ros::spinOnce();
  }

  return true;
}

bool VisualTools::loadEEMarker(const std::string& ee_group_name, const std::string& planning_group)
{
  // Always load the robot state before using
  loadSharedRobotState();

  // -----------------------------------------------------------------------------------------------
  // Get end effector group

  // Create color to use for EE markers
  std_msgs::ColorRGBA marker_color = getColor( GREY );

  // Get robot model
  robot_model::RobotModelConstPtr robot_model = shared_robot_state_->getRobotModel();
  // Get joint state group
  const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(ee_group_name);
  if( joint_model_group == NULL ) // make sure EE_GROUP exists
  {
    ROS_ERROR_STREAM_NAMED("visual_tools","Unable to find joint model group '" << ee_group_name << "'");
    return false;
  }
  // Get link names that are in end effector
  const std::vector<std::string> &ee_link_names = joint_model_group->getLinkModelNames();
  //ROS_DEBUG_STREAM_NAMED("visual_tools","Number of links in group " << ee_group_name << ": " << ee_link_names.size());
  //std::copy(ee_link_names.begin(), ee_link_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

  // Robot Interaction - finds the end effector associated with a planning group
  robot_interaction::RobotInteraction robot_interaction( robot_model );

  // Decide active end effectors
  robot_interaction.decideActiveComponents(planning_group);

  // Get active EE
  std::vector<robot_interaction::RobotInteraction::EndEffector> active_eef =
    robot_interaction.getActiveEndEffectors();

  ROS_DEBUG_STREAM_NAMED("visual_tools","Number of active end effectors: " << active_eef.size());
  if( !active_eef.size() )
  {
    ROS_ERROR_STREAM_NAMED("visual_tools","No active end effectors found! Make sure kinematics.yaml is loaded in this node's namespace!");
    return false;
  }

  // Just choose the first end effector \todo better logic?
  robot_interaction::RobotInteraction::EndEffector eef = active_eef[0];

  // -----------------------------------------------------------------------------------------------
  // Get EE link markers for Rviz

  shared_robot_state_->getRobotMarkers(ee_marker_array_, ee_link_names, marker_color, eef.eef_group, ros::Duration());
  ROS_DEBUG_STREAM_NAMED("visual_tools","Number of rviz markers in end effector: " << ee_marker_array_.markers.size());

  // Change pose from Eigen to TF
  try
  {
    //ee_parent_link_ = eef.parent_link; // save the name of the link for later use
    tf::poseEigenToTF(shared_robot_state_->getGlobalLinkTransform(eef.parent_link), tf_root_to_link_);
  }
  catch(...)
  {
    ROS_ERROR_STREAM_NAMED("visual_tools","Didn't find link state for " << eef.parent_link);
  }

  // Copy original marker poses to a vector
  for (std::size_t i = 0 ; i < ee_marker_array_.markers.size() ; ++i)
  {
    marker_poses_.push_back( ee_marker_array_.markers[i].pose );
  }

  return true;
}

void VisualTools::resetMarkerCounts()
{
  arrow_id_ = 0;
  sphere_id_ = 0;
  block_id_ = 0;
  cylinder_id_ = 0;
  text_id_ = 0;
  rectangle_id_ = 0;
  line_id_ = 0;
}

bool VisualTools::publishEEMarkers(const geometry_msgs::Pose &pose, const rviz_colors &color, const std::string &ns)
{
  if(muted_)
    return true;

  // Check if we have already loaded the EE markers
  if( ee_marker_array_.markers.empty() ) 
  {
    ROS_ERROR_STREAM_NAMED("visual_tools","Unable to publish EE marker because marker has not been loaded yet");
    return false;
  }

  // -----------------------------------------------------------------------------------------------
  // Change the end effector pose to frame of reference of this custom end effector

  // Convert to Eigen
  Eigen::Affine3d ee_pose_eigen;
  Eigen::Affine3d eef_conversion_pose;
  tf::poseMsgToEigen(pose, ee_pose_eigen);
  tf::poseMsgToEigen(grasp_pose_to_eef_pose_, eef_conversion_pose);

  // Transform the grasp pose
  ee_pose_eigen = ee_pose_eigen * eef_conversion_pose;

  // Convert back to message
  geometry_msgs::Pose ee_pose = convertPose(ee_pose_eigen);

  // -----------------------------------------------------------------------------------------------
  // Process each link of the end effector
  for (std::size_t i = 0 ; i < ee_marker_array_.markers.size() ; ++i)
  {
    // Make sure ROS is still spinning
    if( !ros::ok() )
      break;

    // Header
    ee_marker_array_.markers[i].header.frame_id = base_link_;
    ee_marker_array_.markers[i].header.stamp = ros::Time::now();

    // Namespace
    ee_marker_array_.markers[i].ns = ns;

    // Lifetime
    ee_marker_array_.markers[i].lifetime = marker_lifetime_;

    // Color
    ee_marker_array_.markers[i].color = getColor( color );

    // Options for meshes
    if( ee_marker_array_.markers[i].type == visualization_msgs::Marker::MESH_RESOURCE )
    {
      ee_marker_array_.markers[i].mesh_use_embedded_materials = true;
    }

    // -----------------------------------------------------------------------------------------------
    // Do some math for the offset
    // pose             - our generated grasp
    // markers[i].pose        - an ee link's pose relative to the whole end effector
    // REMOVED grasp_pose_to_eef_pose_ - the offset from the grasp pose to eef_pose - probably nothing
    tf::Pose tf_root_to_marker;
    tf::Pose tf_root_to_mesh;
    tf::Pose tf_pose_to_eef;

    // Simple conversion from geometry_msgs::Pose to tf::Pose
    tf::poseMsgToTF(pose, tf_root_to_marker);
    tf::poseMsgToTF(marker_poses_[i], tf_root_to_mesh);

    // Conversions
    tf::Pose tf_eef_to_mesh = tf_root_to_link_.inverse() * tf_root_to_mesh;
    tf::Pose tf_root_to_mesh_new = tf_root_to_marker * tf_eef_to_mesh;
    tf::poseTFToMsg(tf_root_to_mesh_new, ee_marker_array_.markers[i].pose);
    // -----------------------------------------------------------------------------------------------

    //ROS_INFO_STREAM("Marker " << i << ":\n" << ee_marker_array_.markers[i]);

    loadMarkerPub(); // always check this before publishing
    pub_rviz_marker_.publish( ee_marker_array_.markers[i] );
    ros::spinOnce();
  }

  return true;
}

bool VisualTools::publishSphere(const Eigen::Affine3d &pose, const rviz_colors color, const rviz_scales scale)
{
  return publishSphere(convertPose(pose), color, scale);
}

bool VisualTools::publishSphere(const Eigen::Vector3d &point, const rviz_colors color, const rviz_scales scale)
{
  geometry_msgs::Pose pose_msg;
  tf::pointEigenToMsg(point, pose_msg.position);
  return publishSphere(pose_msg, color, scale);
}

bool VisualTools::publishSphere(const geometry_msgs::Point &point, const rviz_colors color, const rviz_scales scale)
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position = point;
  return publishSphere(pose_msg, color, scale);
}

bool VisualTools::publishSphere(const geometry_msgs::Pose &pose, const rviz_colors color, const rviz_scales scale)
{
  if(muted_)
    return true; // this function will only work if we have loaded the publishers

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  sphere_marker_.header.stamp = ros::Time::now();

  sphere_marker_.id = ++sphere_id_;
  sphere_marker_.color = getColor(color);
  sphere_marker_.scale = getScale(scale, false, 0.1);

  // Update the single point with new pose
  sphere_marker_.points[0] = pose.position;
  sphere_marker_.colors[0] = getColor(color);

  // Publish
  loadMarkerPub(); // always check this before publishing
  pub_rviz_marker_.publish( sphere_marker_ );
  ros::spinOnce();

  return true;
}

bool VisualTools::publishArrow(const Eigen::Affine3d &pose, const rviz_colors color, const rviz_scales scale)
{
  return publishArrow(convertPose(pose), color, scale);
}

bool VisualTools::publishArrow(const geometry_msgs::Pose &pose, const rviz_colors color, const rviz_scales scale)
{
  if(muted_)
    return true;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  arrow_marker_.header.stamp = ros::Time::now();

  arrow_marker_.id = ++arrow_id_;
  arrow_marker_.pose = pose;
  arrow_marker_.color = getColor(color);
  arrow_marker_.scale = getScale(scale, true);

  loadMarkerPub(); // always check this before publishing
  pub_rviz_marker_.publish( arrow_marker_ );
  ros::spinOnce();

  return true;
}

bool VisualTools::publishBlock(const geometry_msgs::Pose &pose, const rviz_colors color, const double &block_size)
{
  if(muted_)
    return true;

  // Set the timestamp
  block_marker_.header.stamp = ros::Time::now();

  block_marker_.id = ++block_id_;

  // Set the pose
  block_marker_.pose = pose;

  // Set marker size
  block_marker_.scale.x = block_size;
  block_marker_.scale.y = block_size;
  block_marker_.scale.z = block_size;

  // Set marker color
  block_marker_.color = getColor( color );

  loadMarkerPub(); // always check this before publishing
  pub_rviz_marker_.publish( block_marker_ );
  ros::spinOnce();

  return true;
}

bool VisualTools::publishCylinder(const geometry_msgs::Pose &pose, const rviz_colors color, double height, double radius)
{
  if(muted_)
    return true;

  // Set the timestamp
  cylinder_marker_.header.stamp = ros::Time::now();

  cylinder_marker_.id = ++cylinder_id_;

  // Set the pose
  cylinder_marker_.pose = pose;

  // Set marker size
  cylinder_marker_.scale.x = radius;
  cylinder_marker_.scale.y = radius;
  cylinder_marker_.scale.z = height;

  // Set marker color
  cylinder_marker_.color = getColor( color );

  loadMarkerPub(); // always check this before publishing
  pub_rviz_marker_.publish( cylinder_marker_ );
  ros::spinOnce();

  return true;
}

bool VisualTools::publishGraph(const graph_msgs::GeometryGraph &graph, const rviz_colors color, double radius)
{
  // Track which pairs of nodes we've already connected since graph is bi-directional
  typedef std::pair<std::size_t, std::size_t> node_ids;
  std::set<node_ids> added_edges;
  std::pair<std::set<node_ids>::iterator,bool> return_value;

  Eigen::Vector3d a, b;
  for (std::size_t i = 0; i < graph.nodes.size(); ++i)
  {
    for (std::size_t j = 0; j < graph.edges[i].node_ids.size(); ++j)
    {
      // Check if we've already added this pair of nodes (edge)
      return_value = added_edges.insert( node_ids(i,j) );
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
        added_edges.insert( node_ids(j,i) );

        // Distance between two points
        double height = (a - b).lpNorm<2>();

        // Find center point
        Eigen::Vector3d pt_center = getCenterPoint(a, b);

        // Create vector
        Eigen::Affine3d pose;
        pose = getVectorBetweenPoints(pt_center, b);

        // Convert pose to be normal to cylindar axis
        Eigen::Affine3d rotation;
        rotation = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitY());
        pose = pose * rotation;

        // Publish individually
        publishCylinder(convertPose(pose), color, height, radius);
      }
    }
  }

}

bool VisualTools::publishRectangle(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2, const rviz_colors color)
{
  if(muted_)
    return true;

  // Set the timestamp
  rectangle_marker_.header.stamp = ros::Time::now();

  rectangle_marker_.id = ++rectangle_id_;
  rectangle_marker_.color = getColor(color);

  // Calculate pose
  geometry_msgs::Pose pose;
  pose.position.x = (point1.x - point2.x) / 2.0 + point2.x;
  pose.position.y = (point1.y - point2.y) / 2.0 + point2.y;
  pose.position.z = (point1.z - point2.z) / 2.0 + point2.z;
  rectangle_marker_.pose = pose;

  // Calculate scale
  rectangle_marker_.scale.x = fabs(point1.x - point2.x);
  rectangle_marker_.scale.y = fabs(point1.y - point2.y);
  rectangle_marker_.scale.z = fabs(point1.z - point2.z);

  loadMarkerPub(); // always check this before publishing
  pub_rviz_marker_.publish( rectangle_marker_ );
  ros::spinOnce();

  return true;
}

bool VisualTools::publishLine(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
  const rviz_colors color, const rviz_scales scale)
{
  if(muted_)
    return true;

  // Set the timestamp
  line_marker_.header.stamp = ros::Time::now();

  line_marker_.id = ++line_id_;
  line_marker_.color = getColor(color);
  line_marker_.scale = getScale( scale, false, 0.1 );

  line_marker_.points.clear();
  line_marker_.points.push_back(point1);
  line_marker_.points.push_back(point2);

  loadMarkerPub(); // always check this before publishing
  pub_rviz_marker_.publish( line_marker_ );
  ros::spinOnce();

  return true;
}

bool VisualTools::publishText(const geometry_msgs::Pose &pose, const std::string &text, const rviz_colors &color)
{
  if(muted_)
    return true;

  text_marker_.id = 0;

  text_marker_.header.stamp = ros::Time::now();
  text_marker_.text = text;
  text_marker_.pose = pose;
  text_marker_.color = getColor( color );
  text_marker_.scale.z = 0.01;    // only z is required (size of an "A")

  loadMarkerPub(); // always check this before publishing
  pub_rviz_marker_.publish( text_marker_ );
  ros::spinOnce();

  return true;
}

bool VisualTools::publishGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
  const std::string &ee_parent_link)
{
  if(muted_)
  {
    ROS_DEBUG_STREAM_NAMED("visual_tools","Not visualizing grasps - muted.");
    return false;
  }

  ROS_DEBUG_STREAM_NAMED("visual_tools","Visualizing " << possible_grasps.size() << " grasps with parent link " 
    << ee_parent_link);

  // Loop through all grasps
  for (std::size_t i = 0; i < possible_grasps.size(); ++i)
  {
    if( !ros::ok() )  // Check that ROS is still ok and that user isn't trying to quit
      break;

    ROS_DEBUG_STREAM_NAMED("grasp","Visualizing grasp pose " << i);

    publishEEMarkers(possible_grasps[i].grasp_pose.pose);

    ros::Duration(0.1).sleep();
  }

  return true;
}

bool VisualTools::publishAnimatedGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
  const std::string &ee_parent_link, double animate_speed)
{
  if(muted_)
  {
    ROS_DEBUG_STREAM_NAMED("visual_tools","Not visualizing grasps - muted.");
    return false;
  }

  ROS_DEBUG_STREAM_NAMED("visual_tools","Visualizing " << possible_grasps.size() << " grasps with parent link " 
    << ee_parent_link << " at speed " << animate_speed);

  // Loop through all grasps
  for (std::size_t i = 0; i < possible_grasps.size(); ++i)
  {
    if( !ros::ok() )  // Check that ROS is still ok and that user isn't trying to quit
      break;

    publishAnimatedGrasp(possible_grasps[i], ee_parent_link, animate_speed);

    ros::Duration(0.1).sleep();
  }

  return true;
}

bool VisualTools::publishAnimatedGrasp(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link, double animate_speed)
{
  // Grasp Pose Variables
  geometry_msgs::Pose grasp_pose = grasp.grasp_pose.pose;
  Eigen::Affine3d grasp_pose_eigen;
  tf::poseMsgToEigen(grasp_pose, grasp_pose_eigen);

  // Pre-grasp pose variables
  geometry_msgs::Pose pre_grasp_pose;
  Eigen::Affine3d pre_grasp_pose_eigen;

  // Approach direction variables
  Eigen::Vector3d pre_grasp_approach_direction_local;

  // Display Grasp Score
  std::string text = "Grasp Quality: " + boost::lexical_cast<std::string>(int(grasp.grasp_quality*100)) + "%";
  publishText(grasp_pose, text);

  // Convert the grasp pose into the frame of reference of the approach/retreat frame_id

  // Animate the movement - for ee approach direction
  double animation_resulution = 0.1; // the lower the better the resolution
  for(double percent = 0; percent < 1; percent += animation_resulution)
  {
    if( !ros::ok() ) // Check that ROS is still ok and that user isn't trying to quit
      break;

    // Copy original grasp pose to pre-grasp pose
    pre_grasp_pose_eigen = grasp_pose_eigen;

    // The direction of the pre-grasp
    // Calculate the current animation position based on the percent
    Eigen::Vector3d pre_grasp_approach_direction = Eigen::Vector3d(
      -1 * grasp.pre_grasp_approach.direction.vector.x * grasp.pre_grasp_approach.desired_distance * (1-percent),
      -1 * grasp.pre_grasp_approach.direction.vector.y * grasp.pre_grasp_approach.desired_distance * (1-percent),
      -1 * grasp.pre_grasp_approach.direction.vector.z * grasp.pre_grasp_approach.desired_distance * (1-percent)
    );

    // Decide if we need to change the approach_direction to the local frame of the end effector orientation
    if( grasp.pre_grasp_approach.direction.header.frame_id == ee_parent_link )
    {
      // Apply/compute the approach_direction vector in the local frame of the grasp_pose orientation
      pre_grasp_approach_direction_local = grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
    }
    else
    {
      pre_grasp_approach_direction_local = pre_grasp_approach_direction; //grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
    }

    // Update the grasp matrix usign the new locally-framed approach_direction
    pre_grasp_pose_eigen.translation() += pre_grasp_approach_direction_local;

    // Convert eigen pre-grasp position back to regular message
    tf::poseEigenToMsg(pre_grasp_pose_eigen, pre_grasp_pose);

    //publishArrow(pre_grasp_pose, moveit_visual_tools::BLUE);
    publishEEMarkers(pre_grasp_pose);

    ros::Duration(animate_speed).sleep();
  }
  return true;
}

bool VisualTools::publishIKSolutions(const std::vector<trajectory_msgs::JointTrajectoryPoint> &ik_solutions, 
  const std::string& planning_group, double display_time)
{
  if(muted_)
  {
    ROS_DEBUG_STREAM_NAMED("visual_tools","Not visualizing inverse kinematic solutions - muted.");
    return false;
  }

  if (ik_solutions.empty())
  {
    ROS_WARN_STREAM_NAMED("visual_tools","Empty ik_solutions vector passed into publishIKSolutions()");
    return false;
  }

  loadSharedRobotState();

  ROS_DEBUG_STREAM_NAMED("visual_tools","Visualizing " << ik_solutions.size() << " inverse kinematic solutions");

  // Get robot model
  robot_model::RobotModelConstPtr robot_model = shared_robot_state_->getRobotModel();
  // Get joint state group
  const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(planning_group);

  if (joint_model_group == NULL) // not found
  {
    ROS_ERROR_STREAM_NAMED("publishIKSolutions","Could not find joint model group " << planning_group);
    return false;
  }

  // Apply the time to the trajectory
  trajectory_msgs::JointTrajectoryPoint trajectory_pt_timed;

  // Create a trajectory with one point
  moveit_msgs::RobotTrajectory trajectory_msg;
  trajectory_msg.joint_trajectory.header.frame_id = base_link_;
  trajectory_msg.joint_trajectory.joint_names = joint_model_group->getJointModelNames();

  // Overall length of trajectory
  double running_time = 0;

  // Loop through all inverse kinematic solutions
  for (std::size_t i = 0; i < ik_solutions.size(); ++i)
  {
    if( !ros::ok() )  // Check that ROS is still ok and that user isn't trying to quit
      break;

    trajectory_pt_timed = ik_solutions[i];
    trajectory_pt_timed.time_from_start = ros::Duration(running_time);
    trajectory_msg.joint_trajectory.points.push_back(trajectory_pt_timed);

    running_time += display_time;

    //ROS_DEBUG_STREAM_NAMED("grasp","Visualizing ik solution " << i);
  }

  // Re-add final position so the last point is displayed fully
  trajectory_pt_timed = trajectory_msg.joint_trajectory.points.back();
  trajectory_pt_timed.time_from_start = ros::Duration(running_time);
  trajectory_msg.joint_trajectory.points.push_back(trajectory_pt_timed);

  return publishTrajectoryPath(trajectory_msg, true);
}

bool VisualTools::removeAllCollisionObjects()
{
  // Publish an empty REMOVE message so as to remove all of them

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.world.collision_objects.clear();

  // Clean up old collision objects
  moveit_msgs::CollisionObject remove_object;
  remove_object.header.frame_id = base_link_;
  remove_object.operation = moveit_msgs::CollisionObject::REMOVE;

  planning_scene.world.collision_objects.push_back(remove_object);

  // Publish
  loadPlanningPub(); // always call this before publishing
  pub_planning_scene_diff_.publish(planning_scene);
  ros::spinOnce();

  return true;
}

bool VisualTools::cleanupCO(std::string name)
{
  // Clean up old collision objects
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = base_link_;
  co.id = name;
  co.operation = moveit_msgs::CollisionObject::REMOVE;

  loadCollisionPub(); // always call this before publishing
  pub_collision_obj_.publish(co);
  ros::spinOnce();
  return true;
}

bool VisualTools::cleanupACO(const std::string& name)
{
  // Clean up old attached collision object
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.header.stamp = ros::Time::now();
  aco.object.header.frame_id = base_link_;

  //aco.object.id = name;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;

  //aco.link_name = ee_parent_link_;

  loadAttachedPub(); // always call this before publishing
  pub_attach_collision_obj_.publish(aco);
  ros::spinOnce();
  return true;
}

bool VisualTools::attachCO(const std::string& name, const std::string& ee_parent_link)
{
  // Clean up old attached collision object
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.header.stamp = ros::Time::now();
  aco.object.header.frame_id = base_link_;

  aco.object.id = name;
  aco.object.operation = moveit_msgs::CollisionObject::ADD;

  // Link to attach the object to
  aco.link_name = ee_parent_link;

  loadAttachedPub(); // always call this before publishing
  pub_attach_collision_obj_.publish(aco);
  ros::spinOnce();
  return true;
}

bool VisualTools::publishCollisionBlock(geometry_msgs::Pose block_pose, std::string block_name, double block_size)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_link_;
  collision_obj.id = block_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = block_size;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = block_size;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = block_size;
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = block_pose;

  //ROS_INFO_STREAM_NAMED("pick_place","CollisionObject: \n " << collision_obj);

  loadCollisionPub(); // always call this before publishing
  pub_collision_obj_.publish(collision_obj);
  ros::spinOnce();

  ROS_DEBUG_STREAM_NAMED("visual_tools","Published collision object " << block_name);
  return true;
}

bool VisualTools::publishCollisionCylinder(geometry_msgs::Point a, geometry_msgs::Point b, std::string object_name, double radius)
{
  return publishCollisionCylinder(convertPoint(a), convertPoint(b), object_name, radius);
}

bool VisualTools::publishCollisionCylinder(Eigen::Vector3d a, Eigen::Vector3d b, std::string object_name, double radius)
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
  rotation = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitY());
  pose = pose * rotation;

  return publishCollisionCylinder(pose, object_name, radius, height);
}

bool VisualTools::publishCollisionCylinder(Eigen::Affine3d object_pose, std::string object_name, double radius, double height)
{
  return publishCollisionCylinder(convertPose(object_pose), object_name, radius, height);
}

bool VisualTools::publishCollisionCylinder(geometry_msgs::Pose object_pose, std::string object_name, double radius, double height)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_link_;
  collision_obj.id = object_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = object_pose;

  //ROS_INFO_STREAM_NAMED("pick_place","CollisionObject: \n " << collision_obj);

  loadCollisionPub(); // always call this before publishing
  pub_collision_obj_.publish(collision_obj);
  ros::spinOnce();

  //ROS_DEBUG_STREAM_NAMED("visual_tools","Published collision object " << object_name);
  return true;
}

bool VisualTools::publishCollisionGraph(const graph_msgs::GeometryGraph &graph, const std::string &object_name, double radius)
{
  ROS_INFO_STREAM_NAMED("publishCollisionGraph","Preparing to create collision graph");

  // The graph is one collision object with many primitives
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_link_;
  collision_obj.id = object_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;

  // Track which pairs of nodes we've already connected since graph is bi-directional
  typedef std::pair<std::size_t, std::size_t> node_ids;
  std::set<node_ids> added_edges;
  std::pair<std::set<node_ids>::iterator,bool> return_value;

  Eigen::Vector3d a, b;
  for (std::size_t i = 0; i < graph.nodes.size(); ++i)
  {
    for (std::size_t j = 0; j < graph.edges[i].node_ids.size(); ++j)
    {
      // Check if we've already added this pair of nodes (edge)
      return_value = added_edges.insert( node_ids(i,j) );
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
        added_edges.insert( node_ids(j,i) );

        // Distance between two points
        double height = (a - b).lpNorm<2>();

        // Find center point
        Eigen::Vector3d pt_center = getCenterPoint(a, b);

        // Create vector
        Eigen::Affine3d pose;
        pose = getVectorBetweenPoints(pt_center, b);

        // Convert pose to be normal to cylindar axis
        Eigen::Affine3d rotation;
        rotation = Eigen::AngleAxisd(0.5*M_PI, Eigen::Vector3d::UnitY());
        pose = pose * rotation;

        // Create the solid primitive
        shape_msgs::SolidPrimitive cylinder;
        cylinder.type = shape_msgs::SolidPrimitive::CYLINDER;
        cylinder.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
        cylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
        cylinder.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;

        // Add to the collision object
        collision_obj.primitives.push_back(cylinder);

        // Add the pose
        collision_obj.primitive_poses.push_back(convertPose(pose));
      }
    }
  }

  loadCollisionPub(); // always call this before publishing
  pub_collision_obj_.publish(collision_obj);
  ros::spinOnce();

  return true;
}

bool VisualTools::publishCollisionWall(double x, double y, double angle, double width, const std::string name)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_link_;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);

  geometry_msgs::Pose rec_pose;

  // ----------------------------------------------------------------------------------
  // Name
  collision_obj.id = name;

  double depth = 0.1;
  double height = 2.5;

  // Position
  rec_pose.position.x = x;
  rec_pose.position.y = y;
  rec_pose.position.z = height / 2 + floor_to_base_height_;

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

  loadCollisionPub(); // always call this before publishing
  pub_collision_obj_.publish(collision_obj);
  ros::spinOnce();

  return true;
}

bool VisualTools::publishCollisionTable(double x, double y, double angle, double width, double height,
  double depth, const std::string name)
{
  geometry_msgs::Pose table_pose;

  // Position
  table_pose.position.x = x;
  table_pose.position.y = y;
  table_pose.position.z = height / 2 + floor_to_base_height_;

  // Orientation
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  table_pose.orientation.x = quat.x();
  table_pose.orientation.y = quat.y();
  table_pose.orientation.z = quat.z();
  table_pose.orientation.w = quat.w();

  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_link_;
  collision_obj.id = name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);

  // Size
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = depth;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = height;

  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = table_pose;

  loadCollisionPub(); // always call this before publishing
  pub_collision_obj_.publish(collision_obj);
  ros::spinOnce();

  return true;
}

bool VisualTools::publishTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& trajectory_pt,
  const std::string &group_name, double display_time)
{
  loadSharedRobotState();

  // Get robot model
  robot_model::RobotModelConstPtr robot_model = shared_robot_state_->getRobotModel();
  // Get joint state group
  const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(group_name);

  if (joint_model_group == NULL) // not found
  {
    ROS_ERROR_STREAM_NAMED("publishTrajectoryPoint","Could not find joint model group " << group_name);
    return false;
  }

  // Apply the time to the trajectory
  trajectory_msgs::JointTrajectoryPoint trajectory_pt_timed = trajectory_pt;
  trajectory_pt_timed.time_from_start = ros::Duration(display_time);

  // Create a trajectory with one point
  moveit_msgs::RobotTrajectory trajectory_msg;
  trajectory_msg.joint_trajectory.header.frame_id = base_link_;
  trajectory_msg.joint_trajectory.joint_names = joint_model_group->getJointModelNames();
  trajectory_msg.joint_trajectory.points.push_back(trajectory_pt);
  trajectory_msg.joint_trajectory.points.push_back(trajectory_pt_timed);

  return publishTrajectoryPath(trajectory_msg, true);
}

bool VisualTools::publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg,
  bool blocking)
{
  // Create the message
  moveit_msgs::DisplayTrajectory rviz_display;
  rviz_display.model_id = getPlanningSceneMonitor()->getPlanningScene()->getRobotModel()->getName();

  //    rviz_display.trajectory_start = start_state;
  rviz_display.trajectory.resize(1);
  rviz_display.trajectory[0] = trajectory_msg;

  // Publish message
  loadTrajectoryPub(); // always call this before publishing
  pub_display_path_.publish(rviz_display);
  ros::spinOnce();

  // Wait the duration of the trajectory
  if( blocking )
  {
    ROS_INFO_STREAM_NAMED("visual_tools","Waiting for trajectory animation "
      << trajectory_msg.joint_trajectory.points.back().time_from_start << " seconds");

    // Check if ROS is ok in intervals
    double counter = 0;
    while (ros::ok() && counter < trajectory_msg.joint_trajectory.points.back().time_from_start.toSec())
    {
      counter += 0.25; // check every fourth second
      ros::Duration(0.25).sleep();
    }
  }

  return true;
}

bool VisualTools::publishRobotState(const robot_state::RobotState &robot_state)
{
  robot_state::robotStateToRobotStateMsg(robot_state, display_robot_msg_.state);
  loadRobotPub(); // always call this before publishinga
  pub_robot_state_.publish( display_robot_msg_ );
  ros::spinOnce();

  return true;
}

bool VisualTools::publishRobotState(const trajectory_msgs::JointTrajectoryPoint& trajectory_pt,
  const std::string &group_name)
{
  // Always oad the robot state before using
  loadSharedRobotState();

  // Set robot state
  shared_robot_state_->setToDefaultValues(); // reset the state just in case
  shared_robot_state_->setJointGroupPositions(group_name, trajectory_pt.positions);

  // Publish robot state
  publishRobotState(*shared_robot_state_);

  return true;
}

bool VisualTools::publishTest()
{
  // Create pose
  geometry_msgs::Pose pose;
  generateRandomPose(pose);

  // Publish arrow vector of pose
  ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
  publishArrow(pose, moveit_visual_tools::RED, moveit_visual_tools::LARGE);
}

geometry_msgs::Pose VisualTools::convertPose(const Eigen::Affine3d &pose)
{
  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(pose, pose_msg);
  return pose_msg;
}

Eigen::Vector3d VisualTools::convertPoint(const geometry_msgs::Point &point)
{
  Eigen::Vector3d point_eigen;
  point_eigen[0] = point.x;
  point_eigen[1] = point.y;
  point_eigen[2] = point.z;
  return point_eigen;
}

void VisualTools::generateRandomPose(geometry_msgs::Pose& pose)
{
  // Position
  pose.position.x = dRand(0, 1);
  pose.position.y = dRand(0, 1);
  pose.position.z = dRand(0, 1);

  // Orientation on place
  double angle = M_PI * dRand(0.1,1);
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();
}

double VisualTools::dRand(double dMin, double dMax)
{
  double d = (double)rand() / RAND_MAX;
  return dMin + d * (dMax - dMin);
}

void VisualTools::print()
{
  ROS_WARN_STREAM_NAMED("visual_tools","Debug Visual Tools variable values:");  
  std::cout << "marker_topic_: " << marker_topic_ << std::endl;
  std::cout << "base_link_: " << base_link_ << std::endl;
  std::cout << "floor_to_base_height_: " << floor_to_base_height_ << std::endl;
  std::cout << "marker_lifetime_: " << marker_lifetime_.toSec() << std::endl;
  std::cout << "muted_: " << muted_ << std::endl;
  std::cout << "alpha_: " << alpha_ << std::endl;
}

/**
 * \brief Move the robot arm to the ik solution in rviz
 * \param joint_values - the in-order list of values to set the robot's joints
 * \return true if it is successful
 *
 */
/*
  bool VisualTools::publishPlanningScene(std::vector<double> joint_values)
  {
  if(muted_)
  return true; // this function will only work if we have loaded the publishers

  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing planning scene");

  // Output debug
  //ROS_INFO_STREAM_NAMED("visual_tools","Joint values being sent to planning scene:");
  //std::copy(joint_values.begin(),joint_values.end(), std::ostream_iterator<double>(std::cout, "\n"));

  // Update planning scene
  robot_state::JointStateGroup* joint_state_group = getPlanningSceneMonitor()->getPlanningScene()->getCurrentStateNonConst()
  .getJointStateGroup(planning_group_name_);
  joint_state_group->setVariableValues(joint_values);

  //    getPlanningSceneMonitor()->updateFrameTransforms();
  getPlanningSceneMonitor()->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

  return true;
  }
*/

/**
 * \brief Publish an marker of a mesh to rviz
 * \return true if it is successful
 *
 bool publishMesh(const geometry_msgs::Pose &pose, const rviz_colors &color = WHITE,
 const std::string &ns="mesh")
 {
 if(muted_)
 return true; // this function will only work if we have loaded the publishers

 visualization_msgs::Marker marker;
 // Set the frame ID and timestamp.  See the TF tutorials for information on these.
 marker.header.frame_id = base_link_;
 marker.header.stamp = ros::Time::now();

 // Set the namespace and id for this marker.  This serves to create a unique ID
 marker.ns = "Mesh";

 // Set the marker type.
 marker.type = visualization_msgs::Marker::MESH_RESOURCE;
 marker.mesh_resource = "package://clam_description/stl/gripper_base_link.STL";

 // Set the marker action.  Options are ADD and DELETE
 marker.action = visualization_msgs::Marker::ADD;
 marker.id = 0;

 marker.pose.position.x = x;
 marker.pose.position.y = y;
 marker.pose.position.z = z;

 marker.pose.orientation.x = qx;
 marker.pose.orientation.y = qy;
 marker.pose.orientation.z = qz;
 marker.pose.orientation.w = qw;

 marker.scale.x = 0.001;
 marker.scale.y = 0.001;
 marker.scale.z = 0.001;

 marker.color = getColor( RED );

 // Make line color
 std_msgs::ColorRGBA color = getColor( RED );

 // Point
 geometry_msgs::Point point_a;
 point_a.x = x;
 point_a.y = y;
 point_a.z = z;

 // Add the point pair to the line message
 marker.points.push_back( point_a );
 marker.colors.push_back( color );

 marker.lifetime = marker_lifetime_;

 loadMarkerPub(); // always check this before publishing
 pub_rviz_marker_.publish( marker );
 ros::spinOnce();

 return true;
 }
*/


} // namespace
