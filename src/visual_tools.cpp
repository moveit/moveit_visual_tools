/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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

namespace moveit_visual_tools
{

VisualTools::VisualTools(std::string base_link,
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
  std::string marker_topic)
  : planning_scene_monitor_(planning_scene_monitor)
{
  // Pass to next contructor
  VisualTools(base_link, marker_topic);
}

VisualTools::VisualTools(std::string base_link, std::string marker_topic)
  : nh_("~"),
    marker_topic_(marker_topic),
    ee_group_name_("unknown"),
    planning_group_name_("unknown"),
    base_link_(base_link),
    floor_to_base_height_(0),
    marker_lifetime_(ros::Duration(30.0)),
    muted_(true),
    alpha_(0.8)
{
  // Initialize counters to zero
  resetMarkerCounts();

  // Rviz Visualizations
  pub_rviz_marker_ = nh_.advertise<visualization_msgs::Marker>(marker_topic_, 10);
  ROS_DEBUG_STREAM_NAMED("viz_tools","Visualizing rviz markers on topic " << marker_topic_);

  // Collision object creator
  pub_collision_obj_ = nh_.advertise<moveit_msgs::CollisionObject>(COLLISION_TOPIC, 10);
  ROS_DEBUG_STREAM_NAMED("viz_tools","Publishing collision objects on topic " << COLLISION_TOPIC);

  // Collision object attacher
  pub_attach_collision_obj_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>
    (ATTACHED_COLLISION_TOPIC, 10);
  ROS_DEBUG_STREAM_NAMED("viz_tools","Publishing attached collision objects on topic "
    << ATTACHED_COLLISION_TOPIC);

  // Trajectory paths
  pub_display_path_ = nh_.advertise<moveit_msgs::DisplayTrajectory>
    ("/move_group/display_planned_path", 10, false);

  // Cache the reusable markers
  loadRvizMarkers();

  // Wait
  ros::spinOnce();
  ros::Duration(0.5).sleep();
}

VisualTools::~VisualTools()
{
}

void VisualTools::setFloorToBaseHeight(double floor_to_base_height)
{
  floor_to_base_height_ = floor_to_base_height;
}

void VisualTools::setGraspPoseToEEFPose(geometry_msgs::Pose grasp_pose_to_eef_pose)
{
  grasp_pose_to_eef_pose_ = grasp_pose_to_eef_pose;
}

void VisualTools::loadRvizMarkers()
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
  // ---------------------------------------------------------------------------------------------
  // Create planning scene monitor
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION));

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  ros::spinOnce();

  if (planning_scene_monitor_->getPlanningScene())
  {
    //planning_scene_monitor_->startWorldGeometryMonitor();
    //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
    //planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
    //planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
    //  "dave_planning_scene");
  }
  else
  {
    ROS_FATAL_STREAM_NAMED("rviz_tools","Planning scene not configured");
    return false;
  }

  ros::spinOnce();
  ros::Duration(0.5).sleep();
  ros::spinOnce();

  return true;
}


/**  NEEDS TO BE PORTED TO THE NEW MOVEIT FUNCTIONALITY
 * \brief Move the robot arm to the ik solution in rviz
 * \param joint_values - the in-order list of values to set the robot's joints
 * \return true if it is successful
 *
 bool publishPlanningScene(std::vector<double> joint_values)
 {
 if(muted_)
 return true; // this function will only work if we have loaded the publishers

 ROS_DEBUG_STREAM_NAMED("viz_tools","Publishing planning scene");

 // Output debug
 //ROS_INFO_STREAM_NAMED("viz_tools","Joint values being sent to planning scene:");
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

bool VisualTools::loadRobotMarkers()
{
  ROS_ERROR_STREAM_NAMED("temp","loading robot markers");



  // Get robot model
  robot_model::RobotModelConstPtr robot_model = getPlanningSceneMonitor()->getRobotModel();

  /*
  // Get joint state group
  //robot_state::JointStateGroup* joint_state_group = robot_state.getJointStateGroup(ee_group_name_);
  const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(ee_group_name_);

  if( joint_model_group == NULL ) // make sure EE_GROUP exists
  {
  ROS_ERROR_STREAM_NAMED("viz_tools","Unable to find joint model group " << ee_group_name_ );
  return false;
  }
  */

  // Get all link names
  const std::vector<std::string> &link_names = robot_model->getLinkModelNames();;

  ROS_DEBUG_STREAM_NAMED("viz_tools","Number of links in baxter: " << link_names.size());
  //    std::copy(link_names.begin(), link_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

  // -----------------------------------------------------------------------------------------------
  // Get EE link markers for Rviz
  //    robot_state::RobotState robot_state = getPlanningSceneMonitor()->getPlanningScene()->getCurrentState();
  robot_state::RobotState robot_state(robot_model);

  robot_state.updateLinkTransforms();
  //robot_state.printStateInfo();
  //robot_state.printTransforms();

  visualization_msgs::MarkerArray robot_marker_array;
  robot_state.getRobotMarkers(robot_marker_array, link_names, getColor( GREY ), "dave test", ros::Duration());

  ROS_DEBUG_STREAM_NAMED("viz_tools","Number of rviz markers: " << robot_marker_array.markers.size());

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

bool VisualTools::loadEEMarker()
{
  // Check if we have already loaded the EE markers
  if( ee_marker_array_.markers.size() > 0 ) // already loaded
    return true;

  // -----------------------------------------------------------------------------------------------
  // Get end effector group

  // Create color to use for EE markers
  std_msgs::ColorRGBA marker_color = getColor( GREY );

  // Get robot model
  robot_model::RobotModelConstPtr robot_model = getPlanningSceneMonitor()->getRobotModel();
  // Get joint state group
  //robot_state::JointStateGroup* joint_state_group = robot_state.getJointStateGroup(ee_group_name_);
  const robot_model::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(ee_group_name_);
  if( joint_model_group == NULL ) // make sure EE_GROUP exists
  {
    ROS_ERROR_STREAM_NAMED("viz_tools","Unable to find joint model group " << ee_group_name_ );
    return false;
  }

  // Get link names that are in end effector
  const std::vector<std::string> &ee_link_names = joint_model_group->getLinkModelNames();

  ROS_DEBUG_STREAM_NAMED("viz_tools","Number of links in group " << ee_group_name_ << ": " << ee_link_names.size());
  //std::copy(ee_link_names.begin(), ee_link_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

  // Robot Interaction - finds the end effector associated with a planning group
  //robot_interaction::RobotInteraction robot_interaction( getPlanningSceneMonitor()->getRobotModel() );
  robot_interaction::RobotInteraction robot_interaction( robot_model );

  // Decide active end effectors
  robot_interaction.decideActiveComponents(planning_group_name_);

  // Get active EE
  std::vector<robot_interaction::RobotInteraction::EndEffector> active_eef =
    robot_interaction.getActiveEndEffectors();

  ROS_DEBUG_STREAM_NAMED("viz_tools","Number of active end effectors: " << active_eef.size());
  if( !active_eef.size() )
  {
    ROS_ERROR_STREAM_NAMED("viz_tools","No active end effectors found! Make sure kinematics.yaml is loaded in this node's namespace!");
    return false;
  }

  // Just choose the first end effector \todo better logic?
  robot_interaction::RobotInteraction::EndEffector eef = active_eef[0];

  // -----------------------------------------------------------------------------------------------
  // Get EE link markers for Rviz
  robot_state::RobotState robot_state = getPlanningSceneMonitor()->getPlanningScene()->getCurrentState();
  //    robot_state.updateTransforms();

  /*ROS_ERROR_STREAM_NAMED("temp","before printing");
    robot_state.printStateInfo();
    robot_state.printTransforms();
  */

  robot_state.getRobotMarkers(ee_marker_array_, ee_link_names, marker_color, eef.eef_group, ros::Duration());
  ROS_DEBUG_STREAM_NAMED("viz_tools","Number of rviz markers in end effector: " << ee_marker_array_.markers.size());

  // Change pose from Eigen to TF
  try
  {
    ee_parent_link_ = eef.parent_link; // save the name of the link for later use
    tf::poseEigenToTF(robot_state.getGlobalLinkTransform(eef.parent_link), tf_root_to_link_);
    //tf::poseEigenToTF(robot_state.getLinkState(eef.parent_link)->getGlobalLinkTransform(), tf_root_to_link_);
  }
  catch(...)
  {
    ROS_ERROR_STREAM_NAMED("viz_tools","Didn't find link state for " << ee_parent_link_);
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
  text_id_ = 0;
  rectangle_id_ = 0;
  line_id_ = 0;
}

bool VisualTools::publishEEMarkers(const geometry_msgs::Pose &pose,
  const rviz_colors &color, const std::string &ns)
{
  if(muted_)
    return true;

  // Load EE Markers
  if( !loadEEMarker() )
  {
    ROS_ERROR_STREAM_NAMED("viz_tools","Unable to publish EE marker");
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
  geometry_msgs::Pose ee_pose;     // Non const version
  tf::poseEigenToMsg(ee_pose_eigen, ee_pose);

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
    // tf::poseMsgToTF(grasp_pose_to_eef_pose_, tf_pose_to_eef); // \todo REMOVE

    // Conversions
    tf::Pose tf_eef_to_mesh = tf_root_to_link_.inverse() * tf_root_to_mesh;
    // REMOVED tf::Pose tf_marker_to_mesh = tf_pose_to_eef * tf_eef_to_mesh;
    //tf::Pose tf_root_to_mesh_new = tf_root_to_marker * tf_marker_to_mesh;
    tf::Pose tf_root_to_mesh_new = tf_root_to_marker * tf_eef_to_mesh;
    tf::poseTFToMsg(tf_root_to_mesh_new, ee_marker_array_.markers[i].pose);
    // -----------------------------------------------------------------------------------------------

    //ROS_INFO_STREAM("Marker " << i << ":\n" << ee_marker_array_.markers[i]);

    pub_rviz_marker_.publish( ee_marker_array_.markers[i] );
    ros::spinOnce();
  }

  return true;
}

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
 ROS_ERROR_STREAM_NAMED("temp","TODO - add mesh resource");

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

 pub_rviz_marker_.publish( marker );
 ros::spinOnce();

 return true;
 }
*/

bool VisualTools::publishSphere(const Eigen::Affine3d &pose, const rviz_colors color, const rviz_scales scale)
{
  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(pose, pose_msg);
  publishSphere(pose_msg, color, scale);
}

bool VisualTools::publishSphere(const Eigen::Vector3d &point, const rviz_colors color, const rviz_scales scale)
{
  geometry_msgs::Pose pose_msg;
  tf::pointEigenToMsg(point, pose_msg.position);
  publishSphere(pose_msg, color, scale);
}

bool VisualTools::publishSphere(const geometry_msgs::Point &point, const rviz_colors color, const rviz_scales scale)
{
  geometry_msgs::Pose pose_msg;
  pose_msg.position = point;
  publishSphere(pose_msg, color, scale);
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
  pub_rviz_marker_.publish( sphere_marker_ );
  ros::spinOnce();

  return true;
}

bool VisualTools::publishArrow(const Eigen::Affine3d &pose, const rviz_colors color, const rviz_scales scale)
{
  geometry_msgs::Pose pose_msg;
  tf::poseEigenToMsg(pose, pose_msg);
  publishArrow(pose_msg, color, scale);
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

  pub_rviz_marker_.publish( arrow_marker_ );
  ros::spinOnce();

  return true;
}

bool VisualTools::publishBlock(const geometry_msgs::Pose &pose, const double &block_size, const bool isRed)
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
  if(isRed)
  {
    block_marker_.color = getColor( RED );
  }
  else
  {
    block_marker_.color = getColor( GREEN );
  }

  pub_rviz_marker_.publish( block_marker_ );
  //ros::Duration(0.05).sleep(); // Sleep to prevent markers from being 'skipped' in rviz

  return true;
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

  pub_rviz_marker_.publish( text_marker_ );

  return true;
}

void VisualTools::cleanupCO(std::string name)
{
  // Clean up old collision objects
  moveit_msgs::CollisionObject co;
  co.header.stamp = ros::Time::now();
  co.header.frame_id = base_link_;
  co.id = name;
  co.operation = moveit_msgs::CollisionObject::REMOVE;
  ros::WallDuration(0.1).sleep();
  pub_collision_obj_.publish(co);
  ros::WallDuration(0.1).sleep();
  pub_collision_obj_.publish(co);
}

void VisualTools::cleanupACO(const std::string& name)
{
  // Clean up old attached collision object
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.header.stamp = ros::Time::now();
  aco.object.header.frame_id = base_link_;

  //aco.object.id = name;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;

  aco.link_name = ee_parent_link_;

  ros::WallDuration(0.1).sleep();
  pub_attach_collision_obj_.publish(aco);
  ros::WallDuration(0.1).sleep();
  pub_attach_collision_obj_.publish(aco);

}
void VisualTools::attachCO(const std::string& name)
{
  // Clean up old attached collision object
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.header.stamp = ros::Time::now();
  aco.object.header.frame_id = base_link_;

  aco.object.id = name;
  aco.object.operation = moveit_msgs::CollisionObject::ADD;

  // Link to attach the object to
  aco.link_name = ee_parent_link_;

  ros::WallDuration(0.1).sleep();
  pub_attach_collision_obj_.publish(aco);
  ros::WallDuration(0.1).sleep();
  pub_attach_collision_obj_.publish(aco);

}

void VisualTools::publishCollisionBlock(geometry_msgs::Pose block_pose, std::string block_name, double block_size)
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

  pub_collision_obj_.publish(collision_obj);

  ROS_DEBUG_STREAM_NAMED("simple_pick_place","Published collision object " << block_name);
}

void VisualTools::publishCollisionWall(double x, double y, double angle, double width, const std::string name)
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

  pub_collision_obj_.publish(collision_obj);
}

void VisualTools::publishCollisionTable(double x, double y, double angle, double width, double height,
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

  pub_collision_obj_.publish(collision_obj);
}

bool VisualTools::publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg,
  bool waitTrajectory)
{
  // Create the message
  moveit_msgs::DisplayTrajectory rviz_display;
  rviz_display.model_id = getPlanningSceneMonitor()->getPlanningScene()->getRobotModel()->getName();

  //    rviz_display.trajectory_start = start_state;
  rviz_display.trajectory.resize(1);
  rviz_display.trajectory[0] = trajectory_msg;

  // Publish message
  pub_display_path_.publish(rviz_display);

  ros::Duration(0.1).sleep();

  // Wait the duration of the trajectory
  if( waitTrajectory )
  {
    ros::Duration wait_sec = trajectory_msg.joint_trajectory.points.back().time_from_start * 4;
    ROS_INFO_STREAM_NAMED("viz_tools","Waiting for trajectory animation " << wait_sec.toSec() << " seconds");
    wait_sec.sleep();
  }

  return true;
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

} // namespace
