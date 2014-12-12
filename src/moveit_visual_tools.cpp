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

#include <moveit_visual_tools/moveit_visual_tools.h>
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

MoveItVisualTools::MoveItVisualTools(const std::string& base_frame,
                                     const std::string& marker_topic,
                                     planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
  :  RvizVisualTools::RvizVisualTools(base_frame, marker_topic),
     planning_scene_monitor_(planning_scene_monitor)
{

}

MoveItVisualTools::MoveItVisualTools(const std::string& base_frame,
                                     const std::string& marker_topic,
                                     robot_model::RobotModelConstPtr robot_model)
  :  RvizVisualTools::RvizVisualTools(base_frame, marker_topic),
     robot_model_(robot_model)
{

}

bool MoveItVisualTools::loadPlanningSceneMonitor()
{
  // Check if we already have one
  if (planning_scene_monitor_)
  {
    ROS_WARN_STREAM_NAMED("visual_tools","Will not load a new planning scene monitor when one has already been set for Visual Tools");
    return false;
  }
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
  ros::Duration(0.1).sleep();
  ros::spinOnce();

  if (planning_scene_monitor_->getPlanningScene())
  {
    // Optional monitors to start:
    //planning_scene_monitor_->startWorldGeometryMonitor();
    //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
    //planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");

    static const std::string PLANNING_SCENE_TOPIC = "/planning_scene";
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, 
                                                          PLANNING_SCENE_TOPIC);
    ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing planning scene on " << PLANNING_SCENE_TOPIC);
  }
  else
  {
    ROS_ERROR_STREAM_NAMED("visual_tools","Planning scene not configured");
    return false;
  }

  return true;
}

bool MoveItVisualTools::processCollisionObjectMsg(const moveit_msgs::CollisionObject& msg)
{
  // Apply command directly to planning scene to avoid a ROS msg call
  {
    planning_scene_monitor::LockedPlanningSceneRW scene(getPlanningSceneMonitor());
    scene->processCollisionObjectMsg(msg);
  }

  // Trigger an update
  getPlanningSceneMonitor()->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

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
      // \todo Deprecated?
      ROS_WARN_STREAM_NAMED("temp","Falling back to using planning scene monitor for loading a robot state");
      planning_scene_monitor::PlanningSceneMonitorPtr psm = getPlanningSceneMonitor();
      robot_model_ = psm->getRobotModel();
    }
    shared_robot_state_.reset(new robot_state::RobotState(robot_model_));
  }

  return shared_robot_state_;
}

bool MoveItVisualTools::loadRobotMarkers()
{
  // Always load the robot state before using
  loadSharedRobotState();

  // Get all link names
  const std::vector<std::string> &link_names = shared_robot_state_->getRobotModel()->getLinkModelNames();;

  ROS_DEBUG_STREAM_NAMED("visual_tools","Number of links in robot: " << link_names.size());
  //    std::copy(link_names.begin(), link_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));

  // Get EE link markers for Rviz
  visualization_msgs::MarkerArray robot_marker_array;
  shared_robot_state_->getRobotMarkers(robot_marker_array, link_names, getColor( rviz_visual_tools::GREY ), "test", ros::Duration());

  ROS_DEBUG_STREAM_NAMED("visual_tools","Number of rviz markers: " << robot_marker_array.markers.size());

  // Publish the markers
  for (std::size_t i = 0 ; i < robot_marker_array.markers.size() ; ++i)
  {
    // Make sure ROS is still spinning
    if( !ros::ok() )
      break;

    // Header
    robot_marker_array.markers[i].header.frame_id = base_frame_;
    robot_marker_array.markers[i].header.stamp = ros::Time::now();

    // Options for meshes
    if( robot_marker_array.markers[i].type == visualization_msgs::Marker::MESH_RESOURCE )
      robot_marker_array.markers[i].mesh_use_embedded_materials = true;

    // Helper for publishing rviz markers
    publishMarker( robot_marker_array.markers[i] );
  }

  return true;
}

robot_state::RobotStatePtr& MoveItVisualTools::getSharedRobotState()
{
  // Always load the robot state before using
  loadSharedRobotState();
  return shared_robot_state_;
}

// \todo handle different types of end effectors better
bool MoveItVisualTools::loadEEMarker(const std::string& ee_group_name, const std::string& planning_group)
{
  // Always load the robot state before using
  loadSharedRobotState();
  shared_robot_state_->setToDefaultValues(); // make sure the ee joint values are reasonable

  // Clear old EE markers
  marker_poses_.clear();
  ee_marker_array_.markers.clear();

  // -----------------------------------------------------------------------------------------------
  // Get end effector group

  // Create color to use for EE markers
  std_msgs::ColorRGBA marker_color = getColor( rviz_visual_tools::GREY );

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

/*void MoveItVisualTools::loadCollisionPub()
{
  if (pub_collision_obj_)
    return;

  // Collision object creator
  pub_collision_obj_ = nh_.advertise<moveit_msgs::CollisionObject>(COLLISION_TOPIC, 10);
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing collision objects on topic " << pub_collision_obj_.getTopic());

  // Wait for topic to be ready
  waitForSubscriber(pub_collision_obj_);
}
*/

void MoveItVisualTools::loadAttachedPub()
{
  if (pub_attach_collision_obj_)
    return;

  // Collision object attacher
  pub_attach_collision_obj_ = nh_.advertise<moveit_msgs::AttachedCollisionObject>(ATTACHED_COLLISION_TOPIC, 10);
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing attached collision objects on topic " << pub_attach_collision_obj_.getTopic());

  // Wait for topic to be ready
  waitForSubscriber(pub_attach_collision_obj_);
}

void MoveItVisualTools::loadPlanningPub()
{
  if (pub_planning_scene_diff_)
    return;

  // Planning scene diff publisher
  pub_planning_scene_diff_ = nh_.advertise<moveit_msgs::PlanningScene>(PLANNING_SCENE_TOPIC, 1);
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing planning scene on topic " << pub_planning_scene_diff_.getTopic());

  // Wait for topic to be ready
  waitForSubscriber(pub_planning_scene_diff_);
}

void MoveItVisualTools::loadTrajectoryPub()
{
  if (pub_display_path_)
    return;

  // Trajectory paths
  pub_display_path_ = nh_.advertise<moveit_msgs::DisplayTrajectory>(DISPLAY_PLANNED_PATH_TOPIC, 10, false);
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing MoveIt trajectory on topic " << pub_display_path_.getTopic());

  // Wait for topic to be ready
  waitForSubscriber(pub_display_path_);  
}

void MoveItVisualTools::loadRobotStatePub(const std::string &marker_topic)
{
  if (pub_robot_state_)
    return;

  // RobotState Message
  pub_robot_state_ = nh_.advertise<moveit_msgs::DisplayRobotState>(marker_topic, 1 );
  ROS_DEBUG_STREAM_NAMED("visual_tools","Publishing MoveIt robot state on topic " << pub_robot_state_.getTopic());

  // Wait for topic to be ready
  waitForSubscriber(pub_robot_state_);
}

planning_scene_monitor::PlanningSceneMonitorPtr MoveItVisualTools::getPlanningSceneMonitor()
{
  if( !planning_scene_monitor_ )
  {
    loadPlanningSceneMonitor();
    ros::spinOnce();
    ros::Duration(1).sleep();
  }
  return planning_scene_monitor_;
}

bool MoveItVisualTools::publishEEMarkers(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color, 
                                         const std::string &ns)
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
    ee_marker_array_.markers[i].header.frame_id = base_frame_;
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

    //ROS_INFO_STREAM("Marker " << i << " ------------- \n" << ee_marker_array_.markers[i]);

    // Helper for publishing rviz markers
    publishMarker( ee_marker_array_.markers[i] );
  }

  return true;
}

bool MoveItVisualTools::publishGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
                                      const std::string &ee_parent_link, double animate_speed)
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

    //ROS_DEBUG_STREAM_NAMED("grasp","Visualizing grasp pose " << i);

    publishEEMarkers(possible_grasps[i].grasp_pose.pose);

    ros::Duration(animate_speed).sleep();
  }

  return true;
}

bool MoveItVisualTools::publishAnimatedGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
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

bool MoveItVisualTools::publishAnimatedGrasp(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link, 
                                             double animate_speed)
{
  if(muted_)
    return true;

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

bool MoveItVisualTools::publishIKSolutions(const std::vector<trajectory_msgs::JointTrajectoryPoint> &ik_solutions,
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
  trajectory_msg.joint_trajectory.header.frame_id = base_frame_;
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

bool MoveItVisualTools::publishRemoveAllCollisionObjects()
{
  // Publish an empty REMOVE message so as to remove all of them

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.is_diff = true;
  planning_scene.world.collision_objects.clear();

  // Clean up old collision objects
  moveit_msgs::CollisionObject remove_object;
  remove_object.header.frame_id = base_frame_;
  remove_object.operation = moveit_msgs::CollisionObject::REMOVE;

  planning_scene.world.collision_objects.push_back(remove_object);

  // Publish
  loadPlanningPub(); // always call this before publishing
  pub_planning_scene_diff_.publish(planning_scene);
  ros::spinOnce();

  return true;
}

bool MoveItVisualTools::removeAllCollisionObjectsPS()
{
  // Clean up old collision objects
  moveit_msgs::CollisionObject remove_object;
  remove_object.header.frame_id = base_frame_;
  remove_object.operation = moveit_msgs::CollisionObject::REMOVE;

  return processCollisionObjectMsg(remove_object);
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

  //aco.object.id = name;
  aco.object.operation = moveit_msgs::CollisionObject::REMOVE;

  //aco.link_name = ee_parent_link_;

  loadAttachedPub(); // always call this before publishing
  pub_attach_collision_obj_.publish(aco);
  ros::spinOnce();

  return true;
}

bool MoveItVisualTools::attachCO(const std::string& name, const std::string& ee_parent_link)
{
  // Clean up old attached collision object
  moveit_msgs::AttachedCollisionObject aco;
  aco.object.header.stamp = ros::Time::now();
  aco.object.header.frame_id = base_frame_;

  aco.object.id = name;
  aco.object.operation = moveit_msgs::CollisionObject::ADD;

  // Link to attach the object to
  aco.link_name = ee_parent_link;

  loadAttachedPub(); // always call this before publishing
  pub_attach_collision_obj_.publish(aco);
  ros::spinOnce();

  return true;
}

bool MoveItVisualTools::publishCollisionBlock(const geometry_msgs::Pose& block_pose, const std::string& block_name, double block_size)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
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

  //ROS_INFO_STREAM_NAMED("visual_tools","CollisionObject: \n " << collision_obj);
  //ROS_DEBUG_STREAM_NAMED("visual_tools","Published collision object " << block_name);
  return processCollisionObjectMsg(collision_obj);
}

bool MoveItVisualTools::publishCollisionRectangle(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2, 
                                                  const std::string& rectangle_name)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = rectangle_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;

  // Calculate center pose
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0].position.x = (point1.x - point2.x) / 2.0 + point2.x;
  collision_obj.primitive_poses[0].position.y = (point1.y - point2.y) / 2.0 + point2.y;
  collision_obj.primitive_poses[0].position.z = (point1.z - point2.z) / 2.0 + point2.z;

  // Calculate scale
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
  collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = fabs(point1.x - point2.x);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = fabs(point1.y - point2.y);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = fabs(point1.z - point2.z);

  // Prevent scale from being zero
  if (!collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X]) 
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = rviz_visual_tools::SMALL_SCALE;
  if (!collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y]) 
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = rviz_visual_tools::SMALL_SCALE;
  if (!collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z]) 
    collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = rviz_visual_tools::SMALL_SCALE;

  //ROS_INFO_STREAM_NAMED("visual_tools","CollisionObject: \n " << collision_obj);
  return processCollisionObjectMsg(collision_obj);
}

bool MoveItVisualTools::publishCollisionFloor(double z, const std::string& plane_name)
{
  /*
  TODO: I don't think collision planes are implemented... could not get this to work
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = plane_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;  

  // Representation of a plane, using the plane equation ax + by + cz + d = 0
  collision_obj.planes.resize(1);
  collision_obj.planes[0].coef[0] = -2; // a
  collision_obj.planes[0].coef[1] = 3; // b
  collision_obj.planes[0].coef[2] = 5; // c
  collision_obj.planes[0].coef[3] = 6; // d

  // Pose
  geometry_msgs::Pose floor_pose;

  // Position
  floor_pose.position.x = 0;
  floor_pose.position.y = 0;
  floor_pose.position.z = z;

  // Orientation
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(0.0, Eigen::Vector3d::UnitZ()));
  floor_pose.orientation.x = quat.x();
  floor_pose.orientation.y = quat.y();
  floor_pose.orientation.z = quat.z();
  floor_pose.orientation.w = quat.w();

  collision_obj.plane_poses.resize(1);
  collision_obj.plane_poses[0] = floor_pose;
  */

  // Instead just generate a rectangle
  geometry_msgs::Point point1;
  geometry_msgs::Point point2;
  
  point1.x = rviz_visual_tools::LARGE_SCALE;
  point1.y = rviz_visual_tools::LARGE_SCALE;
  point1.z = z;

  point2.x = -rviz_visual_tools::LARGE_SCALE;
  point2.y = -rviz_visual_tools::LARGE_SCALE;
  point2.z = z-rviz_visual_tools::SMALL_SCALE;;
  
  return publishCollisionRectangle(point1, point2, plane_name);
}

bool MoveItVisualTools::publishCollisionCylinder(const geometry_msgs::Point &a, const geometry_msgs::Point& b,
                                                 const std::string& object_name, double radius)
{
  return publishCollisionCylinder(convertPoint(a), convertPoint(b), object_name, radius);
}

bool MoveItVisualTools::publishCollisionCylinder(const Eigen::Vector3d &a, const Eigen::Vector3d &b,
                                                 const std::string& object_name, double radius)
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

bool MoveItVisualTools::publishCollisionCylinder(const Eigen::Affine3d& object_pose, const std::string& object_name, double radius, double height)
{
  return publishCollisionCylinder(convertPose(object_pose), object_name, radius, height);
}

bool MoveItVisualTools::publishCollisionCylinder(const geometry_msgs::Pose& object_pose, const std::string& object_name, double radius, double height)
{
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
  collision_obj.id = object_name;
  collision_obj.operation = moveit_msgs::CollisionObject::ADD;
  collision_obj.primitives.resize(1);
  collision_obj.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
  collision_obj.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = height;
  collision_obj.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = radius;
  collision_obj.primitive_poses.resize(1);
  collision_obj.primitive_poses[0] = object_pose;

  //ROS_INFO_STREAM_NAMED("visual_tools","CollisionObject: \n " << collision_obj);
  return processCollisionObjectMsg(collision_obj);
}

bool MoveItVisualTools::publishCollisionGraph(const graph_msgs::GeometryGraph &graph, const std::string &object_name, double radius)
{
  ROS_INFO_STREAM_NAMED("publishCollisionGraph","Preparing to create collision graph");

  // The graph is one collision object with many primitives
  moveit_msgs::CollisionObject collision_obj;
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
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

  return processCollisionObjectMsg(collision_obj);
}

void MoveItVisualTools::getCollisionWallMsg(double x, double y, double angle, double width, const std::string name,
                                      moveit_msgs::CollisionObject &collision_obj)
{
  collision_obj.header.stamp = ros::Time::now();
  collision_obj.header.frame_id = base_frame_;
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
}

bool MoveItVisualTools::publishCollisionWall(double x, double y, double angle, double width, const std::string name)
{
  moveit_msgs::CollisionObject collision_obj;
  getCollisionWallMsg(x, y, angle, width, name, collision_obj);

  return processCollisionObjectMsg(collision_obj);
}

bool MoveItVisualTools::publishCollisionTable(double x, double y, double angle, double width, double height,
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
  collision_obj.header.frame_id = base_frame_;
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

  return processCollisionObjectMsg(collision_obj);
}

bool MoveItVisualTools::publishCollisionTests()
{
  // Create pose
  geometry_msgs::Pose pose1;
  geometry_msgs::Pose pose2;

  // Test all shapes ----------

  ROS_INFO_STREAM_NAMED("test","Publishing Collision Block");
  generateRandomPose(pose1);
  publishCollisionBlock(pose1, "Block", 0.1);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Collision Rectangle");
  generateRandomPose(pose1);
  generateRandomPose(pose2);
  publishCollisionRectangle(pose1.position, pose2.position, "Rectangle");
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Collision Floor");
  publishCollisionFloor(0, "Floor");
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Collision Cylinder");
  generateRandomPose(pose1);
  generateRandomPose(pose2);
  publishCollisionCylinder(pose1.position, pose2.position, "Cylinder", 0.1);
  ros::Duration(1.0).sleep();

  // TODO: test publishCollisionGraph

  ROS_INFO_STREAM_NAMED("test","Publishing Collision Wall");
  generateRandomPose(pose1);
  publishCollisionWall(pose1.position.x, pose1.position.y, 0, 1, "Wall");
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM_NAMED("test","Publishing Collision Table");
  generateRandomPose(pose1);
  publishCollisionTable(pose1.position.x, pose1.position.y, 0, 0.5, 0.5, 0.5, "Table");
  ros::Duration(1.0).sleep();

}

bool MoveItVisualTools::loadCollisionSceneFromFile(const std::string &path, double x_offset, double y_offset)
{
  {
    // Load directly to the planning scene
    planning_scene_monitor::LockedPlanningSceneRW scene(getPlanningSceneMonitor());
    if (scene)
    {

      std::ifstream fin(path.c_str());
      if (fin.good())
      {
        //scene->loadGeometryFromStream(fin, x_offset, y_offset);
        scene->loadGeometryFromStream(fin);
        fin.close();
        ROS_INFO("Loaded scene geometry from '%s'", path.c_str());
      }
      else
        ROS_WARN("Unable to load scene geometry from '%s'", path.c_str());
    }
    else
      ROS_WARN_STREAM_NAMED("temp","Unable to get locked planning scene RW");
  }
  getPlanningSceneMonitor()->triggerSceneUpdateEvent(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
}

bool MoveItVisualTools::publishTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& trajectory_pt,
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
  trajectory_msg.joint_trajectory.header.frame_id = base_frame_;
  trajectory_msg.joint_trajectory.joint_names = joint_model_group->getJointModelNames();
  trajectory_msg.joint_trajectory.points.push_back(trajectory_pt);
  trajectory_msg.joint_trajectory.points.push_back(trajectory_pt_timed);

  return publishTrajectoryPath(trajectory_msg, true);
}

bool MoveItVisualTools::publishTrajectoryPath(const robot_trajectory::RobotTrajectory& trajectory, bool blocking)
{
  moveit_msgs::RobotTrajectory trajectory_msg;
  trajectory.getRobotTrajectoryMsg(trajectory_msg);

  // Add time from start if none specified
  if (trajectory_msg.joint_trajectory.points.size() > 1)
  {
    if (trajectory_msg.joint_trajectory.points[1].time_from_start == ros::Duration(0)) // assume no timestamps exist
    {
      for (std::size_t i = 0; i < trajectory_msg.joint_trajectory.points.size(); ++i)
      {
        trajectory_msg.joint_trajectory.points[i].time_from_start = ros::Duration(i*2); // 1 hz
      }
    }
  }

  //std::cout << "trajectory_msg:\n " << trajectory_msg << std::endl;

  publishTrajectoryPath(trajectory_msg, blocking);
}

bool MoveItVisualTools::publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg, bool blocking)
{
  loadSharedRobotState();

  // Check if we have enough points
  if (!trajectory_msg.joint_trajectory.points.size())
  {
    ROS_WARN_STREAM_NAMED("temp","Unable to publish trajectory path because trajectory has zero points");
    return false;
  }

  // Create the message  TODO move to member function to load less often
  moveit_msgs::DisplayTrajectory display_trajectory_msg;
  display_trajectory_msg.model_id = robot_model_->getName();

  //    display_trajectory_msg.trajectory_start = start_state;
  display_trajectory_msg.trajectory.resize(1);
  display_trajectory_msg.trajectory[0] = trajectory_msg;

  // Publish message
  loadTrajectoryPub(); // always call this before publishing
  //std::cout << "visual_tools: " << display_trajectory_msg << std::endl;
  pub_display_path_.publish(display_trajectory_msg);
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

bool MoveItVisualTools::publishRobotState(const robot_state::RobotStatePtr &robot_state,
                                          const rviz_visual_tools::colors &color)
{
  publishRobotState(*robot_state.get(), color);
}

bool MoveItVisualTools::publishRobotState(const robot_state::RobotState &robot_state,
                                          const rviz_visual_tools::colors &color)
{
  // Reference to the correctly colored version of message (they are cached)
  // May not exist yet but this will create it
  moveit_msgs::DisplayRobotState& display_robot_msg = display_robot_msgs_[color]; 

  // Check if a robot state message already exists for this color
  if (display_robot_msg.highlight_links.size() == 0) // has not been colored yet, lets create that
  {
    // Get links names
    const std::vector<const moveit::core::LinkModel*>& link_names = robot_state.getRobotModel()->getLinkModelsWithCollisionGeometry();
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

  // Convert state to message
  robot_state::robotStateToRobotStateMsg(robot_state, display_robot_msg.state);

  // Publish
  loadRobotStatePub();
  pub_robot_state_.publish( display_robot_msg );
  ros::spinOnce();

  return true;
}

bool MoveItVisualTools::publishRobotState(const trajectory_msgs::JointTrajectoryPoint& trajectory_pt,
                                    const std::string &group_name)
{
  // Always load the robot state before using
  loadSharedRobotState();

  // Set robot state
  shared_robot_state_->setToDefaultValues(); // reset the state just in case
  shared_robot_state_->setJointGroupPositions(group_name, trajectory_pt.positions);

  // Publish robot state
  publishRobotState(*shared_robot_state_);

  return true;
}

bool MoveItVisualTools::hideRobot()
{
  // Always load the robot state before using
  loadSharedRobotState();

  shared_robot_state_->setVariablePosition("virtual_joint/trans_x", rviz_visual_tools::LARGE_SCALE);
  shared_robot_state_->setVariablePosition("virtual_joint/trans_y", rviz_visual_tools::LARGE_SCALE);
  shared_robot_state_->setVariablePosition("virtual_joint/trans_z", rviz_visual_tools::LARGE_SCALE);
      
  publishRobotState(shared_robot_state_);
}



} // namespace
