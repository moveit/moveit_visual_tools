// Copyright 2016 University of Colorado, Boulder
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

/* Author: Dave Coleman
   Desc:   Class to encapsule a visualized robot state that can be controlled using an interactive marker
*/

// Boost
#include <boost/filesystem.hpp>

// C++
#include <string>

// Conversions
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

// this package
#include <moveit_visual_tools/imarker_robot_state.h>
#include <moveit_visual_tools/imarker_end_effector.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("imarker_end_effector");
static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
namespace
{
bool isStateValid(const planning_scene::PlanningScene* planning_scene, bool verbose, bool only_check_self_collision,
                  const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools, moveit::core::RobotState* robot_state,
                  const moveit::core::JointModelGroup* group, const double* ik_solution)
{
  // Apply IK solution to robot state
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();

#if 0  // Ensure there are objects in the planning scene
  const std::size_t num_collision_objects = planning_scene->getCollisionEnv()->getWorld()->size();
  if (num_collision_objects == 0)
  {
    RCLCPP_ERROR(LOGGER, "No collision objects exist in world, you need at least a table "
                           "modeled for the controller to work");
    RCLCPP_ERROR(LOGGER, "To fix this, relaunch the teleop/head tracking/whatever MoveIt "
                           "node to publish the collision objects");
    return false;
  }
#endif

  if (!planning_scene)
  {
    RCLCPP_ERROR(LOGGER, "No planning scene provided");
    return false;
  }
  if (only_check_self_collision)
  {
    // No easy API exists for only checking self-collision, so we do it here.
    // TODO(davetcoleman): move this into planning_scene.cpp
    collision_detection::CollisionRequest req;
    req.verbose = verbose;
    req.group_name = group->getName();
    collision_detection::CollisionResult res;
    planning_scene->checkSelfCollision(req, res, *robot_state);
    if (!res.collision)
      return true;  // not in collision
  }
  else if (!planning_scene->isStateColliding(*robot_state, group->getName()))
    return true;  // not in collision

  // Display more info about the collision
  if (verbose)
  {
    visual_tools->publishRobotState(*robot_state, rviz_visual_tools::RED);
    planning_scene->isStateColliding(*robot_state, group->getName(), true);
    visual_tools->publishContactPoints(*robot_state, planning_scene);
  }
  RCLCPP_WARN_THROTTLE(LOGGER, steady_clock, 2000, "Collision");
  return false;
}

}  // namespace

namespace moveit_visual_tools
{
IMarkerEndEffector::IMarkerEndEffector(IMarkerRobotState* imarker_parent, const std::string& imarker_name,
                                       ArmData arm_data, rviz_visual_tools::Colors color)
  : name_(imarker_name)
  , imarker_parent_(imarker_parent)
  , imarker_state_(imarker_parent_->imarker_state_)
  , psm_(imarker_parent_->psm_)
  , visual_tools_(imarker_parent_->visual_tools_)
  , arm_data_(arm_data)
  , color_(color)
  , imarker_server_(imarker_parent_->imarker_server_)
  , clock_(RCL_ROS_TIME)
{
  // Get pose from robot state
  imarker_pose_ = imarker_state_->getGlobalLinkTransform(arm_data_.ee_link_);

  // Create imarker
  initializeInteractiveMarkers();

  RCLCPP_INFO_STREAM(LOGGER, "IMarkerEndEffector " << name_ << " tracking ee link '" << arm_data_.ee_link_->getName()
                                                   << "' ready.");
}

void IMarkerEndEffector::getPose(Eigen::Isometry3d& pose)
{
  pose = imarker_pose_;
}

bool IMarkerEndEffector::setPoseFromRobotState()
{
  imarker_pose_ = imarker_state_->getGlobalLinkTransform(arm_data_.ee_link_);

  sendUpdatedIMarkerPose();

  return true;
}

void IMarkerEndEffector::iMarkerCallback(
    const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback)
{
  if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP)
  {
    // Save pose to file if its been long enough
    double save_every_sec = 0.1;
    if (time_since_last_save_ < clock_.now() - rclcpp::Duration::from_seconds(save_every_sec))
    {
      imarker_parent_->saveToFile();
      time_since_last_save_ = clock_.now();
    }
    return;
  }

  // Ignore if not pose update
  if (feedback->event_type != visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE)
    return;

  // Only allow one feedback to be processed at a time
  {
    // boost::unique_lock<boost::mutex> scoped_lock(imarker_mutex_);
    if (!imarker_ready_to_process_)
    {
      return;
    }
    imarker_ready_to_process_ = false;
  }

  // Convert
  Eigen::Isometry3d robot_ee_pose;
  tf2::fromMsg(feedback->pose, robot_ee_pose);

  // Update robot
  solveIK(robot_ee_pose);

  // Redirect to base class
  if (imarker_callback_)
    imarker_callback_(feedback, robot_ee_pose);

  // Allow next feedback to be processed
  {
    // boost::unique_lock<boost::mutex> scoped_lock(imarker_mutex_);
    imarker_ready_to_process_ = true;
  }
}

void IMarkerEndEffector::solveIK(Eigen::Isometry3d& pose)
{
  // Cartesian settings
  const double timeout = 1.0 / 30.0;  // 30 fps

  // Optionally collision check
  moveit::core::GroupStateValidityCallbackFn constraint_fn;
  if (use_collision_checking_)
  {
    // TODO(davetcoleman): this is currently not working, the locking seems to cause segfaults
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(psm_));
    constraint_fn = boost::bind(&isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(),
                                collision_checking_verbose_, only_check_self_collision_, visual_tools_,
                                boost::placeholders::_1, boost::placeholders::_2, boost::placeholders::_3);
  }

  // Attempt to set robot to new pose
  if (imarker_state_->setFromIK(arm_data_.jmg_, pose, arm_data_.ee_link_->getName(), timeout, constraint_fn))
  {
    imarker_state_->update();
    // if (psm_->getPlanningScene()->isStateValid(*imarker_state_))
    //{
    // ROS_INFO_STREAM_NAMED(name_, "Solved IK");
    imarker_parent_->publishRobotState();
    //}
    // else
    // {
    //   visual_tools_->publishRobotState(imarker_state_, rviz_visual_tools::RED);
    //   exit(0);
    // }
  }
}

void IMarkerEndEffector::initializeInteractiveMarkers()
{
  // Convert
  const geometry_msgs::msg::Pose pose_msg = tf2::toMsg(imarker_pose_);

  // marker
  make6DofMarker(pose_msg);
}

void IMarkerEndEffector::updateIMarkerPose(const Eigen::Isometry3d& /*pose*/)
{
  // Move marker to tip of fingers
  // imarker_pose_ = pose * imarker_offset_.inverse();
  sendUpdatedIMarkerPose();
}

void IMarkerEndEffector::sendUpdatedIMarkerPose()
{
  // Convert
  const geometry_msgs::msg::Pose pose_msg = tf2::toMsg(imarker_pose_);

  imarker_server_->setPose(int_marker_.name, pose_msg);
  imarker_server_->applyChanges();
}

void IMarkerEndEffector::make6DofMarker(const geometry_msgs::msg::Pose& pose)
{
  RCLCPP_DEBUG_STREAM(LOGGER, "Making 6dof interactive marker named " << name_);

  int_marker_.header.frame_id = "world";
  int_marker_.pose = pose;
  int_marker_.scale = 0.2;

  int_marker_.name = name_;
  // int_marker_.description = "imarker_" + name_; // TODO: unsure, but I think this causes a caption in Rviz that I
  // don't want

  // insert a box
  // makeBoxControl(int_marker_);

  // int_marker_.controls[0].interaction_mode = InteractiveMarkerControl::MENU;

  InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker_.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker_.controls.push_back(control);

  imarker_server_->insert(int_marker_, std::bind(&IMarkerEndEffector::iMarkerCallback, this, std::placeholders::_1));

  // menu_handler_.apply(*imarker_server_, int_marker_.name);
}

visualization_msgs::msg::InteractiveMarkerControl&
IMarkerEndEffector::makeBoxControl(visualization_msgs::msg::InteractiveMarker& msg)
{
  visualization_msgs::msg::InteractiveMarkerControl control;
  control.always_visible = true;

  visualization_msgs::msg::Marker marker;
  marker.type = visualization_msgs::msg::Marker::CUBE;
  marker.scale.x = msg.scale * 0.3;   // x direction
  marker.scale.y = msg.scale * 0.10;  // y direction
  marker.scale.z = msg.scale * 0.10;  // height
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  control.markers.push_back(marker);
  msg.controls.push_back(control);

  return msg.controls.back();
}

}  // namespace moveit_visual_tools
