/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, University of Colorado, Boulder
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
 *   * Neither the name of Univ of Colorado nor the names of its
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

/* Author: Dave Coleman
   Desc:   Class to encapsule a visualized robot state that can be controlled using an interactive marker
*/

#ifndef MOVEIT_VISUAL_TOOLS_IMARKER_END_EFFECTOR_H
#define MOVEIT_VISUAL_TOOLS_IMARKER_END_EFFECTOR_H

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/menu_handler.h>

// MoveIt
#include <moveit/robot_state/robot_state.h>

// this package
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_visual_tools/imarker_robot_state.h>

// C++
#include <string>
#include <vector>

namespace moveit_visual_tools
{
using visualization_msgs::InteractiveMarkerFeedback;
using visualization_msgs::InteractiveMarkerControl;

typedef std::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &, const Eigen::Affine3d &)>
    IMarkerCallback;

class IMarkerRobotState;

class IMarkerEndEffector
{
public:
  /**
   * \brief Constructor
   */
  IMarkerEndEffector(IMarkerRobotState *imarker_parent, const std::string &imarker_name, ArmData arm_data,
                     rviz_visual_tools::colors color);

  ~IMarkerEndEffector()
  {
  }

  /** \brief Get the current end effector pose */
  void getPose(Eigen::Affine3d &pose);

  bool setPoseFromRobotState();

  void iMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void solveIK(Eigen::Affine3d &pose);

  void initializeInteractiveMarkers();

  void updateIMarkerPose(const Eigen::Affine3d &pose);

  void sendUpdatedIMarkerPose();

  void make6DofMarker(const geometry_msgs::Pose &pose);

  visualization_msgs::InteractiveMarkerControl &makeBoxControl(visualization_msgs::InteractiveMarker &msg);

  void setCollisionCheckingVerbose(bool collision_checking_verbose)
  {
    collision_checking_verbose_ = collision_checking_verbose;
  }

  void setOnlyCheckSelfCollision(bool only_check_self_collision)
  {
    only_check_self_collision_ = only_check_self_collision;
  }

  void setUseCollisionChecking(bool use_collision_checking)
  {
    use_collision_checking_ = use_collision_checking;
  }

  void setIMarkerCallback(IMarkerCallback callback)
  {
    imarker_callback_ = callback;
  }

  const moveit::core::LinkModel *getEELink()
  {
    return arm_data_.ee_link_;
  }

private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_;

  // Pointer to parent
  IMarkerRobotState *imarker_parent_;

  // State
  moveit::core::RobotStatePtr imarker_state_;
  Eigen::Affine3d imarker_pose_;

  // Core MoveIt components
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  // Visual tools
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Settings
  ArmData arm_data_;
  rviz_visual_tools::colors color_ = rviz_visual_tools::PURPLE;

  // File saving
  ros::Time time_since_last_save_;

  // Interactive markers
  // interactive_markers::MenuHandler menu_handler_;
  visualization_msgs::InteractiveMarker int_marker_;
  bool imarker_ready_to_process_ = true;
  boost::mutex imarker_mutex_;

  InteractiveMarkerServerPtr imarker_server_;

  // Verbose settings
  bool collision_checking_verbose_ = false;
  bool only_check_self_collision_ = false;
  bool use_collision_checking_ = false;

  // Hook to parent class
  IMarkerCallback imarker_callback_;
};  // end class

// Create std pointers for this class
typedef std::shared_ptr<IMarkerEndEffector> IMarkerEndEffectorPtr;
typedef std::shared_ptr<const IMarkerEndEffector> IMarkerEndEffectorConstPtr;
}  // namespace moveit_visual_tools

namespace
{
/** \brief Collision checking handle for IK solvers */
bool isStateValid(const planning_scene::PlanningScene *planning_scene, bool verbose, bool only_check_self_collision,
                  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_, robot_state::RobotState *state,
                  const robot_state::JointModelGroup *group, const double *ik_solution);
}

#endif  // MOVEIT_VISUAL_TOOLS_IMARKER_END_EFFECTOR_H
