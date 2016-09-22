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

#ifndef MOVEIT_VISUAL_TOOLS_IMARKER_ROBOT_STATE_H
#define MOVEIT_VISUAL_TOOLS_IMARKER_ROBOT_STATE_H

// ROS
#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/menu_handler.h>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// C++
#include <string>
#include <vector>

namespace moveit_visual_tools
{
using visualization_msgs::InteractiveMarkerFeedback;
using visualization_msgs::InteractiveMarkerControl;

typedef std::function<void(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &, const Eigen::Affine3d &)>
    IMarkerCallback;

class IMarkerRobotState
{
public:
  /**
   * \brief Constructor
   */
  IMarkerRobotState(planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                    const std::string &imarker_name, const moveit::core::JointModelGroup *jmg,
                    moveit::core::LinkModel *ee_link, rviz_visual_tools::colors color, const std::string &package_path);

  ~IMarkerRobotState()
  {
    output_file_.close();
  }

  /** \brief Set where in the parent class the feedback should be sent */
  void setIMarkerCallback(IMarkerCallback callback);

  /** \brief Get the current end effector pose */
  void getPose(Eigen::Affine3d &pose);

  /** \brief Get a pointer to the current robot state */
  moveit::core::RobotStatePtr getRobotState();

  bool loadFromFile(const std::string &file_name);

  bool saveToFile();

  bool setPoseFromRobotState();

  void iMarkerCallback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void solveIK(Eigen::Affine3d &pose);

  void initializeInteractiveMarkers(const Eigen::Affine3d &pose);

  void updateIMarkerPose(const Eigen::Affine3d &pose);

  void sendUpdatedIMarkerPose();

  void make6DofMarker(const geometry_msgs::Pose &pose);

  visualization_msgs::InteractiveMarkerControl &makeBoxControl(visualization_msgs::InteractiveMarker &msg);

  bool getFilePath(std::string &file_path, const std::string &file_name, const std::string &subdirectory) const;

  bool setToRandomState();

  moveit_visual_tools::MoveItVisualToolsPtr getVisualTools();

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



private:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_;

  // A shared node handle
  ros::NodeHandle nh_;

  // State
  moveit::core::RobotStatePtr imarker_state_;
  Eigen::Affine3d imarker_pose_;

  // Trajetory
  std::vector<moveit::core::RobotStatePtr> trajectory_;

  // Core MoveIt components
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Visual tools
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Settings
  std::size_t refresh_rate_ = 30;
  const moveit::core::JointModelGroup *jmg_;
  moveit::core::LinkModel *ee_link_;
  rviz_visual_tools::colors color_ = rviz_visual_tools::PURPLE;

  // File location of this package
  std::string package_path_;

  // File saving
  std::string file_path_;
  ros::Time time_since_last_save_;

  // Interactive markers
  std::string imarker_topic_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imarker_server_;
  // interactive_markers::MenuHandler menu_handler_;
  visualization_msgs::InteractiveMarker int_marker_;
  bool imarker_ready_to_process_ = true;
  boost::mutex imarker_mutex_;

  // hook to parent class
  IMarkerCallback imarker_callback_;

  // Amount to move interactive marker from tip link of kinematic chain
  //Eigen::Affine3d imarker_offset_ = Eigen::Affine3d::Identity();

  double total_duration_ = 0;
  std::size_t total_saves_ = 0;

  std::ofstream output_file_;

  // Verbose settings
  bool collision_checking_verbose_ = false;
  bool only_check_self_collision_ = false;
  bool use_collision_checking_ = false;

};  // end class

// Create boost pointers for this class
typedef boost::shared_ptr<IMarkerRobotState> IMarkerRobotStatePtr;
typedef boost::shared_ptr<const IMarkerRobotState> IMarkerRobotStateConstPtr;
}  // namespace moveit_visual_tools

namespace
{
/** \brief Collision checking handle for IK solvers */
bool isStateValid(const planning_scene::PlanningScene *planning_scene, bool verbose, bool only_check_self_collision,
                  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_, robot_state::RobotState *state,
                  const robot_state::JointModelGroup *group, const double *ik_solution);
}

#endif  // MOVEIT_VISUAL_TOOLS_IMARKER_ROBOT_STATE_H
