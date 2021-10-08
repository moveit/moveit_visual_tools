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

#ifndef MOVEIT_VISUAL_TOOLS_IMARKER_ROBOT_STATE_H
#define MOVEIT_VISUAL_TOOLS_IMARKER_ROBOT_STATE_H

// ROS
#include <rclcpp/rclcpp.hpp>
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// C++
#include <string>
#include <vector>

namespace moveit_visual_tools
{
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::InteractiveMarkerFeedback;

typedef std::function<void(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr&,
                           const Eigen::Isometry3d&)>
    IMarkerCallback;

typedef std::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;

class IMarkerEndEffector;
typedef std::shared_ptr<IMarkerEndEffector> IMarkerEndEffectorPtr;
typedef std::shared_ptr<const IMarkerEndEffector> IMarkerEndEffectorConstPtr;

/** \brief I wish this struct wasn't necessary, but the SRDF does not seem to support choosing one particular link
 *         as an end effector tip - instead it does so automatically.
 */
struct ArmData
{
  ArmData(moveit::core::JointModelGroup* jmg, moveit::core::LinkModel* ee_link) : jmg_(jmg), ee_link_(ee_link){};

  moveit::core::JointModelGroup* jmg_;
  moveit::core::LinkModel* ee_link_;
};

class IMarkerRobotState
{
  friend class IMarkerEndEffector;

public:
  /**
   * \brief Constructor
   */
  IMarkerRobotState(rclcpp::Node::SharedPtr node, planning_scene_monitor::PlanningSceneMonitorPtr psm,
                    const std::string& imarker_name, std::vector<ArmData> arm_datas, rviz_visual_tools::Colors color,
                    const std::string& package_path);

  ~IMarkerRobotState()
  {
    output_file_.close();
  }

  bool loadFromFile(const std::string& file_name);

  bool saveToFile();

  /** \brief Set where in the parent class the feedback should be sent */
  void setIMarkerCallback(const IMarkerCallback& callback);

  /** \brief Get a pointer to the current robot state */
  moveit::core::RobotStateConstPtr getRobotState()
  {
    return imarker_state_;
  }
  moveit::core::RobotStatePtr getRobotStateNonConst()
  {
    return imarker_state_;
  }

  /** \brief Set the robot state */
  void setRobotState(const moveit::core::RobotStatePtr& state);

  /** \brief Set the robot state to current in planning scene monitor */
  void setToCurrentState();

  /**
   * \brief Set the robot to a random position
   * \param clearance - optional value to ensure random state is not too close to obstacles. 0 is disable
   * \return true on success
   */
  bool setToRandomState(double clearance = 0);

  /** \brief Return true if the currently solved IK solution is valid */
  bool isStateValid(bool verbose = false);

  /** \brief Show current state in Rviz */
  void publishRobotState();

  moveit_visual_tools::MoveItVisualToolsPtr getVisualTools();

  bool getFilePath(std::string& file_path, const std::string& file_name, const std::string& subdirectory) const;

  IMarkerEndEffectorPtr getEEF(const std::string& name)
  {
    return name_to_eef_[name];
  }

  bool setFromPoses(const EigenSTL::vector_Isometry3d& poses, const moveit::core::JointModelGroup* group);

protected:
  // --------------------------------------------------------

  // The short name of this class
  std::string name_;

  // State
  moveit::core::RobotStatePtr imarker_state_;

  // End effectors
  std::vector<ArmData> arm_datas_;
  std::vector<IMarkerEndEffectorPtr> end_effectors_;
  std::map<std::string, IMarkerEndEffectorPtr> name_to_eef_;

  // Core MoveIt components
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  // Visual tools
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Settings
  std::size_t refresh_rate_ = 30;
  rviz_visual_tools::Colors color_ = rviz_visual_tools::PURPLE;

  // Interactive markers
  InteractiveMarkerServerPtr imarker_server_;

  // File saving
  std::string file_path_;
  std::ofstream output_file_;
  std::string package_path_;  // File location of this package

};  // end class

// Create std pointers for this class
typedef std::shared_ptr<IMarkerRobotState> IMarkerRobotStatePtr;
typedef std::shared_ptr<const IMarkerRobotState> IMarkerRobotStateConstPtr;
}  // namespace moveit_visual_tools

#endif  // MOVEIT_VISUAL_TOOLS_IMARKER_ROBOT_STATE_H
