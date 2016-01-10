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

/* \author  Dave Coleman
 * \desc    Helper functions for displaying and debugging MoveIt! data in Rviz via published markers
 *          and MoveIt! collision objects. Very useful for debugging complex software
 *
 *          See README.md for developers notes.
 */

#ifndef MOVEIT_VISUAL_TOOLS__MOVEIT_VISUAL_TOOLS_H_
#define MOVEIT_VISUAL_TOOLS__MOVEIT_VISUAL_TOOLS_H_

// Rviz Visualization Tool
#include <rviz_visual_tools/rviz_visual_tools.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
//#include <moveit/macros/deprecation.h>
#include <rviz_visual_tools/deprecation.h>

// MoveIt Messages
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/WorkspaceParameters.h>

// ROS Messages
#include <trajectory_msgs/JointTrajectory.h>

namespace moveit_visual_tools
{
// Default constants
static const std::string ROBOT_DESCRIPTION = "robot_description";  // this is the default used in ROS
static const std::string DISPLAY_PLANNED_PATH_TOPIC =
    "/move_group/display_planned_path";  // this is the default when adding the Rviz plugin
static const std::string DISPLAY_ROBOT_STATE_TOPIC =
    "display_robot_state";                                         // this is the default when adding the Rviz plugin
static const std::string PLANNING_SCENE_TOPIC = "planning_scene";  // this is the default when adding the Rviz plugin

class MoveItVisualTools : public rviz_visual_tools::RvizVisualTools
{
public:
  /**
   * \brief Constructor
   * \param base_frame - common base for all visualization markers, usually "/world" or "/odom"
   * \param marker_topic - rostopic to publish markers to - your Rviz display should match
   * \param planning_scene_monitor - optionally pass in a pre-loaded planning scene monitor to
   *        avoid having to re-load the URDF, kinematic solvers, etc
   */
  MoveItVisualTools(const std::string &base_frame, const std::string &marker_topic,
                    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);

  /**
   * \brief Constructor
   * \param base_frame - common base for all visualization markers, usually "/world" or "/odom"
   * \param marker_topic - rostopic to publish markers to - your Rviz display should match
   * \param robot_model - load robot model pointer so that we don't have do re-parse it here
   */
  MoveItVisualTools(const std::string &base_frame,
                    const std::string &marker_topic = rviz_visual_tools::RVIZ_MARKER_TOPIC,
                    robot_model::RobotModelConstPtr robot_model = robot_model::RobotModelConstPtr());

  /**
   * \brief Deconstructor
   */
  ~MoveItVisualTools(){};

  /**
   * \brief Set the ROS topic for publishing a robot state
   * \param topic
   */
  void setRobotStateTopic(const std::string &robot_state_topic)
  {
    robot_state_topic_ = robot_state_topic;
  }

  /**
   * \brief Set the planning scene topic
   * \param topic
   */
  void setPlanningSceneTopic(const std::string &planning_scene_topic)
  {
    planning_scene_topic_ = planning_scene_topic;
  }

  /**
   * \brief Load a planning scene monitor if one was not passed into the constructor
   * \return true if successful in loading
   */
  bool loadPlanningSceneMonitor();

  /**
   * \brief Skip a ROS message call by sending directly to planning scene monitor
   * \param collision object message
   * \param color to display the collision object with
   * \return true on success
   */
  bool processCollisionObjectMsg(const moveit_msgs::CollisionObject &msg,
                                 const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);

  /**
   * \brief Skip a ROS message call by sending directly to planning scene monitor
   * \param attached collision object message
   * \return true on success
   */
  bool processAttachedCollisionObjectMsg(const moveit_msgs::AttachedCollisionObject &msg);

  /**
   * \brief When mannual_trigger_update_ is true, use this to tell the planning scene to send
   *        an update out. Do not use otherwise
   */
  bool triggerPlanningSceneUpdate();

  /**
   * \brief Load robot state only as needed
   * \return true if successful in loading
   */
  bool loadSharedRobotState();

  /**
   * \brief Allow robot state to be altered.
   * \return shared pointer to robot state
   */
  moveit::core::RobotStatePtr &getSharedRobotState();

  /**
   * \brief Get a pointer to the robot model
   * \return const RobotModel
   */
  moveit::core::RobotModelConstPtr getRobotModel();

  /**
   * \brief Call this once at begining to load the robot marker
   * \param ee_group_name - name of planning_group for the end effector
   * \return true if it is successful
   */
  RVIZ_VISUAL_TOOLS_DEPRECATED
  bool loadEEMarker(const std::string &ee_group_name)
  {
    return loadEEMarker(robot_model_->getJointModelGroup(ee_group_name));
  }
  /**
   * \brief Call this once at begining to load the robot marker
   * \param ee_jmg - the set of joints to use, e.g. the MoveIt! planning group, e.g. "left_arm"
   * \return true if it is successful
   */
  bool loadEEMarker(const robot_model::JointModelGroup *ee_jmg);

  /**
   * \brief Load publishers as needed
   */
  void loadTrajectoryPub(const std::string &display_planned_path_topic = DISPLAY_PLANNED_PATH_TOPIC);
  void loadRobotStatePub(const std::string &robot_state_topic = "");

  /**
   * \brief Allow a pre-configured planning scene monitor to be set for publishing collision objects, etc
   * \param a pointer to a load planning scen
   */
  void setPlanningSceneMonitor(planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
  {
    planning_scene_monitor_ = planning_scene_monitor;
  }

  /**
   * \brief Prevent the planning scene from always auto-pushing, but rather do it manually
   * \param bool true to enable manual mode
   */
  void setManualSceneUpdating(bool enable_manual)
  {
    mannual_trigger_update_ = enable_manual;
  }

  /**
   * \brief Publish an end effector to rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \param ee_jmg - joint model group of the end effector, e.g. "left_hand"
   * \param color to display the collision object with
   * \return true on success
   */
  bool publishEEMarkers(const Eigen::Affine3d &pose, const robot_model::JointModelGroup *ee_jmg,
                        const rviz_visual_tools::colors &color = rviz_visual_tools::CLEAR,
                        const std::string &ns = "end_effector")
  {
    return publishEEMarkers(convertPose(pose), ee_jmg, color, ns);
  }
  bool publishEEMarkers(const geometry_msgs::Pose &pose, const robot_model::JointModelGroup *ee_jmg,
                        const rviz_visual_tools::colors &color = rviz_visual_tools::CLEAR,
                        const std::string &ns = "end_effector");

  /**
   * \brief Show grasps generated from moveit_simple_grasps or other MoveIt Grasp message sources
   * \param possible_grasps - a set of grasp positions to visualize
   * \param ee_jmg - the set of joints to use, e.g. the MoveIt! planning group, e.g. "left_arm"
   * \param animate_speed - how fast the gripper approach is animated, optional
   */
  bool publishGrasps(const std::vector<moveit_msgs::Grasp> &possible_grasps, const robot_model::JointModelGroup *ee_jmg,
                     double animate_speed = 0.1);

  /**
   * \brief Display an animated vector of grasps including its approach movement in Rviz
   * \param possible_grasps - a set of grasp positions to visualize
   * \param ee_jmg - the set of joints to use, e.g. the MoveIt! planning group, e.g. "left_arm"
   * \param animate_speed - how fast the gripper approach is animated, optional
   */
  bool publishAnimatedGrasps(const std::vector<moveit_msgs::Grasp> &possible_grasps,
                             const robot_model::JointModelGroup *ee_jmg, double animate_speed = 0.01);

  /**
   * \brief Animate a single grasp in its movement direction
   * \param grasp
   * \param ee_jmg - the set of joints to use, e.g. the MoveIt! planning group, e.g. "left_arm"
   * \param animate_speed - how fast the gripper approach is animated
   * \return true on sucess
   */
  bool publishAnimatedGrasp(const moveit_msgs::Grasp &grasp, const robot_model::JointModelGroup *ee_jmg,
                            double animate_speed);

  /**
   * \brief Display an vector of inverse kinematic solutions for the IK service in Rviz
   * Note: this is published to the 'Planned Path' section of the 'MotionPlanning' display in Rviz
   * \param ik_solutions - a set of corresponding arm positions to achieve each grasp
   * \param arm_jmg - the set of joints to use, e.g. the MoveIt! planning group, e.g. "left_arm"
   * \param display_time - amount of time to sleep between sending trajectories, optional
   */
  bool publishIKSolutions(const std::vector<trajectory_msgs::JointTrajectoryPoint> &ik_solutions,
                          const robot_model::JointModelGroup *arm_jmg, double display_time = 0.4);

  /**
   * \brief Remove all collision objects that this class has added to the MoveIt! planning scene
   *        Communicates directly to a planning scene monitor e.g. if this is the move_group node
   * \param  the scene to directly clear the collision objects from
   * \return true on sucess
   */
  bool removeAllCollisionObjects();

  /**
   * \brief Remove a collision object from the planning scene
   * \param Name of object
   * \return true on sucess
   */
  bool cleanupCO(const std::string &name);

  /**
   * \brief Remove an active collision object from the planning scene
   * \param Name of object
   * \return true on sucess
   */
  bool cleanupACO(const std::string &name);

  /**
   * \brief Attach a collision object from the planning scene
   * \param Name of object
   * \param
   * \return true on sucess
   */
  bool attachCO(const std::string &name, const std::string &ee_parent_link);

  /**
   * \brief Make the floor a collision object
   * \param z location of floor
   * \param name of floor
   * \param color to display the collision object with
   * \return true on success
   */
  bool publishCollisionFloor(double z = 0.0, const std::string &plane_name = "Floor",
                             const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);

  /**
   * \brief Create a MoveIt Collision block at the given pose
   * \param pose - location of center of block
   * \param name - semantic name of MoveIt collision object
   * \param size - height=width=depth=size
   * \param color to display the collision object with
   * \return true on sucess
   **/
  bool publishCollisionBlock(const geometry_msgs::Pose &block_pose, const std::string &block_name, double block_size,
                             const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);

  /**
   * \brief Create a MoveIt collision rectangular cuboid at the given pose
   * \param point1 - top left of rectangle
   * \param point2 - bottom right of rectangle
   * \param name - semantic name of MoveIt collision object
   * \param color to display the collision object with
   * \return true on sucess
   **/
  RVIZ_VISUAL_TOOLS_DEPRECATED
  bool publishCollisionRectangle(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2, const std::string &name,
                                 const rviz_visual_tools::colors &color)
  {
    return publishCollisionCuboid(convertPoint(point1), convertPoint(point2), name, color);
  }
  RVIZ_VISUAL_TOOLS_DEPRECATED
  bool publishCollisionRectangle(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                                 const std::string &name, const rviz_visual_tools::colors &color)
  {
    return publishCollisionCuboid(convertPoint(point1), convertPoint(point2), name, color);
  }
  bool publishCollisionCuboid(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2, const std::string &name,
                              const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);
  bool publishCollisionCuboid(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                              const std::string &name,
                              const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);

  /**
   * \brief Create a MoveIt Collision cylinder between two points
   * \param point a - x,y,z in space of a point
   * \param point b - x,y,z in space of a point
   * \param name - semantic name of MoveIt collision object
   * \param radius - size of cylinder
   * \param color to display the collision object with
   * \return true on sucess
   */
  bool publishCollisionCylinder(const geometry_msgs::Point &a, const geometry_msgs::Point &b,
                                const std::string &object_name, double radius,
                                const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);
  bool publishCollisionCylinder(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const std::string &object_name,
                                double radius, const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);

  /**
   * \brief Create a MoveIt Collision cylinder with a center point pose
   * \param pose - vector pointing along axis of cylinder
   * \param name - semantic name of MoveIt collision object
   * \param radius - size of cylinder
   * \param height - size of cylinder
   * \param color to display the collision object with
   * \return true on sucess
   */
  bool publishCollisionCylinder(const Eigen::Affine3d &object_pose, const std::string &object_name, double radius,
                                double height, const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);
  bool publishCollisionCylinder(const geometry_msgs::Pose &object_pose, const std::string &object_name, double radius,
                                double height, const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);

  /**
   * \brief Create a collision object using a mesh
   * \param pose - location of center of mesh
   * \param name - semantic name of MoveIt collision object
   * \param peth - file location to an .stl file
   * \param color to display the collision object with
   * \return true on success
   */
  bool publishCollisionMesh(const geometry_msgs::Pose &object_pose, const std::string &object_name,
                            const std::string &mesh_path,
                            const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);
  bool publishCollisionMesh(const Eigen::Affine3d &object_pose, const std::string &object_name,
                            const std::string &mesh_path,
                            const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);
  bool publishCollisionMesh(const Eigen::Affine3d &object_pose, const std::string &object_name,
                            const shape_msgs::Mesh &mesh_msg,
                            const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);
  bool publishCollisionMesh(const geometry_msgs::Pose &object_pose, const std::string &object_name,
                            const shape_msgs::Mesh &mesh_msg,
                            const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);

  /**
   * \brief Publish a connected birectional graph
   * \param graph of nodes and edges
   * \param name of collision object
   * \param color to display the collision object with
   * \return true on sucess
   */
  bool publishCollisionGraph(const graph_msgs::GeometryGraph &graph, const std::string &object_name, double radius,
                             const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);

  /**
   * \brief Helper for publishCollisionWall
   */
  void getCollisionWallMsg(double x, double y, double angle, double width, const std::string name,
                           moveit_msgs::CollisionObject &collision_obj);

  /**
   * \brief Publish a typical room wall
   * \param x
   * \param y
   * \param angle
   * \param width
   * \param name
   * \param color to display the collision object with
   * \return true on sucess
   */
  bool publishCollisionWall(double x, double y, double angle, double width, const std::string name,
                            const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);

  /**
   * \brief Publish a typical room table
   * \param x
   * \param y
   * \param angle
   * \param width
   * \param height
   * \param depth
   * \param name
   * \param color to display the collision object with
   * \return true on sucess
   */
  bool publishCollisionTable(double x, double y, double angle, double width, double height, double depth,
                             const std::string name, const rviz_visual_tools::colors &color = rviz_visual_tools::GREEN);

  /**
   * \brief Load a planning scene to a planning_scene_monitor from file
   * \param path - path to planning scene, e.g. as exported from Rviz Plugin
   * \param offset for scene to be placed
   * \return true on success
   */
  bool loadCollisionSceneFromFile(const std::string &path);
  bool loadCollisionSceneFromFile(const std::string &path, const Eigen::Affine3d &offset);

  /**
   * \brief Simple tests for collision testing
   * \return true on success
   */
  RVIZ_VISUAL_TOOLS_DEPRECATED
  bool publishCollisionTests();

  /**
   * \brief Display size of workspace used for planning with OMPL, etc. Important for virtual joints
   * \param display bounds of workspace
   * \return true on sucess
   */
  bool publishWorkspaceParameters(const moveit_msgs::WorkspaceParameters &params);

  /**
   * \brief Given a planning scene and robot state, publish any collisions
   * \param robot_state
   * \param planning_scene
   * \param color - display color of markers
   * \return true on success
   */
  bool publishContactPoints(const moveit::core::RobotState &robot_state,
                            const planning_scene::PlanningScene *planning_scene,
                            const rviz_visual_tools::colors &color = rviz_visual_tools::RED);

  /**
   * \brief Move a joint group in MoveIt for visualization
   *  make sure you have already set the planning group name
   *  this assumes the trajectory_pt position is the size of the number of joints in the planning group
   *  This will be displayed in the Planned Path section of the MoveIt Rviz plugin
   *
   * \param trajectory_pts - a single joint configuration
   * \param planning_group - the MoveIt planning group the trajectory applies to
   * \param display_time - amount of time for the trajectory to "execute"
   * \return true on success
   */
  bool publishTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint &trajectory_pt,
                              const std::string &planning_group, double display_time = 0.1);

  /**
   * \brief Animate trajectory in rviz
   * \param trajectory the actual plan
   * \param blocking whether we need to wait for the animation to complete
   * \param robot_state - the base state to add the joint trajectory message to
   * \param ee_jmg - the set of joints to use, e.g. the MoveIt! planning group, e.g. "left_arm"
   * \return true on success
   */
  bool publishTrajectoryPath(const std::vector<robot_state::RobotStatePtr> &trajectory,
                             const moveit::core::JointModelGroup *jmg, double speed = 0.01, bool blocking = false);
  bool publishTrajectoryPath(const robot_trajectory::RobotTrajectory &trajectory, bool blocking = false);
  bool publishTrajectoryPath(const moveit_msgs::RobotTrajectory &trajectory_msg,
                             const robot_state::RobotStateConstPtr robot_state, bool blocking);

  /**
   * \brief Display a line of the end effector path from a robot trajectory path
   * \param trajectory_msg - the robot plan
   * \param ee_parent_link - the link that we should trace a path of, e.g. the gripper link
   * \param arm_jmg - the set of joints to use, e.g. the MoveIt! planning group, e.g. "left_arm"
   * \param color - display color of markers
   * \param clear_all_markers - optionally ability to delete all existing markers in Rviz before adding the trajectory path
   * \return true on success
   */
  bool publishTrajectoryLine(const moveit_msgs::RobotTrajectory &trajectory_msg,
                             const moveit::core::LinkModel *ee_parent_link, const robot_model::JointModelGroup *arm_jmg,
                             const rviz_visual_tools::colors &color, bool clear_all_markers = false);
  bool publishTrajectoryLine(const robot_trajectory::RobotTrajectoryPtr robot_trajectory,
                             const moveit::core::LinkModel* ee_parent_link,
                             const rviz_visual_tools::colors& color,
                             bool clear_all_markers);

  /**
   * \brief Display trajectory as series of end effector position points
   * \param trajectory the actual plan
   * \param color - display color of markers
   * \return true on success
   */
  bool publishTrajectoryPoints(const std::vector<robot_state::RobotStatePtr> &robot_state_trajectory,
                               const moveit::core::LinkModel *ee_parent_link,
                               const rviz_visual_tools::colors &color = rviz_visual_tools::YELLOW);

  /**
   * \brief Publish a complete robot state to Rviz
   *        To use, add a RobotState marker to Rviz and subscribe to the DISPLAY_ROBOT_STATE_TOPIC, above
   * \param robot_state - joint values of robot
   * \param color - how to highlight the robot (solid-ly) if desired, default keeps color as specified in URDF
   */
  bool publishRobotState(const robot_state::RobotState &robot_state,
                         const rviz_visual_tools::colors &color = rviz_visual_tools::DEFAULT);
  bool publishRobotState(const robot_state::RobotStatePtr &robot_state,
                         const rviz_visual_tools::colors &color = rviz_visual_tools::DEFAULT);

  /**
   * \brief Publish a MoveIt robot state to a topic that the Rviz "RobotState" display can show
   * \param trajectory_pt of joint positions
   * \param jmg - the set of joints to use, e.g. the MoveIt! planning group, e.g. "left_arm"
   * \param color - how to highlight the robot (solid-ly) if desired, default keeps color as specified in URDF
   * \return true on success
   */
  bool publishRobotState(const trajectory_msgs::JointTrajectoryPoint &trajectory_pt,
                         const robot_model::JointModelGroup *jmg,
                         const rviz_visual_tools::colors &color = rviz_visual_tools::DEFAULT);

  /**
   * \brief Fake removing a Robot State display in Rviz by simply moving it very far away
   * \return true on success
   */
  bool hideRobot();

private:
  /**
   * @brief Get the planning scene monitor that this class is using
   * @return a ptr to a planning scene
   */
  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor();

protected:
  // Short name for this class
  std::string name_;

  // ROS publishers
  ros::Publisher pub_display_path_;  // for MoveIt trajectories
  ros::Publisher pub_robot_state_;   // publish a RobotState message

  // Pointer to a Planning Scene Monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_model_loader::RobotModelLoaderPtr rm_loader_;  // so that we can specify our own options

  // End Effector Markers
  std::map<const robot_model::JointModelGroup *, visualization_msgs::MarkerArray> ee_markers_map_;
  std::map<const robot_model::JointModelGroup *, EigenSTL::vector_Affine3d> ee_poses_map_;

  // Cached robot state marker - cache the colored links.
  // Note: Only allows colors provided in rviz_visual_tools to prevent too many robot state messages from being loaded
  // and ensuring efficiency
  std::map<rviz_visual_tools::colors, moveit_msgs::DisplayRobotState> display_robot_msgs_;

  // Pointer to the robot model
  robot_state::RobotModelConstPtr robot_model_;

  // Note: call loadSharedRobotState() before using this
  robot_state::RobotStatePtr shared_robot_state_;

  // Note: call loadSharedRobotState() before using this. Use only for hiding the robot
  robot_state::RobotStatePtr hidden_robot_state_;

  // Prevent the planning scene from always auto-pushing, but rather do it manually
  bool mannual_trigger_update_;

  // ROS topic names to use when starting publishers
  std::string robot_state_topic_;
  std::string planning_scene_topic_;

};  // class

typedef boost::shared_ptr<MoveItVisualTools> MoveItVisualToolsPtr;
typedef boost::shared_ptr<const MoveItVisualTools> MoveItVisualToolsConstPtr;

}  // namespace

#endif
