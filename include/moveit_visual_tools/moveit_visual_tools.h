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

/* \author  Dave Coleman
 * \desc    Helper functions for displaying and debugging MoveIt data in Rviz via published markers
 *          and MoveIt collision objects. Very useful for debugging complex software
 */

#ifndef MOVEIT_VISUAL_TOOLS_MOVEIT_VISUAL_TOOLS_H
#define MOVEIT_VISUAL_TOOLS_MOVEIT_VISUAL_TOOLS_H

// Rviz Visualization Tool
#include <rviz_visual_tools/rviz_visual_tools.hpp>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// MoveIt Messages
#include <moveit_msgs/msg/grasp.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/workspace_parameters.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

// ROS Messages
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <graph_msgs/msg/geometry_graph.hpp>

// C++
#include <map>
#include <string>
#include <utility>
#include <vector>

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
   *
   * All Markers will be rendered in the planning frame of the model ROBOT_DESCRIPTION
   * and are published to rviz_visual_tools::RVIZ_MARKER_TOPIC
   */
  MoveItVisualTools(const rclcpp::Node::SharedPtr& node);

  /**
   * \brief Constructor
   * \param base_frame - common base for all visualization markers, usually "/world" or "/odom"
   * \param marker_topic - rostopic to publish markers to - your Rviz display should match
   * \param planning_scene_monitor - optionally pass in a pre-loaded planning scene monitor to
   *        avoid having to re-load the URDF, kinematic solvers, etc
   */
  MoveItVisualTools(const rclcpp::Node::SharedPtr& node, const std::string& base_frame, const std::string& marker_topic,
                    planning_scene_monitor::PlanningSceneMonitorPtr psm);

  /**
   * \brief Constructor
   * \param base_frame - common base for all visualization markers, usually "/world" or "/odom"
   * \param marker_topic - rostopic to publish markers to - your Rviz display should match
   * \param robot_model - load robot model pointer so that we don't have do re-parse it here
   */
  MoveItVisualTools(const rclcpp::Node::SharedPtr& node, const std::string& base_frame,
                    const std::string& marker_topic = rviz_visual_tools::RVIZ_MARKER_TOPIC,
                    moveit::core::RobotModelConstPtr robot_model = moveit::core::RobotModelConstPtr());

  /**
   * \brief Set the ROS topic for publishing a robot state
   * \param topic
   */
  void setRobotStateTopic(const std::string& robot_state_topic)
  {
    robot_state_topic_ = robot_state_topic;
  }

  /**
   * \brief Set the planning scene topic
   * \param topic
   */
  void setPlanningSceneTopic(const std::string& planning_scene_topic)
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
  bool processCollisionObjectMsg(const moveit_msgs::msg::CollisionObject& msg,
                                 const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);

  /**
   * \brief Skip a ROS message call by sending directly to planning scene monitor
   * \param attached collision object message
   * \return true on success
   */
  bool processAttachedCollisionObjectMsg(const moveit_msgs::msg::AttachedCollisionObject& msg);

  /**
   * \brief Move an already published collision object to a new locaiton in space
   * \param pose - location of center of object
   * \param name - semantic name of MoveIt collision object
   * \return true on success
   */
  bool moveCollisionObject(const Eigen::Isometry3d& pose, const std::string& name,
                           const rviz_visual_tools::Colors& color);
  bool moveCollisionObject(const geometry_msgs::msg::Pose& pose, const std::string& name,
                           const rviz_visual_tools::Colors& color);

  /**
   * \brief When manual_trigger_update_ is true, use this to tell the planning scene to send
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
  moveit::core::RobotStatePtr& getSharedRobotState();

  /**
   * \brief Allow robot state to be altered.
   * \return shared pointer to robot state
   */
  moveit::core::RobotStatePtr& getRootRobotState()
  {
    return root_robot_state_;
  }

  /**
   * \brief Get a pointer to the robot model
   * \return const RobotModel
   */
  moveit::core::RobotModelConstPtr getRobotModel();

  /**
   * \brief Call this once at begining to load the end effector markers and then whenever a joint changes
   * \param ee_jmg - the set of joints to use, e.g. the MoveIt planning group, e.g. "left_gripper"
   * \param ee_joint_pos - the values of all active joints in this planning group
   * \return true if it is successful
   */
  bool loadEEMarker(const moveit::core::JointModelGroup* ee_jmg, const std::vector<double>& ee_joint_pos = {});

  /**
   * \brief Load publishers as needed
   */
  void loadTrajectoryPub(const std::string& display_planned_path_topic = DISPLAY_PLANNED_PATH_TOPIC,
                         bool blocking = true);
  void loadRobotStatePub(const std::string& robot_state_topic = DISPLAY_ROBOT_STATE_TOPIC, bool blocking = true);

  /**
   * \brief Allow a pre-configured planning scene monitor to be set for publishing collision objects, etc
   * \param a pointer to a load planning scene
   */
  void setPlanningSceneMonitor(planning_scene_monitor::PlanningSceneMonitorPtr psm)
  {
    psm_ = std::move(psm);
  }

  /**
   * \brief Prevent the planning scene from always auto-pushing, but rather do it manually
   * \param bool true to enable manual mode
   */
  void setManualSceneUpdating(bool enable_manual = true)
  {
    manual_trigger_update_ = enable_manual;
  }

  /**
   * \brief Publish an end effector to rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \param ee_jmg - joint model group of the end effector, e.g. "left_hand"
   * \param ee_joint_pos - position of all active joints in the end effector
   * \param color to display the collision object with
   * \return true on success
   */
  bool publishEEMarkers(const Eigen::Isometry3d& pose, const moveit::core::JointModelGroup* ee_jmg,
                        const std::vector<double>& ee_joint_pos,
                        const rviz_visual_tools::Colors& color = rviz_visual_tools::DEFAULT,
                        const std::string& ns = "end_effector")
  {
    return publishEEMarkers(convertPose(pose), ee_jmg, ee_joint_pos, color, ns);
  }
  bool publishEEMarkers(const Eigen::Isometry3d& pose, const moveit::core::JointModelGroup* ee_jmg,
                        const rviz_visual_tools::Colors& color = rviz_visual_tools::DEFAULT,
                        const std::string& ns = "end_effector")
  {
    return publishEEMarkers(convertPose(pose), ee_jmg, {}, color, ns);
  }
  bool publishEEMarkers(const geometry_msgs::msg::Pose& pose, const moveit::core::JointModelGroup* ee_jmg,
                        const rviz_visual_tools::Colors& color = rviz_visual_tools::DEFAULT,
                        const std::string& ns = "end_effector")
  {
    return publishEEMarkers(pose, ee_jmg, {}, color, ns);
  }
  bool publishEEMarkers(const geometry_msgs::msg::Pose& pose, const moveit::core::JointModelGroup* ee_jmg,
                        const std::vector<double>& ee_joint_pos,
                        const rviz_visual_tools::Colors& color = rviz_visual_tools::DEFAULT,
                        const std::string& ns = "end_effector");

  /**
   * \brief Show grasps generated from moveit_simple_grasps or other MoveIt Grasp message sources
   * \param possible_grasps - a set of grasp positions to visualize
   * \param ee_jmg - the set of joints to use, e.g. the MoveIt planning group, e.g. "left_arm"
   * \param animate_speed - how fast the gripper approach is animated in seconds, optional
   */
  bool publishGrasps(const std::vector<moveit_msgs::msg::Grasp>& possible_grasps,
                     const moveit::core::JointModelGroup* ee_jmg, double animate_speed = 0.1 /* seconds */);

  /**
   * \brief Display an animated vector of grasps including its approach movement in Rviz
   *        Note this function calls publish() automatically in order to achieve animations
   * \param possible_grasps - a set of grasp positions to visualize
   * \param ee_jmg - the set of joints to use, e.g. the MoveIt planning group, e.g. "left_arm"
   * \param animate_speed - how fast the gripper approach is animated in seconds, optional
   */
  bool publishAnimatedGrasps(const std::vector<moveit_msgs::msg::Grasp>& possible_grasps,
                             const moveit::core::JointModelGroup* ee_jmg, double animate_speed = 0.01 /* seconds */);

  /**
   * \brief Animate a single grasp in its movement direction
   *        Note this function calls publish() automatically in order to achieve animations
   * \param grasp
   * \param ee_jmg - the set of joints to use, e.g. the MoveIt planning group, e.g. "left_arm"
   * \param animate_speed - how fast the gripper approach is animated in seconds
   * \return true on sucess
   */
  bool publishAnimatedGrasp(const moveit_msgs::msg::Grasp& grasp, const moveit::core::JointModelGroup* ee_jmg,
                            double animate_speed /* seconds */);

  /**
   * \brief Display an vector of inverse kinematic solutions for the IK service in Rviz
   * Note: this is published to the 'Planned Path' section of the 'MotionPlanning' display in Rviz
   * \param ik_solutions - a set of corresponding arm positions to achieve each grasp
   * \param arm_jmg - the set of joints to use, e.g. the MoveIt planning group, e.g. "left_arm"
   * \param display_time - amount of time to sleep between sending trajectories, optional
   */
  bool publishIKSolutions(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& ik_solutions,
                          const moveit::core::JointModelGroup* arm_jmg, double display_time = 0.4);

  /**
   * \brief Remove all collision objects that this class has added to the MoveIt planning scene
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
  bool cleanupCO(const std::string& name);

  /**
   * \brief Remove an active collision object from the planning scene
   * \param Name of object
   * \return true on sucess
   */
  bool cleanupACO(const std::string& name);

  /**
   * \brief Attach a collision object from the planning scene
   * \param Name of object
   * \param
   * \return true on sucess
   */
  bool attachCO(const std::string& name, const std::string& ee_parent_link);

  /**
   * \brief Make the floor a collision object
   * \param z location of floor
   * \param name of floor
   * \param color to display the collision object with
   * \return true on success
   */
  bool publishCollisionFloor(double z = 0.0, const std::string& plane_name = "Floor",
                             const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);

  /**
   * \brief Create a MoveIt Collision block at the given pose
   * \param pose - location of center of block
   * \param name - semantic name of MoveIt collision object
   * \param size - height=width=depth=size
   * \param color to display the collision object with
   * \return true on sucess
   **/
  bool publishCollisionBlock(const geometry_msgs::msg::Pose& block_pose, const std::string& block_name = "block",
                             double block_size = 0.1,
                             const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);

  /**
   * \brief Create a MoveIt collision rectangular cuboid at the given pose
   * \param point1 - top left of rectangle
   * \param point2 - bottom right of rectangle
   * \param name - semantic name of MoveIt collision object
   * \param color to display the collision object with
   * \return true on sucess
   **/
  bool publishCollisionCuboid(const Eigen::Vector3d& point1, const Eigen::Vector3d& point2, const std::string& name,
                              const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);
  bool publishCollisionCuboid(const geometry_msgs::msg::Point& point1, const geometry_msgs::msg::Point& point2,
                              const std::string& name,
                              const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);

  /**
   * \brief Create a MoveIt collision rectangular cuboid at the given pose
   * \param pose - position of the centroid of the cube
   * \param width - width of the object in its local frame
   * \param depth - depth of the object in its local frame
   * \param height - height of the object in its local frame
   * \param name - semantic name of MoveIt collision object
   * \param color to display the collision object with
   * \return true on sucess
   **/
  bool publishCollisionCuboid(const Eigen::Isometry3d& pose, double width, double depth, double height,
                              const std::string& name, const rviz_visual_tools::Colors& color);

  bool publishCollisionCuboid(const geometry_msgs::msg::Pose& pose, double width, double depth, double height,
                              const std::string& name, const rviz_visual_tools::Colors& color);

  /**
   * \brief Create a MoveIt collision rectangular cuboid at the given pose
   * \param pose - position of the centroid of the cube
   * \param size - the size (x,y,z) of the object in its local frame
   * \param name - semantic name of MoveIt collision object
   * \param color to display the collision object with
   * \return true on sucess
   **/
  bool publishCollisionCuboid(const Eigen::Isometry3d& pose, const Eigen::Vector3d& size, const std::string& name,
                              const rviz_visual_tools::Colors& color)
  {
    return publishCollisionCuboid(pose, size.x(), size.y(), size.z(), name, color);
  }
  bool publishCollisionCuboid(const geometry_msgs::msg::Pose& pose, const geometry_msgs::msg::Vector3& size,
                              const std::string& name, const rviz_visual_tools::Colors& color)
  {
    return publishCollisionCuboid(pose, size.x, size.y, size.z, name, color);
  }

  /**
   * \brief Create a MoveIt Collision cylinder between two points
   * \param point a - x,y,z in space of a point
   * \param point b - x,y,z in space of a point
   * \param name - semantic name of MoveIt collision object
   * \param radius - size of cylinder
   * \param color to display the collision object with
   * \return true on sucess
   */
  bool publishCollisionCylinder(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b,
                                const std::string& object_name, double radius,
                                const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);
  bool publishCollisionCylinder(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const std::string& object_name,
                                double radius, const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);

  /**
   * \brief Create a MoveIt Collision cylinder with a center point pose
   * \param pose - vector pointing along axis of cylinder
   * \param name - semantic name of MoveIt collision object
   * \param radius - size of cylinder
   * \param height - size of cylinder
   * \param color to display the collision object with
   * \return true on sucess
   */
  bool publishCollisionCylinder(const Eigen::Isometry3d& object_pose, const std::string& object_name, double radius,
                                double height, const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);
  bool publishCollisionCylinder(const geometry_msgs::msg::Pose& object_pose, const std::string& object_name,
                                double radius, double height,
                                const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);

  /**
   * \brief Create a collision object using a mesh
   * \param pose - location of center of mesh
   * \param name - semantic name of MoveIt collision object
   * \param peth - file location to an .stl file
   * \param color to display the collision object with
   * \return true on success
   */
  bool publishCollisionMesh(const geometry_msgs::msg::Pose& object_pose, const std::string& object_name,
                            const std::string& mesh_path,
                            const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);
  bool publishCollisionMesh(const Eigen::Isometry3d& object_pose, const std::string& object_name,
                            const std::string& mesh_path,
                            const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);
  bool publishCollisionMesh(const Eigen::Isometry3d& object_pose, const std::string& object_name,
                            const shape_msgs::msg::Mesh& mesh_msg,
                            const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);
  bool publishCollisionMesh(const geometry_msgs::msg::Pose& object_pose, const std::string& object_name,
                            const shape_msgs::msg::Mesh& mesh_msg,
                            const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);

  /**
   * \brief Publish a connected birectional graph
   * \param graph of nodes and edges
   * \param name of collision object
   * \param color to display the collision object with
   * \return true on sucess
   */
  bool publishCollisionGraph(const graph_msgs::msg::GeometryGraph& graph, const std::string& object_name, double radius,
                             const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);

  /**
   * \brief Helper for publishCollisionWall
   */
  void getCollisionWallMsg(double x, double y, double z, double angle, double width, double height,
                           const std::string& name, moveit_msgs::msg::CollisionObject& collision_obj);

  /**
   * \brief Publish a typical room wall
   * \param x
   * \param y
   * \param angle
   * \param width
   * \param height
   * \param name
   * \param color to display the collision object with
   * \return true on sucess
   */
  bool publishCollisionWall(double x, double y, double angle = 0.0, double width = 2.0, double height = 1.5,
                            const std::string& name = "wall",
                            const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);
  bool publishCollisionWall(double x, double y, double z, double angle = 0.0, double width = 2.0, double height = 1.5,
                            const std::string& name = "wall",
                            const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);

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
  bool publishCollisionTable(double x, double y, double z, double angle, double width, double height, double depth,
                             const std::string& name,
                             const rviz_visual_tools::Colors& color = rviz_visual_tools::GREEN);

  /**
   * \brief Load a planning scene to a planning_scene_monitor from file
   * \param path - path to planning scene, e.g. as exported from Rviz Plugin
   * \param offset for scene to be placed
   * \return true on success
   */
  bool loadCollisionSceneFromFile(const std::string& path);
  bool loadCollisionSceneFromFile(const std::string& path, const Eigen::Isometry3d& offset);

  /**
   * \brief Display size of workspace used for planning with OMPL, etc. Important for virtual joints
   * \param display bounds of workspace
   * \return true on sucess
   */
  bool publishWorkspaceParameters(const moveit_msgs::msg::WorkspaceParameters& params);

  /**
   * \brief Check if the robot state is in collision inside the planning scene and visualize the result.
   * If the state is not colliding only the robot state is published. If there is a collision the colliding
   * links are ghlighted and the contact points are visualized with markers.
   * \param robot_state - The robot state to check for collisions
   * \param planning_scene - The planning scene to use for collision checks
   * \param highlight_link_color - The color to use for highligting colliding links
   * \param contact_point_color - The color to use for contact point markers
   * \result - True if there is a collision
   */
  bool checkAndPublishCollision(const moveit::core::RobotState& robot_state,
                                const planning_scene::PlanningScene* planning_scene,
                                const rviz_visual_tools::Colors& highlight_link_color = rviz_visual_tools::RED,
                                const rviz_visual_tools::Colors& contact_point_color = rviz_visual_tools::PURPLE);

  /**
   * \brief Given a planning scene and robot state, publish any collisions
   * \param robot_state
   * \param planning_scene
   * \param color - display color of markers
   * \return true on success
   */
  bool publishContactPoints(const moveit::core::RobotState& robot_state,
                            const planning_scene::PlanningScene* planning_scene,
                            const rviz_visual_tools::Colors& color = rviz_visual_tools::RED);

  /**
   * \brief Given a contact map and planning scene, publish the contact points
   * \param contacts - The populated contacts to visualize
   * \param planning_scene
   * \param color - display color of markers
   * \return true on success
   */
  bool publishContactPoints(const collision_detection::CollisionResult::ContactMap& contacts,
                            const planning_scene::PlanningScene* planning_scene,
                            const rviz_visual_tools::Colors& color = rviz_visual_tools::RED);

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
  bool publishTrajectoryPoint(const trajectory_msgs::msg::JointTrajectoryPoint& trajectory_pt,
                              const std::string& planning_group, double display_time = 0.1);

  /**
   * \brief Animate trajectory in rviz. These functions do not need a trigger() called because use different publisher
   * \param trajectory the actual plan
   * \param blocking whether we need to wait for the animation to complete
   * \param robot_state - the base state to add the joint trajectory message to
   * \param ee_jmg - the set of joints to use, e.g. the MoveIt planning group, e.g. "left_arm"
   * \return true on success
   */
  bool publishTrajectoryPath(const std::vector<moveit::core::RobotStatePtr>& trajectory,
                             const moveit::core::JointModelGroup* jmg, double speed = 0.01, bool blocking = false);
  bool publishTrajectoryPath(const robot_trajectory::RobotTrajectoryPtr& trajectory, bool blocking = false);
  bool publishTrajectoryPath(const robot_trajectory::RobotTrajectory& trajectory, bool blocking = false);
  bool publishTrajectoryPath(const moveit_msgs::msg::RobotTrajectory& trajectory_msg,
                             const moveit::core::RobotStateConstPtr& robot_state, bool blocking = false);
  bool publishTrajectoryPath(const moveit_msgs::msg::RobotTrajectory& trajectory_msg,
                             const moveit::core::RobotState& robot_state, bool blocking = false);
  bool publishTrajectoryPath(const moveit_msgs::msg::RobotTrajectory& trajectory_msg,
                             const moveit_msgs::msg::RobotState& robot_state, bool blocking = false);
  void publishTrajectoryPath(const moveit_msgs::msg::DisplayTrajectory& display_trajectory_msg);

  /**
   * \brief Display a line of the end effector path from a robot trajectory path
   * \param trajectory_msg - the robot plan
   * \param ee_parent_link - the link that we should trace a path of, e.g. the gripper link
   * \param arm_jmg - the set of joints to use, e.g. the MoveIt planning group, e.g. "left_arm"
   * \param color - display color of markers
   * \return true on success
   */
  bool publishTrajectoryLine(const moveit_msgs::msg::RobotTrajectory& trajectory_msg,
                             const moveit::core::LinkModel* ee_parent_link,
                             const moveit::core::JointModelGroup* arm_jmg,
                             const rviz_visual_tools::Colors& color = rviz_visual_tools::LIME_GREEN);
  bool publishTrajectoryLine(const robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
                             const moveit::core::LinkModel* ee_parent_link,
                             const rviz_visual_tools::Colors& color = rviz_visual_tools::LIME_GREEN);
  bool publishTrajectoryLine(const robot_trajectory::RobotTrajectory& robot_trajectory,
                             const moveit::core::LinkModel* ee_parent_link,
                             const rviz_visual_tools::Colors& color = rviz_visual_tools::LIME_GREEN);

  /**
   * \brief Display a line of the end effector(s) path(s) from a robot trajectory path
   *        This version can visualize multiple end effectors
   * \param trajectory_msg - the robot plan
   * \param arm_jmg - the set of joints to use, e.g. the MoveIt planning group, e.g. "left_arm".
   * \param color - display color of markers
   * \return true on success
   */
  bool publishTrajectoryLine(const moveit_msgs::msg::RobotTrajectory& trajectory_msg,
                             const moveit::core::JointModelGroup* arm_jmg,
                             const rviz_visual_tools::Colors& color = rviz_visual_tools::LIME_GREEN);
  bool publishTrajectoryLine(const robot_trajectory::RobotTrajectoryPtr& robot_trajectory,
                             const moveit::core::JointModelGroup* arm_jmg,
                             const rviz_visual_tools::Colors& color = rviz_visual_tools::LIME_GREEN);
  bool publishTrajectoryLine(const robot_trajectory::RobotTrajectory& robot_trajectory,
                             const moveit::core::JointModelGroup* arm_jmg,
                             const rviz_visual_tools::Colors& color = rviz_visual_tools::LIME_GREEN);

  /**
   * \brief Display trajectory as series of end effector position points
   * \param trajectory the actual plan
   * \param color - display color of markers
   * \return true on success
   */
  bool publishTrajectoryPoints(const std::vector<moveit::core::RobotStatePtr>& robot_state_trajectory,
                               const moveit::core::LinkModel* ee_parent_link,
                               const rviz_visual_tools::Colors& color = rviz_visual_tools::YELLOW);

  /** \brief All published robot states will have their virtual joint moved by offset */
  void enableRobotStateRootOffet(const Eigen::Isometry3d& offset);

  /** \brief Turn off the root offset feature */
  void disableRobotStateRootOffet();

  /**
   * \brief Publish a MoveIt robot state to a topic that the Rviz "RobotState" display can show
   * \param trajectory_pt of joint positions
   * \param jmg - the set of joints to use, e.g. the MoveIt planning group, e.g. "left_arm"
   * \param color - how to highlight the robot (solid-ly) if desired, default keeps color as specified in URDF
   * \return true on success
   */
  bool publishRobotState(const trajectory_msgs::msg::JointTrajectoryPoint& trajectory_pt,
                         const moveit::core::JointModelGroup* jmg,
                         const rviz_visual_tools::Colors& color = rviz_visual_tools::DEFAULT);

  /**
   * \brief Publish a MoveIt robot state to a topic that the Rviz "RobotState" display can show
   * \param joint_positions - a vector of doubles corresponding 1-to-1 to the kinematic chain named in "jmg"
   * \param jmg - the set of joints to use, e.g. the MoveIt planning group, e.g. "left_arm"
   * \param color - how to highlight the robot (solid-ly) if desired, default keeps color as specified in URDF
   * \return true on success
   */
  bool publishRobotState(const std::vector<double>& joint_positions, const moveit::core::JointModelGroup* jmg,
                         const rviz_visual_tools::Colors& color = rviz_visual_tools::DEFAULT);

  /**
   * \brief Publish a complete robot state to Rviz
   *        To use, add a RobotState marker to Rviz and subscribe to the DISPLAY_ROBOT_STATE_TOPIC, above
   * \param robot_state - joint values of robot
   * \param color - how to highlight the robot (solid-ly) if desired, default keeps color as specified in URDF
   * \param highlight_links - if the |color| is not |DEFAULT|, allows selective robot links to be highlighted.
   * By default (empty) all links are highlighted.
   */
  bool publishRobotState(const moveit::core::RobotState& robot_state,
                         const rviz_visual_tools::Colors& color = rviz_visual_tools::DEFAULT,
                         const std::vector<std::string>& highlight_links = {});
  bool publishRobotState(const moveit::core::RobotStatePtr& robot_state,
                         const rviz_visual_tools::Colors& color = rviz_visual_tools::DEFAULT,
                         const std::vector<std::string>& highlight_links = {});
  void publishRobotState(const moveit_msgs::msg::DisplayRobotState& display_robot_state_msg);

  /**
   * \brief Hide robot in RobotState display in Rviz
   * \return true on success
   */
  bool hideRobot();

  /** \brief Before publishing a robot state, optionally change its root transform */
  static bool applyVirtualJointTransform(moveit::core::RobotState& robot_state, const Eigen::Isometry3d& offset);

  /**
   * \brief Print to console the current robot state's joint values within its limits visually
   * \param robot_state - the robot to show
   */
  void showJointLimits(const moveit::core::RobotStatePtr& robot_state);

  /**
   * @brief Get the planning scene monitor that this class is using
   * @return a ptr to a planning scene
   */
  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor();

private:
  /**
   * \brief Error check that the robot's SRDF was properly setup with a virtual joint that was named a certain way
   * \return true on success
   */
  static bool checkForVirtualJoint(const moveit::core::RobotState& robot_state);

protected:
  // Pointer to a Planning Scene Monitor
  planning_scene_monitor::PlanningSceneMonitorPtr psm_;

  // Prevent the planning scene from always auto-pushing, but rather do it manually
  bool manual_trigger_update_ = false;

  // Pointer to the robot model
  moveit::core::RobotModelConstPtr robot_model_;

  // ROS topic names to use when starting publishers
  std::string robot_state_topic_;
  std::string planning_scene_topic_;

  // ROS publishers
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr pub_display_path_;  // for MoveIt trajectories
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr pub_robot_state_;   // publish a RobotState message

  // ROS Node
  rclcpp::Node::SharedPtr node_;

  robot_model_loader::RobotModelLoaderPtr rm_loader_;  // so that we can specify our own options

  // End Effector Markers
  std::map<const moveit::core::JointModelGroup*, visualization_msgs::msg::MarkerArray> ee_markers_map_;
  std::map<const moveit::core::JointModelGroup*, EigenSTL::vector_Isometry3d> ee_poses_map_;
  std::map<const moveit::core::JointModelGroup*, std::vector<double> > ee_joint_pos_map_;

  // Cached robot state marker - cache the colored links.
  // Note: Only allows colors provided in rviz_visual_tools to prevent too many robot state messages from being loaded
  // and ensuring efficiency
  std::map<rviz_visual_tools::Colors, moveit_msgs::msg::DisplayRobotState> display_robot_msgs_;

  // Note: call loadSharedRobotState() before using this
  moveit::core::RobotStatePtr shared_robot_state_;

  // Note: call loadSharedRobotState() before using this. Use only for hiding the robot
  moveit::core::RobotStatePtr hidden_robot_state_;

  // A robot state whose virtual_joint remains at identity so that getGlobalLinkTransform() isn't tainted
  // Note: call loadSharedRobotState() before using this
  moveit::core::RobotStatePtr root_robot_state_;

  // Optional offset that can be applied to all outgoing/published robot states
  bool robot_state_root_offset_enabled_ = false;
  Eigen::Isometry3d robot_state_root_offset_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // http://eigen.tuxfamily.org/dox/group__TopicStructHavingEigenMembers.html
};                                 // class

typedef std::shared_ptr<MoveItVisualTools> MoveItVisualToolsPtr;
typedef std::shared_ptr<const MoveItVisualTools> MoveItVisualToolsConstPtr;

}  // namespace moveit_visual_tools

#endif  // MOVEIT_VISUAL_TOOLS_MOVEIT_VISUAL_TOOLS_H
