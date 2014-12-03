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

/* \author  Dave Coleman
 * \desc    Helper functions for displaying and debugging MoveIt! data in Rviz via published markers
 *          and MoveIt! collision objects. Very useful for debugging complex software
 *
 *          See README.md for developers notes.
 *
 */

#ifndef MOVEIT_VISUAL_TOOLS__MOVEIT_VISUAL_TOOLS_H_
#define MOVEIT_VISUAL_TOOLS__MOVEIT_VISUAL_TOOLS_H_

// Rviz Visualization Tool
#include <rviz_visual_tools/rviz_visual_tools.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/macros/deprecation.h>

// Messages
#include <trajectory_msgs/JointTrajectory.h>

namespace moveit_visual_tools
{

// Default constants
static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string COLLISION_TOPIC = "/collision_object";
static const std::string ATTACHED_COLLISION_TOPIC = "/attached_collision_object";
static const std::string PLANNING_SCENE_TOPIC = "/move_group/monitored_planning_scene";
static const std::string DISPLAY_PLANNED_PATH_TOPIC = "/move_group/display_planned_path";
static const std::string DISPLAY_ROBOT_STATE_TOPIC = "/move_group/robot_state";

class MoveItVisualTools : public rviz_visual_tools::RvizVisualTools
{
protected:

  // ROS publishers
  ros::Publisher pub_collision_obj_; // for MoveIt collision objects
  ros::Publisher pub_attach_collision_obj_; // for MoveIt attached objects
  ros::Publisher pub_display_path_; // for MoveIt trajectories
  ros::Publisher pub_planning_scene_diff_; // for adding and removing collision objects
  ros::Publisher pub_robot_state_; // publish a RobotState message

  // Pointer to a Planning Scene Monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_model_loader::RobotModelLoaderPtr rm_loader_; // so that we can specify our own options

  // End Effector Markers
  visualization_msgs::MarkerArray ee_marker_array_;
  tf::Pose tf_root_to_link_;
  geometry_msgs::Pose grasp_pose_to_eef_pose_; // Convert generic grasp pose to this end effector's frame of reference
  std::vector<geometry_msgs::Pose> marker_poses_;

  // MoveIt cached markers
  moveit_msgs::DisplayRobotState display_robot_msg_;

  // MoveIt cached objects
  robot_state::RobotStatePtr shared_robot_state_; // Note: call loadSharedRobotState() before using this
  robot_state::RobotModelConstPtr robot_model_;

public:

  /**
   * \brief Constructor
   * \param base_frame - common base for all visualization markers, usually "/world" or "/odom"
   * \param marker_topic - rostopic to publish markers to - your Rviz display should match
   * \param planning_scene_monitor - optionally pass in a pre-loaded planning scene monitor to avoid having to re-load
   *        the URDF, kinematic solvers, etc
   */
  MoveItVisualTools(const std::string& base_frame,
              const std::string& marker_topic,
              planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);

  /**
   * \brief Constructor
   * \param base_frame - common base for all visualization markers, usually "/world" or "/odom"
   * \param marker_topic - rostopic to publish markers to - your Rviz display should match
   * \param robot_model - load robot model pointer so that we don't have do re-parse it here
   */
  MoveItVisualTools(const std::string& base_frame,
              const std::string& marker_topic = rviz_visual_tools::RVIZ_MARKER_TOPIC,
              robot_model::RobotModelConstPtr robot_model = robot_model::RobotModelConstPtr());

  /**
   * \brief Deconstructor
   */
  ~MoveItVisualTools() {};

  /**
   * \brief Load a planning scene monitor if one was not passed into the constructor
   * \return true if successful in loading
   */
  bool loadPlanningSceneMonitor();

  /**
   * \brief Skip a ROS message call by sending directly to planning scene monitor
   * \param collision object message
   * \return true on success
   */
  bool processCollisionObjectMsg(const moveit_msgs::CollisionObject& msg);

  /**
   * \brief Load robot state only as needed
   * \return true if successful in loading
   */
  bool loadSharedRobotState();

  /**
   * \brief Caches the meshes and geometry of a robot. NOTE: perhaps not maintained...
   * \return true if successful in loading
   */
  bool loadRobotMarkers();

  /**
   * \brief Allow robot state to be altered.
   * \return shared pointer to robot state
   */
  robot_state::RobotStatePtr& getSharedRobotState();

  /**
   * \brief Call this once at begining to load the robot marker
   * \param
   * \param
   * \return true if it is successful
   */
  bool loadEEMarker(const std::string& ee_group_name, const std::string& planning_group);

  /**
   * \brief Load publishers as needed
   */
  void loadAttachedPub();
  void loadPlanningPub();
  void loadTrajectoryPub();
  void loadRobotStatePub(const std::string &marker_topic = DISPLAY_ROBOT_STATE_TOPIC);

  /**
   * \brief Allow a pre-configured planning scene monitor to be set for publishing collision objects, etc
   * \param a pointer to a load planning scen
   */
  void setPlanningSceneMonitor(planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor)
  {
    planning_scene_monitor_ = planning_scene_monitor;
  }

  /**
   * \brief Publish an end effector to rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \return true on success
   */
  bool publishEEMarkers(const geometry_msgs::Pose &pose, const rviz_visual_tools::colors &color = rviz_visual_tools::WHITE, const std::string &ns="end_effector");

  /**
   * \brief Show grasps generated from moveit_simple_grasps or other MoveIt Grasp message sources
   * \param possible_grasps - a set of grasp positions to visualize
   * \param ee_parent_link - end effector's attachment link
   * \param animate_speed - how fast the gripper approach is animated, optional
   */
  bool publishGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
                     const std::string &ee_parent_link, double animate_speed = 0.1);

  /**
   * \brief Display an animated vector of grasps including its approach movement in Rviz
   * \param possible_grasps - a set of grasp positions to visualize
   * \param ee_parent_link - end effector's attachment link
   * \param animate_speed - how fast the gripper approach is animated, optional
   */
  bool publishAnimatedGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
                             const std::string &ee_parent_link, double animate_speed = 0.01);

  /**
   * \brief Animate a single grasp in its movement direction
   * \param grasp
   * \param ee_parent_link - end effector's attachment link
   * \param animate_speed - how fast the gripper approach is animated
   * \return true on sucess
   */
  bool publishAnimatedGrasp(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link, double animate_speed);

  /**
   * \brief Display an vector of inverse kinematic solutions for the IK service in Rviz
   * Note: this is published to the 'Planned Path' section of the 'MotionPlanning' display in Rviz
   * \param ik_solutions - a set of corresponding arm positions to achieve each grasp
   * \param display_time - amount of time to sleep between sending trajectories, optional
   */
  bool publishIKSolutions(const std::vector<trajectory_msgs::JointTrajectoryPoint> &ik_solutions,
                          const std::string& planning_group, double display_time = 0.4);

  /**
   * \brief Remove all collision objects that this class has added to the MoveIt! planning scene
   *        Communicates to a remote move_group node through a ROS message
   * \return true on sucess
   */
  MOVEIT_DEPRECATED bool removeAllCollisionObjects()
  {
    publishRemoveAllCollisionObjects();
  }
  bool publishRemoveAllCollisionObjects();

  /**
   * \brief Remove all collision objects that this class has added to the MoveIt! planning scene
   *        Communicates directly to a planning scene monitor e.g. if this is the move_group node
   * \param  the scene to directly clear the collision objects from
   * \return true on sucess
   */
  bool removeAllCollisionObjectsPS();

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
   * \return true on success
   */
  bool publishCollisionFloor(double z = 0.0, const std::string& plane_name = "Floor");

  /**
   * \brief Create a MoveIt Collision block at the given pose
   * \param pose - location of center of block
   * \param name - semantic name of MoveIt collision object
   * \param size - height=width=depth=size
   * \return true on sucess
   **/
  bool publishCollisionBlock(const geometry_msgs::Pose& block_pose, const std::string& block_name, double block_size);

  /**
   * \brief Create a MoveIt Collision block at the given pose
   * \param point1 - top left of rectangle
   * \param point2 - bottom right of rectangle
   * \param name - semantic name of MoveIt collision object
   * \return true on sucess
   **/
  bool publishCollisionRectangle(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2, 
                                 const std::string& block_name);

  /**
   * \brief Create a MoveIt Collision cylinder between two points
   * \param point a - x,y,z in space of a point
   * \param point b - x,y,z in space of a point
   * \param name - semantic name of MoveIt collision object
   * \param radius - size of cylinder
   * \return true on sucess
   */
  bool publishCollisionCylinder(const geometry_msgs::Point &a, const geometry_msgs::Point &b, 
                                const std::string& object_name, double radius);
  bool publishCollisionCylinder(const Eigen::Vector3d &a, const Eigen::Vector3d &b, 
                                const std::string& object_name, double radius);

  /**
   * \brief Create a MoveIt Collision cylinder with a center point pose
   * \param pose - vector pointing along axis of cylinder
   * \param name - semantic name of MoveIt collision object
   * \param radius - size of cylinder
   * \param height - size of cylinder
   * \return true on sucess
   */
  bool publishCollisionCylinder(const Eigen::Affine3d& object_pose, const std::string& object_name, double radius, double height);
  bool publishCollisionCylinder(const geometry_msgs::Pose& object_pose, const std::string& object_name, double radius, double height);

  /**
   * \brief Publish a connected birectional graph
   * \param graph of nodes and edges
   * \param name of collision object
   * \return true on sucess
   */
  bool publishCollisionGraph(const graph_msgs::GeometryGraph &graph, const std::string &object_name, double radius);

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
   * \return true on sucess
   */
  bool publishCollisionWall(double x, double y, double angle, double width, const std::string name);

  /**
   * \brief Publish a typical room table
   * \param x
   * \param y
   * \param angle
   * \param width
   * \param height
   * \param depth
   * \param name
   * \return true on sucess
   */
  bool publishCollisionTable(double x, double y, double angle, double width, double height,
                             double depth, const std::string name);

  /**
   * \brief Load a planning scene to a planning_scene_monitor from file
   * \param path - path to planning scene, e.g. as exported from Rviz Plugin
   * \param planning scene monitor that is already setup
   * \return true on success
   */
  bool loadCollisionSceneFromFile(const std::string &path, double x_offset = 0, double y_offset = 0);

  /**
   * \brief Simple tests for collision testing
   * \return true on success
   */
  bool publishCollisionTests();

  /**
   * \brief Move a joint group in MoveIt for visualization
   *  make sure you have already set the planning group name
   *  this assumes the trajectory_pt position is the size of the number of joints in the planning group
   *  This will be displayed in the Planned Path section of the MoveIt Rviz plugin
   *
   * \param trajectory_pts - a single joint configuration
   * \param group_name - the MoveIt planning group the trajectory applies to
   * \param display_time - amount of time for the trajectory to "execute"
   * \return true on success
   */
  bool publishTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& trajectory_pt, const std::string &group_name,
                              double display_time = 0.1);

  /**
   * \brief Animate trajectory in rviz
   * \param trajectory_msg the actual plan
   * \param blocking whether we need to wait for the animation to complete
   * \return true on success
   */
  bool publishTrajectoryPath(const robot_trajectory::RobotTrajectory& trajectory, bool blocking = false);
  bool publishTrajectoryPath(const moveit_msgs::RobotTrajectory& trajectory_msg, bool blocking = false);

  /**
   * \brief Publish a complete robot state to Rviz
   *        To use, add a RobotState marker to Rviz and subscribe to the DISPLAY_ROBOT_STATE_TOPIC, above
   * \param robot_state
   */
  bool publishRobotState(const robot_state::RobotState &robot_state);
  bool publishRobotState(const robot_state::RobotStatePtr &robot_state);

  /**
   * \brief Publish a MoveIt robot state to a topic that the Rviz "RobotState" display can show
   * \param robot_state
   * \return true on success
   */
  bool publishRobotState(const trajectory_msgs::JointTrajectoryPoint& trajectory_pt, const std::string &group_name);

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


}; // class

typedef boost::shared_ptr<MoveItVisualTools> MoveItVisualToolsPtr;
typedef boost::shared_ptr<const MoveItVisualTools> MoveItVisualToolsConstPtr;

} // namespace

#endif
