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

#ifndef MOVEIT_VISUAL_TOOLS__VISUAL_TOOLS_H_
#define MOVEIT_VISUAL_TOOLS__VISUAL_TOOLS_H_

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/Grasp.h>
#include <moveit_msgs/DisplayRobotState.h>
//#include <moveit/macros/deprecation.h>

// Boost
#include <boost/shared_ptr.hpp>

// Messages
#include <std_msgs/ColorRGBA.h>
#include <graph_msgs/GeometryGraph.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Polygon.h>
#include <trajectory_msgs/JointTrajectory.h>

namespace moveit_visual_tools
{

// Default constants
static const std::string ROBOT_DESCRIPTION = "robot_description";
static const std::string COLLISION_TOPIC = "/collision_object";
static const std::string ATTACHED_COLLISION_TOPIC = "/attached_collision_object";
static const std::string RVIZ_MARKER_TOPIC = "/end_effector_marker";
static const std::string PLANNING_SCENE_TOPIC = "/move_group/monitored_planning_scene";
static const std::string DISPLAY_PLANNED_PATH_TOPIC = "/move_group/display_planned_path";
static const std::string DISPLAY_ROBOT_STATE_TOPIC = "/move_group/robot_state";

enum rviz_colors { RED, GREEN, BLUE, GREY, WHITE, ORANGE, BLACK, YELLOW, TRANSLUCENT, RAND };
enum rviz_scales { XXSMALL, XSMALL, SMALL, REGULAR, LARGE, XLARGE, XXLARGE };

class VisualTools
{
protected:

  // A shared node handle
  ros::NodeHandle nh_;

  // ROS publishers
  ros::Publisher pub_rviz_marker_; // for rviz visualization markers
  ros::Publisher pub_collision_obj_; // for MoveIt collision objects
  ros::Publisher pub_attach_collision_obj_; // for MoveIt attached objects
  ros::Publisher pub_display_path_; // for MoveIt trajectories
  ros::Publisher pub_planning_scene_diff_; // for adding and removing collision objects
  ros::Publisher pub_robot_state_; // publish a RobotState message

  // Pointer to a Planning Scene Monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  robot_model_loader::RobotModelLoaderPtr rm_loader_; // so that we can specify our own options

  // Strings
  std::string marker_topic_; // topic to publish to rviz
  std::string base_link_; // name of base link of robot

  // TODO rename this
  double floor_to_base_height_; // allows an offset between base link and floor where objects are built

  // Duration to have Rviz markers persist, 0 for infinity
  ros::Duration marker_lifetime_;

  // End Effector Markers
  visualization_msgs::MarkerArray ee_marker_array_;
  tf::Pose tf_root_to_link_;
  geometry_msgs::Pose grasp_pose_to_eef_pose_; // Convert generic grasp pose to this end effector's frame of reference
  std::vector<geometry_msgs::Pose> marker_poses_;

  // Library settings
  bool muted_; // Whether to actually publish to rviz or not
  double alpha_; // opacity of all markers
  double global_scale_; // allow all markers to be increased by a constanct factor

  // Cached Rviz markers
  visualization_msgs::Marker arrow_marker_;
  visualization_msgs::Marker sphere_marker_;
  visualization_msgs::Marker block_marker_;
  visualization_msgs::Marker cylinder_marker_;
  visualization_msgs::Marker text_marker_;
  visualization_msgs::Marker rectangle_marker_;
  visualization_msgs::Marker line_marker_;
  visualization_msgs::Marker path_marker_;
  visualization_msgs::Marker spheres_marker_;
  visualization_msgs::Marker reset_marker_;

  // MoveIt cached markers
  moveit_msgs::DisplayRobotState display_robot_msg_;

  // MoveIt cached objects
  robot_state::RobotStatePtr shared_robot_state_; // Note: call loadSharedRobotState() before using this
  robot_state::RobotModelConstPtr robot_model_;

private:
  /**
   * \brief Shared function for initilization by constructors
   */
  void initialize();

public:

  /**
   * \brief Constructor
   * \param base_link - common base for all visualization markers, usually "base_link"
   * \param marker_topic - rostopic to publish markers to - your Rviz display should match
   * \param planning_scene_monitor - optionally pass in a pre-loaded planning scene monitor to avoid having to re-load
   *        the URDF, kinematic solvers, etc
   */
  VisualTools(const std::string& base_link,
              const std::string& marker_topic,
              planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor);

  /**
   * \brief Constructor
   * \param base_link - common base for all visualization markers, usually "base_link"
   * \param marker_topic - rostopic to publish markers to - your Rviz display should match
   * \param robot_model - load robot model pointer so that we don't have do re-parse it here
   */
  VisualTools(const std::string& base_link,
              const std::string& marker_topic = RVIZ_MARKER_TOPIC,
              robot_model::RobotModelConstPtr robot_model = robot_model::RobotModelConstPtr());

  /**
   * \brief Deconstructor
   */
  ~VisualTools() {};

  /**
   * \brief Tell Rviz to clear all markers on a particular display. Note: only works on ROS Indigo and newer
   */
  void deleteAllMarkers();

  /**
   * \brief Reset the id's of all published markers so that they overwrite themselves in the future
   *        NOTE you may prefer deleteAllMarkers()
   */
  void resetMarkerCounts();

  /**
   * \brief Pre-load rviz markers for better efficiency
   * \return converted pose   * \return true on sucess
   */
  bool loadRvizMarkers();

  /**
   * \brief Load a planning scene monitor if one was not passed into the constructor
   * \return true if successful in loading
   */
  bool loadPlanningSceneMonitor();

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
   * \brief Call this once at begining to load the robot marker
   * \param
   * \param
   * \return true if it is successful
   */
  bool loadEEMarker(const std::string& ee_group_name, const std::string& planning_group);

  /**
   * \brief Load publishers as needed
   */
  void loadMarkerPub();
  void loadCollisionPub();
  void loadAttachedPub();
  void loadPlanningPub();
  void loadTrajectoryPub();
  void loadRobotStatePub(const std::string &marker_topic = DISPLAY_ROBOT_STATE_TOPIC);

  /**
   * \brief Return if we are in verbose mode
   */
  bool isMuted()
  {
    return muted_;
  }

  /**
   * \brief Set this class to not actually publish anything to Rviz.
   * \param muted true if verbose
   */
  void setMuted(bool muted)
  {
    muted_ = muted;
  }

  /**
   * \brief Allows an offset between base link and floor where objects are built. Default is zero
   * \param floor_to_base_height - the offset
   */
  void setFloorToBaseHeight(double floor_to_base_height);

  /**
   * \brief Convert generic grasp pose to this end effector's frame of reference
   * \param pose - the transform
   */
  void setGraspPoseToEEFPose(geometry_msgs::Pose grasp_pose_to_eef_pose);

  /**
   * \brief Change the transparency of all markers published
   * \param alpha - value 0 - 1 where 0 is invisible
   */
  void setAlpha(double alpha)
  {
    alpha_ = alpha;
  }

  /**
   * \brief Set the lifetime of markers published to rviz
   * \param lifetime seconds of how long to show markers. 0 for inifinity
   */
  void setLifetime(double lifetime);

  /**
   * \brief Get the RGB value of standard colors
   * \param color - an enum pre-defined name of a color
   * \return the RGB message equivalent
   */
  std_msgs::ColorRGBA getColor(const rviz_colors &color);

  /**
   * \brief Get the rviz marker scale of standard sizes
   * \param scale - an enum pre-defined name of a size
   * \param arrow_scale - they do not have an even scaling, compensate
   * \param marker_scale - amount to scale the scale for accounting for different types of markers
   * \return vector of 3 scales
   */
  geometry_msgs::Vector3 getScale(const rviz_scales &scale, bool arrow_scale = false, double marker_scale = 1.0);

  /**
   * \brief Create a vector that points from point a to point b
   * \param point a - x,y,z in space of a point
   * \param point b - x,y,z in space of a point
   * \return vector from a to b
   */
  Eigen::Affine3d getVectorBetweenPoints(Eigen::Vector3d a, Eigen::Vector3d b);

  /**
   * \brief Find the center between to points
   * \param point a - x,y,z in space of a point
   * \param point b - x,y,z in space of a point
   * \return center point
   */
  Eigen::Vector3d getCenterPoint(Eigen::Vector3d a, Eigen::Vector3d b);

  /**
   * \brief Get the base frame
   * \return name of base frame
   */
  const std::string getBaseLink()
  {
    return base_link_;
  }

  /**
   * \brief Getter for the global scale used for changing size of all markers
   */
  double getGlobalScale()
  {
    return global_scale_;
  }

  /**
   * \brief Setter for the global scale used for changing size of all markers
   */
  void setGlobalScale(double global_scale)
  {
    global_scale_ = global_scale;
  }

  /**
   * \brief Publish an end effector to rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \return true on success
   */
  bool publishEEMarkers(const geometry_msgs::Pose &pose, const rviz_colors &color = WHITE, const std::string &ns="end_effector");

  /**
   * \brief Publish a marker of a sphere to rviz
   * \param pose - the location to publish the sphere with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishSphere(const Eigen::Affine3d &pose, const rviz_colors color = BLUE, const rviz_scales scale = REGULAR, const std::string& ns = "Sphere");
  bool publishSphere(const Eigen::Vector3d &point, const rviz_colors color = BLUE, const rviz_scales scale = REGULAR, const std::string& ns = "Sphere");
  bool publishSphere(const geometry_msgs::Point &point, const rviz_colors color = BLUE, const rviz_scales scale = REGULAR, const std::string& ns = "Sphere");
  bool publishSphere(const geometry_msgs::Pose &pose, const rviz_colors color = BLUE, const rviz_scales scale = REGULAR, const std::string& ns = "Sphere");

  /**
   * \brief Publish a marker of an arrow to rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishArrow(const Eigen::Affine3d &pose, const rviz_colors color = BLUE, const rviz_scales scale = REGULAR);
  bool publishArrow(const geometry_msgs::Pose &pose, const rviz_colors color = BLUE, const rviz_scales scale = REGULAR);

  /**
   * \brief Publish a marker of rectangle to rviz
   * \param point1 - x,y,z top corner location of box
   * \param point2 - x,y,z bottom opposite corner location of box
   * \param color - an enum pre-defined name of a color
   * \return true on success
   */
  bool publishRectangle(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2, const rviz_colors color = BLUE);

  /**
   * \brief Publish a marker of line to rviz
   * \param point1 - x,y,z of start of line
   * \param point2 - x,y,z of end of line
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishLine(const geometry_msgs::Point &point1, const geometry_msgs::Point &point2,
                   const rviz_colors color = BLUE, const rviz_scales scale = REGULAR);

  /**
   * \brief Publish a marker of a series of connected lines to rviz
   * \param path - a series of points to connect with lines
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishPath(const std::vector<geometry_msgs::Point> &path, const rviz_colors color = RED, const rviz_scales scale = REGULAR,
                   const std::string& ns = "Path");

  /**
   * \brief Publish a marker of a polygon to Rviz
   * \param polygon - a series of points to connect with lines
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishPolygon(const geometry_msgs::Polygon &polygon, const rviz_colors color = RED, const rviz_scales scale = REGULAR,
                      const std::string& ns = "Polygon");

  /**
   * \brief Publish a marker of a block to Rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param size - height=width=depth=size
   * \return true on success
   */
  bool publishBlock(const geometry_msgs::Pose &pose, const rviz_colors color = BLUE, const double &block_size = 0.1);

  /**
   * \brief Publish a marker of a cylinder to Rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \param color - an enum pre-defined name of a color
   * \param height - geometry of cylinder
   * \param radius - geometry of cylinder
   * \return true on success
   */
  bool publishCylinder(const geometry_msgs::Pose &pose, const rviz_colors color = BLUE, double height = 0.1, double radius = 0.1);

  /**
   * \brief Publish a graph
   * \param graph of nodes and edges
   * \param color - an enum pre-defined name of a color
   * \param radius - width of cylinders
   * \return true on success
   */
  bool publishGraph(const graph_msgs::GeometryGraph &graph, const rviz_colors color, double radius);


  /**
   * \brief Publish a marker of a series of spheres to rviz
   * \param spheres - where to publish them
   * \param color - an enum pre-defined name of a color
   * \param scale - an enum pre-defined name of a size
   * \param ns - namespace of marker
   * \return true on success
   */
  bool publishSpheres(const std::vector<geometry_msgs::Point> &points, const rviz_colors color = BLUE, const rviz_scales scale = REGULAR,
                      const std::string& ns = "Spheres");

  /**
   * \brief Publish a marker of a text to Rviz
   * \param pose - the location to publish the marker with respect to the base frame
   * \param text - what to display
   * \param color - an enum pre-defined name of a colo
   * \param scale - an enum pre-defined name of a size
   * \return true on success
   */
  bool publishText(const geometry_msgs::Pose &pose, const std::string &text,
                   const rviz_colors &color = WHITE, const rviz_scales scale = REGULAR);

  /**
   * \brief Publish a visualization_msgs Marker of a custom type. Allows reuse of the ros publisher
   * \param marker - a pre-made marker ready to be published
   * \return true on success
   */
  bool publishMarker(const visualization_msgs::Marker &marker);

  /**
   * \brief Show grasps generated from moveit_simple_grasps or other MoveIt Grasp message sources
   * \param possible_grasps - a set of grasp positions to visualize
   * \param ee_parent_link - end effector's attachment link
   */
  bool publishGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps,
                     const std::string &ee_parent_link);

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
   *        Note: this is published to the 'Planned Path' section of the 'MotionPlanning' display in Rviz
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
  bool removeAllCollisionObjects();

  /**
   * \brief Remove all collision objects that this class has added to the MoveIt! planning scene
   *        Communicates directly to a planning scene monitor e.g. if this is the move_group node
   * \param  the scene to directly clear the collision objects from
   * \return true on sucess
   */
  bool removeAllCollisionObjects(planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

  /**
   * \brief Remove a collision object from the planning scene
   * \param Name of object
   * \return true on sucess
   */
  bool cleanupCO(std::string name);

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
   * \brief Create a MoveIt Collision block at the given pose
   * \param pose - location of center of block
   * \param name - semantic name of MoveIt collision object
   * \param size - height=width=depth=size
   * \return true on sucess
   **/
  bool publishCollisionBlock(geometry_msgs::Pose block_pose, std::string block_name, double block_size);

  /**
   * \brief Create a MoveIt Collision cylinder between two points
   * \param point a - x,y,z in space of a point
   * \param point b - x,y,z in space of a point
   * \param name - semantic name of MoveIt collision object
   * \param radius - size of cylinder
   * \return true on sucess
   */
  bool publishCollisionCylinder(geometry_msgs::Point a, geometry_msgs::Point b, std::string object_name, double radius);
  bool publishCollisionCylinder(Eigen::Vector3d a, Eigen::Vector3d b, std::string object_name, double radius);

  /**
   * \brief Create a MoveIt Collision cylinder with a center point pose
   * \param pose - vector pointing along axis of cylinder
   * \param name - semantic name of MoveIt collision object
   * \param radius - size of cylinder
   * \param height - size of cylinder
   * \return true on sucess
   */
  bool publishCollisionCylinder(Eigen::Affine3d object_pose, std::string object_name, double radius, double height);
  bool publishCollisionCylinder(geometry_msgs::Pose object_pose, std::string object_name, double radius, double height);

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
  bool publishCollisionSceneFromFile(const std::string &path, planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor);

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

  /**
   * \brief Run a simple test of all visual_tool's features
   * \return true on success
   */
  bool publishTest();

  /**
   * \brief Convert an Eigen pose to a geometry_msg pose
   *        Note: NOT memory efficient
   * \param pose
   * \return converted pose
   */
  static geometry_msgs::Pose convertPose(const Eigen::Affine3d &pose);

  /**
   * \brief Convert a geometry_msg pose to an Eigen pose
   *        Note: NOT memory efficient
   * \param pose
   * \return converted pose
   */
  static Eigen::Affine3d convertPose(const geometry_msgs::Pose &pose);

  /**
   * \brief Convert a geometry_msg point (32bit) to an Eigen pose
   *        Note: NOT memory efficient
   * \param pose
   * \return converted point with default rotation matrix
   */
  static Eigen::Affine3d convertPoint32ToPose(const geometry_msgs::Point32 &point);

  /**
   * \brief Convert an Eigen pose to a geometry_msg point
   *        Note: NOT memory efficient
   * \param pose
   * \return converted point with orientation discarded
   */
  static geometry_msgs::Point convertPoseToPoint(const Eigen::Affine3d &pose);

  /**
   * \brief Convert a geometry_msg point to an Eigen point
   *        Note: NOT memory efficient
   * \param point
   * \return converted pose
   */
  static Eigen::Vector3d convertPoint(const geometry_msgs::Point &point);

  /**
   * \brief Convert a geometry_msg point to an Eigen point
   *        Note: NOT memory efficient
   * \param point
   * \return converted pose
   */
  static Eigen::Vector3d convertPoint32(const geometry_msgs::Point32 &point);

  /**
   * \brief Convert an Eigen point to a 32 bit geometry_msg point
   *        Note: NOT memory efficient
   * \param point
   * \return converted pose
   */
  static geometry_msgs::Point32 convertPoint32(const Eigen::Vector3d &point);

  /**
   * \brief Create a random pose
   * \param Pose to fill in
   */
  void generateRandomPose(geometry_msgs::Pose& pose);

  /**
   * \brief Get random between min and max
   */
  static double dRand(double dMin, double dMax);
  static float fRand(float dMin, float dMax);
  static int iRand(int dMin, int dMax);

  /**
   * \brief Debug variables to console
   */
  void print();

private:

  /**
   * \brief Get the end effector parent link as loaded from the SRDF
   * \return string of name of end effector parent link
   */
  const std::string& getEEParentLink();

  /**
   * @brief Get the planning scene monitor that this class is using
   * @return a ptr to a planning scene
   */
  planning_scene_monitor::PlanningSceneMonitorPtr getPlanningSceneMonitor();


}; // class

typedef boost::shared_ptr<VisualTools> VisualToolsPtr;
typedef boost::shared_ptr<const VisualTools> VisualToolsConstPtr;

} // namespace

#endif
