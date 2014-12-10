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

/* Author: Dave Coleman
   Desc:   Demo implementation of moveit_visual_tools
           To use, add a Rviz Marker Display subscribed to topic /moveit_visual_tools
*/

// ROS
#include <ros/ros.h>

// For visualizing things in rviz
#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace moveit_visual_tools
{

// Baxter specific
static const std::string EE_PARENT_LINK = "right_wrist";
static const std::string PLANNING_GROUP_NAME = "right_arm_torso_grasping";
static const std::string EE_GROUP = "right_hand";

class VisualToolsTest
{
private:

  // A shared node handle
  ros::NodeHandle nh_;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // MoveIt Components
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

public:

  /**
   * \brief Constructor
   */
  VisualToolsTest()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base","/moveit_visual_tools"));
    visual_tools_->loadPlanningSceneMonitor();

    // Allow time to publish messages
    ros::spinOnce();
    ros::Duration(1.0).sleep();

    // Run through test
    visual_tools_->publishCollisionTests();
  }

  /**
   * \brief Destructor
   */
  ~VisualToolsTest()
  {
  }

  bool loadPlanningSceneMonitor()
  {
    // Allows us to sycronize to Rviz and also publish collision objects to ourselves
    ROS_DEBUG_STREAM_NAMED("r2_demos","Loading Planning Scene Monitor");
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor(ROBOT_DESCRIPTION, boost::shared_ptr<tf::Transformer>()));
    ros::spinOnce();
    ros::Duration(0.5).sleep();

    if (planning_scene_monitor_->getPlanningScene())
    {
      // Optional monitors to start:
      bool use_octomap_monitor = false; // this prevents a /tf warning
      planning_scene_monitor_->startWorldGeometryMonitor(planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
                                                         planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                                                         use_octomap_monitor);
      //planning_scene_monitor_->startSceneMonitor("/move_group/monitored_planning_scene");
      //planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "/planning_scene");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("r2_demos","Planning scene not configured");
      return false;
    }
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    ros::spinOnce();

    visual_tools_->setPlanningSceneMonitor(planning_scene_monitor_);

    return true;
  }

}; // end class

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_tools_test");
  ROS_INFO_STREAM("Visual Tools Test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit_visual_tools::VisualToolsTest tester;

  ROS_INFO_STREAM("Shutting down.");

  return 0;
}
