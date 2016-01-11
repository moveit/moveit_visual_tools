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

class VisualToolsDemo
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
  VisualToolsDemo()
  {
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base","/moveit_visual_tools"));
    visual_tools_->loadPlanningSceneMonitor();

    // Allow time to publish messages
    ros::spinOnce();
    ros::Duration(5.0).sleep();

    // Create pose
    geometry_msgs::Pose pose1;
    geometry_msgs::Pose pose2;

    // Demo all collision shapes ----------
    for (std::size_t i = 0; i < 10; ++i)
    {
      if (!ros::ok())
        break;

      ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Mesh");
      visual_tools_->generateRandomPose(pose1);
      static const std::string package = "moveit_visual_tools";
      std::string path = "file://" + ros::package::getPath(package);
      if( path == "file://" )
        ROS_FATAL_STREAM_NAMED("visual_tools", "Unable to get " << package << " package path " );
      path.append("/resources/demo_mesh.stl");
      visual_tools_->publishCollisionMesh(pose1, "Mesh", path, rviz_visual_tools::RAND);
      ros::Duration(1.0).sleep();

      ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Block");
      visual_tools_->generateRandomPose(pose1);
      visual_tools_->publishCollisionBlock(pose1, "Block", 0.1, rviz_visual_tools::RAND);
      ros::Duration(1.0).sleep();

      ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Rectanglular Cuboid");
      visual_tools_->generateRandomPose(pose1);
      visual_tools_->generateRandomPose(pose2);
      visual_tools_->publishCollisionCuboid(pose1.position, pose2.position, "Rectangle", rviz_visual_tools::RAND);
      ros::Duration(1.0).sleep();

      ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Floor");
      visual_tools_->publishCollisionFloor(0, "Floor", rviz_visual_tools::RAND);
      ros::Duration(1.0).sleep();

      ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Cylinder");
      visual_tools_->generateRandomPose(pose1);
      visual_tools_->generateRandomPose(pose2);
      visual_tools_->publishCollisionCylinder(pose1.position, pose2.position, "Cylinder", 0.1, rviz_visual_tools::RAND);
      ros::Duration(1.0).sleep();

      // TODO: test publishCollisionGraph

      ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Wall");
      visual_tools_->generateRandomPose(pose1);
      visual_tools_->publishCollisionWall(pose1.position.x, pose1.position.y, 0, 1, "Wall", rviz_visual_tools::RAND);
      ros::Duration(1.0).sleep();

      ROS_INFO_STREAM_NAMED("visual_tools","Publishing Collision Table");
      visual_tools_->generateRandomPose(pose1);
      visual_tools_->publishCollisionTable(pose1.position.x, pose1.position.y, 0, 0.5, 0.5, 0.5, "Table", rviz_visual_tools::RAND);
      ros::Duration(1.0).sleep();
    }
  }

  /**
   * \brief Destructor
   */
  ~VisualToolsDemo()
  {
  }

}; // end class

} // end namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_tools_demo");
  ROS_INFO_STREAM("Visual Tools Demo");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit_visual_tools::VisualToolsDemo demoer;

  ROS_INFO_STREAM("Shutting down.");

  return 0;
}
