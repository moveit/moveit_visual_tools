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

/* Author: Dave Coleman
   Desc:   quick test
*/

// ROS
#include <ros/ros.h>
#include <moveit_visual_tools/visualization_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ROS_INFO_STREAM("Test");

  // Load the Robot Viz Tools for publishing to Rviz
  moveit_visual_tools::VisualizationToolsPtr visual_tools_;
  visual_tools_.reset(new moveit_visual_tools::VisualizationTools("/base","/grasp_markers"));
  visual_tools_->setMuted(false);
  visual_tools_->setLifetime(20.0);
  ros::Duration(2).sleep();

  // TEST points
  geometry_msgs::Point pt1;
  pt1.x = 0.1;
  pt1.y = 0.1;
  pt1.z = 0.11;

  geometry_msgs::Point pt2;
  pt2.x = 0.15;
  pt2.y = 0.2;
  pt2.z = 0.3;

  // Visualize line
  visual_tools_->publishLine(pt1, pt2, moveit_visual_tools::GREY, moveit_visual_tools::REGULAR);

  Eigen::Vector3d a, b;
  tf::pointMsgToEigen(pt1,a);
  tf::pointMsgToEigen(pt2,b);

  // Visualize points
  visual_tools_->publishSphere(a, moveit_visual_tools::BLUE, moveit_visual_tools::LARGE);
  visual_tools_->publishSphere(b, moveit_visual_tools::BLUE, moveit_visual_tools::LARGE);

  Eigen::Quaterniond q;
  q.setFromTwoVectors(a, b);

  //Eigen::Affine3d output_vector;
  //output_vector.rotate(q);
  //output_vector.translate(-a);

  //Eigen::Affine3d(q.matrix()) * Eigen::Translationd(-a)
  Eigen::Affine3d output_vector = q * Eigen::Translation3d(-b);

  visual_tools_->publishArrow(output_vector, moveit_visual_tools::RED, moveit_visual_tools::REGULAR);
  ros::Duration(1.0).sleep();

  ROS_INFO_STREAM("Shutting down.");

  return 0;
}

