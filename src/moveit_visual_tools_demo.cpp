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

/* Author: Dave Coleman
   Desc:   Demo implementation of moveit_visual_tools
           To use, add a Rviz Marker Display subscribed to topic /moveit_visual_tools
*/

// ROS
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// For visualizing things in rviz
#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// C++
#include <string>

// Boost
#include <boost/lexical_cast.hpp>

using namespace std::literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_visual_tools_demo");

namespace rvt = rviz_visual_tools;

namespace moveit_visual_tools
{
static const std::string PLANNING_GROUP_NAME = "arm";  // RRBot Specific
static const std::string THIS_PACKAGE = "moveit_visual_tools";

class VisualToolsDemo
{
public:
  /**
   * \brief Constructor
   */
  VisualToolsDemo(const rclcpp::Node::SharedPtr& node) : node_(node)
  {
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(node_, "world", "/moveit_visual_tools");
    visual_tools_->loadPlanningSceneMonitor();
    visual_tools_->loadMarkerPub(true);
    visual_tools_->loadRobotStatePub("display_robot_state");
    visual_tools_->setManualSceneUpdating();

    robot_state_ = visual_tools_->getSharedRobotState();
    jmg_ = robot_state_->getJointModelGroup(PLANNING_GROUP_NAME);
    // Allow time to publish messages
    rclcpp::spin_some(node);
    rclcpp::sleep_for(1000ms);

    // Clear collision objects and markers
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->triggerPlanningSceneUpdate();
    rclcpp::sleep_for(1000ms);

    // Show message
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 4;
    visual_tools_->publishText(text_pose, "MoveIt Visual Tools", rvt::WHITE, rvt::XLARGE, /*static_id*/ false);

    runRobotStateTests();

    runDeskTest();

    runCollisionObjectTests();
  }

  void publishLabelHelper(const Eigen::Isometry3d& pose, const std::string& label)
  {
    Eigen::Isometry3d pose_copy = pose;
    pose_copy.translation().x() -= 0.2;
    visual_tools_->publishText(pose_copy, label, rvt::WHITE, rvt::XLARGE, false);
    visual_tools_->trigger();
  }

  void runRobotStateTests()
  {
    // Show 5 random robot states
    RCLCPP_INFO(LOGGER, "Showing 5 random robot states");
    for (std::size_t i = 0; i < 5; ++i)
    {
      robot_state_->setToRandomPositions(jmg_);
      visual_tools_->publishRobotState(robot_state_, rvt::DEFAULT);
      rclcpp::sleep_for(1000ms);
    }

    // Show 5 robot state in different colors
    RCLCPP_INFO(LOGGER, "Showing 5 random robot states in different colors");
    for (std::size_t i = 0; i < 5; ++i)
    {
      robot_state_->setToRandomPositions(jmg_);
      visual_tools_->publishRobotState(robot_state_, visual_tools_->getRandColor());
      rclcpp::sleep_for(1000ms);
    }

    // Hide the robot
    RCLCPP_INFO(LOGGER, "Hiding the robot");
    visual_tools_->hideRobot();
    rclcpp::sleep_for(1000ms);

    // Show the robot
    RCLCPP_INFO(LOGGER, "Showing the robot");
    visual_tools_->publishRobotState(robot_state_, rvt::DEFAULT);
    rclcpp::sleep_for(1000ms);
  }

  void runDeskTest()
  {
    double common_angle = M_PI * 1.1;
    double x_offset = -3.0;

    // --------------------------------------------------------------------
    RCLCPP_INFO_STREAM(LOGGER, "Moving the robot");
    Eigen::Isometry3d robot_pose = Eigen::Isometry3d::Identity();
    robot_pose = robot_pose * Eigen::AngleAxisd(common_angle, Eigen::Vector3d::UnitZ());
    robot_pose.translation().x() = x_offset;
    visual_tools_->applyVirtualJointTransform(*robot_state_, robot_pose);
    visual_tools_->publishRobotState(robot_state_, rvt::DEFAULT);
    rclcpp::sleep_for(1000ms);

    // --------------------------------------------------------------------
    RCLCPP_INFO_STREAM(LOGGER, "Publishing Collision Floor");
    visual_tools_->publishCollisionFloor(-0.5, "Floor", rvt::GREY);
    rclcpp::sleep_for(1000ms);

    // --------------------------------------------------------------------
    RCLCPP_INFO_STREAM(LOGGER, "Publishing Collision Wall");
    double wall_x = x_offset - 1.0;
    double wall_y = -1.0;
    double wall_width = 6.0;
    double wall_height = 4;
    visual_tools_->publishCollisionWall(wall_x, wall_y, common_angle, wall_width, wall_height, "Wall", rvt::GREEN);
    rclcpp::sleep_for(1000ms);

    // --------------------------------------------------------------------
    RCLCPP_INFO_STREAM(LOGGER, "Publishing Collision Table");
    double table_x = x_offset + 1.0;
    double table_y = 0;
    double table_z = 0;
    double table_width = 3;
    double table_height = 1;
    double table_depth = 1;
    visual_tools_->publishCollisionTable(table_x, table_y, table_z, common_angle, table_width, table_height,
                                         table_depth, "Table", rvt::BLUE);
    rclcpp::sleep_for(1000ms);

    // Send ROS messages
    visual_tools_->triggerPlanningSceneUpdate();

    // Show 5 random robot states
    RCLCPP_INFO(LOGGER, "Showing 5 random robot states");
    for (std::size_t i = 0; i < 5; ++i)
    {
      robot_state_->setToRandomPositions(jmg_);
      visual_tools_->publishRobotState(robot_state_, rvt::DEFAULT);
      rclcpp::sleep_for(1000ms);
    }
  }

  void runCollisionObjectTests()
  {
    // Create pose
    Eigen::Isometry3d pose1 = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d pose2 = Eigen::Isometry3d::Identity();

    double space_between_rows = 0.2;
    double y = 0;
    double step;

    // --------------------------------------------------------------------
    RCLCPP_INFO_STREAM(LOGGER, "Publishing Collision Mesh");
    // TODO: Catch exception
    std::string file_path = "file://" + ament_index_cpp::get_package_share_directory(THIS_PACKAGE);
    if (file_path == "file://")
      RCLCPP_FATAL_STREAM(LOGGER, "Unable to get " << THIS_PACKAGE << " package path ");
    file_path.append("/resources/demo_mesh.stl");
    step = 0.4;
    for (double i = 0; i <= 1.0; i += 0.1)
    {
      visual_tools_->publishCollisionMesh(visual_tools_->convertPose(pose1),
                                          "Mesh_" + boost::lexical_cast<std::string>(i), file_path,
                                          visual_tools_->getRandColor());
      if (!i)
        publishLabelHelper(pose1, "Mesh");
      pose1.translation().x() += step;
    }
    rclcpp::sleep_for(1000ms);

    // --------------------------------------------------------------------
    RCLCPP_INFO_STREAM(LOGGER, "Publishing Collision Block");
    pose1 = Eigen::Isometry3d::Identity();
    y += space_between_rows * 2.0;
    pose1.translation().y() = y;
    for (double i = 0; i <= 1.0; i += 0.1)
    {
      double size = 0.1 * (i + 0.5);
      visual_tools_->publishCollisionBlock(visual_tools_->convertPose(pose1),
                                           "Block_" + boost::lexical_cast<std::string>(i), size,
                                           visual_tools_->getRandColor());

      if (!i)
        publishLabelHelper(pose1, "Block");
      pose1.translation().x() += step;
    }
    rclcpp::sleep_for(1000ms);

    // --------------------------------------------------------------------
    RCLCPP_INFO_STREAM(LOGGER, "Publishing Collision Rectanglular Cuboid");
    double cuboid_min_size = 0.05;
    double cuboid_max_size = 0.2;
    pose1 = Eigen::Isometry3d::Identity();
    pose2 = Eigen::Isometry3d::Identity();
    y += space_between_rows * 2.0;
    pose1.translation().y() = y;
    pose2.translation().y() = y;
    for (double i = 0; i <= 1.0; i += 0.1)
    {
      pose2 = pose1;
      pose2.translation().x() += i * cuboid_max_size + cuboid_min_size;
      pose2.translation().y() += (i * cuboid_max_size + cuboid_min_size) / 2.0;
      pose2.translation().z() += i * cuboid_max_size + cuboid_min_size;
      visual_tools_->publishCollisionCuboid(pose1.translation(), pose2.translation(),
                                            "Rectangle_" + boost::lexical_cast<std::string>(i),
                                            visual_tools_->getRandColor());
      if (!i)
        publishLabelHelper(pose1, "Cuboid");
      pose1.translation().x() += step;
    }
    rclcpp::sleep_for(1000ms);

    // --------------------------------------------------------------------
    RCLCPP_INFO_STREAM(LOGGER, "Publishing Collision Cylinder");
    double cylinder_min_size = 0.01;
    double cylinder_max_size = 0.3;
    pose1 = Eigen::Isometry3d::Identity();
    pose2 = Eigen::Isometry3d::Identity();
    y += space_between_rows * 2.0;
    pose1.translation().y() = y;
    pose2.translation().y() = y;
    for (double i = 0; i <= 1.0; i += 0.1)
    {
      double radius = 0.1;
      pose2 = pose1;
      pose2.translation().x() += i * cylinder_max_size + cylinder_min_size;
      // pose2.translation().y() += i * cylinder_max_size + cylinder_min_size;
      // pose2.translation().z() += i * cylinder_max_size + cylinder_min_size;
      visual_tools_->publishCollisionCylinder(pose1.translation(), pose2.translation(),
                                              "Cylinder_" + boost::lexical_cast<std::string>(i), radius,
                                              visual_tools_->getRandColor());

      if (!i)
        publishLabelHelper(pose1, "Cylinder");
      pose1.translation().x() += step;

      pose1 = pose1 * Eigen::AngleAxisd(step * 2 * M_PI, Eigen::Vector3d::UnitZ());
    }

    // TODO(davetcoleman): publishCollisionGraph
    // TODO(davetcoleman): publishContactPoint
    // TODO(davetcoleman): trajectory stuff
    // TODO(davetcoleman): gripper stuff

    // Send ROS messages
    visual_tools_->triggerPlanningSceneUpdate();
    rclcpp::sleep_for(1000ms);
  }

private:
  // A shared node handle
  rclcpp::Node::SharedPtr node_;

  // For visualizing things in rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // MoveIt Components
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  const moveit::core::JointModelGroup* jmg_;

  moveit::core::RobotStatePtr robot_state_;
};  // end class

}  // namespace moveit_visual_tools

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("moveit_visual_tools_demo", node_options);
  RCLCPP_INFO_STREAM(LOGGER, "Visual Tools Demo");

  moveit_visual_tools::VisualToolsDemo demo(node);

  RCLCPP_INFO_STREAM(LOGGER, "Shutting down.");
  rclcpp::shutdown();

  return 0;
}
