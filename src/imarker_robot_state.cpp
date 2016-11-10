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
 *   * Neither the name of PickNik LLC nor the names of its
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

// MoveIt
#include <moveit/robot_state/conversions.h>
#include <moveit/transforms/transforms.h>

// Conversions
// #include <eigen_conversions/eigen_msg.h>
// #include <tf_conversions/tf_eigen.h>

// this package
#include <moveit_visual_tools/imarker_robot_state.h>
#include <moveit_visual_tools/imarker_end_effector.h>

// C++
#include <string>
#include <vector>

namespace moveit_visual_tools
{
IMarkerRobotState::IMarkerRobotState(planning_scene_monitor::PlanningSceneMonitorPtr psm,
                                     const std::string &imarker_name, std::vector<ArmData> arm_datas,
                                     rviz_visual_tools::colors color, const std::string &package_path)
  : name_(imarker_name), nh_("~"), psm_(psm), arm_datas_(arm_datas), color_(color), package_path_(package_path)
{
  // Load Visual tools
  visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
                                                                           psm_->getRobotModel()->getModelFrame(), nh_.getNamespace() + "/" + imarker_name, psm_);
  // visual_tools_->setPlanningSceneMonitor(psm_);
  visual_tools_->loadRobotStatePub(nh_.getNamespace() + "/imarker_" + imarker_name + "_state");
  visual_tools_->enableBatchPublishing();

  // Load robot state
  imarker_state_.reset(new moveit::core::RobotState(psm_->getRobotModel()));
  imarker_state_->setToDefaultValues();

  // Create Marker Server
  const std::string imarker_topic = nh_.getNamespace() + "/" + imarker_name + "_imarker";
  imarker_server_.reset(new interactive_markers::InteractiveMarkerServer(imarker_topic, "", false));

  // Get file name
  if (!getFilePath(file_path_, "imarker_" + name_ + ".csv", "config/imarkers"))
    exit(-1);

  // Load previous pose from file
  if (!loadFromFile(file_path_))
    ROS_INFO_STREAM_NAMED(name_, "Unable to find state from file, setting to default");

  // Show initial robot state loaded from file
  publishRobotState();

  // Create each end effector
  end_effectors_.resize(arm_datas_.size());
  for (std::size_t i = 0; i < arm_datas_.size(); ++i)
  {
    std::string eef_name;
    if (i == 0)
      eef_name = imarker_name + "_right";
    else
      eef_name = imarker_name + "_left";

    end_effectors_[i].reset(new IMarkerEndEffector(this, eef_name, arm_datas_[i], color));

    // Create map from eef name to object
    name_to_eef_[eef_name] = end_effectors_[i];
  }

  // After both end effectors have been added, apply on server
  imarker_server_->applyChanges();

  ROS_DEBUG_STREAM_NAMED(name_, "IMarkerRobotState '" << name_ << "' Ready.");
}

bool IMarkerRobotState::loadFromFile(const std::string &file_name)
{
  if (!boost::filesystem::exists(file_name))
  {
    ROS_WARN_STREAM_NAMED(name_, "File not found: " << file_name);
    return false;
  }
  std::ifstream input_file(file_name);

  std::string line;

  if (!std::getline(input_file, line))
  {
    ROS_ERROR_STREAM_NAMED(name_, "Unable to read line");
    return false;
  }

  // Get robot state from file
  moveit::core::streamToRobotState(*imarker_state_, line);

  return true;
}

bool IMarkerRobotState::saveToFile()
{
  output_file_.open(file_path_);
  moveit::core::robotStateToStream(*imarker_state_, output_file_, false);
  output_file_.close();

  return true;
}

void IMarkerRobotState::setIMarkerCallback(IMarkerCallback callback)
{
  for (const IMarkerEndEffectorPtr ee : end_effectors_)
    ee->setIMarkerCallback(callback);
}

void IMarkerRobotState::setRobotState(moveit::core::RobotStatePtr state)
{
  // Do a copy
  *imarker_state_ = *state;

  // Update the imarkers
  for (IMarkerEndEffectorPtr ee : end_effectors_)
    ee->setPoseFromRobotState();
}

void IMarkerRobotState::setToCurrentState()
{
  // Get the real current state
  planning_scene_monitor::LockedPlanningSceneRO scene(psm_);  // Lock planning scene
  (*imarker_state_) = scene->getCurrentState();

  // TODO: move interactive markers and pose

  // Show new state
  visual_tools_->publishRobotState(imarker_state_, color_);
}

bool IMarkerRobotState::setToRandomState()
{
  ROS_ERROR_STREAM_NAMED(name_, "Not reimplemented yet");
  /*
  static const std::size_t MAX_ATTEMPTS = 1000;
  for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
  {
    imarker_state_->setToRandomPositions(jmg_);
    imarker_state_->update();

    if (isStateValid())
    {
      // ROS_DEBUG_STREAM_NAMED(name_, "Found valid random robot state after " << i << " attempts");

      // Get pose from robot state
      for (std::size_t i = 0; i < arm_datas_.size(); ++i)
        end_effectors_[i]->setPoseFromRobotState();

      // Send to imarker
      for (std::size_t i = 0; i < arm_datas_.size(); ++i)
        end_effectors_[i]->sendUpdatedIMarkerPose();

      return true;
    }

    if (i == 100)
      ROS_WARN_STREAM_NAMED(name_, "Taking long time to find valid random state");
  }

  ROS_ERROR_STREAM_NAMED(name_, "Unable to find valid random robot state for imarker");
  exit(-1);
  */
  return false;
}

bool IMarkerRobotState::isStateValid(bool verbose)
{
  // Update transforms
  imarker_state_->update();

  planning_scene_monitor::LockedPlanningSceneRO planning_scene(psm_);  // Read only lock

  // which planning group to collision check, "" is everything
  if (planning_scene->isStateValid(*imarker_state_, "", verbose))
  {
    return true;
  }

  return false;
}

void IMarkerRobotState::publishRobotState()
{
  visual_tools_->publishRobotState(imarker_state_, color_);
}

moveit_visual_tools::MoveItVisualToolsPtr IMarkerRobotState::getVisualTools()
{
  return visual_tools_;
}

bool IMarkerRobotState::getFilePath(std::string &file_path, const std::string &file_name,
                                    const std::string &subdirectory) const

{
  namespace fs = boost::filesystem;

  // Check that the directory exists, if not, create it
  fs::path rootPath = fs::path(package_path_);
  rootPath = rootPath / fs::path(subdirectory);

  boost::system::error_code returnedError;
  fs::create_directories(rootPath, returnedError);

  if (returnedError)
  {
    // did not successfully create directories
    ROS_ERROR("Unable to create directory %s", subdirectory.c_str());
    return false;
  }

  // directories successfully created, append the group name as the file name
  rootPath = rootPath / fs::path(file_name);
  file_path = rootPath.string();
  // ROS_DEBUG_STREAM_NAMED(name_, "Config file: " << file_path);

  return true;
}

bool IMarkerRobotState::setFromPoses(const EigenSTL::vector_Affine3d poses, const moveit::core::JointModelGroup *group)
{
  std::vector<std::string> tips;
  for (std::size_t i = 0; i < arm_datas_.size(); ++i)
    tips.push_back(arm_datas_[i].ee_link_->getName());

  std::cout << "First pose should be for joint model group: " << arm_datas_[0].ee_link_->getName() << std::endl;

  const std::size_t attempts = 10;
  const double timeout = 1.0 / 30.0;  // 30 fps

  // Optionally collision check
  moveit::core::GroupStateValidityCallbackFn constraint_fn;
  if (true)
  {
    bool collision_checking_verbose_ = false;
    bool only_check_self_collision_ = false;

    // TODO(davetcoleman): this is currently not working, the locking seems to cause segfaults
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRO> ls;
    ls.reset(new planning_scene_monitor::LockedPlanningSceneRO(psm_));
    constraint_fn = boost::bind(&isIKStateValid, static_cast<const planning_scene::PlanningSceneConstPtr &>(*ls).get(),
                                collision_checking_verbose_, only_check_self_collision_, visual_tools_, _1, _2, _3);
  }

  // Solve
  std::size_t outer_attempts = 20;
  for (std::size_t i = 0; i < outer_attempts; ++i)
  {
    if (!imarker_state_->setFromIK(group, poses, tips, attempts, timeout, constraint_fn))
    {
      ROS_DEBUG_STREAM_NAMED(name_, "Failed to find dual arm pose, trying again");

      // Re-seed
      imarker_state_->setToRandomPositions(group);
    }
    else
    {
      ROS_INFO_STREAM_NAMED(name_, "Found solution");

      // Visualize robot
      publishRobotState();

      // Update the imarkers
      for (IMarkerEndEffectorPtr ee : end_effectors_)
        ee->setPoseFromRobotState();

      return true;
    }
  }

  ROS_ERROR_STREAM_NAMED(name_, "Failed to find dual arm pose");
  return false;
}

}  // namespace moveit_visual_tools

namespace
{
bool isIKStateValid(const planning_scene::PlanningScene *planning_scene, bool verbose, bool only_check_self_collision,
                  moveit_visual_tools::MoveItVisualToolsPtr visual_tools, moveit::core::RobotState *robot_state,
                  const moveit::core::JointModelGroup *group, const double *ik_solution)
{
  // Apply IK solution to robot state
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();

  // Ensure there are objects in the planning scene
  if (false)
  {
    const std::size_t num_collision_objects = planning_scene->getCollisionWorld()->getWorld()->size();
    if (num_collision_objects == 0)
    {
      ROS_ERROR_STREAM_NAMED("imarker_robot_state", "No collision objects exist in world, you need at least a table "
                                                  "modeled for the controller to work");
      ROS_ERROR_STREAM_NAMED("imarker_robot_state", "To fix this, relaunch the teleop/head tracking/whatever MoveIt! "
                                                  "node to publish the collision objects");
      return false;
    }
  }

  if (!planning_scene)
  {
    ROS_ERROR_STREAM_NAMED("imarker_robot_state", "No planning scene provided");
    return false;
  }
  if (only_check_self_collision)
  {
    // No easy API exists for only checking self-collision, so we do it here.
    // TODO(davetcoleman): move this into planning_scene.cpp
    collision_detection::CollisionRequest req;
    req.verbose = verbose;
    req.group_name = group->getName();
    collision_detection::CollisionResult res;
    planning_scene->checkSelfCollision(req, res, *robot_state);
    if (!res.collision)
      return true;  // not in collision
  }
  else if (!planning_scene->isStateColliding(*robot_state, group->getName()))
    return true;  // not in collision

  // Display more info about the collision
  if (verbose)
  {
    visual_tools->publishRobotState(*robot_state, rviz_visual_tools::RED);
    planning_scene->isStateColliding(*robot_state, group->getName(), true);
    visual_tools->publishContactPoints(*robot_state, planning_scene);
    ROS_WARN_STREAM_THROTTLE_NAMED(2.0, "imarker_robot_state", "Collision in IK CC callback");
  }

  return false;
}

}  // end annonymous namespace
