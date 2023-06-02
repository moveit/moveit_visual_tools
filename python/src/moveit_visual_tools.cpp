#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <rviz_visual_tools/rviz_visual_tools.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <rviz_visual_tools/binding_utils.hpp>

namespace py = pybind11;
using py::literals::operator""_a;

namespace moveit_visual_tools
{
PYBIND11_MODULE(pymoveit_visual_tools, m)
{
  py::module::import("rviz_visual_tools.pyrviz_visual_tools");
  py::module::import("moveit.core.robot_state");
  py::module::import("moveit.core.robot_model");
  py::module::import("moveit.core.planning_scene");

  py::class_<MoveItVisualTools, rviz_visual_tools::RvizVisualTools>(m, "MoveItVisualTools")
      .def(py::init(
          [](const rviz_visual_tools::RvizVisualToolsNode::SharedPtr& node) { return MoveItVisualTools(node); }))
      .def(py::init([](const rviz_visual_tools::RvizVisualToolsNode::SharedPtr& node, const std::string& base_frame,
                       const std::string& marker_topic) { return MoveItVisualTools(node, base_frame, marker_topic); }),
           "node"_a, "base_frame"_a, "marker_topic"_a = rviz_visual_tools::RVIZ_MARKER_TOPIC)
      .def("set_robot_state_topic", &MoveItVisualTools::setRobotStateTopic)
      .def("set_planning_scene_topic", &MoveItVisualTools::setPlanningSceneTopic)
      .def("load_planning_scene_monitor", &MoveItVisualTools::loadPlanningSceneMonitor)
      .def("process_collision_object_msg", &MoveItVisualTools::processCollisionObjectMsg, "msg"_a,
           "color"_a = rviz_visual_tools::Colors::GREEN)
      .def("process_attached_collision_object_msg", &MoveItVisualTools::processAttachedCollisionObjectMsg)
      .def("move_collision_object",
           py::overload_cast<const geometry_msgs::msg::Pose&, const std::string&, const rviz_visual_tools::Colors&>(
               &MoveItVisualTools::moveCollisionObject))
      .def("trigger_planning_scene_update", &MoveItVisualTools::triggerPlanningSceneUpdate)
      .def("load_shared_robot_state", &MoveItVisualTools::loadSharedRobotState)
      .def("get_shared_robot_state", &MoveItVisualTools::getSharedRobotState)
      .def("get_root_robot_state", &MoveItVisualTools::getSharedRobotState)
      .def("get_robot_model", &MoveItVisualTools::getRobotModel)
      .def("load_ee_marker", &MoveItVisualTools::loadEEMarker)
      .def("load_trajectory_publisher", &MoveItVisualTools::loadTrajectoryPub)
      .def("load_robot_state_publisher", &MoveItVisualTools::loadRobotStatePub, "topic"_a, "blocking"_a = true)
      .def("set_manual_scene_updating", &MoveItVisualTools::setManualSceneUpdating, "enable_manual"_a = true)
      .def("publish_ee_markers",
           py::overload_cast<const geometry_msgs::msg::Pose&, const moveit::core::JointModelGroup*,
                             const std::vector<double>&, const rviz_visual_tools::Colors&, const std::string&>(
               &MoveItVisualTools::publishEEMarkers),
           "pose"_a, "ee_jmq"_a, "ee_joint_pos"_a = std::vector<double>(),
           "color"_a = rviz_visual_tools::Colors::DEFAULT, "ns"_a = "end_effector")
      .def("publish_ik_solutions", &MoveItVisualTools::publishIKSolutions, "ik_solutions"_a, "arm_jmg"_a,
           "display_time"_a = 0.4)
      .def("remove_all_collision_objects", &MoveItVisualTools::removeAllCollisionObjects)
      .def("cleanup_collision_object", &MoveItVisualTools::cleanupCO)
      .def("cleanup_attached_collision_object", &MoveItVisualTools::cleanupACO)
      .def("attach_collision_object", &MoveItVisualTools::attachCO)
      .def("publish_collision_floor", &MoveItVisualTools::publishCollisionFloor, "z"_a = 0.0, "plane_name"_a = "Floor",
           "color"_a = rviz_visual_tools::Colors::GREEN)
      .def("publish_collision_block", &MoveItVisualTools::publishCollisionBlock, "pose"_a, "name"_a = "block",
           "size"_a = 0.1, "color"_a = rviz_visual_tools::Colors::GREEN)
      .def("publish_collision_cuboid",
           py::overload_cast<const geometry_msgs::msg::Point&, const geometry_msgs::msg::Point&, const std::string&,
                             const rviz_visual_tools::Colors&>(&MoveItVisualTools::publishCollisionCuboid))
      .def("publish_collision_cuboid",
           py::overload_cast<const geometry_msgs::msg::Pose&, double, double, double, const std::string&,
                             const rviz_visual_tools::Colors&>(&MoveItVisualTools::publishCollisionCuboid))
      .def("publish_collision_cylinder",
           py::overload_cast<const geometry_msgs::msg::Pose&, const std::string&, double, double,
                             const rviz_visual_tools::Colors&>(&MoveItVisualTools::publishCollisionCylinder),
           "object_pose"_a, "object_name"_a, "radius"_a, "height"_a, "color"_a = rviz_visual_tools::Colors::GREEN)
      .def("publish_collision_mesh",
           py::overload_cast<const geometry_msgs::msg::Pose&, const std::string&, const std::string&,
                             const rviz_visual_tools::Colors&>(&MoveItVisualTools::publishCollisionMesh),
           "object_pose"_a, "object_name"_a, "mesh_path"_a, "color"_a = rviz_visual_tools::Colors::GREEN)
      .def("publish_collision_wall",
           py::overload_cast<double, double, double, double, double, double, const std::string&,
                             const rviz_visual_tools::Colors&>(&MoveItVisualTools::publishCollisionWall),
           "x"_a, "y"_a, "z"_a, "angle"_a = 0.0, "width"_a = 2.0, "height"_a = 1.5, "name"_a = "wall",
           "color"_a = rviz_visual_tools::Colors::GREEN)
      .def("publish_workspace_parameters", &MoveItVisualTools::publishWorkspaceParameters)
      .def("publish_robot_state",
           py::overload_cast<const moveit::core::RobotState&, const rviz_visual_tools::Colors&,
                             const std::vector<std::string>&>(&MoveItVisualTools::publishRobotState),
           "robot_state"_a, "color"_a = rviz_visual_tools::Colors::DEFAULT,
           "highlight_links"_a = std::vector<std::string>())
      .def("hide_robot", &MoveItVisualTools::hideRobot)
      .def("show_joint_limits", &MoveItVisualTools::showJointLimits)
      .def("get_planning_scene_monitor", &MoveItVisualTools::getPlanningSceneMonitor);
}
}  // namespace moveit_visual_tools
