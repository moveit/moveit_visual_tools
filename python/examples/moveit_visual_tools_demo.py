#!/usr/bin/env python3

import moveit_visual_tools as mvt
import rviz_visual_tools as rvt
import rclcpp
import sys
import time
from rclpy import logging
import numpy as np
from ament_index_python.packages import get_package_share_directory
from copy import deepcopy

logger = logging.get_logger("moveit_visual_tools_demo")

from geometry_msgs.msg import Pose, Point, Quaternion

rclcpp.init()

PLANNING_GROUP_NAME = "arm"

node = rclcpp.Node("moveit_visual_tools_demo")
visual_tools = mvt.MoveItVisualTools(node, "world", "/moveit_visual_tools")
visual_tools.load_planning_scene_monitor()
visual_tools.load_marker_publisher(True)
visual_tools.load_robot_state_publisher("display_robot_state")
visual_tools.set_manual_scene_updating()

robot_state = visual_tools.get_shared_robot_state()
robot_model = visual_tools.get_robot_model()
jmg = robot_model.get_joint_model_group(PLANNING_GROUP_NAME)

visual_tools.delete_all_markers()
visual_tools.remove_all_collision_objects()
visual_tools.trigger_planning_scene_update()

visual_tools.publish_text(
    Pose(position=Point(x=0.0, y=0.0, z=4.0)),
    "MoveIt-Visual-Tools",
    scale=rvt.Scales.XXLARGE,
)
visual_tools.trigger()

logger.info("Showing 5 random robot states")
for _ in range(5):
    robot_state.set_to_random_positions(jmg)
    visual_tools.publish_robot_state(robot_state)
    time.sleep(1.0)

logger.info("Showing 5 random robot states in different colors")
for _ in range(5):
    robot_state.set_to_random_positions(jmg)
    visual_tools.publish_robot_state(robot_state, color=visual_tools.get_random_color())
    time.sleep(1.0)

logger.info("Hiding robot state")
visual_tools.hide_robot()
time.sleep(1.0)

logger.info("Showing robot state")
visual_tools.publish_robot_state(robot_state, color=rvt.Colors.DEFAULT)
time.sleep(1.0)

logger.info("Publishing Collision Floor")
visual_tools.publish_collision_floor(-0.5, "Floor", color=rvt.Colors.GREY)
visual_tools.trigger_planning_scene_update()
time.sleep(1.0)

logger.info("Publishing Collision Wall")
visual_tools.publish_collision_wall(
    -4.0, -1.0, 0.0, np.pi * 1.1, 6.0, 4.0, "Wall", color=rvt.Colors.GREY
)
visual_tools.trigger_planning_scene_update()
time.sleep(1.0)


logger.info("Publishing Collision Mesh")
mesh_pose = Pose()
for i in range(5):
    visual_tools.publish_collision_mesh(
        mesh_pose,
        f"Mesh_{i}",
        f"file://{get_package_share_directory('moveit_visual_tools')}/resources/demo_mesh.stl",
        color=visual_tools.get_random_color(),
    )
    mesh_pose.position.x += 0.4
    visual_tools.trigger_planning_scene_update()
time.sleep(1.0)

logger.info("Publishing Collision Block")
block_pose = Pose(position=Point(y=1.0))
for i in np.linspace(0.0, 1.0, 10):
    visual_tools.publish_collision_block(
        block_pose,
        f"Block_{i}",
        color=visual_tools.get_random_color(),
    )
    block_pose.position.x += 0.4
    visual_tools.trigger_planning_scene_update()
time.sleep(1.0)

logger.info("Publishing Collision Rectanglular Cuboid")
cuboid_min_size = 0.05
cuboid_max_size = 0.2
cuboid_point1 = Point(y=2.0)
for i in np.linspace(0.0, 1.0, 10):
    cuboid_point2 = deepcopy(cuboid_point1)
    cuboid_point2.x += i * cuboid_max_size + cuboid_min_size
    cuboid_point2.y += (i * cuboid_max_size + cuboid_min_size) / 2.0
    cuboid_point2.z += i * cuboid_max_size + cuboid_min_size
    visual_tools.publish_collision_cuboid(
        cuboid_point1, cuboid_point2, f"Cuboid_{i}", visual_tools.get_random_color()
    )
    cuboid_point1.x += 0.4
    visual_tools.trigger_planning_scene_update()
time.sleep(1.0)

logger.info("Publishing Collision Cylinder")
cylinder_min_size = 0.01
cylinder_max_size = 0.3
cylinder_pose = Pose(position=Point(y=3.0))
for i in np.linspace(0.0, 1.0, 10):
    visual_tools.publish_collision_cylinder(
        cylinder_pose,
        height=i * cylinder_max_size + cylinder_min_size,
        radius=i * cylinder_max_size + cylinder_min_size,
        object_name=f"Cylinder_{i}",
        color=visual_tools.get_random_color(),
    )
    cylinder_pose.position.x += 0.1 + i
    visual_tools.trigger_planning_scene_update()
time.sleep(1.0)

rclcpp.shutdown()
