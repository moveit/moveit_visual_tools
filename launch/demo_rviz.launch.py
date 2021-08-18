import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():

    robot_description_config = load_file("moveit_visual_tools", "resources/rrbot.urdf")

    robot_description = {"robot_description": robot_description_config}

    robot_description_semantic_config = load_file(
        "moveit_visual_tools", "resources/rrbot.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Start the demo
    moveit_visual_tools_demo = Node(
        package="moveit_visual_tools",
        executable="moveit_visual_tools_demo",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    # RViz
    # TODO: Configure this file
    rviz_config_file = (
        get_package_share_directory("moveit_visual_tools") + "/launch/demo.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
    )

    return LaunchDescription([rviz_node, static_tf, moveit_visual_tools_demo])
