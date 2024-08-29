# MoveIt Visual Tools

Helper functions for displaying and debugging MoveIt data in Rviz via published markers, trajectories, and MoveIt collision objects. It is sometimes hard to understand everything that is going on internally with MoveIt, but using these quick convenience functions allows one to easily visualize their code. This package is built in top of [rviz_visual_tools](https://github.com/davetcoleman/rviz_visual_tools) and all those features are included via class inheritance.

This package helps you visualize:

 - Basic Rviz geometric shapes
 - MoveIt collision objects
 - MoveIt and ROS trajectories
 - Robot states
 - End effectors
 - Interactive markers to move robot arms using IK from remote applications

<img src="https://picknik.ai/assets/images/logo.jpg" width="120">

This open source project was developed at [PickNik Robotics](https://picknik.ai/). Need professional ROS development and consulting? Contact us at projects@picknik.ai for a free consultation.

## Status:

### ROS:
- [![Build and Test](https://github.com/ros-planning/moveit_visual_tools/actions/workflows/build_and_test.yaml/badge.svg?branch=master)](https://github.com/ros-planning/moveit_visual_tools/actions/workflows/build_and_test.yaml?query=branch%3Amaster) [![Formatting](https://github.com/ros-planning/moveit_visual_tools/actions/workflows/format.yaml/badge.svg?branch=master)](https://github.com/ros-planning/moveit_visual_tools/actions/workflows/format.yaml?query=branch%3Amaster) Github Actions
- [![Build Status](http://build.ros.org/buildStatus/icon?job=Nsrc_uF__moveit_visual_tools__ubuntu_focal__source)](http://build.ros.org/job/Nsrc_uF__moveit_visual_tools__ubuntu_focal__source/) ROS Buildfarm - AMD64 Focal Source Build - Ubuntu 20.04 LTS
- [![Build Status](http://build.ros.org/buildStatus/icon?job=Ndev__moveit_visual_tools__ubuntu_focal_amd64)](http://build.ros.org/job/Ndev__moveit_visual_tools__ubuntu_focal_amd64/) ROS Buildfarm - AMD64 Focal Devel Build - Ubuntu 20.04 LTS

![](resources/screenshot.png)

![](resources/demo.png)

## Install

### Ubuntu Debian

    sudo apt-get install ros-kinetic-moveit-visual-tools

### Install From Source

Clone this repository into a catkin workspace, then use the rosdep install tool to automatically download its dependencies. Depending on your current version of ROS, use:

    rosdep install --from-paths src --ignore-src --rosdistro kinetic

## Quick Start Demo

First launch Rviz:

    roslaunch moveit_visual_tools demo_rviz.launch

Then run some demos displaying robot states and collision objects:

    roslaunch moveit_visual_tools demo.launch

## Code API

See [VisualTools Class Reference](http://docs.ros.org/kinetic/api/moveit_visual_tools/html/classmoveit__visual__tools_1_1MoveItVisualTools.html)

## Usage

We'll assume you will be using these helper functions within a class.

### Initialize

Add to your includes:
```
#include <moveit_visual_tools/moveit_visual_tools.h>
```

Add to your class's member variables:
```
// For visualizing things in rviz
moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
```

In your class' constructor add:
```
visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base_frame","/moveit_visual_markers"));
```

### Collision Object Functions

Helpers for adding and removing objects from the MoveIt planning scene. CO stands for Collision Object and ACO stands for Active Collision Object.

 - cleanupCO
 - cleanupACO
 - attachCO
 - publishCollisionBlock
 - publishCollisionCylinder
 - publishCollisionTree
 - publishCollisionTable
 - publishCollisionWall

And more...

### Animate Trajectories

Higher level robot and trajectory functions

 - publishTrajectoryPath
 - publishTrajectoryPoint
 - publishRobotState
 - publishAnimatedGrasps
 - publishIKSolutions

## Show parts of a robot

These functions are a little more complicated

 - publishEEMarkers

## Parent Class

This class is built on top of [rviz_visual_tools](https://github.com/davetcoleman/rviz_visual_tools) so all features and documentation for that package apply here as well.

## Developers Notes

Useful notes for anyone wanting to dig in deeper:

 -  All poses are published with respect to the world frame e.g. /world, /odom, or maybe /base
 -  All publish() ROS topics should be followed by a ``ros::spinOnce();`` but no sleep
 -  Do not want to load any features/publishers until they are actually needed since this library contains so many components

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/):

    catkin lint -W2

Use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/) to run the small amount of available tests:

    catkin run_tests --no-deps --this -i

## Contribute

Please send PRs for new helper functions, fixes, etc!
