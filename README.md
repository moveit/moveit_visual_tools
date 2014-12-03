MoveIt! Visual Tools
==========================

**NOTE: in ROS Indigo API has changed significantly, see 'Upgrade Notes' below**

Helper functions for displaying and debugging MoveIt! data in Rviz via published markers, trajectories, and MoveIt! collision objects. It is sometimes hard to understand everything that is going on internally with MoveIt!, but using these quick convenience functions allows one to easily visualize their code.

This package includes:

 - Basic geometric markers for Rviz
 - MoveIt! collision object tools
 - Trajectory visualization tools
 - Robot state tools

Developed by [Dave Coleman](http://dav.ee) at the Correll Robotics Lab, University of Colorado Boulder with outside contributors.

<img align="right" src="https://raw.github.com/davetcoleman/moveit_visual_tools/indigo-devel/resources/demo.png" />

## Install

### Ubuntu Debian

```
sudo apt-get install ros-indigo-moveit-visual-tools
```

### Install From Source

Clone this repository into a catkin workspace, then use the rosdep install tool to automatically download its dependencies. Depending on your current version of ROS, use:
```
rosdep install --from-paths src --ignore-src --rosdistro indigo
```

## Quick Start

To see random shapes generated in Rviz:

    roslaunch moveit_visual_tools visual_tools_test.launch

You should see something like:

<img align="right" src="https://raw.github.com/davetcoleman/moveit_visual_tools/indigo-devel/resources/screenshot.png" />

You can also test the collision objects generation:

    roslaunch moveit_visual_tools collision_objects_test.launch
	
## Code API

See [VisualTools Class Reference](http://docs.ros.org/indigo/api/moveit_visual_tools/html/classmoveit__visual__tools_1_1VisualTools.html)

## Upgrade Notes

We recently did a major refactor of moveit_visual_tools that caused some API breaking changes. If you do not want to bother, you can still build the old version from source using the branch ``indigo-devel-old-api``.

To upgrade, do the following (or use the upgrade script further down):

Orignal API                                    | New API
---------------------------------------------- | ------------------------------------------------------
#include \<moveit_visual_tools/visual_tools.h\>  | #include \<moveit_visual_tools/moveit_visual_tools.h\>
moveit_visual_tools::VisualTools               | moveit_visual_tools::MoveItVisualTools
moveit_visual_tools::VisualToolsPtr            | moveit_visual_tools::MoveItVisualToolsPtr
moveit_visual_tools::rviz_colors               | rviz_visual_tools::colors
moveit_visual_tools::rviz_scales               | rviz_visual_tools::scales

### Auto Upgrade Script

Run each line in order in the ``/src`` of your catkin workspace:

```
findreplace() { grep -lr -e "$1" * | xargs sed -i "s/$1/$2/g" ;}
findreplace '<moveit_visual_tools\/visual_tools.h>' '<moveit_visual_tools\/moveit_visual_tools.h>'
findreplace moveit_visual_tools::VisualTools moveit_visual_tools::MoveItVisualTools
findreplace 'moveit_visual_tools::' 'rviz_visual_tools::'
findreplace 'rviz_visual_tools::MoveItVisualTools' 'moveit_visual_tools::MoveItVisualTools'
```

## Usage

We'll assume you will be using these helper functions within a class.

### Initialize

Add to your includes:
```
#include <moveit_visual_tools/visual_tools.h>
```

Add to your class's member variables:
```
// For visualizing things in rviz
moveit_visual_tools::VisualToolsPtr visual_tools_;
```

In your class' constructor add:
```
visual_tools_.reset(new moveit_visual_tools::VisualTools("base_frame","/moveit_visual_markers"));
```

Change the first parameter to the name of your robot's base frame, and the second parameter to whatever name you'd like to use for the corresponding Rviz marker ROS topic.

There are several other settings you can adjust, which I might get around to documenting in the future:
```
visual_tools_->setMuted(false);
visual_tools_->setLifetime(20.0);
visual_tools_->setEEGroupName(grasp_data_.ee_group_);
visual_tools_->setPlanningGroupName(planning_group_name_);
visual_tools_->setFloorToBaseHeight(floor_to_base_height);
visual_tools_->setGraspPoseToEEFPose(grasp_pose_to_eef_pose);
visual_tools_->setAlpha(alpha);
visual_tools_->setGlobalScale(scale);
visual_tools_->setBaseFrame(frame_name);
```

### Tools

Now in your code you can easily debug your MoveIt! code using visual markers in Rviz

Start rviz and create a new marker using the 'Add' button at the bottom right. Choose the marker topic to be the same as the topic you specified in the constructor.

### Example Code

In the following snippet we create a pose at xyz (0.1, 0.1, 0.1) and rotate the pose down 45 degrees along the Y axis. Then we publish the pose as a arrow for visualziation in Rviz. Make sure your Rviz fixed frame is the same as the one chosen in the code.

    // Create pose
    Eigen::Affine3d pose;
    pose = Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitY()); // rotate along X axis by 45 degrees
    pose.translation() = Eigen::Vector3d( 0.1, 0.1, 0.1 ); // translate x,y,z

    // Publish arrow vector of pose
    ROS_INFO_STREAM_NAMED("test","Publishing Arrow");
    visual_tools_->publishArrow(pose, moveit_visual_tools::RED, moveit_visual_tools::LARGE);


### Basic Publishing Functions

See ``visual_tools.h`` for more details and documentation on the following functions:

 - publishSphere
 - publishArrow
 - publishRectangle
 - publishLine
 - publishBlock
 - publishText
 - publishTest

And more...

### Collision Object Functions

Helpers for adding and removing objects from the MoveIt! planning scene. CO stands for Collision Object and ACO stands for Active Collision Object.

*DEVELOPER TODO: make it so that to use these functions, you must first instanciate a planning scene monitor outside of moveit_visual_tools. Remove publish collision message*

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

These functions are a little more complicated TODO document more

 - publishEEMarkers

### Helper Functions

Reset function

 - ``deleteAllMarkers`` - tells Rviz to clear out all current markers from being displayed. Only withs in ROS Indigo and newer.

Conversion functions

 - convertPose
 - convertPoint32ToPose
 - convertPoseToPoint
 - convertPoint
 - convertPoint32

Convenience functions

 - generateRandomPose
 - dRand
 - fRand
 - iRand
 - getCenterPoint
 - getVectorBetweenPoints

### Available Colors

This package helps you quickly choose colors - feel free to send PRs with more colors as needed

 - moveit_visual_tools::RED
 - moveit_visual_tools::GREEN
 - moveit_visual_tools::BLUE
 - moveit_visual_tools::GREY
 - moveit_visual_tools::WHITE
 - moveit_visual_tools::ORANGE
 - moveit_visual_tools::BLACK
 - moveit_visual_tools::YELLOW

### Available Marker Sizes

 - moveit_visual_tools::XXSMALL
 - moveit_visual_tools::XSMALL
 - moveit_visual_tools::SMALL
 - moveit_visual_tools::REGULAR
 - moveit_visual_tools::LARGE
 - moveit_visual_tools::XLARGE

### Lifetime

All markers will persist for the duration set by ``setLifetime``, defaulting to 30 seconds. You can reset this earlier by calling
```
resetMarkerCounts();
```
This will cause all new markers to overwrite older ones.

You can also delete all markers (new in ROS Indigo) by calling
```
deleteAllMarkers();
```

## Developers Notes

Useful notes for anyone wanting to dig in deeper:

 -  All poses are published with respect to the world frame e.g. /world, /odom, or maybe /base
 -  All publish() ROS topics should be followed by a ``ros::spinOnce();`` but no sleep
 -  Do not want to load any features/publishers until they are actually needed since this library contains so many components

## Contribute

Feel free to send PRs for new helper functions, fixes, etc. - I'll happily discuss and merge them. I do not, however, want to send much time helping people use this because I am a busy grad student. Use at your own risk.
