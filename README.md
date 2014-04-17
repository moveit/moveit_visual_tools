MoveIt! Visual Tools
==========================

Helper functions for displaying and debugging MoveIt! data in Rviz via published markers and MoveIt! collision objects. Very useful for debugging complex software

By [Dave Coleman](http://dav.ee) at the Correll Robotics Lab, University of Colorado Boulder

<img align="right" src="https://raw.github.com/davetcoleman/moveit_visual_tools/hydro-devel/resources/demo.png" />

### Build Status

[![Build Status](https://travis-ci.org/davetcoleman/moveit_visual_tools.png?branch=hydro-devel)](https://travis-ci.org/davetcoleman/moveit_visual_tools)

## Install

### Ubuntu Debian

Available next hydro release:
```
sudo apt-get install ros-hydro-moveit-visual-tools
```

### Install From Source

```
git clone git@github.com:davetcoleman/moveit_visual_tools.git
```

Dependencies (available in hydro next release):
```
git clone git@github.com:davetcoleman/graph_msgs.git
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
visual_tools_.reset(new moveit_visual_tools::VisualTools("base_link","/moveit_visual_markers"));
```

Change the first parameter to the name of your robot's base link, and the second parameter to whatever name you'd like to use for the corresponding Rviz marker ROS topic.

There are several other settings you can adjust, which I might get around to documenting in the future:
```
visual_tools_->setMuted(false);
visual_tools_->setLifetime(20.0);
visual_tools_->setEEGroupName(grasp_data_.ee_group_);
visual_tools_->setPlanningGroupName(planning_group_name_);
visual_tools_->setFloorToBaseHeight(floor_to_base_height);
visual_tools_->setGraspPoseToEEFPose(grasp_pose_to_eef_pose);
visual_tools_->setAlpha(alpha);
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

 - publishEEMarkers
 - publishSphere
 - publishArrow
 - publishRectangle
 - publishLine
 - publishBlock
 - publishText
 - publishTest

### Collision Object Functions

Helpers for adding and removing objects from the MoveIt! planning scene. CO stands for Collision Object and ACO stands for Active Collision Object.

 - cleanupCO
 - cleanupACO
 - attachCO
 - publishCollisionBlock
 - publishCollisionCylinder
 - publishCollisionTree
 - publishCollisionTable
 - publishCollisionWall

### Animate Trajectories

Higher level robot ans trajectory functions

 - publishTrajectoryPath
 - publishTrajectoryPoint
 - publishRobotState
 - publishAnimatedGrasps
 - publishIKSolutions

### Helper Functions

Convenience functions
 
 - convertPose
 - generateRandomPose
 - dRand
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

## Contribute

Feel free to send PRs for new helper functions, fixes, etc. - I'll happily discuss and merge them. I do not, however, want to send much time helping people use this because I am a busy grad student. Use at your own risk.
