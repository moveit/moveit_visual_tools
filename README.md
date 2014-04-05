moveit_visual_tools
==========================

Helper functions for displaying and debugging MoveIt! data in Rviz via published markers and MoveIt! collision objects

<img align="right" src="https://raw.github.com/davetcoleman/moveit_visual_tools/hydro-devel/resource/demo.png" />

### Build Status

[![Build Status](https://travis-ci.org/davetcoleman/moveit_visual_tools.png?branch=hydro-devel)](https://travis-ci.org/davetcoleman/moveit_visual_tools)

## Install

### Ubuntu Debian

PENDING
```
sudo apt-get install ros-hydro-moveit-visual-tools
```

### Install From Source

```
git clone git@github.com:davetcoleman/moveit_visual_tools.git
```

## Usage

We'll assume you will be using these helper functions within a class.

### Initialize

Add to your includes:
```
#include <moveit_visual_tools/visualization_tools.h>
```

Add to your class's member variables:
```
// For visualizing things in rviz
moveit_visual_tools::VisualizationToolsPtr visual_tools_;
```

In your class' constructor add:
```
visual_tools_.reset(new moveit_visual_tools::VisualizationTools("base_link","/moveit_visual_markers"));
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

### Publishing Functions

See ``moveit_visual_tools.h`` for more details and documentation on the following functions:

 - publishSphere
 - publishEEMarkers
 - publishArrow
 - publishRectangle
 - publishLine
 - publishBlock
 - publishText

### Collision Object Functions

Helpers for adding and removing objects from the MoveIt! planning scene. CO stands for Collision Object and ACO stands for Active Collision Object.

 - cleanupCO
 - cleanupACO
 - attachCO
 - publishCollisionBlock
 - publishCollisionWall
 - publishCollisionTable

### Animate Trajectories

Note: this might not currently be working

 - publishTrajectoryPath

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

Feel free to send PRs for new helper functions, fixes, etc. - I'll happily discuss and merge them. I do not, howver, want to send much time helping people use this because I am a busy grad student. Use at your own risk.