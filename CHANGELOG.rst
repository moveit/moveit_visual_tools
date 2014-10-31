^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_visual_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2014-10-31)
------------------
* Fix for upstream change of RvizVisualTools
* Set animation speed of grasps
* Fix publishing end effector
* New publishCollisionObjectMsg() function
* New getSharedRobotState() accessor function
* Consolidated publish marker functions
* Fixed loadEEMarker() to be called more than once
* Contributors: Dave Coleman

2.0.0 (2014-10-27)
------------------
* Updated README
* API Upgrade Notes
* Renamed to have 'MoveIt' prefix in class and file name, moved base functionality to rviz_visual_tools
* Added new publishSphere function and publish_sphere test script
* Created better test script
* Better static_id handling for publishText
* Added mainpage for API docs
* Enabled colors
* Improved integer random num generation
* New publishSpheres functions
* Contributors: Dave Coleman

1.3.0 (2014-09-17)
------------------
* Added new getRandColor() function
* Added TRANSLUCENT2 color
* Added two new publishSphere() functions
* New convertPointToPose function
* Reduced sleep timer for starting all publishers from 0.5 seconds to 0.2 seconds
* Removed stacktrace tool because already exists in moveit_core
* New publishText function that allows custom scale and id number be passed in
* Removed deprecated getEEParentLink() function
* Added new scale sizes
* Added new processCollisionObvMsg()
* Added new setPlanningSceneMonitor()
* Deprecated removeAllColisionObejcts()
* Created new removeAllCollisionObjectsPS()
* Added new publishCollisionFloor()
* Added new loadCollisionSceneFromFile()
* New color purple
* Added new setBaseFrame() function
* Contributors: Dave Coleman

1.2.1 (2014-08-11)
------------------
* Renamed base_link to base_frame
* Added new getBaseFrame() function
* Deprecated getBaseLink() function
* Contributors: Dave Coleman

1.2.0 (2014-08-08)
------------------
* Added XXLarge size
* Added global_scale feature
* Added hideRobot() functionality
* Added removeAllCollisionObjects from planning scene monitor
* Added publishCollisionSceneFromFile function
* Formatting
* Contributors: Dave Coleman

1.1.0 (2014-07-31)
------------------
* Bug fixes
* Fixed convertPoint32ToPose
* Added scale to publishText
* New publishPolygon, publishMarker, convertPose, convertPointToPose, and convertPoint32 functions
* New deleteAllMarkers, publishPath, publishSpheres, and convertPoseToPoint functions
* Added getCollisionWall
* Made lines darker
* Added reset marker feature
* Namespaces for publishSphere
* New publishTrajectory function
* Merging features from OMPL viewer
* Refactored functions, new robot_model intialization
* Added more rand functions and made them static
* Added graph_msgs generated messages dependence so it waits for it to be compiled
* Updated README
* Contributors: Dave Coleman, Sammy Pfeiffer

1.0.1 (2014-05-30)
------------------
* Updated README
* Indigo support
* Fix for strict cppcheck and g++ warnings/errors
* Compatibilty fix for Eigen package in ROS Indigo
* Fix uninitialized
* Fix functions with no return statement and other cppcheck errors
* Contributors: Bence Magyar, Dave Coleman, Jordi Pages

1.0.0 (2014-05-05)
------------------
* Enabled dual arm manipulation 
* Removed notions of a global planning group, ee group name, or ee parent link. 
* Changed functionality of loadEEMarker
* Added new print function
* Made getPlanningSceneMonitor() private function
* Renamed loadPathPub()
* Added tool for visualizing unmangled stack trace
* Created function for publishing non-animated grasps
* Created new publishGraph function. Renamed publishCollisionTree to publishCollisionGraph
* Created functions for loading publishers with a delay
* Removed old method of removing all collision objects
* Created better testing functionality
* Changed return type from void to bool for many functions
* Changed way trajectory is timed
* Created new publishIKSolutions() function for grasp poses, etc
* Added new MoveIt robot state functionality
* Added visualize grasp functionality
* Removed unnecessary run dependencies
* Updated README

0.2.0 (2014-04-11)
------------------
* Improved header comments are re-ordered functions into groups
* Started to create new trajectory point publisher
* Added getBaseLink function
* Added dependency on graph_msgs
* Added new collision cylinder functionality
* Created example code in README
* Renamed visualization to visual keyword
* Updated README

0.1.0 (2014-04-04)
------------------
* Split moveit_visual_tools from its original usage within block_grasp_generator package
