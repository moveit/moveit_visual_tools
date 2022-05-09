^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package moveit_visual_tools
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

4.1.0 (2022-05-09)
------------------
* Update black version (`#117 <https://github.com/ros-planning/moveit_visual_tools/issues/117>`_)
* Jammy fixes and clang-format-12 (`#116 <https://github.com/ros-planning/moveit_visual_tools/issues/116>`_)
* Fixed deprecated call to PlanningSceneMonitor's constructor (`#108 <https://github.com/ros-planning/moveit_visual_tools/issues/108>`_)
* Remove Foxy CI builds (`#111 <https://github.com/ros-planning/moveit_visual_tools/issues/111>`_)
* Generate license using ament_copyright (`#105 <https://github.com/ros-planning/moveit_visual_tools/issues/105>`_)
* Contributors: Jafar Abdi, Stephanie Eng, Vatan Aksoy Tezer

4.0.0 (2021-09-27)
------------------
* Port to ros2 (`#92 <https://github.com/ros-planning/moveit_visual_tools/issues/92>`_)
* Fix typo in package.xml (`#87 <https://github.com/ros-planning/moveit_visual_tools/issues/87>`_)
* Contributors: Davide Faconti, Felix von Drigalski, Vatan Aksoy Tezer

3.6.0 (2020-10-09)
------------------
* [feature] Unified collision environment used (`#54 <https://github.com/ros-planning/moveit_visual_tools/issues/54>`_)
* [feature] publish cuboids with size (`#59 <https://github.com/ros-planning/moveit_visual_tools/issues/59>`_)
* [feature] hideRobot(): use new .hide member of DisplayRobotState msg (`#56 <https://github.com/ros-planning/moveit_visual_tools/issues/56>`_)
* [feature] Use first trajectory point as start state for visualization (`#49 <https://github.com/ros-planning/moveit_visual_tools/issues/49>`_)
* [feature] exposing the publishers to the user (`#48 <https://github.com/ros-planning/moveit_visual_tools/issues/48>`_)
* [feature] Highlight selected links in publishRobotState() + improved collision visualization (`#45 <https://github.com/ros-planning/moveit_visual_tools/issues/45>`_)
* [fix] Update to Noetic and fix various warnings (`#79 <https://github.com/ros-planning/moveit_visual_tools/issues/79>`_)
* [fix] Fix Eigen alignment (`#63 <https://github.com/ros-planning/moveit_visual_tools/issues/63>`_)
* [fix] missing end effector markers because clear (`#51 <https://github.com/ros-planning/moveit_visual_tools/issues/51>`_)
* [fix] Remove #attempts from setFromIK (`#43 <https://github.com/ros-planning/moveit_visual_tools/issues/43>`_)
* [documentation] Update README.md (`#69 <https://github.com/ros-planning/moveit_visual_tools/issues/69>`_)
* [documentation] update doc of hideRobot() (`#64 <https://github.com/ros-planning/moveit_visual_tools/issues/64>`_)
* [maint] add soname version (`#74 <https://github.com/ros-planning/moveit_visual_tools/issues/74>`_)
* [maint] Replace tf_conversions with tf2's toMsg / fromMsg (`#66 <https://github.com/ros-planning/moveit_visual_tools/issues/66>`_)
* [maint] fix clang-tidy issue (`#68 <https://github.com/ros-planning/moveit_visual_tools/issues/68>`_)
* [maint] Cleanup (`#65 <https://github.com/ros-planning/moveit_visual_tools/issues/65>`_)
  * Replace robot_model and robot_state namespaces by moveit::core
  * Fix clang-tidy issues
  * Remove deprecated function
* [maint] Bump required cmake version (`#62 <https://github.com/ros-planning/moveit_visual_tools/issues/62>`_)
* [maint] moveit.rosinstall: use master branch for all deps (`#57 <https://github.com/ros-planning/moveit_visual_tools/issues/57>`_)
* [maint] drop melodic + kinetic support for master branch (`#58 <https://github.com/ros-planning/moveit_visual_tools/issues/58>`_)
* [maint] Fix Travis config + issues (`#47 <https://github.com/ros-planning/moveit_visual_tools/issues/47>`_)
* [maint] Change 'MoveIt!' to MoveIt
* Contributors: Bjar Ne, Dave Coleman, Henning Kayser, Jafar Abdi, Jens P, Mark Moll, Michael Görner, Mike Lautman, Robert Haschke, Tyler Weaver

3.5.2 (2018-12-10)
------------------
* Use LOGNAME for named logging (`#42 <https://github.com/ros-planning/moveit_visual_tools/issues/42>`_)
* Eigen::Affine3d -> Eigen::Isometry3d (`#39 <https://github.com/ros-planning/moveit_visual_tools/issues/39>`_)
* Contributors: Dave Coleman

3.5.1 (2018-11-14)
------------------
* Adding trigger call to animations when batch_publishing_enabled\_ is enabled (`#36 <https://github.com/ros-planning/moveit_visual_tools/issues/36>`_)
* Contributors: Mike Lautman

3.5.0 (2018-09-05)
------------------
* Add Markdown ROS Buildfarm badges for Melodic
* Fixing melodic branch with tf2 updates (`#34 <https://github.com/ros-planning/moveit_visual_tools/issues/34>`_)
* Additional visualization (`#31 <https://github.com/ros-planning/moveit_visual_tools/issues/31>`_)
  * adding a visualization for publishing a box with width, height, and depth
  * adding additional publishCollisionCuboid method overloads
  * changing x,y,z to width, depth, height
* Fixup CMakeLists and package.xml (`#30 <https://github.com/ros-planning/moveit_visual_tools/issues/30>`_)
* Triggering UPDATE_GEOMETRY is sufficient, dont spinOnce
  - triggering UPDATE_SCENE leads to sending a full planning scene including all meshes. For all use-cases in this lib UPDATE_GEOMETRY is sufficient and sends only diffs
  - calling ros::spinOnce() in random places messes up the event queue (e.g. joint state updates being processed in gui thread instead of the async spinner thread)
* Fix API compatibility by providing a default empty list of end-effector joints
* Allow setting joint values for end-effector marker
  currently end-effector markers are based on the default robot state (ie all zero),
  this allows passing a vector of doubles for all active joint in the end-effector group
  Passing a new set of joint values will invalidate the cache but for our use-case this is
  neglectible and could be optimized later
* Moved boost::shared_ptr to std for tf2
* Converted to use tf2 moveit interfaces
* Fix broken CI for Melodic
* Fix Continuous Integration
* Contributors: Dave Coleman, Ian McMahon, Mike Lautman, Simon Schmeisser

3.4.0 (2017-12-27)
------------------
* Apply current MoveIt clang-format
* Various improvements needed while finishing planning thesis
* Fix greater than/less than issue in clearance check
* Ability to specify clearance for random state
* Small threading fixes
* imarker: Fix setToRandomState()
  imarker: Switch to std::makeshared
* Improve console output
* Contributors: Dave Coleman, Mike Lautman

3.3.0 (2017-06-20)
------------------
* Change error message to warning
* Make planning scene monitor publicly exposed
* Remove label from imarkers
* Ability to move a collision object without removing it first
* IMarkerRobotState: update imarkers location when setting robot state
* IMarkerRobotState: Added setRobotState()
* IMarkerRobotState: Renamed function publishRobotState()
* MoveItVisualTools: renamed variable to psm\_
* Expose verbose collision checking
* Contributors: Dave Coleman

3.2.1 (2016-11-02)
------------------
* New publishTrajectoryPath() functions
* New publishTrajectoryLine() functions
* getRobotState() return by reference
* Trajectory path has smaller vertices
* IMarkerRobotState: added isStateValid()
* Contributors: Dave Coleman

3.2.0 (2016-10-20)
------------------
* Added publishState() to imarker_robot_state
* New publishTrajectoryLine() function that automatically chooses end effectors to visualize
* New collision table function that takes z input
* Fixed callbacks for multiple EEFs
* Allow for two end effectors
* Ability to use two end effectors for interactive markers
* Make ik solving at any end effector link, not just end of kinematic chain
* Better debugging for collision
* Only save when mouse up
* Fix API for changes in rviz_visual_tools
* Allow collision walls to have variable z location
* Make applyVirtualJointTransform() static
* Make checkForVirtualJoint() static
* IMarkerRobotState remove offset capability
* IMarkerRobotState remove imarker box control
* Switched travis to MoveIt CI
* Added new IMarker Robot control method
* Cleaned up code base: catkin lint, roslint
* Fixed bug in planning scene triggering
* Optimize planning scene updates to only update GEOMETRY
* Fix xacro
* Upgrade to Eigen3 per ROS Kinetic requirements
* New publishRobotState() function
* Fix Eigen bugs
* Removed deprecated code
* Converted to C++11
* Optional blocking publisher calls
* Added getter for getRobotRootState()
* Contributors: Dave Coleman

3.1.0 (2016-04-28)
------------------
* Re-factored and fixed visual tools demo!
* Fixes for catkin lint
* Fixes for roslint
* Removed deprecated function call
* Remove deprecated test
* New root_robot_state utilization
* Ablity to move a RobotState's root frame permenatly around in the scene
* Better publishCollisionWall() function
* Deprecated old publishTrajectoryLine() functions - removed clear_all_markers argument
* New publishTrajectoryPath() variant
* Rename namespace of RobotState
* Made INFO into DEBUG output
* New publishTrajectoryLine function
* Switched publishTrajectoryLine to use cylinders instead of lines
* New showJointLimits() function for console debugging a robot state
* Fix publishTrajectoryPath() bug
* Default blocking time for trajectory if not parameterized
* Publish workspace parameters was incorrectly creating a collision object
* Contributors: Dave Coleman

3.0.5 (2016-02-09)
------------------
* Updated README
* Better comment
* Contributors: Dave Coleman

3.0.4 (2016-01-12)
------------------
* Removed stray debug output
* Improved debugging output for the hideRobot() feature and virtual_joints
* Contributors: Dave Coleman

3.0.3 (2016-01-10)
------------------
* Renamed test to demo
* New publishTrajectoryLine() function
* Fix travis
* Deprecated loadEEMarker() that uses string
* Formatted code
* Switched from MOVEIT deprecated to RVIZ_VISUAL_TOOLS deprecated
* Fixed shared_robot_state to initialize correctly every time
* Switched to using name\_ variables
* Add error checks to publishTrajectoryLine
* Added ability for publishTrajectoryLine to clear all previous markers
* Contributors: Dave Coleman

3.0.2 (2015-12-27)
------------------
* Updated README
* Temp fix missing variable
* Contributors: Dave Coleman

3.0.1 (2015-12-05)
------------------
* catkin lint cleanup
* Fix travis
* Contributors: Dave Coleman

3.0.0 (2015-12-02)
------------------
* Release 3.0
* Added travis support
* fix the how to link a demo img
* Updated link to Doxygen API description
* Formatting and better debug output
* Fix hide robot bug
* Remove incompatible humanoid function
* Default color when publishing collision meshes
* Added error check for bad value
* API change for removal of shape_tools
* New publish trajectory line function
* Remove slash from topic name
* Removed mute functionality
* Improved loading efficiency
* publishContactPoints accepts a color
* Change topics to default when opening Rviz
* New publishCollisionMesh() function
* Changed publishCollisionMesh() API
* Renamed publishCollisionRectangle to publishCollisionCuboid()
* Updated rviz_visual_tools API
* New publishMesh from ROS msg function
* publishRobotState() for a RobotStateMsg now allows color
* publishTrajectoryPath() for a ROS msg now requires a RobotState
* New method for attaching collision objects that does not require a publisher
* Specify scene name and cleanup logging
* Fixed error checking for hideRobot() function
* loadTrajectoryPub() allows custom topic
* New publishTrajectoryPoints() function
* New publishContactPoints function
* New publishTrajectoryPath() function
* New getRobotModel() function
* New ability to visualize IK solutions with arbitrary virtual joint
* API Broken: ability to have different end effectors for different arms, auto EE marker loading
* Publish collision meshes
* Added check for virtual joint
* Fixed which arrow gets published
* Publish fixed link arrows to show footstep locations
* Ability to specify robot_state_topic without loading the publisher
* Contributors: Dave Coleman, Daiki Maekawa, simonschmeisser

2.2.0 (2015-01-07)
------------------
* Code cleanup
* Improved naming
* Joint model bug fix
* Improved speed of sending collision objects to Rviz
  Added Manual planning scene update mode
  Ability to apply colors to all collision objects (YAY)
  API: removed removeAllCollisionObjectsPS function
  Removed loadPlanningPub() function
  Removed publishRemoveAllCollisionObjects() function
* Added backwards compatibile loadCollisionSceneFromFile()
* New publishCollisionRectangle function
  API: Changed loadCollisionSceneFromFile() to accept a pose instead of x,y
* Fix for renamed function
* New publishWorkspaceParameters() function
* Added ability to publish robot states with color
* Fixed install method
* Merge pull request `#5 <https://github.com/davetcoleman/moveit_visual_tools/issues/5>`_ from robomakery/feature/fix-collision-objects-test
* Fixes for missing declarations in collision_objects_test.cpp
* Refactored how collision ojects are published
  Created new collision objects test and roslaunch file
  Optimized header file
  Removed loadCollisionPub() function
  Fixed publishCollisionFloor
  Added publishCollisionRectangle
* Contributors: Dave Coleman, Dylan Vaughn

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
* Added TRANSLUCENT color
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
