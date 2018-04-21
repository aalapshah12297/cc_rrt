# Chance-Constrained RRT
This repository contains an implementation of chance-constrained RRT (as a course project for AE640A @IITK) along with a fully functional SLAM + Control ROS stack.


The main algorithm is based on [_**Chance Constrained RRT for Probabilistic. Robustness to Environmental Uncertainty, by Brandon D. Luders, Mangal Kothari and Jonathan P. How**_](http://acl.mit.edu/papers/Luders10_GNC.pdf)

## Contents of the Repository
The packages in the workspace folder **ae640_ws** are:
- **cc_rrt** - This package contains the main path planner and the top-level launch files. It has been fully developed from scratch.
- **costmap_2d** - From simple maps, and the robot's footprint, this package generates a configuration space which includes a costmap (based on distance of each cell from the nearest occupied cell). It is a modified version of the node in the [ROS navigation stack](https://github.com/ros-planning/navigation). Changes from the original include a linear decay function (for giving distance to nearest obstacle), some changed parameters (as per calculations, so as to give a decay rate of 2 per cell) for compatibility with _cc_rrt_ and inclusion of the robot's footprint (based on NEX Robotics' Firebird Delta) in the parameters.
- **lidar_mapping** - This node should be used when gmapping fails to generate maps, which is mostly the case in sparse outdoor environments. It has been forked from [Team IGVC IITK](https://github.com/igvc-iitk).
- **robot_localization** - This node fuses the wheel odometry with the IMU orientation. It has been forked from [Team IGVC IITK](https://github.com/igvc-iitk).
-  **ros0xrobot**, **rplidar_ros** - These are the drivers for the robot and the LiDAR. These have been forked from [Team IGVC IITK](https://github.com/igvc-iitk).
- **unicycle_controller** - This is a non-linear high-level controller for the robot. It has been forked from [Team IGVC IITK](https://github.com/igvc-iitk).

The **extras** folder contains:
- 2 Bagfiles for testing purposes, and
- a script for a certain step within the installation process (mentioned below).


## Dependencies

You can either run the demo in **live on a robot** (supplementary packages and drivers included) **or on bagfiles** (also included). If you only wish to run the demo on a bagfile, then only the _cc_rrt_ package is required. The remaining packages and optional dependencies are only for the live demo. 

### Required:
It is mandatory to have ROS ([Kinetic](http://wiki.ros.org/kinetic/Installation) or higher) installed on your system for this package to work. Full installation is recommended to avoid installing separate packages later.
If you do not install the full version, you will have to install [Eigen 3](http://eigen.tuxfamily.org/index.php?title=Main_Page) separately using the following command:
- `sudo apt install libeigen3-dev`

### Optional:
#### Pixhawk IMU:
You can choose to use a pixhawk IMU along with odometry for localization. If not, then uncomment the appropriate line from _initialization.launch_. Otherwise, run these commands to intall the mavros driver on your system:
- `sudo apt-get update`
- `sudo apt-get install ros-kinetic-mavros`
- `sudo apt-get install ros-kinetic-mavros-msgs`
- `sudo apt-get install ros-kinetic-mavros-extras`

Now go to the directory containing _install_geographiclib_datasets.sh_ (check the _extras_ folder)
- `sudo chmod +x install_geographiclib_datasets.sh`
- `sudo ./install_geographiclib_datasets.sh`

If you are not added to the dialout group, you might have to use this after connecting the IMU:
- `sudo chmod 666 /dev/ttyACM0`

##### slam_gmapping:
You can either install gmapping using the following command:
- `sudo apt-get install ros-kinetic-gmapping`
Or you can use the lidar_mapping node included in the repository. Check _mapping.launch_ to choose.


## Instructions for Running the node
Clone the repository to your system, go to the workspace folder **ae640_ws**, and run `catkin_make` (this could take a long while).
### For the live demo:
- Connect the robot, followed by the LiDAR to your system (and optionally the IMU).
- Read the launchfiles in the _cc_rrt_ package and comment/uncomment the appropriate lines. Then set the parameters (esp. _goal_position_) as required.
- Run the launchfiles in the _cc_rrt_ package in the following order:
1. `roslaunch cc_rrt initialization.launch`
2. `roslaunch cc_rrt mapping.launch` (Now drive the robot using the arrow keys, map the region and bring the robot to a suitable start position.)
3. `roslaunch cc_rrt planning.launch`
4. Terminate `mapping.launch` before the robot begins to move. This leads to growing uncertainty in the robot's state, which demonstrates one of the key use-cases of this algorithm.


### For testing on bagfiles:
- Read planning_test.launch in the _cc_rrt_ package. Then set the parameters (esp. _goal_position_) as required.
- Run roscore then:
- `rosparam set /use_sim_time true`
- `roslaunch cc_rrt planning_test.launch`
Now go to the folder containing the bagfile and then:
- `rosbag play test_0.bag --clock`
