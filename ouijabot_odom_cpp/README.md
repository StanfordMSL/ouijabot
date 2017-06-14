ouijabot_odom_cpp
===============

Author: Eric Cristofalo

Affiliation: Stanford University

Date Created: 2017/06/13; Date Last Modified: 2017/06/13

Tested on: ROS Kinetic

# Required ROS Packages:
* mocap_optitrack
* mocap_interface
* joy
* rviz
* ouijabot_telop_cpp

# ouijabot_odom_cpp
This package contains an node that estimates the odometry of the Ouijabot from published wheel velocities. In the future, we should add a Kalman filter to fuse wheel encoders, IMU, and noisy GPS (motion capture) data.  

## Requirements

## Files
* odom.cpp
	* Reads raw Ouijabot data and Optitrack data to estimate odometry
    * Publishes ground truth odometry from Optitrack and estimated odometry from Ouijabot sensor data 
    * DISPLAY_DATA_BOOL: integer indicator for printing data during testing (1 displays data)
    * USE_OPTITRACK_BOOL: integer indicator for dusing motion capture data into odometry estimate (not yet functional)
    * POSITION_COVARIANCE: double for position covariance
    * ORIENTATION_COVARIANCE: double for orientation covariance
    * VELOCITY_COVARIANCE: double for velocity covariance
    * ROTATION_RATE_COVARIANCE: double for rotation rate covariance
* odom.launch
    * ROS launch file that launches the mocap_optitrack, rviz, and odom nodes simulaneously. The ground truth and odometry estimates are visualized in RViz. 

