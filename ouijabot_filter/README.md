ouijabot_filter
===============

Author: Eric Cristofalo

Affiliation: Stanford University

Date Created: 2017/08/17; Date Last Modified: 2017/08/17

Tested on: ROS Kinetic

# Required ROS Packages:
* mocap_optitrack
* mocap_interface
* rqt
* ouijabot_telop_cpp

# ouijabot_filter
This package contains an node that estimates the pose and odometry of the Ouijabot from published Optitrack data and the onboard IMU/wheel encoders. The estimation is performed with an EKF using the holonomic robot's 2nd order motion model. 

## Requirements

## Files
* filter.cpp
	* Reads raw Ouijabot data and Optitrack data to estimate odometry
    * Publishes ground truth odometry from Optitrack and estimated odometry from Ouijabot sensor data 
    * DISPLAY_DATA_BOOL: integer indicator for printing data during testing (1 displays data)
    * USE_OPTITRACK_BOOL: integer indicator for dusing motion capture data into odometry estimate (not yet functional)
    * POS_COVARIANCE: double for position covariance
    * ORIENT_COVARIANCE: double for orientation covariance
    * VEL_COVARIANCE: double for velocity covariance
    * ROT_RATE_COVARIANCE: double for rotation rate covariance
    * ACCEL_COVARIANCE: double for acceleration covariance
    * ROT_ACCEL_COVARIANCE: double for rotation acceleration covariance
* filter.launch
    * ROS launch file that launches the mocap_optitrack, rviz, and odom nodes simulaneously. The ground truth and odometry estimates are visualized in RViz. 

