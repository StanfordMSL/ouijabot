ouijabot_filter
===============

Author: Eric Cristofalo

Affiliation: Stanford University

Date Created: 2017/08/17; Date Last Modified: 2017/08/17

Tested on: ROS Kinetic

# Required ROS Packages:
* mocap_optitrack
* mocap_interface
* rviz
* rqt
* ouijabot_telop_cpp

# ouijabot_filter
This package contains an node that estimates the pose and odometry of the Ouijabot from published Optitrack data and the onboard IMU/wheel encoders. The estimation is performed with an EKF using the holonomic robot's 2nd order motion model. 

## Requirements

## Files
* filter.cpp
	* Reads raw Ouijabot data and Optitrack data to estimate odometry
    * Publishes ground truth odometry from Optitrack and estimated odometry from Ouijabot sensor data 
    * display_data_flag: integer indicator for printing data during testing (1 displays data)
    * use_optitrack_flag: integer indicator for dusing motion capture data into odometry estimate (not yet functional)
		* wheel_radius
		* body_radius
    * pos_covariance: double for position covariance
    * orient_covariance: double for orientation covariance
    * vel_covariance: double for velocity covariance
    * rot_rate_covariance: double for rotation rate covariance
    * accel_covariance: double for acceleration covariance
    * ROT_accel_covariance: double for rotation acceleration covariance
		* wheel_measurement_covariance
		* gyro_measurement_covariance
		* accel_measurement_covariance

* filter.launch
    * ROS launch file that launches the mocap_optitrack, rviz, rqt, and filter nodes simulaneously. The ground truth and odometry estimates are visualized in RViz. The state estimate is plotted against the ground truth using rqt. 

