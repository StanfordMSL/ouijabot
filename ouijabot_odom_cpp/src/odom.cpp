/*--------------------------------------------------------------------------
 
 File Name:         odom.cpp
 Date Created:      2017/06/13
 Date Modified:     2017/06/13
 
 Author:            Eric Cristofalo
 Contact:           eric.cristofalo@gmail.com
 
 Description:       ROS node for estimating realtime odometry from Ouijibot encoders
 
 -------------------------------------------------------------------------*/

#include <iostream>
#include <stdio.h>
#include <math.h>
#include <random>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <ouijabot/Wheel_Spd.h>

using namespace std;

class mocapInterfaceClass {
    
  ros::NodeHandle nh_;
  
  ros::Subscriber process_mocap_sub_;
  ros::Subscriber process_imu_sub_;
  ros::Subscriber process_wheel_speed_sub_;

  ros::Publisher odom_ground_truth_pub_;
  ros::Publisher odom_pub_;
  
  // Initialize Variables
  int displayData, useOptitrack;
  double positionCovariance, orientationCovariance, velocityCovariance, rotationRateCovariance;
  int addNoise;
  ros::Time timeCur, timePrev, timeCurEst, timePrevEst;
  int initialize, initCount;
  double initTime;
  double xInit, yInit, zInit, phiInit, theInit, psiInit;
  double x_, y_, z_, phi_, the_, psi_;
  double xOdom_, yOdom_, zOdom_, phiOdom_, theOdom_, psiOdom_;
  double imu_x, imu_y, imu_z;
  double gyro_x, gyro_y, gyro_z;
  double xOdomEst_, yOdomEst_, zOdomEst_, phiOdomEst_, theOdomEst_, psiOdomEst_;
    
public:
  mocapInterfaceClass(int a, int b, double c, double d, double e, double f) {

    // Subscribe to Optitrack Topics
    process_mocap_sub_ = nh_.subscribe("/robot/pose", 10, &mocapInterfaceClass::mocapCallback, this);

    // Subscribe to Ouijibot IMU
    process_imu_sub_ = nh_.subscribe("/ouijabot/imu", 10, &mocapInterfaceClass::imuCallback, this);

    // Subscribe to Ouijibot Wheel Encoders
    process_wheel_speed_sub_ = nh_.subscribe("/ouijabot/wheel_spd", 10, &mocapInterfaceClass::wheelSpeedCallback, this);

    // Publish Ground Truth Odometry Message
    odom_ground_truth_pub_ = nh_.advertise<nav_msgs::Odometry>("/robot/odom_gt", 10);

    // Publish Odometry Message
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/robot/odom", 10);

    // Initialize Variables
    displayData = a;
    useOptitrack = b;
    positionCovariance = c;
    orientationCovariance = d;
    velocityCovariance = e;
    rotationRateCovariance = f;
    addNoise = 0;
    if (positionCovariance!=0.0 || orientationCovariance!=0.0 || velocityCovariance!=0.0 || rotationRateCovariance!=0.0 ) {
      addNoise = 1;
    }
    timeCur = ros::Time::now();
    timePrev = ros::Time::now();
    timeCurEst = ros::Time::now();
    timePrevEst = ros::Time::now();
    initialize = 1;
    initCount = 0;
    initTime = 0.0;
    xInit = 0.0; yInit = 0.0; zInit = 0.0; phiInit = 0.0; theInit = 0.0; psiInit = 0.0;
    x_ = 0.0; y_ = 0.0; z_ = 0.0; phi_ = 0.0; the_ = 0.0; psi_ = 0.0;
    xOdom_ = 0.0; yOdom_ = 0.0; zOdom_ = 0.0; phiOdom_ = 0.0; theOdom_ = 0.0; psiOdom_ = 0.0;
    imu_x = 0.0, imu_y = 0.0, imu_z = 0.0;
    gyro_x = 0.0, gyro_y = 0.0, gyro_z = 0.0;
    xOdomEst_ = 0.0; yOdomEst_ = 0.0; zOdomEst_ = 0.0; phiOdomEst_ = 0.0; theOdomEst_ = 0.0; psiOdomEst_ = 0.0;

    std::cout << "Listening to Optitrack for 1 Seconds" << std::endl;
  }

  void mocapCallback(const geometry_msgs::PoseStamped& msg)
  {

    // Extract Data
    double xCur = msg.pose.position.x;
    double yCur = msg.pose.position.y;
    double zCur = msg.pose.position.z;
    double qx = msg.pose.orientation.x;
    double qy = msg.pose.orientation.y;
    double qz = msg.pose.orientation.z;
    double qw = msg.pose.orientation.w;

    // Conversion to Euler Angles
    double phiCur, theCur, psiCur;
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    m.getRPY(phiCur, theCur, psiCur);

    // Initialize Starting Robot Pose
    if (initialize==1) {
      timeCur = ros::Time::now();
      // Add Poses
      double dt = (timeCur-timePrev).toSec();
      initTime = initTime + dt;
      if ( initTime<1.0 ) {
        initCount++;
        xInit = xInit + xCur;
        yInit = yInit + yCur;
        zInit = zInit + zCur;
        phiInit = phiInit + phiCur;
        theInit = theInit + theCur;
        psiInit = psiInit + psiCur;
      }
      else {
        // Average Poses
        xInit = xInit/double(initCount);
        yInit = yInit/double(initCount);
        zInit = zInit/double(initCount);
        phiInit = phiInit/double(initCount);
        theInit = theInit/double(initCount);
        psiInit = psiInit/double(initCount);
        timePrevEst = ros::Time::now();
        initialize = 0;
        std::cout << "Initial Pose:" << endl << "x: " << xInit << endl << "y: " << yInit << endl << "psi: " << psiInit << endl << endl;
        // Set Previous Values
        timePrev = ros::Time::now();
        x_ = xInit;
        y_ = yInit;
        z_ = zInit;
        phi_ = phiInit;
        the_ = theInit;
        psi_ = psiInit;
        xOdom_ = xInit;
        yOdom_ = yInit;
        zOdom_ = 0.0;
        phiOdom_ = 0.0;
        theOdom_ = 0.0;
        psiOdom_ = psiInit;
        // Set Previous Estimate Values
        timePrevEst = ros::Time::now();
        xOdomEst_ = xInit;
        yOdomEst_ = yInit;
        zOdomEst_ = 0.0;
        phiOdomEst_ = 0.0;
        theOdomEst_ = 0.0;
        psiOdomEst_ = psiInit;
      }
    }

    else { // Compute Odometry
      // Current Time
      timeCur = ros::Time::now();

      // Add Noise to Velocity Estimate
      double dt = (timeCur-timePrev).toSec();
      double vx = (xCur-x_)*dt;
      double vy = (yCur-y_)*dt;
      // double vz = (zCur-z_)*dt;
      // double vphi = (phiCur-phi_)*dt;
      // double vthe = (theCur-the_)*dt;
      double vpsi = (psiCur-psi_)*dt;
      if (addNoise==1) {
        const double mu = 0.0;
        std::random_device rd;
        std::mt19937 generator(rd());
        std::normal_distribution<double> vxNoise(mu,velocityCovariance);
        std::normal_distribution<double> vyNoise(mu,velocityCovariance);
        std::normal_distribution<double> vpsiNoise(mu,rotationRateCovariance);
        vx = vx + vxNoise(generator);
        vy = vy + vyNoise(generator);
        vpsi = vpsi + vpsiNoise(generator);
      }

      // Add Noise in Planar Coordinates Only
      double xOdom = xOdom_;
      double yOdom = yOdom_;
      double psiOdom = psiOdom_;
      if (addNoise==1) {
        const double mu = 0.0;
        std::random_device rd;
        std::mt19937 generator(rd());
        std::normal_distribution<double> xNoise(mu,positionCovariance);
        std::normal_distribution<double> yNoise(mu,positionCovariance);
        std::normal_distribution<double> psiNoise(mu,orientationCovariance);
        xOdom = xOdom + xNoise(generator);
        yOdom = yOdom + yNoise(generator);
        psiOdom = psiOdom + psiNoise(generator);
      }
      xOdom = xOdom + vx/dt;
      yOdom = yOdom + vy/dt;
      psiOdom = psiOdom + vpsi/dt;

      // Publish Odometry Transform
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(psiOdom);
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = timeCur;
      odom_trans.header.frame_id = "odom_gt_frame";
      odom_trans.child_frame_id = "base_gt_link";
      odom_trans.transform.translation.x = xOdom;
      odom_trans.transform.translation.y = yOdom;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
      tf::TransformBroadcaster odom_broadcaster;
      odom_broadcaster.sendTransform(odom_trans);

      // Set Final Planar Pose
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = timeCur;
      odomMsg.header.frame_id = "odom_gt_frame";
      odomMsg.pose.pose.position.x = xOdom;
      odomMsg.pose.pose.position.y = yOdom;
      odomMsg.pose.pose.position.z = 0.0;
      odomMsg.pose.pose.orientation = odom_quat;

      // Set Final Planar Velocity
      odomMsg.child_frame_id = "base_gt_link";
      odomMsg.twist.twist.linear.x = vx;
      odomMsg.twist.twist.linear.y = vy;
      odomMsg.twist.twist.linear.z = 0.0;
      odomMsg.twist.twist.angular.x = 0.0;
      odomMsg.twist.twist.angular.y = 0.0;
      odomMsg.twist.twist.angular.z = vpsi;

      // Publish Odometry Message
      odom_pub_.publish(odomMsg);

      // Set Previous Values
      timePrevEst = timeCur;
      x_ = xCur;
      y_ = yCur;
      z_ = zCur;
      phi_ = psiCur;
      the_ = theCur;
      psi_ = psiCur;
      xOdom_ = xOdom;
      yOdom_ = yOdom;
      zOdom_ = 0.0;
      phiOdom_ = 0.0;
      theOdom_ = 0.0;
      psiOdom_ = psiOdom;

      // Display Data
      if (displayData==1) {
        std::cout << "Ground Truth Pose:" << endl << "x: " << xCur << endl << "y: " << yCur << endl << "psi: " << psiCur << endl << endl;
        std::cout << "Odometry Pose:" << endl << "x: " << xOdom << endl << "y: " << yOdom << endl << "psi: " << psiOdom << endl << endl;
        // std::cout << "Velocity:" << endl << "vx: " << vx << endl << "vy: " << vy << endl << "vpsi: " << vpsi << endl << endl;
      }

    } // and if not initializing

  }

  void imuCallback(const sensor_msgs::Imu& msg)
  {
    // Extract Data
    // double qx, qy, qz, qw;
    // qx = msg.orientation.x;
    // qy = msg.orientation.y;
    // qz = msg.orientation.z;
    // qw = msg.orientation.w;
    imu_x = msg.linear_acceleration.x;
    imu_y = msg.linear_acceleration.y;
    imu_z = msg.linear_acceleration.z;
    gyro_x = msg.angular_velocity.x;
    gyro_y = msg.angular_velocity.y;
    gyro_z = msg.angular_velocity.z;
  }

  void wheelSpeedCallback(const ouijabot::Wheel_Spd& msg)
  {
    // Extract Data
    double w1 = msg.wheel_spd[0];
    double w2 = msg.wheel_spd[1];
    double w3 = msg.wheel_spd[2];
    double w4 = msg.wheel_spd[3];

    if ( initialize==0 ) {

      // Current Time
      timeCurEst = ros::Time::now();

      // Odometry from Wheel Speeds
      double rw = 0.08;
      double vx = (sqrt(2.0)*rw/4.0) * (w3-w1+w4-w2);
      double vy = (sqrt(2.0)*rw/4.0) * (w3-w1+w2-w4);
      double vpsi = gyro_z;

      // Estimate Robot Pose
      double dt = (timeCurEst-timePrevEst).toSec();
      double psiOdom = psiOdomEst_ + vpsi*dt;
      double xOdom = xOdomEst_ + cos(psiOdom)*vx*dt - sin(psiOdom)*vy*dt;
      double yOdom = yOdomEst_ + sin(psiOdom)*vx*dt + cos(psiOdom)*vy*dt;

      // Publish Odometry Transform
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(psiOdom);
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = timeCurEst;
      odom_trans.header.frame_id = "/odom_frame";
      odom_trans.child_frame_id = "/base_link";
      odom_trans.transform.translation.x = xOdom;
      odom_trans.transform.translation.y = yOdom;
      odom_trans.transform.translation.z = 0.0;
      odom_trans.transform.rotation = odom_quat;
      tf::TransformBroadcaster odom_broadcaster;
      odom_broadcaster.sendTransform(odom_trans);

      // Set Final Planar Pose
      nav_msgs::Odometry odomMsg;
      odomMsg.header.stamp = timeCurEst;
      odomMsg.header.frame_id = "/odom_frame";
      odomMsg.pose.pose.position.x = xOdom;
      odomMsg.pose.pose.position.y = yOdom;
      odomMsg.pose.pose.position.z = 0.0;
      odomMsg.pose.pose.orientation = odom_quat;

      // Set Final Planar Velocity
      odomMsg.child_frame_id = "/base_link";
      odomMsg.twist.twist.linear.x = vx;
      odomMsg.twist.twist.linear.y = vy;
      odomMsg.twist.twist.linear.z = 0.0;
      odomMsg.twist.twist.angular.x = 0.0;
      odomMsg.twist.twist.angular.y = 0.0;
      odomMsg.twist.twist.angular.z = vpsi;

      // Publish Odometry Message
      odom_pub_.publish(odomMsg);

      // Set Previous Values
      xOdomEst_ = xOdom;
      yOdomEst_ = yOdom;
      zOdomEst_ = 0.0;
      phiOdomEst_ = 0.0;
      theOdomEst_ = 0.0;
      psiOdomEst_ = psiOdom;
      timePrevEst = timeCurEst;

      // Display Data
      if (displayData==1) {
        std::cout << "Estimated Pose:" << endl << "x: " << xOdom << endl << "y: " << yOdom << endl << "psi: " << psiOdom << endl << endl;
        // std::cout << "Estimated Velocity:" << endl << "vx: " << vx << endl << "vy: " << vy << endl << "vpsi: " << vpsi << endl << endl;
      }

    }

  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mocap_interface_odom");
  ros::NodeHandle nh("~");
  
  int displayData;
  nh.param<int>("DISPLAY_DATA_BOOL", displayData, 0);
  int useOptitrack;
  nh.param<int>("USE_OPTITRACK_BOOL", useOptitrack, 0);
  double positionCovariance;
  nh.param<double>("POSITION_COVARIANCE", positionCovariance, 0.0);
  double orientationCovariance;
  nh.param<double>("ORIENTATION_COVARIANCE", orientationCovariance, 0.0);
  double velocityCovariance;
  nh.param<double>("VELOCITY_COVARIANCE", velocityCovariance, 0.0);
  double rotationRateCovariance;
  nh.param<double>("ROTATION_RATE_COVARIANCE", rotationRateCovariance, 0.0);

  mocapInterfaceClass poseEstimation(displayData, useOptitrack, positionCovariance, orientationCovariance, velocityCovariance, rotationRateCovariance);

  while (ros::ok()) {
    ros::spinOnce();
  }
  ros::shutdown();
  
  return 0;
}

